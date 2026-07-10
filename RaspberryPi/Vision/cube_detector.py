#!/usr/bin/env python3
"""
多色方块精确识别与3D定位系统
==============================================
功能：通过摄像头精确识别边长10cm的EVA方块（支持多色），
      输出方块相对于摄像头的3轴坐标（单位：mm）。
      槽位场景：方块下半被遮挡，可见面 100mm×50mm。

算法核心：
  1. HSV多色并行检测
  2. 自适应形态学处理与轮廓筛选（宽高比2:1）
  3. 亚像素级角点精炼
  4. 基于可见面尺寸的针孔相机模型反算
  5. 多测量融合 + 时序滤波 + 离群值剔除
  6. 多块检测 → 自动锁定距离最近的方块

坐标系（右手系，摄像头为原点）：
  X轴 → 右方为正
  Y轴 → 上方为正
  Z轴 → 前方（远离摄像头）为正

使用方法：
  python cube_detector.py [--calibrate] [--camera 1]
"""

import cv2
import numpy as np
import argparse
import json
import os
import sys
from collections import deque
from time import time, strftime

# ============================================================
# 全局配置
# ============================================================
CONFIG = {
    # 方块物理尺寸
    "cube_size_mm": 100.0,

    # 槽位参数：槽深 = 方块边长的一半 → 下半部分被遮挡
    "slot_depth_mm": 50.0,            # 槽深度
    "visible_height_mm": 50.0,        # 可见面高度 = cube_size - slot_depth

    # 相机参数（默认值，通过标定更新）
    "fx": 800.0,          # x方向焦距（像素）
    "fy": 800.0,          # y方向焦距（像素）
    "cx": 320.0,          # 主点x
    "cy": 240.0,          # 主点y

    # ---- 颜色配置表 ----
    # 每个颜色定义 HSV 范围 + 显示名称 + 绘制颜色(BGR)
    "color_profiles": [
        {
            "name": "Orange",
            "hsv_low": np.array([5, 80, 80]),
            "hsv_high": np.array([25, 255, 255]),
            "draw_color": (0, 140, 255),   # 橙色
        },
        {
            "name": "Purple",
            "hsv_low": np.array([130, 60, 60]),
            "hsv_high": np.array([165, 255, 255]),
            "draw_color": (200, 80, 180),  # 紫色
        },
    ],

    # 兼容旧代码：橙色HSV（指向 color_profiles[0]）
    "orange_lower_hsv": np.array([5, 80, 80]),
    "orange_upper_hsv": np.array([25, 255, 255]),

    # 橙色LAB范围（辅助验证，a通道偏红，b通道偏黄）
    "orange_lower_lab": np.array([20, 130, 130]),
    "orange_upper_lab": np.array([255, 180, 180]),

    # 形态学参数
    "morph_kernel_size": 5,
    "morph_iterations": 2,

    # 轮廓筛选
    "min_contour_area": 200,      # 最小轮廓面积（像素²），广角+远距离目标很小
    "max_aspect_ratio_dev": 0.65,  # 宽高比偏离容忍度（广角镜头边缘变形）
    "min_face_width_ratio": 0.10,  # 可见面宽度至少占画面宽度10%（640px→64px, Z≈1250mm）

    # 时序滤波
    "ema_alpha": 0.35,            # 指数移动平均平滑系数
    "history_size": 10,           # 历史窗口大小
    "max_position_jump_mm": 80,   # 单帧位置跳变阈值（用于离群值剔除）

    # 标定文件
    "calib_file": "camera_calib.json",
}

# 运行时状态
STATE = {
    "calibrated": False,
    "fx": CONFIG["fx"],
    "fy": CONFIG["fy"],
    "cx": CONFIG["cx"],
    "cy": CONFIG["cy"],
    "history": deque(maxlen=CONFIG["history_size"]),
    "ema_pos": None,       # (x, y, z) EMA平滑位置
    "locked_color": None,  # 当前锁定的方块颜色名
    "frame_count": 0,
    "fps": 0.0,
    "last_time": time(),
}


# ============================================================
# 相机标定模块
# ============================================================
def load_or_init_calibration():
    """加载标定文件，如不存在则使用默认值并返回未标定状态。"""
    calib_path = os.path.join(os.path.dirname(__file__), CONFIG["calib_file"])
    if os.path.exists(calib_path):
        try:
            with open(calib_path, "r") as f:
                data = json.load(f)
            STATE["fx"] = data["fx"]
            STATE["fy"] = data["fy"]
            STATE["cx"] = data["cx"]
            STATE["cy"] = data["cy"]

            # 加载各颜色的HSV阈值
            if "colors" in data:
                # 新版格式：每个颜色独立存储
                for profile in CONFIG["color_profiles"]:
                    name = profile["name"]
                    if name in data["colors"]:
                        c = data["colors"][name]
                        profile["hsv_low"] = np.array(c["hsv_low"])
                        profile["hsv_high"] = np.array(c["hsv_high"])
                        print(f"  [{name}] HSV: L={c['hsv_low']}, U={c['hsv_high']}")
            elif "hsv_low" in data:
                # 兼容旧版格式：仅橙色有HSV
                CONFIG["color_profiles"][0]["hsv_low"] = np.array(data["hsv_low"])
                CONFIG["color_profiles"][0]["hsv_high"] = np.array(data["hsv_high"])

            STATE["calibrated"] = True
            print(f"[标定] 已加载: fx={STATE['fx']:.1f}, fy={STATE['fy']:.1f}, "
                  f"cx={STATE['cx']:.1f}, cy={STATE['cy']:.1f}")
            return True
        except (KeyError, json.JSONDecodeError) as e:
            print(f"[警告] 标定文件损坏: {e}，使用默认参数")
    return False


def calibrate_from_known_distance(known_distance_mm, pixel_width, pixel_height):
    """
    使用已知距离标定焦距（考虑槽位遮挡）。
    宽度方向完整可见(100mm)，高度方向仅可见上半(50mm)。

    参数:
        known_distance_mm: 摄像头到方块可见面的实际距离
        pixel_width: 检测到的可见面像素宽度（对应100mm）
        pixel_height: 检测到的可见面像素高度（对应50mm）
    """
    fx = known_distance_mm * pixel_width / CONFIG["cube_size_mm"]
    fy = known_distance_mm * pixel_height / CONFIG["visible_height_mm"]
    return fx, fy


def save_calibration(fx, fy, cx, cy):
    """持久化标定参数（含所有颜色的HSV阈值）。"""
    calib_path = os.path.join(os.path.dirname(__file__), CONFIG["calib_file"])
    data = {"fx": fx, "fy": fy, "cx": cx, "cy": cy}
    # 保存每个颜色的HSV
    data["colors"] = {}
    for profile in CONFIG["color_profiles"]:
        data["colors"][profile["name"]] = {
            "hsv_low": [int(v) for v in profile["hsv_low"]],
            "hsv_high": [int(v) for v in profile["hsv_high"]],
        }
    with open(calib_path, "w") as f:
        json.dump(data, f, indent=2)
    STATE["fx"] = fx
    STATE["fy"] = fy
    STATE["cx"] = cx
    STATE["cy"] = cy
    STATE["calibrated"] = True
    print(f"[标定] 已保存标定到 {calib_path}")
    for profile in CONFIG["color_profiles"]:
        print(f"  [{profile['name']}] HSV: L={profile['hsv_low'].tolist()}, "
              f"U={profile['hsv_high'].tolist()}")


def interactive_calibration(cap):
    """
    交互式标定流程（支持分别标定各颜色）：
    - 按 Tab 切换当前标定的颜色
    - 每种颜色独立调整HSV阈值
    - 按空格键采集标定数据（焦距用于所有颜色，HS V独立保存）
    """
    profiles = CONFIG["color_profiles"]
    n_colors = len(profiles)

    print("\n" + "=" * 60)
    print("  相机标定模式 (多颜色)")
    print("=" * 60)
    print(f"  可标定颜色: {', '.join(p['name'] for p in profiles)}")
    print("  按 Tab   切换当前标定的颜色")
    print("  按 空格  采集一帧进行焦距标定")
    print("  按 ESC   保存并退出标定")
    print("=" * 60)

    samples = []
    target_distance = 300
    color_idx = 0  # 当前选中的颜色索引

    cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)
    cv2.createTrackbar("Dist(mm)", "Calibration", target_distance, 1000, lambda v: None)
    cv2.createTrackbar("Color(0=O,1=P...)", "Calibration", 0, n_colors - 1, lambda v: None)
    cv2.createTrackbar("H_Low", "Calibration", 0, 179, lambda v: None)
    cv2.createTrackbar("H_High", "Calibration", 0, 179, lambda v: None)
    cv2.createTrackbar("S_Low", "Calibration", 0, 255, lambda v: None)
    cv2.createTrackbar("V_Low", "Calibration", 0, 255, lambda v: None)

    # 加载当前颜色的HSV到滑块
    def _load_trackbars_for_color(idx):
        p = profiles[idx]
        cv2.setTrackbarPos("Color(0=O,1=P...)", "Calibration", idx)
        cv2.setTrackbarPos("H_Low", "Calibration", int(p["hsv_low"][0]))
        cv2.setTrackbarPos("H_High", "Calibration", int(p["hsv_high"][0]))
        cv2.setTrackbarPos("S_Low", "Calibration", int(p["hsv_low"][1]))
        cv2.setTrackbarPos("V_Low", "Calibration", int(p["hsv_low"][2]))

    _load_trackbars_for_color(0)
    last_color_idx = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 读取 trackbar
        target_distance = cv2.getTrackbarPos("Dist(mm)", "Calibration")
        tk_color = cv2.getTrackbarPos("Color(0=O,1=P...)", "Calibration")
        h_low = cv2.getTrackbarPos("H_Low", "Calibration")
        h_high = cv2.getTrackbarPos("H_High", "Calibration")
        s_low = cv2.getTrackbarPos("S_Low", "Calibration")
        v_low = cv2.getTrackbarPos("V_Low", "Calibration")

        # 检测颜色切换：把修改后的值存回当前颜色，再切换到新颜色
        if tk_color != last_color_idx:
            # 先保存当前滑块值到当前颜色
            profiles[last_color_idx]["hsv_low"] = np.array([h_low, s_low, v_low])
            profiles[last_color_idx]["hsv_high"] = np.array([h_high, 255, 255])
            # 切换到新颜色
            last_color_idx = tk_color
            _load_trackbars_for_color(tk_color)
            # 切完重新读滑块
            h_low = cv2.getTrackbarPos("H_Low", "Calibration")
            h_high = cv2.getTrackbarPos("H_High", "Calibration")
            s_low = cv2.getTrackbarPos("S_Low", "Calibration")
            v_low = cv2.getTrackbarPos("V_Low", "Calibration")
            print(f"\n[切换] 当前标定颜色: {profiles[tk_color]['name']}")

        # 始终把当前滑块值实时写入颜色配置（帧间生效）
        color_idx = tk_color
        profiles[color_idx]["hsv_low"] = np.array([h_low, s_low, v_low])
        profiles[color_idx]["hsv_high"] = np.array([h_high, 255, 255])

        temp_lower = np.array([h_low, s_low, v_low])
        temp_upper = np.array([h_high, 255, 255])

        frame_h, frame_w = frame.shape[:2]
        STATE["cx"] = frame_w / 2.0
        STATE["cy"] = frame_h / 2.0

        # 使用当前颜色检测
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV),
                           temp_lower, temp_upper)
        ksize = CONFIG["morph_kernel_size"]
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel,
                                iterations=CONFIG["morph_iterations"])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel,
                                iterations=CONFIG["morph_iterations"])

        display = cv2.bitwise_and(frame, frame, mask=mask)
        quad = find_quadrilateral(mask, min_area=200)

        if quad is not None and len(quad) == 4:
            try:
                cv2.drawContours(frame, [quad.astype(np.int32)], -1,
                                 profiles[color_idx]["draw_color"], 2)
                for pt in quad:
                    cv2.circle(frame, tuple(pt.astype(int)), 5, (0, 0, 255), -1)
            except cv2.error:
                pass

            w_px, h_px = measure_face_size(quad)
            cv2.putText(frame, f"W:{w_px:.0f}px H:{h_px:.0f}px",
                        (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            fx_test, fy_test = calibrate_from_known_distance(target_distance, w_px, h_px)
            z_test = STATE["fx"] * CONFIG["cube_size_mm"] / max(w_px, 1)
            cv2.putText(frame, f"Est.fx={fx_test:.0f} fy={fy_test:.0f}",
                        (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"Target Z={target_distance}mm, Measured Z={z_test:.0f}mm",
                        (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # 当前标定的颜色名称
        color_name = profiles[color_idx]["name"]
        cv2.putText(frame, f"[{color_name}] Target Distance: {target_distance} mm",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    profiles[color_idx]["draw_color"], 2)

        combined = np.hstack([frame, display])
        cv2.imshow("Calibration", combined)

        key = cv2.waitKey(30) & 0xFF
        if key == 27:  # ESC
            break
        elif key == 32:  # Space - 采集焦距样本
            if quad is not None:
                w_px, h_px = measure_face_size(quad)
                samples.append((w_px, h_px, target_distance))
                print(f"[采集] #{len(samples)} ({color_name}): "
                      f"W={w_px:.1f}px, H={h_px:.1f}px, Dist={target_distance}mm")
            else:
                print(f"[采集] 未检测到 {color_name} 方块，请调整位置或HSV阈值")
        elif key == 9:  # Tab - 也可以用键盘切换颜色
            next_idx = (color_idx + 1) % n_colors
            cv2.setTrackbarPos("Color(0=O,1=P...)", "Calibration", next_idx)
            print(f"\n[切换] 当前标定颜色: {profiles[next_idx]['name']}")

    cv2.destroyWindow("Calibration")

    if len(samples) >= 1:
        fxs = [d * w / CONFIG["cube_size_mm"] for w, h, d in samples]
        fys = [d * h / CONFIG["cube_size_mm"] for w, h, d in samples]
        fx_med = np.median(fxs)
        fy_med = np.median(fys)
        cx = STATE["cx"]
        cy = STATE["cy"]

        print(f"\n[标定结果] {len(samples)} 个焦距样本")
        print(f"  fx = {fx_med:.1f} (std={np.std(fxs):.1f})")
        print(f"  fy = {fy_med:.1f} (std={np.std(fys):.1f})")
        print(f"  cx = {cx:.1f}, cy = {cy:.1f}")

        save_calibration(fx_med, fy_med, cx, cy)
        return True

    return False


# ============================================================
# 橙色区域检测
# ============================================================
def detect_orange_region(frame, lower=None, upper=None):
    """
    多阶段橙色区域检测：
    阶段1: HSV颜色空间阈值
    阶段2: 形态学开闭运算去除噪声
    阶段3: 连通域分析保留最大区域

    返回: 二值掩码
    """
    if lower is None:
        lower = CONFIG["orange_lower_hsv"]
    if upper is None:
        upper = CONFIG["orange_upper_hsv"]

    # 转换到HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # ---- 阶段1: HSV阈值 ----
    # 橙色在HSV中H值在5-25左右，但也会跨过0/180边界
    # 对于跨边界情况做双区间处理
    if lower[0] < 0:
        # 需要处理环绕：例如 H范围 [170-180] ∪ [0-10]
        mask1 = cv2.inRange(hsv, np.array([0, lower[1], lower[2]]),
                            np.array([upper[0], upper[1], upper[2]]))
        mask2 = cv2.inRange(hsv, np.array([180 + lower[0], lower[1], lower[2]]),
                            np.array([179, upper[1], upper[2]]))
        mask_hsv = cv2.bitwise_or(mask1, mask2)
    else:
        mask_hsv = cv2.inRange(hsv, lower, upper)

    # ---- 阶段2: 形态学处理 ----
    ksize = CONFIG["morph_kernel_size"]
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))

    # 先闭运算（填充小洞），再开运算（去除噪点）
    mask = cv2.morphologyEx(mask_hsv, cv2.MORPH_CLOSE, kernel,
                            iterations=CONFIG["morph_iterations"])
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel,
                            iterations=CONFIG["morph_iterations"])

    # ---- 阶段3: 只保留最大连通域 ----
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        mask, connectivity=8)

    if num_labels <= 1:
        return mask  # 无前景

    # 跳过背景(label=0)，找最大面积的前景
    areas = stats[1:, cv2.CC_STAT_AREA]
    max_idx = np.argmax(areas) + 1
    mask_clean = np.zeros_like(mask)
    mask_clean[labels == max_idx] = 255

    return mask_clean


# ============================================================
# 四边形检测与精炼
# ============================================================
def find_quadrilaterals(mask, min_area=None):
    """
    从二值掩码中找到所有有效四边形。
    使用轮廓检测 → 多边形逼近 → 筛选四边形。

    返回: [(4,2) numpy数组, ...]  按面积降序排列，空列表表示无检测
    """
    if min_area is None:
        min_area = CONFIG["min_contour_area"]

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []

    results = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        # 计算凸包
        hull = cv2.convexHull(cnt)

        # 多边形逼近
        peri = cv2.arcLength(hull, True)
        for epsilon_factor in [0.02, 0.03, 0.04, 0.05]:
            approx = cv2.approxPolyDP(hull, epsilon_factor * peri, True)

            if len(approx) == 4:
                quad = approx.reshape(4, 2)

                # 排序角点（左上、右上、右下、左下）
                quad = order_corners(quad)

                # 验证宽高比是否合理（可见面 100mm×50mm → 期望比 ≈ 2.0）
                aspect_ratio = compute_quad_aspect_ratio(quad)
                expected_ar = CONFIG["cube_size_mm"] / CONFIG["visible_height_mm"]
                if abs(aspect_ratio - expected_ar) < CONFIG["max_aspect_ratio_dev"]:
                    results.append((area, quad))
                break  # 找到四边形就不再减小epsilon

    # 按面积降序排列
    results.sort(key=lambda x: x[0], reverse=True)
    return [quad for area, quad in results]


# 兼容旧代码的单四边形检测
def find_quadrilateral(mask, min_area=None):
    """返回最佳四边形（兼容旧接口）。"""
    quads = find_quadrilaterals(mask, min_area)
    return quads[0] if quads else None


def order_corners(pts):
    """
    将4个角点排序为：左上、右上、右下、左下
    """
    # 按x+y排序（左上最小，右下最大）
    rect = np.zeros((4, 2), dtype=np.float32)
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]  # 左上
    rect[2] = pts[np.argmax(s)]  # 右下

    # 按y-x排序（右上最小，左下最大）
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  # 右上
    rect[3] = pts[np.argmax(diff)]  # 左下

    return rect


def compute_quad_aspect_ratio(quad):
    """计算四边形的长宽比。"""
    w1 = np.linalg.norm(quad[1] - quad[0])
    w2 = np.linalg.norm(quad[2] - quad[3])
    h1 = np.linalg.norm(quad[3] - quad[0])
    h2 = np.linalg.norm(quad[2] - quad[1])

    avg_w = (w1 + w2) / 2.0
    avg_h = (h1 + h2) / 2.0

    if avg_h < 1e-6:
        return 999
    return avg_w / avg_h


def measure_face_size(quad):
    """
    测量四边形面的像素尺寸。
    使用两种方法并融合：
      - 对边平均法
      - 面积开方法

    返回: (width_px, height_px)
    """
    w1 = np.linalg.norm(quad[1] - quad[0])
    w2 = np.linalg.norm(quad[2] - quad[3])
    h1 = np.linalg.norm(quad[3] - quad[0])
    h2 = np.linalg.norm(quad[2] - quad[1])

    # 对边平均
    w_avg = (w1 + w2) / 2.0
    h_avg = (h1 + h2) / 2.0

    # 面积开方验证
    area = cv2.contourArea(quad.astype(np.float32).reshape(4, 1, 2))
    size_from_area = np.sqrt(area)

    # 融合（取平均+面积法的加权平均）
    w_final = w_avg * 0.7 + size_from_area * 0.3
    h_final = h_avg * 0.7 + size_from_area * 0.3

    return w_final, h_final


def refine_corners_subpix(gray, quad):
    """
    亚像素级角点精炼。
    使用迭代Lucas-Kanade方法将角点精确到亚像素级别。

    返回: (4, 2) 精炼后的角点数组 或 原始quad（精炼失败时）
    """
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
    try:
        refined = cv2.cornerSubPix(gray, quad.astype(np.float32),
                                   (5, 5), (-1, -1), criteria)
        return refined
    except cv2.error:
        return quad


# ============================================================
# 3D位置计算
# ============================================================
def compute_3d_position(quad, fx, fy, cx, cy):
    """
    根据检测到的四边形和相机内参计算3D位置。

    槽位场景：方块下半部分被遮挡，可见面为 100mm×50mm 矩形。
      - 宽度方向：完整 100mm
      - 高度方向：仅可见 50mm
      - 检测到的面中心比实际方块中心高 25mm

    返回: (x_mm, y_mm, z_mm, confidence)
    """
    full_w = CONFIG["cube_size_mm"]           # 100mm（完整宽度）
    vis_h = CONFIG["visible_height_mm"]       # 50mm（可见高度）
    y_offset = (full_w - vis_h) / 2.0         # 25mm（面心→块心偏移）

    # 四边形中心
    center_u = np.mean(quad[:, 0])
    center_v = np.mean(quad[:, 1])

    # 像素尺寸测量
    w_px, h_px = measure_face_size(quad)

    # ---- 多测量融合计算Z ----
    # 宽度推算（100mm 完整可见，最可靠）
    z_from_w = (fx * full_w) / max(w_px, 1e-6)
    # 高度推算（仅50mm 可见）
    z_from_h = (fy * vis_h) / max(h_px, 1e-6)

    # 检测可靠性：宽高比越接近期望值(2.0)越可靠
    expected_ar = full_w / vis_h  # 2.0
    ar = w_px / max(h_px, 1e-6)
    ar_dev = abs(ar - expected_ar) / expected_ar
    aspect_conf = max(0.0, 1.0 - ar_dev)  # 1.0 = 完美匹配

    # 加权融合：宽度测量更可靠（完整可见），权重更高
    w_weight = aspect_conf * 1.5
    h_weight = aspect_conf * 1.0
    total_weight = w_weight + h_weight

    z_mm = (z_from_w * w_weight + z_from_h * h_weight) / max(total_weight, 1e-6)

    # ---- 计算方块实际中心的 X, Y ----
    # 注意：quad中心是可见面的中心，需加Y偏移得到方块实际中心
    x_mm = (center_u - cx) * z_mm / fx
    y_visible_center = -(center_v - cy) * z_mm / fy   # 可见面中心Y
    y_mm = y_visible_center - y_offset                 # 方块实际中心Y（下方25mm）

    # ---- 可信度评估 ----
    confidence = min(aspect_conf * 100, 100)

    # 合理性检查
    if z_mm < 20 or z_mm > 5000:
        confidence = 0

    return x_mm, y_mm, z_mm, confidence


# ============================================================
# 时序滤波
# ============================================================
def temporal_filter(x_mm, y_mm, z_mm, confidence):
    """
    时序滤波：指数移动平均 + 离群值剔除

    返回: (filtered_x, filtered_y, filtered_z, is_valid)
    """
    # 离群值检测
    if STATE["ema_pos"] is not None and confidence > 30:
        prev_x, prev_y, prev_z = STATE["ema_pos"]
        jump = np.sqrt((x_mm - prev_x)**2 + (y_mm - prev_y)**2 + (z_mm - prev_z)**2)
        if jump > CONFIG["max_position_jump_mm"]:
            # 跳变过大，降低置信度
            confidence *= 0.3

    # EMA平滑
    alpha = CONFIG["ema_alpha"]
    if STATE["ema_pos"] is None:
        STATE["ema_pos"] = (x_mm, y_mm, z_mm)
    else:
        px, py, pz = STATE["ema_pos"]
        alpha_eff = alpha * (confidence / 100.0)
        alpha_eff = np.clip(alpha_eff, 0.05, 0.8)
        STATE["ema_pos"] = (
            px + alpha_eff * (x_mm - px),
            py + alpha_eff * (y_mm - py),
            pz + alpha_eff * (z_mm - pz),
        )

    fx, fy, fz = STATE["ema_pos"]

    # 是否有效
    is_valid = confidence > 25

    return fx, fy, fz, is_valid


# ============================================================
# 可视化
# ============================================================
def draw_overlay(frame, all_blocks, x, y, z, confidence, is_valid, locked_color,
                  fx, fy, cx, cy):
    """绘制多块检测结果的可视化叠加层。"""
    h, w = frame.shape[:2]

    # ---- 绘制所有检测到的四边形 ----
    if all_blocks:

        for i, block in enumerate(all_blocks):
            quad = block["quad"]
            color = block["draw_color"]
            is_locked = (i == 0)

            if quad is not None and len(quad) == 4:
                try:
                    # 锁定方块：粗实线 + 角点标号；其他：细线
                    thickness = 3 if is_locked else 1
                    cv2.drawContours(frame, [quad.astype(np.int32)], -1, color, thickness)

                    if is_locked:
                        # 锁定方块画角点和十字线
                        for pt in quad:
                            cv2.circle(frame, tuple(pt.astype(int)), 4,
                                       (0, 0, 255), -1)
                        center = (int(np.mean(quad[:, 0])), int(np.mean(quad[:, 1])))
                        cv2.circle(frame, center, 6, (0, 255, 255), -1)
                        cv2.line(frame, (center[0] - 20, center[1]),
                                 (center[0] + 20, center[1]), (0, 255, 255), 1)
                        cv2.line(frame, (center[0], center[1] - 20),
                                 (center[0], center[1] + 20), (0, 255, 255), 1)
                except cv2.error:
                    pass

            # 标签：颜色名 + Z距离
            cx_b = int(np.mean(quad[:, 0]))
            cy_b = int(np.mean(quad[:, 1]))
            label = f"{block['color_name']} Z={block['z']:.0f}mm"
            cv2.putText(frame, label, (cx_b - 40, cy_b - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # ---- 绘制图像中心（光轴） ----
    cv2.drawMarker(frame, (int(cx), int(cy)), (128, 128, 128),
                   cv2.MARKER_CROSS, 20, 1)

    # ---- 3D坐标信息面板 ----
    panel_x = 12
    panel_y = h - 185
    panel_w = 340
    panel_h = 170

    # 半透明背景
    overlay = frame.copy()
    cv2.rectangle(overlay, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h),
                  (30, 30, 30), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

    # 状态指示器
    status_color = (0, 255, 0) if is_valid else (0, 0, 255)
    status_text = f"● LOCKED: {locked_color}" if is_valid else "○ SEARCHING"
    cv2.putText(frame, status_text, (panel_x + 10, panel_y + 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 2)

    # 3D坐标 + 检测数量
    detected_count = len(all_blocks)
    cv2.putText(frame, f"Blocks detected: {detected_count}", (panel_x + 10, panel_y + 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    if is_valid:
        lines = [
            f"X: {x:+.1f} mm  (right+)",
            f"Y: {y:+.1f} mm  (up+)",
            f"Z: {z:+.1f} mm  (forward+)",
            f"Distance: {np.sqrt(x**2+y**2+z**2):.1f} mm",
            f"Confidence: {confidence:.0f}%",
        ]
        for i, line in enumerate(lines):
            color = [(0, 200, 255), (0, 255, 200), (255, 200, 0), (255, 255, 255), (200, 200, 200)][i]
            cv2.putText(frame, line, (panel_x + 10, panel_y + 75 + i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    # ---- 顶部信息栏 ----
    cv2.putText(frame, f"FPS: {STATE['fps']:.1f}", (w - 120, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    cv2.putText(frame, f"fx:{STATE['fx']:.0f} fy:{STATE['fy']:.0f}", (w - 120, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    calib_text = "CALIBRATED" if STATE["calibrated"] else "UNCALIBRATED"
    calib_color = (0, 255, 100) if STATE["calibrated"] else (0, 140, 255)
    cv2.putText(frame, calib_text, (w - 160, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, calib_color, 1)

    return frame


# ============================================================
# 多颜色多方块检测流水线
# ============================================================
def detect_all_blocks(frame):
    """
    检测画面中所有颜色配置的方块。

    返回: [block_dict, ...]  按Z距离升序排列（最近的在前）
          block_dict = {
              "quad": (4,2) array,
              "color_name": "Orange"|"Purple",
              "draw_color": (B,G,R),
              "x": float, "y": float, "z": float,
              "confidence": float,
          }
    """
    fx, fy = STATE["fx"], STATE["fy"]
    cx, cy = STATE["cx"], STATE["cy"]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    all_blocks = []

    for profile in CONFIG["color_profiles"]:
        lower = profile["hsv_low"]
        upper = profile["hsv_high"]

        # HSV阈值
        mask = cv2.inRange(hsv, lower, upper)

        # 形态学处理
        ksize = CONFIG["morph_kernel_size"]
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel,
                                iterations=CONFIG["morph_iterations"])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel,
                                iterations=CONFIG["morph_iterations"])

        # 找出所有四边形
        quads = find_quadrilaterals(mask)

        for quad in quads:
            # 尺寸过滤：可见面宽度必须占画面一定比例，排除远距离噪点
            w_px, _ = measure_face_size(quad)
            min_width_px = frame.shape[1] * CONFIG["min_face_width_ratio"]
            if w_px < min_width_px:
                continue

            # 亚像素精炼
            quad_refined = refine_corners_subpix(gray, quad)
            quad = quad_refined

            # 计算3D位置
            x_mm, y_mm, z_mm, confidence = compute_3d_position(
                quad, fx, fy, cx, cy)

            all_blocks.append({
                "quad": quad,
                "color_name": profile["name"],
                "draw_color": profile["draw_color"],
                "x": x_mm,
                "y": y_mm,
                "z": z_mm,
                "confidence": confidence,
            })

    # 按Z距离升序（最近的在前）
    all_blocks.sort(key=lambda b: b["z"])
    return all_blocks


# ============================================================
# 主检测流水线
# ============================================================
def process_frame(frame):
    """
    完整的单帧处理流水线：多颜色检测 → 锁定最近方块 → 时序滤波。

    返回: (processed_frame, (x, y, z, confidence, is_valid, color_name, all_blocks))
    """
    h, w = frame.shape[:2]
    STATE["cx"] = w / 2.0
    STATE["cy"] = h / 2.0
    cx, cy = STATE["cx"], STATE["cy"]
    fx, fy = STATE["fx"], STATE["fy"]

    # Step 1: 多颜色多块检测
    all_blocks = detect_all_blocks(frame)

    x_mm, y_mm, z_mm = 0.0, 0.0, 0.0
    confidence = 0.0
    is_valid = False
    locked_color = None

    if all_blocks:
        # Step 2: 锁定距离最近的方块（已按Z升序排列）
        locked = all_blocks[0]

        if locked["confidence"] > 0:
            x_mm, y_mm, z_mm = locked["x"], locked["y"], locked["z"]
            confidence = locked["confidence"]
            locked_color = locked["color_name"]

            # Step 3: 时序滤波
            fx_mm, fy_mm, fz_mm, is_valid = temporal_filter(
                x_mm, y_mm, z_mm, confidence)
            STATE["locked_color"] = locked_color
        else:
            fx_mm, fy_mm, fz_mm = x_mm, y_mm, z_mm
            is_valid = False
    else:
        # 无检测 → 衰减
        if STATE["ema_pos"] is not None:
            fx_mm, fy_mm, fz_mm = STATE["ema_pos"]
            is_valid = False
        else:
            fx_mm, fy_mm, fz_mm = 0.0, 0.0, 0.0
            is_valid = False

    # Step 4: 可视化
    display = draw_overlay(frame.copy(), all_blocks,
                           fx_mm if is_valid else x_mm,
                           fy_mm if is_valid else y_mm,
                           fz_mm if is_valid else z_mm,
                           confidence, is_valid,
                           locked_color,
                           fx, fy, cx, cy)

    return display, (fx_mm if is_valid else x_mm,
                     fy_mm if is_valid else y_mm,
                     fz_mm if is_valid else z_mm,
                     confidence, is_valid,
                     locked_color, all_blocks)


# ============================================================
# 命令行接口
# ============================================================
def main():
    parser = argparse.ArgumentParser(
        description="橙色EVA方块精确识别与3D定位系统")
    parser.add_argument("--camera", type=int, default=1,
                        help="摄像头索引 (默认: 1)")
    parser.add_argument("--calibrate", action="store_true",
                        help="启动交互式标定模式")
    parser.add_argument("--output", type=str, default=None,
                        help="将坐标输出到指定文件（持续更新）")
    parser.add_argument("--no-gui", action="store_true",
                        help="无GUI模式（仅控制台输出坐标）")
    parser.add_argument("--stream", action="store_true",
                        help="持续流式输出模式（每帧输出，无GUI时自动启用）")
    parser.add_argument("--resolution", type=str, default="640x480",
                        help="摄像头分辨率 WxH (默认: 640x480)")
    args = parser.parse_args()

    # 解析分辨率
    res_w, res_h = map(int, args.resolution.split("x"))

    # 打开摄像头
    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"[错误] 无法打开摄像头 {args.camera}")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, res_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res_h)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # 关闭自动对焦（固定焦距）

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[摄像头] {actual_w}x{actual_h} @ Camera {args.camera}")

    # 加载标定
    load_or_init_calibration()
    STATE["cx"] = actual_w / 2.0
    STATE["cy"] = actual_h / 2.0

    # 标定模式
    if args.calibrate:
        interactive_calibration(cap)
        print("[标定] 标定完成，开始检测...")

    # 输出文件
    out_file = None
    if args.output:
        out_file = open(args.output, "w")
        out_file.write("# t(s), X(mm), Y(mm), Z(mm), Confidence(%), Valid, Color\n")

    print("\n" + "=" * 60)
    print("  多色方块3D定位系统 (Orange + Purple)")
    print("=" * 60)
    print(f"  方块尺寸: {CONFIG['cube_size_mm']}mm 立方体")
    print(f"  焦距: fx={STATE['fx']:.0f}, fy={STATE['fy']:.0f}")
    print(f"  检测颜色: {', '.join(p['name'] for p in CONFIG['color_profiles'])}")
    print(f"  策略: 自动锁定距离最近的方块")
    print(f"  按 ESC  退出")
    print(f"  按 c    重新标定")
    print(f"  按 r    重置EMA滤波器")
    print("=" * 60 + "\n")

    # 主循环
    stream_mode = args.stream or args.no_gui  # 无GUI时自动启用换行流式输出
    frame_idx = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[错误] 读取帧失败")
            break

        # 处理
        display, result = process_frame(frame)
        x, y, z, conf, valid, color_name, all_blocks = result
        frame_idx += 1

        # FPS
        STATE["frame_count"] += 1
        now = time()
        if now - STATE["last_time"] >= 1.0:
            STATE["fps"] = STATE["frame_count"] / (now - STATE["last_time"])
            STATE["frame_count"] = 0
            STATE["last_time"] = now

        # 控制台输出 — 默认每2帧打印一次，持续返回方块位置
        if frame_idx % 2 == 0:
            if valid:
                dist = np.sqrt(x ** 2 + y ** 2 + z ** 2)
                block_info = f"[{color_name}]" if color_name else ""
                if stream_mode:
                    print(f"{block_info} [X:{x:+7.1f}  Y:{y:+7.1f}  Z:{z:+7.1f}] mm  "
                          f"Dist:{dist:6.1f}mm  Conf:{conf:5.1f}%  "
                          f"({len(all_blocks)} blocks)",
                          flush=True)
                else:
                    print(f"\r{block_info} [X:{x:+7.1f}  Y:{y:+7.1f}  Z:{z:+7.1f}] mm  "
                          f"Dist:{dist:6.1f}mm  Conf:{conf:5.1f}%  "
                          f"({len(all_blocks)} blocks)  FPS:{STATE['fps']:4.1f}",
                          end="", flush=True)
            else:
                hint = f"({len(all_blocks)} blocks seen)" if all_blocks else "(放入方块)"
                if stream_mode:
                    print(f"[SEARCHING]  {hint}  FPS:{STATE['fps']:4.1f}", flush=True)
                else:
                    print(f"\r[SEARCHING]  {hint}  FPS:{STATE['fps']:4.1f}  ",
                          end="", flush=True)

        # 输出到文件
        if out_file:
            out_file.write(f"{now:.3f}, {x:.2f}, {y:.2f}, {z:.2f}, {conf:.1f}, "
                           f"{1 if valid else 0}, {color_name or 'N/A'}\n")
            out_file.flush()

        # 显示
        if not args.no_gui:
            cv2.imshow("Orange Cube Detector", display)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
            elif key == ord('c'):
                print("\n[重新标定]")
                interactive_calibration(cap)
            elif key == ord('r'):
                STATE["ema_pos"] = None
                STATE["history"].clear()
                print("\n[EMA滤波器已重置]")
        else:
            # 无GUI模式，按Ctrl+C退出
            try:
                if cv2.waitKey(1) & 0xFF == 27:
                    break
            except KeyboardInterrupt:
                break

    # 清理
    cap.release()
    cv2.destroyAllWindows()
    if out_file:
        out_file.close()
    print("\n[退出] 程序结束")


if __name__ == "__main__":
    main()
