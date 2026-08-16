#!/usr/bin/env python3
"""
多色方块精确识别与3D定位系统 — 集成模块版本
==============================================
从桌面视觉程序重构，支持作为 Robot 子系统运行。

功能：通过摄像头识别边长10cm的EVA方块（多色），
      输出方块相对于摄像头的3轴坐标（mm）。

算法核心：
  1. HSV多色并行检测
  2. 自适应形态学处理与轮廓筛选
  3. 亚像素级角点精炼
  4. 基于可见面尺寸的针孔相机模型反算
  5. 多测量融合 + 时序滤波 + 离群值剔除
  6. 多块检测 → 自动锁定距离最近的方块

坐标系（右手系，摄像头为原点）：
  X轴 → 右方为正
  Y轴 → 上方为正
  Z轴 → 前方（远离摄像头）为正

使用方法：
  from vision import CubeDetector
  det = CubeDetector(camera_id=1, show_gui=False)
  det.start()
  result = det.result  # VisionResult or None
  det.stop()
"""

import cv2
import numpy as np
import json
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

try:
    from .camera_devices import default_camera_selector, resolve_camera_source
except ImportError:  # Direct execution: python vision/cube_detector.py
    from camera_devices import default_camera_selector, resolve_camera_source


# ============================================================
# 数据结构
# ============================================================

@dataclass
class BlockInfo:
    """单块方块的检测结果。"""
    color_name: str = ""
    draw_color: Tuple[int, int, int] = (0, 0, 0)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    confidence: float = 0.0
    quad: Optional[np.ndarray] = field(default=None, repr=False)


@dataclass
class VisionResult:
    """单帧视觉检测结果（线程安全快照）。"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    distance: float = 0.0       # sqrt(x²+y²+z²)
    confidence: float = 0.0
    is_valid: bool = False
    color_name: Optional[str] = None
    all_blocks: List[BlockInfo] = field(default_factory=list)
    timestamp: float = 0.0
    fps: float = 0.0


# ============================================================
# 全局配置
# ============================================================
CONFIG = {
    "cube_size_mm": 100.0,
    "slot_depth_mm": 50.0,
    "visible_height_mm": 50.0,

    "fx": 800.0, "fy": 800.0,
    "cx": 320.0, "cy": 240.0,

    "color_profiles": [
        {
            "name": "Orange",
            # Covers the dark orange front under manual exposure and the
            # brighter yellow-orange top, while excluding the green field.
            "hsv_low": np.array([2, 80, 45]),
            "hsv_high": np.array([25, 255, 255]),
            "draw_color": (0, 140, 255),
        },
        {
            "name": "Purple",
            # With the competition camera's automatic white balance, the
            # purple EVA spans violet to magenta and its lit top has lower
            # saturation than the front face.
            "hsv_low": np.array([110, 50, 30]),
            "hsv_high": np.array([145, 255, 255]),
            "draw_color": (200, 80, 180),
        },
    ],

    "morph_kernel_size": 5,
    "morph_iterations": 2,
    "min_contour_area": 100,
    # The visible orange front changes from about 2.7:1 to 4.4:1 as the
    # foreground rail occludes more of the cube. Keep a broad but bounded
    # range; the minimum width filter rejects small orange distractors.
    "min_front_aspect_ratio": 0.55,
    "max_front_aspect_ratio": 8.0,
    "min_face_width_ratio": 0.06,
    "min_rect_fill_ratio": 0.62,

    "ema_alpha": 0.35,
    "history_size": 10,
    "max_position_jump_mm": 80,

    "calib_file": "camera_calib.json",
}

CAMERA_SETTINGS_FILE = os.path.join(os.path.dirname(__file__),
                                    "camera_settings.json")


def load_camera_settings() -> dict:
    """Load platform-specific UVC controls saved by camera_tuner.py."""
    platform_key = "windows" if sys.platform == "win32" else "linux"
    try:
        with open(CAMERA_SETTINGS_FILE, "r", encoding="utf-8") as stream:
            return json.load(stream).get(platform_key, {})
    except (OSError, json.JSONDecodeError):
        return {}


# ============================================================
# 相机标定
# ============================================================

def load_or_init_calibration(state: dict, calib_path: str) -> bool:
    if os.path.exists(calib_path):
        try:
            with open(calib_path, "r") as f:
                data = json.load(f)
            state["fx"] = data["fx"]
            state["fy"] = data["fy"]
            state["cx"] = data["cx"]
            state["cy"] = data["cy"]
            if "colors" in data:
                for profile in CONFIG["color_profiles"]:
                    name = profile["name"]
                    if name in data["colors"]:
                        c = data["colors"][name]
                        profile["hsv_low"] = np.array(c["hsv_low"])
                        profile["hsv_high"] = np.array(c["hsv_high"])
            state["calibrated"] = True
            print(f"[视觉] 标定已加载: fx={state['fx']:.1f}, fy={state['fy']:.1f}")
            return True
        except (KeyError, json.JSONDecodeError) as e:
            print(f"[视觉] 标定文件损坏: {e}")
    return False


def save_calibration(fx, fy, cx, cy, calib_path: str, state: dict):
    data = {"fx": fx, "fy": fy, "cx": cx, "cy": cy}
    data["colors"] = {}
    for profile in CONFIG["color_profiles"]:
        data["colors"][profile["name"]] = {
            "hsv_low": [int(v) for v in profile["hsv_low"]],
            "hsv_high": [int(v) for v in profile["hsv_high"]],
        }
    with open(calib_path, "w") as f:
        json.dump(data, f, indent=2)
    state["fx"] = fx; state["fy"] = fy
    state["cx"] = cx; state["cy"] = cy
    state["calibrated"] = True
    print(f"[视觉] 标定已保存到 {calib_path}")


def calibrate_from_known_distance(known_distance_mm, pixel_width, pixel_height):
    fx = known_distance_mm * pixel_width / CONFIG["cube_size_mm"]
    fy = known_distance_mm * pixel_height / CONFIG["visible_height_mm"]
    return fx, fy


# ============================================================
# 角点/四边形检测
# ============================================================

def order_corners(pts):
    rect = np.zeros((4, 2), dtype=np.float32)
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect


def compute_quad_aspect_ratio(quad):
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
    w1 = np.linalg.norm(quad[1] - quad[0])
    w2 = np.linalg.norm(quad[2] - quad[3])
    h1 = np.linalg.norm(quad[3] - quad[0])
    h2 = np.linalg.norm(quad[2] - quad[1])
    w_avg = (w1 + w2) / 2.0
    h_avg = (h1 + h2) / 2.0
    # Do not blend sqrt(area) into width. The visible height changes with
    # occlusion, while the horizontal edge still represents the 100 mm cube.
    return w_avg, h_avg


def find_quadrilaterals(mask, min_area=None):
    if min_area is None:
        min_area = CONFIG["min_contour_area"]
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []
    results = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        hull = cv2.convexHull(cnt)
        peri = cv2.arcLength(hull, True)
        accepted = False
        last_approx = None
        for eps in [0.02, 0.03, 0.04, 0.05]:
            approx = cv2.approxPolyDP(hull, eps * peri, True)
            last_approx = approx
            if len(approx) == 4:
                quad = approx.reshape(4, 2)
                quad = order_corners(quad)
                ar = compute_quad_aspect_ratio(quad)
                if (CONFIG["min_front_aspect_ratio"] <= ar <=
                        CONFIG["max_front_aspect_ratio"]):
                    results.append((area, quad))
                    accepted = True
                break

        # Partial occlusion and soft foam edges can produce 5-8 vertices.
        # Recover a stable rotated rectangle only when the contour fills it
        # sufficiently; this avoids accepting arbitrary orange fragments.
        if not accepted and last_approx is not None and 4 <= len(last_approx) <= 8:
            rect = cv2.minAreaRect(hull)
            rect_area = rect[1][0] * rect[1][1]
            fill_ratio = area / max(rect_area, 1.0)
            quad = order_corners(cv2.boxPoints(rect))
            ar = compute_quad_aspect_ratio(quad)
            if ar < 1.0:
                ar = 1.0 / max(ar, 1e-6)
            if (fill_ratio >= CONFIG["min_rect_fill_ratio"] and
                    ar <= CONFIG["max_front_aspect_ratio"]):
                results.append((area * fill_ratio, quad))
    results.sort(key=lambda x: x[0], reverse=True)
    return [q for _, q in results]


def refine_corners_subpix(gray, quad):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
    try:
        return cv2.cornerSubPix(gray, quad.astype(np.float32),
                                (5, 5), (-1, -1), criteria)
    except cv2.error:
        return quad


# ============================================================
# 3D位置计算
# ============================================================

def compute_3d_position(quad, fx, fy, cx, cy):
    full_w = CONFIG["cube_size_mm"]
    vis_h = CONFIG["visible_height_mm"]
    y_offset = (full_w - vis_h) / 2.0

    center_u = np.mean(quad[:, 0])
    center_v = np.mean(quad[:, 1])
    w_px, h_px = measure_face_size(quad)

    # Only cube width is invariant in both requested views. The orange face
    # height varies with rail occlusion and must not participate in distance.
    z_mm = (fx * full_w) / max(w_px, 1e-6)

    visible_ratio = h_px / max(w_px, 1e-6)
    aspect_conf = max(0.0, 1.0 - abs(visible_ratio - 0.30) / 0.45)

    x_mm = (center_u - cx) * z_mm / fx
    y_visible = -(center_v - cy) * z_mm / fy
    y_mm = y_visible - y_offset

    confidence = min((0.55 + 0.45 * aspect_conf) * 100, 100)
    if z_mm < 20 or z_mm > 5000:
        confidence = 0
    return x_mm, y_mm, z_mm, confidence


def temporal_filter(x_mm, y_mm, z_mm, confidence, state):
    if state["ema_pos"] is not None and confidence > 30:
        px, py, pz = state["ema_pos"]
        jump = np.sqrt((x_mm - px)**2 + (y_mm - py)**2 + (z_mm - pz)**2)
        if jump > CONFIG["max_position_jump_mm"]:
            confidence *= 0.3

    alpha = CONFIG["ema_alpha"]
    if state["ema_pos"] is None:
        state["ema_pos"] = (x_mm, y_mm, z_mm)
    else:
        px, py, pz = state["ema_pos"]
        alpha_eff = alpha * (confidence / 100.0)
        alpha_eff = np.clip(alpha_eff, 0.05, 0.8)
        state["ema_pos"] = (
            px + alpha_eff * (x_mm - px),
            py + alpha_eff * (y_mm - py),
            pz + alpha_eff * (z_mm - pz),
        )
    fx, fy, fz = state["ema_pos"]
    is_valid = confidence > 25
    return fx, fy, fz, is_valid


# ============================================================
# 多颜色检测
# ============================================================

def detect_all_blocks(frame, state):
    fx, fy = state["fx"], state["fy"]
    cx, cy = state["cx"], state["cy"]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    all_blocks = []
    for profile in CONFIG["color_profiles"]:
        mask = cv2.inRange(hsv, profile["hsv_low"], profile["hsv_high"])
        ksize = CONFIG["morph_kernel_size"]
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel,
                                iterations=CONFIG["morph_iterations"])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel,
                                iterations=CONFIG["morph_iterations"])
        quads = find_quadrilaterals(mask)
        for quad in quads:
            w_px, _ = measure_face_size(quad)
            min_w = frame.shape[1] * CONFIG["min_face_width_ratio"]
            if w_px < min_w:
                continue
            quad = refine_corners_subpix(gray, quad)
            x_mm, y_mm, z_mm, conf = compute_3d_position(quad, fx, fy, cx, cy)
            all_blocks.append(BlockInfo(
                color_name=profile["name"],
                draw_color=profile["draw_color"],
                x=x_mm, y=y_mm, z=z_mm, confidence=conf,
                quad=quad.copy(),
            ))
    # Sort by true camera-relative distance so the first candidate is the
    # physically nearest cube, including cubes away from the optical axis.
    all_blocks.sort(key=lambda b: b.x * b.x + b.y * b.y + b.z * b.z)
    return all_blocks


# ============================================================
# 可视化（精简版）
# ============================================================

def draw_overlay_simple(frame, result: VisionResult, state):
    h, w = frame.shape[:2]

    for index, block in enumerate(result.all_blocks):
        if block.quad is None or len(block.quad) != 4:
            continue
        quad = np.rint(block.quad).astype(np.int32)
        center = tuple(np.rint(np.mean(block.quad, axis=0)).astype(int))
        is_locked = index == 0 and result.color_name == block.color_name
        box_color = (0, 255, 0) if is_locked else block.draw_color
        thickness = 3 if is_locked else 2
        cv2.polylines(frame, [quad], True, box_color, thickness,
                      lineType=cv2.LINE_AA)
        cv2.drawMarker(frame, center, box_color, cv2.MARKER_CROSS,
                       18, 2, cv2.LINE_AA)
        cv2.circle(frame, center, 4, box_color, -1, cv2.LINE_AA)

        label = (f"{block.color_name} {block.confidence:.0f}% "
                 f"Z={block.z:.0f}mm")
        label_y = max(18, int(np.min(quad[:, 1])) - 8)
        cv2.putText(frame, label, (int(np.min(quad[:, 0])), label_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, box_color, 1,
                    cv2.LINE_AA)

    # Camera optical center helps judge lateral alignment at a glance.
    image_center = (int(state.get("cx", w / 2)), int(state.get("cy", h / 2)))
    cv2.drawMarker(frame, image_center, (255, 255, 0), cv2.MARKER_CROSS,
                   14, 1, cv2.LINE_AA)

    if result.is_valid:
        info = (f"[{result.color_name}] "
                f"X:{result.x:+.0f} Y:{result.y:+.0f} Z:{result.z:+.0f}mm "
                f"D:{result.distance:.0f}mm C:{result.confidence:.0f}%")
        color = (0, 255, 0)
    else:
        info = f"SEARCHING ({len(result.all_blocks)} seen)" if result.all_blocks else "SEARCHING"
        color = (0, 0, 255)
    cv2.putText(frame, f"FPS:{result.fps:.1f} {info}", (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return frame


# ============================================================
# CubeDetector 类 — 后台线程运行
# ============================================================

class CubeDetector:
    """
    多色方块3D定位检测器（后台线程）。

    Usage:
        det = CubeDetector(camera_id=1, show_gui=False)
        det.start()
        # ... robot control loop ...
        r = det.result  # VisionResult or None
        det.stop()
    """

    def __init__(self, camera_id=None,
                 resolution: Tuple[int, int] = (640, 480),
                 calibration_file: str = None,
                 show_gui: bool = False,
                 exposure: Optional[float] = None,
                 gain: Optional[float] = None,
                 white_balance: Optional[float] = None):
        self._camera_id = (default_camera_selector()
                           if camera_id is None else camera_id)
        self._camera_source = None
        self._resolution = resolution
        self._show_gui = show_gui
        camera_settings = load_camera_settings()
        self._exposure = (camera_settings.get("exposure")
                          if exposure is None else exposure)
        self._gain = camera_settings.get("gain") if gain is None else gain
        self._white_balance = (camera_settings.get("white_balance")
                               if white_balance is None else white_balance)
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Calibration file path
        if calibration_file is None:
            calibration_file = os.path.join(os.path.dirname(__file__),
                                           "camera_calib.json")
        self._calib_path = calibration_file

        # Per-instance state
        self._state = {
            "calibrated": False,
            "fx": CONFIG["fx"], "fy": CONFIG["fy"],
            "cx": CONFIG["cx"], "cy": CONFIG["cy"],
            "history": deque(maxlen=CONFIG["history_size"]),
            "ema_pos": None,
            "locked_color": None,
            "frame_count": 0,
            "fps": 0.0,
            "last_time": time.time(),
        }

        # Thread-safe result
        self._result_lock = threading.Lock()
        self._result: Optional[VisionResult] = None

        # Camera handle (opened in start)
        self._cap: Optional[cv2.VideoCapture] = None

    # ================== Lifecycle ===========================================

    def start(self) -> bool:
        """打开摄像头并启动后台检测线程。"""
        if self._running:
            return True

        # CAP_ANY may select the GStreamer backend on Raspberry Pi.  For UVC
        # cameras this can produce a 0x0 stream or fail after opening; use the
        # V4L2 backend explicitly on Linux.
        backend = (cv2.CAP_DSHOW if sys.platform == "win32"
                   else cv2.CAP_V4L2)
        try:
            self._camera_source = resolve_camera_source(self._camera_id)
        except (FileNotFoundError, RuntimeError, ValueError) as exc:
            print(f"[视觉] 摄像头配置错误: {exc}")
            return False
        self._cap = cv2.VideoCapture(self._camera_source, backend)
        if not self._cap.isOpened():
            print(f"[视觉] 无法打开摄像头 {self._camera_id} "
                  f"({self._camera_source})")
            self._cap = None
            return False

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._resolution[0])
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._resolution[1])
        self._cap.set(cv2.CAP_PROP_FPS, 30)
        self._cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

        if self._exposure is not None:
            # OpenCV backend convention: DSHOW manual=0.25, V4L2 manual=1.
            manual_mode = 0.25 if sys.platform == "win32" else 1.0
            self._cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, manual_mode)
            self._cap.set(cv2.CAP_PROP_EXPOSURE, self._exposure)
        if self._gain is not None:
            self._cap.set(cv2.CAP_PROP_GAIN, self._gain)
        if self._white_balance is not None:
            self._cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            self._cap.set(cv2.CAP_PROP_WB_TEMPERATURE, self._white_balance)

        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"[视觉] 摄像头 {self._camera_id} "
              f"({self._camera_source}): {actual_w}x{actual_h}")

        load_or_init_calibration(self._state, self._calib_path)
        self._state["cx"] = actual_w / 2.0
        self._state["cy"] = actual_h / 2.0

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        print("[视觉] 检测线程已启动")
        return True

    def stop(self):
        """停止检测线程并释放摄像头。"""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._cap:
            self._cap.release()
            self._cap = None
        cv2.destroyAllWindows()
        print("[视觉] 已停止")

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def result(self) -> Optional[VisionResult]:
        """获取最新检测结果（线程安全）。"""
        with self._result_lock:
            return self._result

    @property
    def calibrated(self) -> bool:
        return self._state["calibrated"]

    def reset_filter(self):
        """重置 EMA 时序滤波器。"""
        self._state["ema_pos"] = None
        self._state["history"].clear()
        print("[视觉] EMA 滤波器已重置")

    # ================== Background Loop =====================================

    def _capture_loop(self):
        """后台线程：采集 → 检测 → 存储结果。"""
        state = self._state

        while self._running:
            if self._cap is None:
                break

            ret, frame = self._cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            h, w = frame.shape[:2]
            state["cx"] = w / 2.0
            state["cy"] = h / 2.0

            detected_blocks = detect_all_blocks(frame, state)
            # Keep the nearest cube as the general-purpose lock, but expose all
            # candidates so competition strategy can select a required color.
            nearest_block = detected_blocks[0] if detected_blocks else None
            result_blocks = detected_blocks

            x_mm, y_mm, z_mm = 0.0, 0.0, 0.0
            confidence = 0.0
            is_valid = False
            color_name = None

            if nearest_block is not None:
                locked = nearest_block
                if locked.confidence > 0:
                    x_mm, y_mm, z_mm = locked.x, locked.y, locked.z
                    confidence = locked.confidence
                    color_name = locked.color_name
                    fx_mm, fy_mm, fz_mm, is_valid = temporal_filter(
                        x_mm, y_mm, z_mm, confidence, state)
                    state["locked_color"] = color_name
                else:
                    fx_mm, fy_mm, fz_mm = x_mm, y_mm, z_mm
            else:
                if state["ema_pos"] is not None:
                    fx_mm, fy_mm, fz_mm = state["ema_pos"]
                else:
                    fx_mm, fy_mm, fz_mm = 0.0, 0.0, 0.0

            result_x = fx_mm if is_valid else x_mm
            result_y = fy_mm if is_valid else y_mm
            result_z = fz_mm if is_valid else z_mm
            result = VisionResult(
                x=result_x,
                y=result_y,
                z=result_z,
                distance=np.sqrt(result_x**2 + result_y**2 + result_z**2),
                confidence=confidence,
                is_valid=is_valid,
                color_name=color_name,
                all_blocks=result_blocks,
                timestamp=time.time(),
                fps=state["fps"],
            )

            with self._result_lock:
                self._result = result

            state["frame_count"] += 1
            now = time.time()
            if now - state["last_time"] >= 1.0:
                state["fps"] = state["frame_count"] / (now - state["last_time"])
                state["frame_count"] = 0
                state["last_time"] = now

            if self._show_gui:
                display = draw_overlay_simple(frame.copy(), result, state)
                cv2.imshow("Cube Detector", display)
                if cv2.waitKey(1) & 0xFF == 27:
                    self._running = False
                    break

        if self._show_gui:
            cv2.destroyAllWindows()


# ============================================================
# 独立运行入口
# ============================================================

if __name__ == "__main__":
    import argparse
    import sys

    parser = argparse.ArgumentParser(description="多色方块3D定位系统")
    parser.add_argument("--camera", default=default_camera_selector(),
                        help="camera role (cube/tag), stable path, or diagnostic index")
    parser.add_argument("--no-gui", action="store_true")
    parser.add_argument("--resolution", type=str, default="640x480")
    parser.add_argument("--exposure", type=float, default=None,
                        help="manual camera exposure; backend-specific value")
    parser.add_argument("--gain", type=float, default=None,
                        help="manual camera gain; backend-specific value")
    parser.add_argument("--white-balance", type=float, default=None,
                        help="manual white-balance temperature")
    args = parser.parse_args()

    res_w, res_h = map(int, args.resolution.split("x"))

    det = CubeDetector(
        camera_id=args.camera,
        resolution=(res_w, res_h),
        show_gui=not args.no_gui,
        exposure=args.exposure,
        gain=args.gain,
        white_balance=args.white_balance,
    )

    if not det.start():
        sys.exit(1)

    print(f"\n多色方块3D定位系统")
    print(f"焦距: fx={det._state['fx']:.0f}, fy={det._state['fy']:.0f}")
    print(f"检测颜色: {', '.join(p['name'] for p in CONFIG['color_profiles'])}")
    print("按 Ctrl+C 退出\n")

    try:
        while det.is_running:
            r = det.result
            if r:
                if r.is_valid:
                    print(f"\r[{r.color_name}] X:{r.x:+7.1f} Y:{r.y:+7.1f} "
                          f"Z:{r.z:+7.1f}mm Dist:{r.distance:6.1f}mm "
                          f"Conf:{r.confidence:5.1f}% FPS:{r.fps:4.1f}",
                          end="", flush=True)
                else:
                    hint = f"({len(r.all_blocks)} seen)" if r.all_blocks else ""
                    print(f"\r[SEARCHING] {hint} FPS:{r.fps:4.1f}  ",
                          end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[退出]")
    finally:
        det.stop()
