"""橙色拾取管线，移植自桌面视觉 demo detect.py。

比赛适配：使用赛前固定平面映射，运行时只做轮廓几何定位。

每帧流程：
  颜色分割(HSV 橙色) -> 连通域(簇) -> 簇长轴 PCA(槽向) -> 旋正到水平
  -> Otsu 二值分割取"顶面条带"(顶面最亮，前面/侧面较暗)
  -> 亮区四边(远棱/折痕/左/右)拟合，交点得四角点
  -> 固定平面单应映射到位姿 (R,t)
  -> 只输出每簇左侧第一个方块（由左缘定位），供机器人对准

简化策略：机器人从左向右搜索，最先看到的是每簇的左缘。纯视觉拆分多块
不可靠，因此不做拆分——簇的左缘就是首块左边界，首块跨 [start_x, start_x+CUBE]。
左缘被画面裁剪时不输出坐标，机器人继续向右搜索直到左缘完整入画。
"""
import numpy as np
import cv2
try:
    from .orange_fixed_geometry import image_to_world, world_to_camera, world_to_image
except ImportError:
    from orange_fixed_geometry import image_to_world, world_to_camera, world_to_image


class Cube:
    __slots__ = ("index", "center_px", "world_xy", "cam_xyz",
                 "distance", "along_cm", "top_quad", "clipped",
                 "visible_ratio", "position_valid", "left_cm")

    def __init__(self, index, center_px, world_xy, cam_xyz, top_quad,
                 clipped=False, visible_ratio=1.0, position_valid=True,
                 left_cm=None):
        self.index = index
        self.center_px = center_px
        self.world_xy = world_xy
        self.cam_xyz = cam_xyz
        self.position_valid = bool(position_valid)
        self.distance = float(np.linalg.norm(cam_xyz)) if self.position_valid else float("nan")
        self.along_cm = float(world_xy[0])
        self.top_quad = top_quad
        self.clipped = bool(clipped)
        self.visible_ratio = float(np.clip(visible_ratio, 0.0, 1.0))
        self.left_cm = float(left_cm) if left_cm is not None else None


def _fit_line_general(pts):
    """最小二乘拟合直线 A*x+B*y+C=0（A²+B²=1），带两轮离群剔除。返回 (A,B,C)。"""
    pts = np.asarray(pts, np.float64)
    if pts.ndim != 2 or len(pts) < 5:
        return None
    pts = pts[np.isfinite(pts).all(axis=1)]
    if len(pts) < 5:
        return None
    for _ in range(3):
        c = pts.mean(axis=0)
        M = pts - c
        w, v = np.linalg.eigh(M.T @ M)
        n = v[:, 0]
        A, B = float(n[0]), float(n[1])
        C = -(A * c[0] + B * c[1])
        d = A * pts[:, 0] + B * pts[:, 1] + C
        sigma = float(d.std())
        if sigma < 1e-9:
            break
        keep = np.abs(d) <= 2.5 * sigma
        if keep.mean() > 0.5:
            pts = pts[keep]
    return np.array([A, B, C], np.float64)


def _line_intersect(l1, l2):
    if l1 is None or l2 is None:
        return None
    p = np.cross(l1, l2)
    if abs(p[2]) < 1e-9:
        return None
    return p[:2] / p[2]


class Detector:
    def __init__(self, cfg):
        self.cfg = cfg
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg.MORPH_KERNEL,) * 2)

    def _merge_nearby_contours(self, contours):
        """Join top/front color patches belonging to one physical cube."""
        if len(contours) < 2:
            return contours
        gap_limit = 12
        groups = []
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            placed = False
            for group in groups:
                gx = min(cv2.boundingRect(c)[0] for c in group)
                gy = min(cv2.boundingRect(c)[1] for c in group)
                gr = max(cv2.boundingRect(c)[0] + cv2.boundingRect(c)[2] for c in group)
                gb = max(cv2.boundingRect(c)[1] + cv2.boundingRect(c)[3] for c in group)
                overlap = min(x + w, gr) - max(x, gx)
                vertical_gap = max(0, max(y - gb, gy - (y + h)))
                if overlap >= 0.45 * min(w, gr - gx) and vertical_gap <= gap_limit:
                    group.append(cnt)
                    placed = True
                    break
            if not placed:
                groups.append([cnt])
        merged = []
        for group in groups:
            if len(group) == 1:
                merged.append(group[0])
            else:
                merged.append(cv2.convexHull(np.vstack(group)))
        return merged

    def _why_filtered(self, contours_by_area, contours_raw,
                      edge, W_img, H_img, mask_px):
        """诊断：检测到橙色但无簇通过筛选时的原因摘要（仅失败帧调用）。"""
        if not contours_by_area:
            largest = max((cv2.contourArea(c) for c in contours_raw), default=0)
            return ("掩码碎片化(总%dpx 最大连通%dpx<面积门槛)"
                    % (mask_px, int(largest)))
        best_area = -1
        best = ""
        for cnt in contours_by_area:
            area = cv2.contourArea(cnt)
            x, y, w, h = cv2.boundingRect(cnt)
            reasons = []
            if not (y > edge and y + h < H_img - edge):
                reasons.append("顶/底贴边y=%d" % y)
            aspect = w / float(max(1, h))
            if aspect < float(getattr(self.cfg, "MIN_CLUSTER_ASPECT", 0.9)):
                reasons.append("竖长w/h=%.2f" % aspect)
            if area < best_area:
                continue
            best_area = area
            best = "最大%dpx(%dx%d): %s" % (
                int(area), w, h, ";".join(reasons) if reasons else "相对面积")
        return best

    # ---------------- 对外 ----------------
    def detect(self, frame):
        """返回 (cubes, info)。info 为诊断信息 dict。"""
        orig_w = frame.shape[1]
        # 降采样加速：视野不变（各分辨率 FOV 一致），处理像素少了约 4 倍
        max_w = getattr(self.cfg, "PROCESS_WIDTH", 0)
        if max_w and frame.shape[1] > max_w:
            scale = max_w / float(frame.shape[1])
            frame = cv2.resize(frame, (max_w, max(1, int(round(frame.shape[0] * scale)))),
                               interpolation=cv2.INTER_AREA)
        H_img, W_img = frame.shape[:2]
        # Robot/camera moves: never reuse a previous frame plane pose.
        mask = self._orange_mask(frame)
        cubes = []
        info = {"status": "empty", "n_clusters": 0, "message": "",
                "mask": mask, "scale": frame.shape[1] / float(orig_w)}

        if np.count_nonzero(mask) < self.cfg.MIN_MASK_FRAC * H_img * W_img:
            info["message"] = "视野中无橙色方块"
            return cubes, info

        contours_raw, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours = [cnt for cnt in contours_raw
                    if cv2.contourArea(cnt) >= self.cfg.MIN_CLUSTER_AREA]
        contours_by_area = contours   # 诊断用：过了面积筛选的候选
        # Slot cubes may enter from left/right, but a target contour should not
        # originate at the top/bottom frame edge (skin and nearby orange props
        # commonly do). Then suppress small detached patches relative to the
        # dominant cube body in this frame.
        edge = max(1, int(getattr(self.cfg, "FRAME_EDGE_MARGIN_PX", 3)))
        contours = [cnt for cnt in contours
                    if (lambda r: r[1] > max(edge, round(H_img * getattr(self.cfg, "ROI_TOP_RATIO", 0.0)) + edge) and r[1] + r[3] < H_img - edge)
                    (cv2.boundingRect(cnt))]
        min_aspect = float(getattr(self.cfg, "MIN_CLUSTER_ASPECT", 0.0))
        if min_aspect > 0:
            contours = [cnt for cnt in contours
                        if (lambda r: r[2] / float(max(1, r[3])) >= min_aspect)
                        (cv2.boundingRect(cnt))]
        if contours:
            largest_area = max(cv2.contourArea(cnt) for cnt in contours)
            min_relative = float(getattr(self.cfg, "MIN_RELATIVE_CLUSTER_AREA", 0.35))
            kept = []
            for cnt in contours:
                x, _, w, _ = cv2.boundingRect(cnt)
                touches_side = x <= edge or x + w >= W_img - edge
                if touches_side or cv2.contourArea(cnt) >= min_relative * largest_area:
                    kept.append(cnt)
            contours = kept
        contours = self._merge_nearby_contours(contours)
        # 诊断：检测到橙色但最终无簇时，记录被哪级滤掉（排查闪烁）
        if not contours:
            info["filter_reason"] = self._why_filtered(
                contours_by_area, contours_raw,
                edge, W_img, H_img, int(mask.sum() / 255))
            info["message"] = (info.get("message", "") + " 筛选:%s;" % info["filter_reason"])
        n_clusters = 0
        # 大簇优先处理，保持左首方块的稳定边缘拟合。
        for cnt in sorted(contours, key=lambda c: -cv2.contourArea(c)):
            n_clusters += 1
            try:
                found = self._process_cluster(frame, mask, cnt, info)
            except Exception as e:  # 单簇失败不影响其他簇
                info["message"] = (info.get("message", "") + f" 簇异常:{e}")
                found = None
            if found:
                cubes.extend(found)

        # 过滤异常结果（NaN / 离原点太近 / 太远）
        cubes = [c for c in cubes
                 if (not c.position_valid)
                 or (np.isfinite(c.cam_xyz).all()
                     and 5.0 < c.distance < 500.0)]

        info["n_clusters"] = n_clusters
        # 按沿槽位置升序排序：index 0 = 画面最左的方块 = 机器人从左向右搜索
        # 时最先遇到的左首方块
        cubes.sort(key=lambda c: c.along_cm)
        for i, c in enumerate(cubes):      # 全局编号（跨簇）
            c.index = i
        info["status"] = "ok" if cubes else "empty"
        if not cubes and not info["message"]:
            info["message"] = "检测到橙色但左缘未定位"
        return cubes, info

    # ---------------- 分割 ----------------
    def _orange_mask(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.cfg.HSV_ORANGE_LOW, self.cfg.HSV_ORANGE_HIGH)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        return mask

    # ---------------- 簇处理 ----------------
    def _process_cluster(self, frame, mask, cnt, info):
        cfg = self.cfg
        # Each disconnected cluster can have a different depth.
        H_img, W_img = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        cluster_mask = np.zeros((H_img, W_img), np.uint8)
        cv2.drawContours(cluster_mask, [cnt], -1, 255, -1)
        if cluster_mask.sum() < 100 * 255:
            info["message"] = (info.get("message", "") + " 簇过小;")
            return None

        # 1) Otsu 顶面提取（原始坐标，与旋转无关）：45° 光照下顶面最亮
        vals = gray[cluster_mask > 0]
        thr, _ = cv2.threshold(vals.astype(np.uint8), 0, 255,
                               cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        bright0 = ((gray > thr) & (cluster_mask > 0)).astype(np.uint8) * 255
        k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        bright0 = cv2.morphologyEx(bright0, cv2.MORPH_OPEN, k3)
        bright0 = cv2.morphologyEx(bright0, cv2.MORPH_CLOSE, k3)
        min_top_frac = float(getattr(cfg, "MIN_TOP_FRAC", 0.35))
        if bright0.sum() < min_top_frac * float(cluster_mask.sum()):
            bright0 = cluster_mask.copy()   # 弱光照/顶面与前面同亮度时兜底
            info["message"] = (info.get("message", "") + " 顶面分割弱，整簇兜底;")

        # 2) 槽向 PCA：用顶面亮区（单块时整簇会因前面/侧面把长轴带偏）
        ys, xs = np.nonzero(bright0)
        if len(ys) < 100:
            info["message"] = (info.get("message", "") + " 顶面亮区过小;")
            return None
        pts = np.column_stack([xs.astype(np.float64), ys.astype(np.float64)])
        center = pts.mean(axis=0)
        w, v = np.linalg.eigh(np.cov(pts.T))
        axis = v[:, int(np.argmax(w))]
        aspect = float(np.sqrt(w.max() / (w.min() + 1e-9)))
        if axis[0] < 0 or (axis[0] == 0 and axis[1] < 0):
            axis = -axis          # 固定轴指向 +X 象限，消除 180° 翻转歧义
        theta = float(np.degrees(np.arctan2(axis[1], axis[0])))

        # 3) 旋正：槽向 -> 水平 X
        rot = cv2.getRotationMatrix2D((float(center[0]), float(center[1])), theta, 1.0)
        mw = cv2.warpAffine(cluster_mask, rot, (W_img, H_img), flags=cv2.INTER_NEAREST)
        bright = cv2.warpAffine(bright0, rot, (W_img, H_img), flags=cv2.INTER_NEAREST)

        bcols = np.nonzero(bright.any(axis=0))[0]
        brows = np.nonzero(bright.any(axis=1))[0]
        if len(bcols) < 15 or len(brows) < 8:
            info["message"] = (info.get("message", "") + " 条带过窄;")
            return None
        edge_margin = max(1, int(getattr(cfg, "FRAME_EDGE_MARGIN_PX", 3)))
        x_original, _, _, _ = cv2.boundingRect(cnt)
        clipped_left = x_original <= edge_margin or bcols[0] <= edge_margin
        clipped_right = (x_original + cv2.boundingRect(cnt)[2] >=
                         W_img - edge_margin or bcols[-1] >= W_img - edge_margin)

        # 4) 亮区(顶面)上下边界
        y_b_top = np.full(W_img, -1, np.int32)
        y_b_bot = np.full(W_img, -1, np.int32)
        for x in bcols:
            nz = np.nonzero(bright[:, x])[0]
            if len(nz):
                y_b_top[x] = int(nz[0])
                y_b_bot[x] = int(nz[-1])

        # 5) 方向：远棱的外邻像素不在簇内（背景），折痕的外邻像素在簇内（前面）。
        #    该判据与图像是否被 180° 翻转无关，比"远棱更窄"更可靠。
        n_top_out = 0
        n_bot_out = 0
        for x in bcols:
            t = y_b_top[x]
            b = y_b_bot[x]
            if t > 0 and mw[t - 1, x] == 0:
                n_top_out += 1
            if b < H_img - 1 and mw[b + 1, x] == 0:
                n_bot_out += 1
        if n_top_out >= n_bot_out:
            far_rows = y_b_top      # 远棱在亮区上边界
            fold_rows = y_b_bot
        else:
            far_rows = y_b_bot      # 远棱在亮区下边界（旋正时被翻转 180°）
            fold_rows = y_b_top

        # 6) 亮区每行的左右边界
        x_left_row, x_right_row = {}, {}
        for y in brows:
            nz = np.nonzero(bright[y, :])[0]
            if len(nz):
                x_left_row[y] = int(nz[0])
                x_right_row[y] = int(nz[-1])
        x_lo = int(np.median([x_left_row[y] for y in brows]))
        x_hi = int(np.median([x_right_row[y] for y in brows]))

        # 7) 四条边界线
        cw = max(2, int(0.2 * (x_hi - x_lo)))
        xs_core = [x for x in bcols if x_lo + cw <= x <= x_hi - cw]
        top_pts = [[float(x), float(far_rows[x])] for x in xs_core]
        fold_pts = [[float(x), float(fold_rows[x])] for x in xs_core]
        y_far_t = int(np.median([far_rows[x] for x in bcols]))
        y_fold_t = int(np.median([fold_rows[x] for x in bcols]))
        rh = abs(y_fold_t - y_far_t)
        y_lo_b = min(y_far_t, y_fold_t) + int(0.2 * rh)
        y_hi_b = max(y_far_t, y_fold_t) - max(2, int(0.15 * rh))
        if y_hi_b <= y_lo_b:          # 顶面条带过矮时放宽到整条
            y_lo_b, y_hi_b = min(y_far_t, y_fold_t), max(y_far_t, y_fold_t)
        left_pts = [[float(x_left_row[y]), float(y)] for y in brows if y_lo_b <= y <= y_hi_b]
        right_pts = [[float(x_right_row[y]), float(y)] for y in brows if y_lo_b <= y <= y_hi_b]

        # 8) 角点 = 边界线交点
        top_line = _fit_line_general(top_pts)
        fold_line = _fit_line_general(fold_pts)
        left_line = _fit_line_general(left_pts)
        right_line = _fit_line_general(right_pts)

        FL = _line_intersect(top_line, left_line)
        FR = _line_intersect(top_line, right_line)
        NL = _line_intersect(fold_line, left_line)
        NR = _line_intersect(fold_line, right_line)
        if any(p is None for p in (FL, FR, NL, NR)):
            info["message"] = (info.get("message", "") + " 角点拟合失败;")
            return None

        inv = cv2.invertAffineTransform(rot)

        def back(p):
            return inv @ np.array([p[0], p[1], 1.0])

        corners = np.array([back(FL), back(FR), back(NR), back(NL)], np.float32)
        # 条带可能横贯画面、角点贴边（合法），容差放宽到 8% 帧宽/高；靠 dep/L 校验兜底
        mx = max(100, int(0.08 * W_img))
        my = max(100, int(0.08 * H_img))
        if (corners[:, 0] < -mx).any() or (corners[:, 1] < -my).any() or \
           (corners[:, 0] > W_img + mx).any() or \
           (corners[:, 1] > H_img + my).any():
            info["message"] = (info.get("message", "") + " 角点越界;")
            return None

        # 9) 左缘必须在画面内：机器人从左向右搜索，找到左缘后才对准左首方块。
        #    右端被裁剪不影响左首方块；左缘必须完整才能确定首块。
        if clipped_left:
            info["message"] = (info.get("message", "") + " 左缘未入画，继续搜索;")
            return None

        # 10) 左首方块：只输出每簇左侧第一个方块（紧贴左缘），供机器人对准。
        #     不做多块拆分——簇左缘即首块左边界，首块跨 [start_x, start_x+CUBE]。
        CUBE = cfg.CUBE_SIZE_CM
        geometry_profile = getattr(cfg, "GEOMETRY_PROFILE", "default")
        start_world = image_to_world(corners[3], W_img, H_img, geometry_profile)
        start_x = float(start_world[0])
        xw = start_x + CUBE / 2.0
        yw = CUBE / 2.0
        cam = world_to_camera((xw, yw), geometry_profile)
        px = world_to_image((xw, yw), W_img, H_img, geometry_profile)
        quad_world = np.array([[start_x, 0.0], [start_x + CUBE, 0.0],
                               [start_x + CUBE, CUBE], [start_x, CUBE]])
        quad = np.array([world_to_image(p, W_img, H_img, geometry_profile)
                         for p in quad_world], np.float32)
        return [Cube(0, px, (xw, yw), cam, quad, left_cm=start_x)]

