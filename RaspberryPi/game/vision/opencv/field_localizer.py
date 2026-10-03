"""AprilTag-only full-field localization using the dedicated tag camera."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import argparse
import json
import math
import os
import sys
import threading
import time
import traceback
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

try:
    from .camera_devices import resolve_camera_source
except ImportError:  # Direct execution: python vision/field_localizer.py
    from camera_devices import resolve_camera_source


MODULE_DIR = os.path.dirname(__file__)
DEFAULT_MAP_FILE = os.path.join(MODULE_DIR, "field_map.json")
DEFAULT_CALIB_FILE = os.path.join(MODULE_DIR, "tag_camera_calib.json")


@dataclass(frozen=True)
class TagSolution:
    tag_id: int
    x_m: float
    y_m: float
    yaw_deg: float
    camera_height_m: float
    reprojection_error_px: float
    area_px: float
    score: float
    distance_m: float = 0.0
    lateral_m: float = 0.0
    relative_yaw_deg: float = 0.0


@dataclass(frozen=True)
class FieldPose:
    valid: bool = False
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_deg: float = 0.0
    camera_height_m: float = 0.0
    reprojection_error_px: float = 0.0
    tag_ids: Tuple[int, ...] = field(default_factory=tuple)
    timestamp: float = 0.0
    fps: float = 0.0
    calibrated: bool = False
    tag_solutions: Tuple[TagSolution, ...] = field(default_factory=tuple)
    captured_monotonic: float = 0.0
    raw_tag_ids: Tuple[int, ...] = field(default_factory=tuple)
    rejection_reasons: Tuple[str, ...] = field(default_factory=tuple)
    capture_error: str = ""


def _wrap_angle(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def load_field_config(path: str = DEFAULT_MAP_FILE) -> dict:
    with open(path, "r", encoding="utf-8") as stream:
        config = json.load(stream)
    required = ("field_width_m", "field_height_m", "tag_size_m",
                "tag_center_height_m", "camera", "tags", "solver")
    missing = [name for name in required if name not in config]
    if missing:
        raise ValueError(f"field map missing keys: {', '.join(missing)}")
    return config


def load_camera_model(path: str, width: int, height: int):
    with open(path, "r", encoding="utf-8") as stream:
        data = json.load(stream)
    calibrated = bool(data.get("calibrated", False))
    configured_size = tuple(data.get("resolution", (width, height)))
    if calibrated:
        if configured_size != (width, height):
            raise ValueError(
                f"tag calibration resolution {configured_size} does not match "
                f"camera resolution {(width, height)}")
        camera_matrix = np.asarray(data["camera_matrix"], dtype=np.float64)
        dist_coeffs = np.asarray(data["dist_coeffs"], dtype=np.float64).reshape(-1, 1)
    else:
        fov_deg = float(data.get("horizontal_fov_deg", 60.0))
        focal = width / (2.0 * math.tan(math.radians(fov_deg / 2.0)))
        camera_matrix = np.array([
            [focal, 0.0, width / 2.0],
            [0.0, focal, height / 2.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        dist_coeffs = np.zeros((5, 1), dtype=np.float64)
    return camera_matrix, dist_coeffs, calibrated


def _object_points(tag_size_m: float) -> np.ndarray:
    half = tag_size_m / 2.0
    return np.array([
        [-half, half, 0.0],
        [half, half, 0.0],
        [half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float32)


def _reprojection_error(obj_pts, img_pts, rvec, tvec,
                        camera_matrix, dist_coeffs) -> float:
    projected, _ = cv2.projectPoints(
        obj_pts, rvec, tvec, camera_matrix, dist_coeffs)
    delta = projected.reshape(-1, 2) - img_pts.reshape(-1, 2)
    return float(np.mean(np.linalg.norm(delta, axis=1)))


def _candidate_to_world(rvec, tvec, tag: dict, config: dict):
    rotation, _ = cv2.Rodrigues(rvec)
    camera_local = (-rotation.T @ tvec).reshape(3)
    lateral_m = float(camera_local[0])
    tag_center_height_m = float(
        tag.get("center_height_m", config["tag_center_height_m"]))
    height_m = float(tag_center_height_m + camera_local[1])
    distance_m = float(camera_local[2])
    normal_deg = float(tag["normal_deg"])
    normal_rad = math.radians(normal_deg)
    camera_x = (float(tag["x_m"]) + distance_m * math.cos(normal_rad)
                - lateral_m * math.sin(normal_rad))
    camera_y = (float(tag["y_m"]) + distance_m * math.sin(normal_rad)
                + lateral_m * math.cos(normal_rad))

    relative_yaw = math.degrees(math.atan2(rotation[2, 0], rotation[2, 2]))
    camera_yaw = _wrap_angle(normal_deg + relative_yaw)
    camera_cfg = config["camera"]
    robot_yaw = _wrap_angle(camera_yaw - float(camera_cfg["yaw_offset_deg"]))
    heading = math.radians(robot_yaw)
    forward = float(camera_cfg["forward_offset_m"])
    left = float(camera_cfg["left_offset_m"])
    robot_x = camera_x - (forward * math.cos(heading) - left * math.sin(heading))
    robot_y = camera_y - (forward * math.sin(heading) + left * math.cos(heading))
    return (robot_x, robot_y, robot_yaw, height_m, distance_m,
            lateral_m, relative_yaw)


def solve_tag_pose(corners: np.ndarray, tag_id: int, config: dict,
                   camera_matrix: np.ndarray,
                   dist_coeffs: np.ndarray,
                   rejection_reasons: Optional[List[str]] = None) -> Optional[TagSolution]:
    def reject(reason):
        if rejection_reasons is not None:
            rejection_reasons.append(f"tag{tag_id}:{reason}")

    tag = config["tags"].get(str(tag_id))
    if tag is None:
        reject("unknown_tag")
        return None
    solver = config["solver"]
    points = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    points = points[np.asarray(solver["corner_order"], dtype=np.int32)]
    sides = [np.linalg.norm(points[(i + 1) % 4] - points[i]) for i in range(4)]
    if float(np.mean(sides)) < float(solver["min_tag_side_px"]):
        reject("too_small")
        return None

    obj_pts = _object_points(float(config["tag_size_m"]))
    img_pts = points.reshape(4, 1, 2)
    try:
        count, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            obj_pts, img_pts, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except cv2.error:
        reject("pnp_error")
        return None
    if not count:
        reject("pnp_no_solution")
        return None

    half_w = float(config["field_width_m"]) / 2.0
    half_h = float(config["field_height_m"]) / 2.0
    margin = float(solver["field_margin_m"])
    expected_height = float(config["camera"]["height_m"])
    max_height_error = float(tag.get(
        "max_camera_height_error_m",
        solver["max_camera_height_error_m"]))
    height_weight = float(solver["height_error_weight_px_per_m"])
    # The tag camera currently uses an FOV-estimated model.  Allow a map
    # entry to override the global quality gates for tags whose mounting or
    # near-field perspective is known to be less stable.
    max_reproj = float(tag.get(
        "max_reprojection_error_px",
        solver["max_reprojection_error_px"]))
    area = abs(float(cv2.contourArea(points)))
    best = None

    for rvec, tvec in zip(rvecs, tvecs):
        if float(tvec[2, 0]) <= 0.01:
            reject("behind_camera")
            continue
        (x_m, y_m, yaw_deg, height_m, distance_m,
         lateral_m, relative_yaw) = _candidate_to_world(
            rvec, tvec, tag, config)
        if distance_m <= 0.01:
            reject("nonpositive_distance")
            continue
        if not (-half_w - margin <= x_m <= half_w + margin
                and -half_h - margin <= y_m <= half_h + margin):
            reject(f"outside_field(x={x_m:.3f},y={y_m:.3f})")
            continue
        height_error = abs(height_m - expected_height)
        if height_error > max_height_error:
            reject(f"height_error({height_error:.3f}>{max_height_error:.3f}m)")
            continue
        reproj = _reprojection_error(
            obj_pts, img_pts, rvec, tvec, camera_matrix, dist_coeffs)
        if reproj > max_reproj:
            reject(f"reprojection({reproj:.3f}>{max_reproj:.3f}px)")
            continue
        score = reproj + height_weight * height_error
        solution = TagSolution(
            tag_id=tag_id, x_m=x_m, y_m=y_m, yaw_deg=yaw_deg,
            camera_height_m=height_m, reprojection_error_px=reproj,
            area_px=area, score=score, distance_m=distance_m,
            lateral_m=lateral_m, relative_yaw_deg=relative_yaw)
        if best is None or solution.score < best.score:
            best = solution
    return best


def fuse_tag_solutions(solutions: Sequence[TagSolution], config: dict):
    if not solutions:
        return None
    max_distance = float(config["solver"]["fusion_outlier_distance_m"])
    clusters = [
        [item for item in solutions
         if math.hypot(item.x_m - seed.x_m,
                       item.y_m - seed.y_m) <= max_distance]
        for seed in solutions
    ]
    used = max(clusters, key=lambda cluster: (
        len(cluster), -sum(item.score for item in cluster)))
    weights = [item.area_px / (item.reprojection_error_px + 0.5)
               for item in used]
    weight_sum = sum(weights)
    x_m = sum(item.x_m * weight for item, weight in zip(used, weights)) / weight_sum
    y_m = sum(item.y_m * weight for item, weight in zip(used, weights)) / weight_sum
    sin_yaw = sum(math.sin(math.radians(item.yaw_deg)) * weight
                  for item, weight in zip(used, weights))
    cos_yaw = sum(math.cos(math.radians(item.yaw_deg)) * weight
                  for item, weight in zip(used, weights))
    yaw_deg = math.degrees(math.atan2(sin_yaw, cos_yaw))
    camera_height = sum(item.camera_height_m * weight
                        for item, weight in zip(used, weights)) / weight_sum
    mean_error = sum(item.reprojection_error_px for item in used) / len(used)
    return x_m, y_m, yaw_deg, camera_height, mean_error, used


class FieldLocalizer:
    """Background AprilTag detector exposing thread-safe field poses."""

    FRAME_WAIT_NS = 200_000_000
    REOPEN_AFTER_S = 0.6
    REOPEN_RETRY_S = 1.0

    def __init__(self, camera="tag", map_file: str = DEFAULT_MAP_FILE,
                 calibration_file: str = DEFAULT_CALIB_FILE,
                 show_gui: bool = False):
        self._config = load_field_config(map_file)
        self._camera_selector = camera or self._config["camera"]["role"]
        self._calibration_file = calibration_file
        self._show_gui = show_gui
        self._running = False
        self._thread = None
        self._cap = None
        self._result_lock = threading.Lock()
        self._filter_lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._result = None
        self._smoothed = None
        self._fps = 0.0
        self._frame_count = 0
        self._fps_started = time.monotonic()
        self._last_frame_monotonic = self._fps_started
        self._capture_error = ""
        self._last_error_log = -math.inf
        self._last_error_key = ""
        self._reopen_count = 0

    def start(self) -> bool:
        with self._lifecycle_lock:
            return self._start()

    def _start(self) -> bool:
        if self._thread is not None and self._thread.is_alive():
            # A timed-out stop must never create a second camera owner.
            return self._running
        if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "ArucoDetector"):
            print("[定位] OpenCV 缺少 aruco/AprilTag 支持")
            return False
        dictionary_id = getattr(cv2.aruco, self._config["tag_family"])
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(dictionary, params)
        self._stop_event.clear()
        self._capture_error = ""
        self._reopen_count = 0
        self._last_error_log = -math.inf
        self._last_error_key = ""
        self._fps = 0.0
        self._frame_count = 0
        self._fps_started = time.monotonic()
        with self._result_lock:
            self._result = None
        self.reset_filter()
        if not self._open_camera():
            return False
        self._running = True
        self._thread = threading.Thread(
            target=self._capture_loop, name="tag-camera", daemon=True)
        self._thread.start()
        return True

    def _open_camera(self) -> bool:
        """Open the stable role again after disconnect/re-enumeration."""
        try:
            source = resolve_camera_source(self._camera_selector)
            camera_cfg = self._config["camera"]
            width, height = map(int, camera_cfg["resolution"])
            self._cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
            if not self._cap.isOpened():
                raise RuntimeError(f"cannot open {source}")
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self._cap.set(cv2.CAP_PROP_FPS, int(camera_cfg["fps"]))
            self._cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            if camera_cfg.get("exposure") is not None:
                self._cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)
                self._cap.set(
                    cv2.CAP_PROP_EXPOSURE, float(camera_cfg["exposure"]))
            if camera_cfg.get("gain") is not None:
                self._cap.set(cv2.CAP_PROP_GAIN, float(camera_cfg["gain"]))
            actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self._camera_matrix, self._dist_coeffs, self._calibrated = \
                load_camera_model(self._calibration_file, actual_w, actual_h)
        except Exception as exc:
            self._record_error("open_failed", exc)
            self._release_camera()
            return False
        self._last_frame_monotonic = time.monotonic()
        quality = "calibrated" if self._calibrated else "FOV estimate"
        print(f"[定位] tag 摄像头 {source}: {actual_w}x{actual_h}, {quality}")
        return True

    def _release_camera(self):
        cap, self._cap = self._cap, None
        if cap is not None:
            cap.release()

    def _record_error(self, reason, exc=None):
        self._capture_error = reason + (f":{type(exc).__name__}:{exc}" if exc else "")
        now = time.monotonic()
        key = (reason, type(exc).__name__ if exc else "")
        if key != self._last_error_key or now - self._last_error_log >= 5.0:
            print(f"[定位] Tag camera {self._capture_error}; "
                  f"reopens={self._reopen_count}", flush=True)
            if exc is not None:
                # Python-level run loggers may not capture native stderr.
                print(traceback.format_exc(), flush=True)
            self._last_error_log = now
            self._last_error_key = key

    def stop(self):
        with self._lifecycle_lock:
            self._running = False
            self._stop_event.set()
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=2.0)
            if self._thread and self._thread.is_alive():
                # Only the owner thread may release a capture during native I/O.
                print("[定位] Tag camera stopping; waiting for native I/O", flush=True)
            else:
                self._release_camera()
        print("[定位] 已停止")

    @property
    def is_running(self) -> bool:
        return bool(self._running and self._thread and self._thread.is_alive())

    @property
    def result(self) -> Optional[FieldPose]:
        with self._result_lock:
            result = self._result
        if result is None:
            return None
        error = self._capture_error
        if not self.is_running:
            error = error or "capture_thread_stopped"
        elif time.monotonic() - self._last_frame_monotonic > self.REOPEN_AFTER_S:
            error = error or "capture_stalled"
        if error:
            # Never refresh the timestamp of a failed read or serve an old pose
            # as valid during recovery.
            return replace(result, valid=False, tag_solutions=(), capture_error=error)
        return result

    def reset_filter(self):
        with self._filter_lock:
            self._smoothed = None

    def _smooth(self, x_m: float, y_m: float, yaw_deg: float):
        with self._filter_lock:
            return self._smooth_locked(x_m, y_m, yaw_deg)

    def _smooth_locked(self, x_m: float, y_m: float, yaw_deg: float):
        if self._smoothed is None:
            self._smoothed = (x_m, y_m, yaw_deg)
            return self._smoothed
        old_x, old_y, old_yaw = self._smoothed
        step = math.hypot(x_m - old_x, y_m - old_y)
        solver = self._config["solver"]
        alpha = min(float(solver["ema_max_alpha"]),
                    float(solver["ema_min_alpha"]) + step)
        yaw_delta = _wrap_angle(yaw_deg - old_yaw)
        self._smoothed = (
            old_x + alpha * (x_m - old_x),
            old_y + alpha * (y_m - old_y),
            _wrap_angle(old_yaw + alpha * yaw_delta),
        )
        return self._smoothed

    def _read_frame(self):
        # V4L2 poll bounds normal inter-frame waits. Its first grab/open may
        # still take the backend timeout; never release it from another thread.
        if hasattr(cv2.VideoCapture, "waitAny"):
            ready, indices = cv2.VideoCapture.waitAny(
                [self._cap], timeoutNs=self.FRAME_WAIT_NS)
            if not ready or len(indices) == 0:
                return False, None
            return self._cap.retrieve()
        return self._cap.read()

    def _capture_loop(self):
        try:
            while self._running and not self._stop_event.is_set():
                if self._cap is None:
                    if not self._open_camera():
                        self._stop_event.wait(self.REOPEN_RETRY_S)
                        continue
                    self.reset_filter()
                try:
                    ok, frame = self._read_frame()
                except Exception as exc:
                    self._record_error("read_exception", exc)
                    ok, frame = False, None
                if not self._running or self._stop_event.is_set():
                    break
                if not ok or frame is None or frame.size == 0:
                    if not self._capture_error:
                        self._record_error("read_failed")
                    if time.monotonic() - self._last_frame_monotonic >= self.REOPEN_AFTER_S:
                        self._record_error("capture_stalled_reopening")
                        self._release_camera()
                        self._reopen_count += 1
                        self._stop_event.wait(0.05)
                    else:
                        self._stop_event.wait(0.01)
                    continue
                captured_at = time.time()
                self._last_frame_monotonic = time.monotonic()
                try:
                    self._process_frame(frame, captured_at,
                                        captured_monotonic=self._last_frame_monotonic)
                except Exception as exc:
                    # A single corrupt frame or detector exception must not
                    # permanently kill localization for all remaining tasks.
                    self._record_error("processing_error", exc)
                    self._stop_event.wait(0.01)
                    continue
                if self._capture_error:
                    print(f"[定位] Tag camera recovered; reopens={self._reopen_count}",
                          flush=True)
                self._capture_error = ""
        except Exception as exc:
            self._record_error("capture_thread_error", exc)
        finally:
            self._running = False
            self._release_camera()
            if self._show_gui:
                cv2.destroyWindow("Field Localization")

    def _process_frame(self, frame, captured_at, *, captured_monotonic=None):
        # Preserve game moving-Tag guidance: acquisition time precedes solving.
        # Exposure/driver latency still needs separate hardware calibration.
        if captured_monotonic is None:
            captured_monotonic = time.monotonic()
        corners, ids, _ = self._detector.detectMarkers(frame)
        detected_ids = tuple(int(value) for value in ids.flatten()) \
            if ids is not None else ()
        solutions: List[TagSolution] = []
        rejected = []
        if ids is not None:
            for marker_corners, tag_id in zip(corners, ids.flatten()):
                reasons = []
                solution = solve_tag_pose(
                    marker_corners.reshape(4, 2), int(tag_id), self._config,
                    self._camera_matrix, self._dist_coeffs, reasons)
                if solution is not None:
                    solutions.append(solution)
                else:
                    rejected.extend(reasons)

        fused = fuse_tag_solutions(solutions, self._config)
        now = captured_at
        if fused is None:
            result = FieldPose(
                valid=False, tag_ids=detected_ids, timestamp=now,
                fps=self._fps, calibrated=self._calibrated,
                raw_tag_ids=detected_ids, rejection_reasons=tuple(rejected),
                captured_monotonic=captured_monotonic)
        else:
            x_m, y_m, yaw_deg, camera_height, error, used = fused
            x_m, y_m, yaw_deg = self._smooth(x_m, y_m, yaw_deg)
            result = FieldPose(
                valid=True, x_m=x_m, y_m=y_m, yaw_deg=yaw_deg,
                camera_height_m=camera_height,
                reprojection_error_px=error,
                tag_ids=tuple(item.tag_id for item in used),
                timestamp=now, fps=self._fps,
                calibrated=self._calibrated,
                tag_solutions=tuple(solutions),
                raw_tag_ids=detected_ids, rejection_reasons=tuple(rejected),
                captured_monotonic=captured_monotonic)
        with self._result_lock:
            self._result = result

        self._frame_count += 1
        elapsed = time.monotonic() - self._fps_started
        if elapsed >= 1.0:
            self._fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_started = time.monotonic()
        if self._show_gui:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            if result.valid:
                cv2.putText(
                    frame, f"x={result.x_m:.2f} y={result.y_m:.2f} m",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 0), 2)
            cv2.imshow("Field Localization", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                self._running = False
                self._stop_event.set()


def main() -> int:
    parser = argparse.ArgumentParser(description="AprilTag field localization")
    parser.add_argument("--camera", default="tag")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--duration", type=float, default=0.0)
    args = parser.parse_args()
    localizer = FieldLocalizer(camera=args.camera, show_gui=args.gui)
    if not localizer.start():
        return 1
    started = time.monotonic()
    try:
        while args.duration <= 0.0 or time.monotonic() - started < args.duration:
            pose = localizer.result
            if pose is not None and pose.valid:
                print(f"\r[定位] x={pose.x_m:+.3f} y={pose.y_m:+.3f} m "
                      f"yaw={pose.yaw_deg:+.1f} deg tags={pose.tag_ids} "
                      f"err={pose.reprojection_error_px:.2f}px "
                      f"fps={pose.fps:.1f}", end="", flush=True)
            elif pose is not None:
                print(f"\r[定位] SEARCHING visible={pose.tag_ids} "
                      f"fps={pose.fps:.1f}", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        localizer.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
