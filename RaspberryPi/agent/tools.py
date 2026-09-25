"""Tool definitions and safe adapters for the natural-language agent.

The model only sees these high-level tools.  It never receives direct access
to PWM, CAN frames, serial writes, or individual wheel commands.
"""

from __future__ import annotations

import base64
import json
import os
import threading
import time
from dataclasses import asdict, is_dataclass
from typing import Any, Dict, Optional


MAX_MOVE_DISTANCE_MM = 2000.0
MAX_MOVE_SPEED_MM_S = 750.0
MAX_ROTATION_DEG = 180.0
MAX_ROTATION_SPEED_DEG_S = 120.0
STALE_TELEMETRY_S = 0.5


def _json_default(value: Any):
    if is_dataclass(value):
        return asdict(value)
    if hasattr(value, "item"):
        return value.item()
    if isinstance(value, tuple):
        return list(value)
    raise TypeError(f"cannot serialize {type(value).__name__}")


def json_text(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, default=_json_default)


def _field(obj: Any, name: str, default=None):
    return getattr(obj, name, default) if obj is not None else default


def tool_definitions() -> list[dict]:
    """Return strict Responses API function definitions."""
    return [
        {
            "type": "function",
            "name": "grab_cube",
            "description": "按已确认的比赛上下文抓取一个方块。task_context 和 color 必须明确；禁止对‘抓一个方块’自行猜测动作。Task1橙色=grap3，Task2紫色=grap2，Task2橙色=grap1。",
            "parameters": {"type": "object", "properties": {
                "task_context": {"type": "string", "enum": ["task1", "task2"]},
                "color": {"type": "string", "enum": ["orange", "purple"]},
            }, "required": ["task_context", "color"], "additionalProperties": False},
            "strict": True,
        },
        {
            "type": "function",
            "name": "approach_wall",
            "description": "低速驱动直到电机堵转确认顶墙，然后停车；用于碰墙定位，最多4秒。",
            "parameters": {"type": "object", "properties": {
                "direction": {"type": "string", "enum": ["forward", "backward", "left", "right"]},
                "speed_mm_s": {"type": "number", "minimum": 50, "maximum": 250},
                "timeout_s": {"type": "number", "minimum": 0.5, "maximum": 4},
            }, "required": ["direction", "speed_mm_s", "timeout_s"], "additionalProperties": False},
            "strict": True,
        },
        {
            "type": "function",
            "name": "get_robot_state",
            "description": "读取机器人连接、遥测、姿态、视觉和当前策略状态。只读。",
            "parameters": {
                "type": "object", "properties": {}, "required": [],
                "additionalProperties": False,
            },
            "strict": True,
        },
        {
            "type": "function",
            "name": "get_camera_snapshot",
            "description": "获取一个或两个摄像头的最新静态图像，用于视觉问答。只读。",
            "parameters": {
                "type": "object",
                "properties": {
                    "cameras": {
                        "type": "array", "items": {
                            "type": "string", "enum": ["cube", "tag"]
                        }, "minItems": 1, "maxItems": 2,
                    },
                    "fresh": {"type": "boolean"},
                },
                "required": ["cameras", "fresh"],
                "additionalProperties": False,
            },
            "strict": True,
        },
        {
            "type": "function",
            "name": "detect_cubes",
            "description": "读取本地方块视觉结果，包括颜色、坐标、距离和置信度。只读。",
            "parameters": {
                "type": "object",
                "properties": {"color": {"type": ["string", "null"]}},
                "required": ["color"], "additionalProperties": False,
            },
            "strict": True,
        },
        {
            "type": "function",
            "name": "detect_tags",
            "description": "读取本地 AprilTag 识别和定位结果，包括代码、距离、横向偏差和位姿。只读。",
            "parameters": {
                "type": "object",
                "properties": {"tag_id": {"type": ["integer", "null"]}},
                "required": ["tag_id"], "additionalProperties": False,
            },
            "strict": True,
        },
        {
            "type": "function",
            "name": "move_chassis",
            "description": "让底盘沿一个方向移动指定距离。会阻塞直到完成或超时；距离和速度受本地安全限制。",
            "parameters": {
                "type": "object",
                "properties": {
                    "direction": {"type": "string", "enum": ["forward", "backward", "left", "right"]},
                    "distance_mm": {"type": "number", "minimum": 1, "maximum": MAX_MOVE_DISTANCE_MM},
                    "speed_mm_s": {"type": "number", "minimum": 1, "maximum": MAX_MOVE_SPEED_MM_S},
                },
                "required": ["direction", "distance_mm", "speed_mm_s"],
                "additionalProperties": False,
            },
            "strict": True,
        },
        {
            "type": "function",
            "name": "rotate_chassis",
            "description": "让底盘按顺时针为正旋转指定角度。会阻塞直到完成或超时。",
            "parameters": {
                "type": "object",
                "properties": {
                    "angle_deg": {"type": "number", "minimum": -MAX_ROTATION_DEG, "maximum": MAX_ROTATION_DEG},
                    "speed_deg_s": {"type": "number", "minimum": 1, "maximum": MAX_ROTATION_SPEED_DEG_S},
                },
                "required": ["angle_deg", "speed_deg_s"],
                "additionalProperties": False,
            },
            "strict": True,
        },
        {
            "type": "function",
            "name": "execute_arm_action",
            "description": "执行一套已经标定好的机械臂动作。动作名只能是 home、hatch_open、hatch_close、grap1、grap2、grap3、build。",
            "parameters": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["home", "hatch_open", "hatch_close", "grap1", "grap2", "grap3", "build"]},
                },
                "required": ["action"], "additionalProperties": False,
            },
            "strict": True,
        },
        {
            "type": "function",
            "name": "run_strategy",
            "description": "启动一个已有的比赛策略。策略在后台运行，可用 get_robot_state 查看状态。",
            "parameters": {
                "type": "object",
                "properties": {
                    "selection": {"type": "string", "enum": ["all", "round1", "round2", "task1", "task2", "task1-r1", "task2-r1", "task1-r2", "task2-r2"]},
                },
                "required": ["selection"], "additionalProperties": False,
            },
            "strict": True,
        },
        {
            "type": "function",
            "name": "cancel_current_action",
            "description": "请求停止当前策略或动作，并发送急停。",
            "parameters": {
                "type": "object", "properties": {}, "required": [],
                "additionalProperties": False,
            },
            "strict": True,
        },
        {
            "type": "function",
            "name": "emergency_stop",
            "description": "立即停止底盘和机械动作。任何时候都可以调用，优先级最高。",
            "parameters": {
                "type": "object", "properties": {}, "required": [],
                "additionalProperties": False,
            },
            "strict": True,
        },
    ]


class ToolResult:
    """Text plus optional image content returned to the model."""

    def __init__(self, value: Any, images: Optional[list[str]] = None):
        self.value = value
        self.images = images or []

    def response_output(self):
        # Responses accepts a normal string for text-only results.  Image
        # results use the structured content form so the model can inspect the
        # returned frame instead of receiving a base64 string as text.
        if not self.images:
            return json_text(self.value)
        content = [{"type": "input_text", "text": json_text(self.value)}]
        content.extend({"type": "input_image", "image_url": image}
                       for image in self.images)
        return content


class RobotToolExecutor:
    """Execute the model's high-level tools against a Robot instance."""

    def __init__(self, robot=None, *, dry_run: bool = False):
        self.robot = robot
        self.dry_run = dry_run
        self._lock = threading.RLock()
        self._strategy_thread = None
        self._strategy_name = None
        self._strategy_result = None
        self._strategy_started_at = None

    def _require_robot(self):
        if self.robot is None:
            raise RuntimeError("机器人未连接；请先使用真实硬件启动，或使用 --dry-run")

    def _strategy_state(self) -> dict:
        thread = self._strategy_thread
        return {
            "name": self._strategy_name,
            "running": bool(thread and thread.is_alive()),
            "result": self._strategy_result,
            "started_at": self._strategy_started_at,
        }

    def get_robot_state(self, **_):
        if self.dry_run:
            return ToolResult({"dry_run": True, "connected": False,
                               "strategy": self._strategy_state()})
        self._require_robot()
        telem = self.robot.telem
        received_at = getattr(self.robot, "_telem_received_at", None)
        age_s = None if received_at is None else max(0.0, time.monotonic() - received_at)
        state = {
            "connected": bool(self.robot.transport.connected),
            "telemetry_available": telem is not None,
            "telemetry_age_s": age_s,
            "vision_active": self.robot.has_vision,
            "localization_active": self.robot.has_field_localization,
            "strategy": self._strategy_state(),
        }
        if telem is not None:
            state.update({
                "uptime_ms": _field(telem, "uptime_ms"),
                "yaw_deg": _field(telem, "yaw_deg"),
                "yaw_rate_ds": _field(telem, "yaw_rate_ds"),
                "motor_speed_rpm": [_field(item, "speed_rpm") for item in (_field(telem, "motors", []) or [])],
                "stepper_busy": _field(telem, "stepper_busy"),
            })
        return ToolResult(state)

    def _camera_source(self, role: str):
        try:
            from vision import resolve_camera_source
        except ImportError:
            from Vision.opencv.camera_devices import resolve_camera_source
        return resolve_camera_source(role)

    def _snapshot(self, role: str) -> tuple[dict, str]:
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("OpenCV 未安装，无法获取摄像头图像") from exc
        source = self._camera_source(role)
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2 if os.name != "nt" else 0)
        try:
            if not cap.isOpened():
                raise RuntimeError(f"无法打开 {role} 摄像头: {source}")
            frame = None
            for _ in range(3):
                ok, candidate = cap.read()
                if ok:
                    frame = candidate
            if frame is None:
                raise RuntimeError(f"无法读取 {role} 摄像头画面")
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
            if not ok:
                raise RuntimeError(f"无法编码 {role} 摄像头画面")
            image = "data:image/jpeg;base64," + base64.b64encode(encoded.tobytes()).decode("ascii")
            return {"camera": role, "source": str(source), "width": int(frame.shape[1]), "height": int(frame.shape[0]), "timestamp": time.time()}, image
        finally:
            cap.release()

    def get_camera_snapshot(self, cameras, fresh=True):
        if self.dry_run:
            return ToolResult({"dry_run": True, "cameras": cameras,
                               "message": "dry-run 未读取真实摄像头"})
        snapshots = []
        images = []
        for role in cameras:
            metadata, image = self._snapshot(role)
            snapshots.append(metadata)
            images.append(image)
        return ToolResult({"snapshots": snapshots}, images)

    def detect_cubes(self, color=None):
        if self.dry_run:
            return ToolResult({"dry_run": True, "color": color, "blocks": []})
        self._require_robot()
        result = self.robot.vision_result
        blocks = [] if result is None else list(_field(result, "all_blocks", []) or [])
        if color:
            color_lower = color.lower()
            blocks = [b for b in blocks if str(_field(b, "color_name", "")).lower() == color_lower]
        value = {
            "active": self.robot.has_vision,
            "timestamp": _field(result, "timestamp"),
            "selected": None if result is None else {
                "color": _field(result, "color_name"), "x_mm": _field(result, "x"),
                "y_mm": _field(result, "y"), "z_mm": _field(result, "z"),
                "distance_mm": _field(result, "distance"), "confidence": _field(result, "confidence"),
            },
            "blocks": [{"color": _field(b, "color_name"), "x_mm": _field(b, "x"),
                        "y_mm": _field(b, "y"), "z_mm": _field(b, "z"),
                        "confidence": _field(b, "confidence")} for b in blocks],
        }
        return ToolResult(value)

    def detect_tags(self, tag_id=None):
        if self.dry_run:
            return ToolResult({"dry_run": True, "tag_id": tag_id, "tags": []})
        self._require_robot()
        pose = self.robot.field_pose
        solutions = [] if pose is None else list(_field(pose, "tag_solutions", []) or [])
        if tag_id is not None:
            solutions = [s for s in solutions if _field(s, "tag_id") == tag_id]
        value = {
            "active": self.robot.has_field_localization,
            "valid": bool(_field(pose, "valid", False)),
            "visible_tag_ids": list(_field(pose, "tag_ids", ()) or ()),
            "timestamp": _field(pose, "timestamp"),
            "pose": None if pose is None else {
                "x_m": _field(pose, "x_m"), "y_m": _field(pose, "y_m"),
                "yaw_deg": _field(pose, "yaw_deg"), "reprojection_error_px": _field(pose, "reprojection_error_px"),
            },
            "tags": [{"id": _field(s, "tag_id"), "distance_m": _field(s, "distance_m"),
                      "lateral_m": _field(s, "lateral_m"), "relative_yaw_deg": _field(s, "relative_yaw_deg"),
                      "reprojection_error_px": _field(s, "reprojection_error_px")} for s in solutions],
        }
        return ToolResult(value)

    def _guard_motion(self):
        self._require_robot()
        if self._strategy_thread and self._strategy_thread.is_alive():
            raise RuntimeError("比赛策略正在运行，不能同时发送独立底盘动作")
        if not self.robot.transport.connected:
            raise RuntimeError("STM32 未连接")
        received_at = getattr(self.robot, "_telem_received_at", None)
        if received_at is None or time.monotonic() - received_at > STALE_TELEMETRY_S:
            raise RuntimeError("遥测数据过期，拒绝执行运动")

    def move_chassis(self, direction, distance_mm, speed_mm_s):
        if direction not in {"forward", "backward", "left", "right"}:
            raise ValueError("direction 必须是 forward、backward、left 或 right")
        if not 0.0 < float(distance_mm) <= MAX_MOVE_DISTANCE_MM:
            raise ValueError(f"distance_mm 必须在 0 到 {MAX_MOVE_DISTANCE_MM:.0f} 之间")
        if not 0.0 < float(speed_mm_s) <= MAX_MOVE_SPEED_MM_S:
            raise ValueError(f"speed_mm_s 必须在 0 到 {MAX_MOVE_SPEED_MM_S:.0f} 之间")
        if self.dry_run:
            return ToolResult({"dry_run": True, "would_move": {"direction": direction, "distance_mm": distance_mm, "speed_mm_s": speed_mm_s}})
        with self._lock:
            self._guard_motion()
            result = self.robot.move_chassis(direction, float(distance_mm), float(speed_mm_s), hold_ms=0)
            return ToolResult({"state": "cancelled" if result.cancelled else "timeout" if result.timed_out else "complete",
                               "direction": direction, "requested_mm": distance_mm,
                               "encoder_mm": result.encoder_distance_mm,
                               "estimated_chassis_mm": result.estimated_chassis_distance_mm,
                               "elapsed_ms": result.elapsed_ms})

    def rotate_chassis(self, angle_deg, speed_deg_s):
        if not -MAX_ROTATION_DEG <= float(angle_deg) <= MAX_ROTATION_DEG:
            raise ValueError(f"angle_deg 必须在 ±{MAX_ROTATION_DEG:.0f} 以内")
        if not 0.0 < float(speed_deg_s) <= MAX_ROTATION_SPEED_DEG_S:
            raise ValueError(f"speed_deg_s 必须在 0 到 {MAX_ROTATION_SPEED_DEG_S:.0f} 之间")
        if self.dry_run:
            return ToolResult({"dry_run": True, "would_rotate": {"angle_deg": angle_deg, "speed_deg_s": speed_deg_s}})
        with self._lock:
            self._guard_motion()
            self.robot.chassis.turn(float(angle_deg), float(speed_deg_s), hold_ms=0)
            return ToolResult({"state": "complete", "angle_deg": angle_deg, "speed_deg_s": speed_deg_s})

    def execute_arm_action(self, action):
        if self.dry_run:
            return ToolResult({"dry_run": True, "would_execute": action})
        with self._lock:
            self._require_robot()
            if self._strategy_thread and self._strategy_thread.is_alive():
                raise RuntimeError("比赛策略正在运行，不能同时执行独立机械动作")
            self.robot.run_action(action)
            return ToolResult({"state": "complete", "action": action})

    def grab_cube(self, task_context, color):
        if task_context == "task1" and color == "orange": action = "grap3"
        elif task_context == "task2" and color == "purple": action = "grap2"
        elif task_context == "task2" and color == "orange": action = "grap1"
        else:
            raise ValueError("当前任务没有这个颜色的安全抓取动作；请确认任务和方块颜色")
        return self.execute_arm_action(action)

    def approach_wall(self, direction, speed_mm_s, timeout_s):
        if self.dry_run:
            return ToolResult({"dry_run": True, "would_approach_wall": {"direction": direction, "speed_mm_s": speed_mm_s, "timeout_s": timeout_s}})
        with self._lock:
            self._guard_motion()
            from Strategy.competition import CompetitionProgram
            program = CompetitionProgram(self.robot)
            program._drive_until_wall(direction=direction, speed_mm_s=float(speed_mm_s), timeout_s=float(timeout_s), context="Agent wall approach")
            return ToolResult({"state": "wall_contact", "direction": direction})

    def run_strategy(self, selection):
        if self.dry_run:
            return ToolResult({"dry_run": True, "would_start_strategy": selection})
        with self._lock:
            self._require_robot()
            if self._strategy_thread and self._strategy_thread.is_alive():
                raise RuntimeError("已有比赛策略正在运行")
            from Strategy.runner import run_tasks
            self._strategy_name = selection
            self._strategy_result = None
            self._strategy_started_at = time.time()
            def target():
                try:
                    self._strategy_result = run_tasks(self.robot, selection)
                except BaseException as exc:  # report through state tool
                    self._strategy_result = {"error": str(exc)}
                finally:
                    self._strategy_name = selection
            self._strategy_thread = threading.Thread(target=target, name="agent-strategy", daemon=True)
            self._strategy_thread.start()
            return ToolResult({"state": "started", "selection": selection})

    def cancel_current_action(self, **_):
        if self.dry_run:
            return ToolResult({"dry_run": True, "state": "stop_requested"})
        self._require_robot()
        self.robot.transport.emergency_stop()
        return ToolResult({"state": "stop_requested", "message": "已发送急停；请用 get_robot_state 检查策略状态"})

    def emergency_stop(self, **_):
        if self.dry_run:
            return ToolResult({"dry_run": True, "state": "stopped"})
        self._require_robot()
        self.robot.transport.emergency_stop()
        return ToolResult({"state": "stopped"})

    def execute(self, name: str, arguments: Dict[str, Any]) -> ToolResult:
        method = getattr(self, name, None)
        if method is None or name.startswith("_"):
            raise ValueError(f"未知工具: {name}")
        return method(**arguments)
