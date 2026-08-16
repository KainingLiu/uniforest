"""Real-time keyboard teleoperation with bounded, ramped chassis velocity."""

import math
import threading
import time
from typing import Callable, Dict, Optional, Set

from protocol.transport import Transport
from .actions import Actions, ActionCancelled
from .chassis import Chassis
from .stepper import STEPPER_HORIZ, STEPPER_VERT


class KeyboardController:
    """Drive the chassis while dispatching the original mechanical actions."""

    SPEED_GEARS_CM_S = (25.0, 60.0, 100.0)
    ANGULAR_GEARS_DEG_S = (45.0, 65.0, 90.0)
    MAX_WHEEL_RPM = 2500.0
    LINEAR_ACCEL_CM_S2 = 200.0
    LINEAR_DECEL_CM_S2 = 260.0
    CONTROL_HZ = 50.0
    HEADING_KP = 8.0
    HEADING_KI = 0.15
    HEADING_INTEGRAL_LIMIT = 20.0
    HEADING_MAX_LEAD_DEG = 15.0
    HEADING_CORRECTION_RESERVE_DEG_S = 45.0
    MAX_ANGULAR_OUTPUT_DEG_S = 135.0

    def __init__(self, transport: Transport, chassis: Chassis,
                 actions: Actions, action_map: Dict[str, Callable[[], None]]):
        self._transport = transport
        self._chassis = chassis
        self._actions = actions
        self._action_map = action_map

        self._lock = threading.Lock()
        self._pressed: Set[str] = set()
        self._pending_action: Optional[str] = None
        self._speed_gear = 0

        self._exit_event = threading.Event()
        self._emergency_event = threading.Event()
        self._action_cancel = threading.Event()
        self._action_active = threading.Event()
        self._action_thread: Optional[threading.Thread] = None
        self._actions.set_cancel_event(self._action_cancel)

        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._heading_ref: Optional[float] = None
        self._heading_integral = 0.0
        self._imu_warning_shown = False

    @staticmethod
    def _slew(current: float, target: float, accel: float,
              decel: float, dt: float) -> float:
        slowing = abs(target) < abs(current) or current * target < 0.0
        limit = (decel if slowing else accel) * dt
        delta = target - current
        if abs(delta) <= limit:
            return target
        return current + math.copysign(limit, delta)

    @staticmethod
    def _token(key) -> Optional[str]:
        char = getattr(key, 'char', None)
        if char:
            return char.lower()
        name = getattr(key, 'name', None)
        return name.lower() if name else None

    def _on_press(self, key):
        token = self._token(key)
        if token is None:
            return None

        with self._lock:
            first_press = token not in self._pressed
            self._pressed.add(token)

            if token == 'esc':
                self._exit_event.set()
                self._action_cancel.set()
                return False
            if token == 'space' and first_press:
                self._pressed.clear()
                self._pending_action = None
                self._action_cancel.set()
                self._emergency_event.set()
            elif token in ('f1', 'f2', 'f3') and first_press:
                self._speed_gear = int(token[-1]) - 1
                speed = self.SPEED_GEARS_CM_S[self._speed_gear]
                angular = self.ANGULAR_GEARS_DEG_S[self._speed_gear]
                print(f"[Keyboard] Speed gear {self._speed_gear + 1}: "
                      f"{speed:.0f} cm/s, {angular:.0f} deg/s")
            elif token in self._action_map and first_press:
                if not self._action_active.is_set():
                    self._pending_action = token
        return None

    def _on_release(self, key):
        token = self._token(key)
        if token is not None:
            with self._lock:
                self._pressed.discard(token)

    def _start_action(self, token: str):
        action = self._action_map[token]
        self._action_cancel.clear()
        self._action_active.set()

        def worker():
            try:
                action()
            except ActionCancelled:
                print("[Keyboard] Mechanical action cancelled")
            except Exception as exc:
                print(f"[Keyboard] Mechanical action failed: {exc}")
                self._emergency_event.set()
            finally:
                self._action_active.clear()

        self._action_thread = threading.Thread(
            target=worker, name='mechanical-action', daemon=True)
        self._action_thread.start()

    def _handle_emergency(self):
        self._vx = self._vy = self._wz = 0.0
        self._reset_heading_reference()
        self._transport.emergency_stop()
        self._transport.stepper_stop(STEPPER_HORIZ)
        self._transport.stepper_stop(STEPPER_VERT)
        print("[Keyboard] EMERGENCY STOP")
        self._emergency_event.clear()

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle

    def _reset_heading_reference(self):
        telem = self._chassis.telem
        self._heading_ref = telem.yaw_deg if telem is not None else None
        self._heading_integral = 0.0

    def _closed_loop_wz(self, dt: float, output_limit: float,
                        enabled: bool) -> float:
        """Integrate turn input into a target heading; PI alone drives yaw."""
        telem = self._chassis.telem
        if not enabled:
            self._reset_heading_reference()
            return 0.0
        if telem is None:
            if not self._imu_warning_shown:
                print("[Keyboard] IMU telemetry unavailable; yaw is open-loop")
                self._imu_warning_shown = True
            return self._wz

        self._imu_warning_shown = False
        yaw = telem.yaw_deg
        if self._heading_ref is None:
            self._heading_ref = yaw

        # Q/E directly changes the target heading. On key release _wz becomes
        # zero immediately, so the target freezes in the same control cycle.
        self._heading_ref = self._wrap_angle(
            self._heading_ref + self._wz * dt)
        error = self._wrap_angle(self._heading_ref - yaw)

        # A sustained turn can be slower than the requested yaw rate under
        # load. Do not let the integrated target heading run indefinitely
        # ahead of the measured yaw; retain only a bounded tracking lead.
        if abs(error) > self.HEADING_MAX_LEAD_DEG:
            error = math.copysign(self.HEADING_MAX_LEAD_DEG, error)
            self._heading_ref = self._wrap_angle(yaw + error)

        self._heading_integral += error * dt
        self._heading_integral = max(
            -self.HEADING_INTEGRAL_LIMIT,
            min(self.HEADING_INTEGRAL_LIMIT, self._heading_integral))

        command = (self.HEADING_KP * error
                   + self.HEADING_KI * self._heading_integral)
        return max(-output_limit, min(output_limit, command))

    def _send_chassis(self, dt: float, target_angular_speed: float,
                      heading_enabled: bool = True):
        angular_output_limit = min(
            self.MAX_ANGULAR_OUTPUT_DEG_S,
            target_angular_speed + self.HEADING_CORRECTION_RESERVE_DEG_S)
        closed_loop_wz = self._closed_loop_wz(
            dt, angular_output_limit, heading_enabled)
        rpm = self._chassis.mecanum_rpm(
            self._vx, self._vy, closed_loop_wz)
        peak = max(abs(value) for value in rpm)
        if peak > self.MAX_WHEEL_RPM:
            scale = self.MAX_WHEEL_RPM / peak
            rpm = [value * scale for value in rpm]
        self._chassis.set_speeds(rpm)

    @staticmethod
    def print_help():
        print("""
Keyboard control (keep this terminal focused)
  W/S       forward / backward       A/D       left / right
  Q/E       rotate left / right       F1/F2/F3  speed 25/60/100 cm/s
  IMU       Q/E changes target heading; PI tracks and holds after release
  1/2/3     Grap1 / Grap2 / Grap3     4         Build
  H         servo home                O/P       hatch open / close
  Z/X       gripper open / close      R/F       front arm up / down
  SPACE     emergency stop            ESC       stop and exit
""")

    def run(self):
        try:
            from pynput import keyboard
        except ImportError as exc:
            raise RuntimeError(
                "Keyboard mode needs pynput: python -m pip install pynput") from exc

        self.print_help()
        listener = keyboard.Listener(
            on_press=self._on_press, on_release=self._on_release)
        listener.start()

        period = 1.0 / self.CONTROL_HZ
        last = time.monotonic()
        try:
            while not self._exit_event.is_set():
                loop_start = time.monotonic()
                dt = min(max(loop_start - last, 0.001), 0.1)
                last = loop_start

                if self._emergency_event.is_set():
                    self._handle_emergency()

                with self._lock:
                    keys = set(self._pressed)
                    pending = self._pending_action
                    speed = self.SPEED_GEARS_CM_S[self._speed_gear]
                    angular_speed = self.ANGULAR_GEARS_DEG_S[self._speed_gear]

                action_waiting = pending is not None
                action_running = self._action_active.is_set()
                if action_waiting or action_running:
                    target_vx = target_vy = target_wz = 0.0
                else:
                    forward = float('w' in keys) - float('s' in keys)
                    lateral = float('d' in keys) - float('a' in keys)
                    magnitude = math.hypot(forward, lateral)
                    if magnitude > 1.0:
                        forward /= magnitude
                        lateral /= magnitude
                    target_vx = forward * speed
                    target_vy = lateral * speed
                    target_wz = ((float('q' in keys) - float('e' in keys))
                                 * angular_speed)

                self._vx = self._slew(
                    self._vx, target_vx, self.LINEAR_ACCEL_CM_S2,
                    self.LINEAR_DECEL_CM_S2, dt)
                self._vy = self._slew(
                    self._vy, target_vy, self.LINEAR_ACCEL_CM_S2,
                    self.LINEAR_DECEL_CM_S2, dt)
                # Turn input changes target heading directly. Do not slew this
                # value: a release must freeze the target in the same cycle.
                self._wz = target_wz
                self._send_chassis(
                    dt, angular_speed, heading_enabled=not action_running)

                stopped = max(abs(self._vx), abs(self._vy), abs(self._wz)) < 0.01
                if pending is not None and stopped and not action_running:
                    with self._lock:
                        if self._pending_action == pending:
                            self._pending_action = None
                    self._start_action(pending)

                elapsed = time.monotonic() - loop_start
                if elapsed < period:
                    self._exit_event.wait(period - elapsed)
        finally:
            self._exit_event.set()
            self._action_cancel.set()
            listener.stop()
            self._handle_emergency()
            if self._action_thread and self._action_thread.is_alive():
                self._action_thread.join(timeout=1.0)
