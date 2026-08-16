#!/usr/bin/env python3
"""
Uniforest robot facade and subsystem lifecycle.

Architecture:
    Pi (上位机) ←→ STM32 A-board (下位机) via UART7 (115200 bps)

    Pi handles all decision logic:
    - Path planning & navigation
    - Action sequences (Build, Grap, etc.)
    - Vision processing
    - PID parameter tuning

    STM32 executes low-level commands:
    - CAN bus motor control (speed PID @ 1 kHz)
    - Servo PWM
    - Stepper pulse generation (non-blocking, TIM7 @ 100 kHz)
    - IMU / RC data acquisition & telemetry

The official competition entry is main.py. This module also retains the
manual diagnostic console used by tools/debug_console.py.
"""

import sys
import time
import threading
import argparse
import glob
import os
from typing import Optional

from protocol import Transport, TelemBatch
from control import (
    Chassis, Servo, Stepper, Actions,
    DEFAULT_MOVE_SPEED_MM_S, LinearMoveResult,
)

# Vision is optional — only imported if enabled
try:
    from vision import (
        CubeDetector, VisionResult, FieldLocalizer, FieldPose,
        default_camera_selector,
    )
    HAS_VISION = True
except ImportError:
    HAS_VISION = False
    CubeDetector = None
    VisionResult = None
    FieldLocalizer = None
    FieldPose = None


def default_serial_port() -> str:
    """Return the stable A-board serial path for the current platform."""
    if sys.platform == 'win32':
        return 'COM5'

    daplink_ports = sorted(glob.glob(
        '/dev/serial/by-id/*CMSIS-DAP*-if02'))
    if daplink_ports:
        return daplink_ports[0]
    if os.path.exists('/dev/ttyACM0'):
        return '/dev/ttyACM0'
    return '/dev/serial0'


class Robot:
    """
    Main robot controller — owns all subsystems.
    """

    SERIAL_PORT = default_serial_port()
    SERIAL_BAUD = 115200

    def __init__(self, port: str = None, baud: int = None,
                 enable_vision: bool = False, camera_id=None,
                 vision_gui: bool = False,
                 vision_exposure: float = None,
                 vision_gain: float = None,
                 enable_localization: bool = False,
                 localization_camera='tag',
                 localization_gui: bool = False,
                 debug: bool = False):
        if port is None:
            port = self.SERIAL_PORT
        if baud is None:
            baud = self.SERIAL_BAUD

        # Transport layer
        self.transport = Transport(port, baud, debug=debug)

        # Control subsystems
        self.chassis = Chassis(self.transport)
        self.servo = Servo(self.transport)
        self.stepper = Stepper(self.transport)
        self.actions = Actions(self.servo, self.stepper,
                               telem_getter=lambda: self.telem)

        # Vision subsystem (optional)
        self._vision: Optional['CubeDetector'] = None
        if enable_vision and HAS_VISION:
            self._vision = CubeDetector(
                camera_id=camera_id,
                show_gui=vision_gui,
                exposure=vision_exposure,
                gain=vision_gain,
            )
        elif enable_vision and not HAS_VISION:
            print("[Robot] 视觉模块不可用（opencv-python 未安装）")

        # Latest telemetry
        self._telem: Optional[TelemBatch] = None
        self._telem_lock = threading.Lock()
        self._pong_event = threading.Event()

        # Dedicated AprilTag camera and full-field localization subsystem.
        self._localizer: Optional['FieldLocalizer'] = None
        if enable_localization and HAS_VISION:
            self._localizer = FieldLocalizer(
                camera=localization_camera,
                show_gui=localization_gui,
            )
        elif enable_localization and not HAS_VISION:
            print("[Robot] Field localization unavailable (OpenCV missing)")

        # State
        self._running = False

    # ==================== Lifecycle ==========================================

    def connect(self) -> bool:
        """Connect to STM32 and start telemetry."""
        if not self.transport.connect():
            print("[Robot] Failed to connect to STM32")
            return False

        # Register telemetry callback
        self.transport.on_telemetry(self._on_telem)
        self.transport.on_ack(self._on_ack)
        self.transport.on_pong(self._on_pong)

        # Opening a stale DAPLink COM handle can succeed after target power loss,
        # even though writes time out.  Require a real STM32 PONG before claiming
        # that the robot is connected; retry while the MCU finishes booting.
        for _ in range(5):
            self._pong_event.clear()
            if self.transport.ping() and self._pong_event.wait(0.7):
                print(f"[Robot] Connected to STM32 on {self.transport._port} "
                      f"@ {self.transport._baudrate}")
                return True
            time.sleep(0.2)

        print("[Robot] COM port opened but STM32 did not answer PING. "
              "Replug DAPLink USB after restoring robot power.")
        self.transport.disconnect()
        return False

    def start(self, telem_rate: int = 50):
        """Start telemetry streaming and vision (if enabled)."""
        self._running = True
        time.sleep(0.1)  # let connection settle
        self.transport.set_telemetry_rate(telem_rate)
        print(f"[Robot] Telemetry streaming at {telem_rate} Hz")

        # Start vision if configured
        if self._vision is not None:
            if self._vision.start():
                print("[Robot] Vision subsystem active")
            else:
                print("[Robot] Vision failed to start")
        if self._localizer is not None:
            if self._localizer.start():
                print("[Robot] Field localization active")
            else:
                print("[Robot] Field localization failed to start")

    def stop(self):
        """Emergency stop and disconnect."""
        self._running = False
        # Stop vision first
        if self._vision is not None:
            self._vision.stop()
        if self._localizer is not None:
            self._localizer.stop()
        self.transport.emergency_stop()
        time.sleep(0.1)
        self.transport.disconnect()
        print("[Robot] Disconnected")

    # ==================== Telemetry Callbacks ================================

    def _on_telem(self, telem: TelemBatch):
        with self._telem_lock:
            self._telem = telem
            # Forward to chassis for position tracking
            self.chassis.update_telem(telem)

    def _on_ack(self, ack):
        if ack.status != 0:
            print(f"[Robot] ACK error: cmd=0x{ack.echoed_cmd:02X} status={ack.status}")

    def _on_pong(self, uptime_ms: int):
        self._pong_event.set()
        print(f"[Robot] PONG — STM32 uptime: {uptime_ms} ms")

    @property
    def telem(self) -> Optional[TelemBatch]:
        with self._telem_lock:
            return self._telem

    @property
    def vision_result(self) -> Optional['VisionResult']:
        """Get latest vision detection result (thread-safe)."""
        if self._vision is not None:
            return self._vision.result
        return None

    def reset_vision_filter(self):
        """Clear stale temporal tracking before a new vision task."""
        if self._vision is not None:
            self._vision.reset_filter()

    @property
    def has_vision(self) -> bool:
        return self._vision is not None and self._vision.is_running

    @property
    def field_pose(self) -> Optional['FieldPose']:
        """Latest pure-vision field pose from the dedicated tag camera."""
        if self._localizer is not None:
            return self._localizer.result
        return None

    @property
    def has_field_localization(self) -> bool:
        return self._localizer is not None and self._localizer.is_running

    def reset_field_localization_filter(self):
        if self._localizer is not None:
            self._localizer.reset_filter()

    # ==================== Telemetry Display ==================================

    def print_telem(self):
        """Print one telemetry snapshot."""
        t = self.telem
        if t is None:
            print("No telemetry yet...")
            return

        print(f"\n{'='*60}")
        print(f"Uptime: {t.uptime_ms/1000:.1f}s  Yaw: {t.yaw_deg:.1f}°  YawRate: {t.yaw_rate_ds:.1f}°/s")
        print(f"{'='*60}")
        print(f"{'Motor':>8} {'Angle':>6} {'RPM':>8} {'Torque':>8} {'Temp':>5}")
        print(f"{'-'*40}")
        names = ["TR", "TL", "BL", "BR"]
        for i, m in enumerate(t.motors):
            print(f"{names[i]:>8} {m.angle:>6} {m.speed_rpm:>8} {m.torque_current:>8} {m.temperature:>4}°C")
        print(f"{'-'*40}")
        print(f"RC: CH1={t.rc_channels[0]:>5} CH2={t.rc_channels[1]:>5} "
              f"CH3={t.rc_channels[2]:>5} CH4={t.rc_channels[3]:>5}")
        print(f"Stepper: H={'BUSY' if t.stepper_busy & 1 else 'idle'} "
              f"V={'BUSY' if t.stepper_busy & 2 else 'idle'} "
              f"Pos=({t.stepper_pos[0]}, {t.stepper_pos[1]})")

        # Vision status
        if self.has_vision:
            vr = self.vision_result
            if vr and vr.is_valid:
                print(f"Vision: [{vr.color_name}] X={vr.x:+.0f} Y={vr.y:+.0f} "
                      f"Z={vr.z:+.0f}mm Dist={vr.distance:.0f}mm "
                      f"Conf={vr.confidence:.0f}% FPS={vr.fps:.1f}")
            elif vr:
                print(f"Vision: SEARCHING ({len(vr.all_blocks)} blocks) FPS={vr.fps:.1f}")
            else:
                print(f"Vision: initializing...")
        if self.has_field_localization:
            pose = self.field_pose
            if pose and pose.valid:
                quality = "cal" if pose.calibrated else "FOV"
                print(f"Field: X={pose.x_m:+.3f} Y={pose.y_m:+.3f} m "
                      f"Yaw={pose.yaw_deg:+.1f} deg Tags={pose.tag_ids} "
                      f"Err={pose.reprojection_error_px:.2f}px {quality}")
            elif pose:
                print(f"Field: SEARCHING visible={pose.tag_ids} FPS={pose.fps:.1f}")

    def telemetry_monitor(self, duration_s: float = 0):
        """Display telemetry continuously for duration_s (0 = forever)."""
        t0 = time.time()
        try:
            while self._running:
                self.print_telem()
                time.sleep(0.5)
                if duration_s > 0 and (time.time() - t0) > duration_s:
                    break
        except KeyboardInterrupt:
            pass

    # ==================== Vision-Guided Navigation =========================

    def move_chassis(self, direction: str, distance_mm: float,
                     speed_mm_s: float = DEFAULT_MOVE_SPEED_MM_S,
                     hold_ms: Optional[int] = None,
                     accel_ms: Optional[int] = None,
                     ) -> LinearMoveResult:
        """Run a blocking calibrated position-loop chassis move."""
        direction = direction.lower()
        if distance_mm <= 0.0:
            raise ValueError('distance must be positive')
        if speed_mm_s <= 0.0:
            raise ValueError('speed must be positive')

        move_kwargs = {}
        if hold_ms is not None:
            move_kwargs['hold_ms'] = hold_ms
        if accel_ms is not None:
            move_kwargs['accel_ms'] = accel_ms
        moves = {
            'forward': lambda: self.chassis.move_forward(
                distance_mm, speed_mm_s, **move_kwargs),
            'backward': lambda: self.chassis.move_forward(
                -distance_mm, speed_mm_s, **move_kwargs),
            'left': lambda: self.chassis.move_right(
                -distance_mm, speed_mm_s, **move_kwargs),
            'right': lambda: self.chassis.move_right(
                distance_mm, speed_mm_s, **move_kwargs),
        }
        if direction not in moves:
            raise ValueError(
                'direction must be forward, backward, left, or right')

        result = moves[direction]()
        state = ('cancelled' if result.cancelled else
                 'timeout' if result.timed_out else 'complete')
        print(f'[Chassis] {state}: {direction} {distance_mm:.1f} mm, '
              f'wheel={result.encoder_distance_mm:.1f} mm, '
              f'chassis_est={result.estimated_chassis_distance_mm:.1f} mm, '
              f'time={result.elapsed_ms:.0f} ms')
        return result

    def approach_cube(self, target_z: float = 200.0,
                      speed_mm_s: float = 150.0,
                      timeout_s: float = 30.0):
        """
        Navigate chassis toward detected cube using vision feedback.

        Strategy:
          1. Turn to center cube X in camera (reduce |X| below threshold)
          2. Drive forward until Z ≈ target_z
          3. Re-center as needed

        Args:
            target_z: stop when cube is this far away (mm)
            speed_mm_s: approach speed
            timeout_s: max duration before giving up
        """
        if not self.has_vision:
            print("[Robot] Vision not available — cannot approach")
            return

        print(f"[Robot] Approaching cube (target Z={target_z:.0f}mm)...")
        t0 = time.time()

        X_THRESHOLD = 30.0   # mm — acceptable lateral error
        Z_THRESHOLD = 40.0   # mm — acceptable depth error
        TURN_K = 0.8         # °/s per mm of X error

        while (time.time() - t0) < timeout_s:
            vr = self.vision_result
            if vr is None or not vr.is_valid:
                print("  [Vision] Lost target — searching...")
                self.chassis.stop()
                time.sleep(0.1)
                continue

            x_err = vr.x    # lateral error in mm
            z_err = vr.z - target_z  # depth error

            # Check if we've arrived
            if abs(z_err) < Z_THRESHOLD and abs(x_err) < X_THRESHOLD:
                self.chassis.stop()
                print(f"  [Arrived] X={vr.x:+.0f} Z={vr.z:.0f}mm "
                      f"(errors: X={x_err:+.0f} Z={z_err:+.0f}mm)")
                return

            # Compute control
            # 1. Turn to center (priority if X is large)
            wz = -x_err * TURN_K   # negative: if cube is to the right (+X), turn right (+wz)
            wz = max(-120.0, min(120.0, wz))  # clamp

            # 2. Forward speed (proportional to Z error, limited)
            vx = z_err * 0.3   # cm/s per mm
            vx = max(-speed_mm_s / 10.0, min(speed_mm_s / 10.0, vx))

            # Mecanum: vx forward + wz rotation
            rpm = self.chassis.mecanum_rpm(vx, 0.0, wz)
            self.chassis.set_speeds(rpm)

            # Status
            if int(time.time() * 4) % 4 == 0:  # print ~4 Hz
                print(f"  [{vr.color_name}] X={x_err:+.0f}mm Z={z_err:+.0f}mm "
                      f"→ vx={vx:.1f}cm/s wz={wz:.1f}°/s")

            time.sleep(0.02)  # 50 Hz control

        self.chassis.stop()
        print(f"[Robot] Approach timeout ({timeout_s}s)")

    # ==================== Action Runner ======================================

    def run_action(self, name: str):
        """Run a named action sequence."""
        actions = {
            'home': self.actions.servo_home,
            'hatch_open': self.actions.hatch_open,
            'hatch_close': self.actions.hatch_close,
            'grap1': self.actions.grap1,
            'grap2': self.actions.grap2,
            'grap3': self.actions.grap3,
            'build': self.actions.build,
            'approach': lambda: self.approach_cube(),
        }
        if name not in actions:
            print(f"Unknown action: {name}")
            print(f"Available: {list(actions.keys())}")
            return

        print(f"[Robot] Running action: {name}")
        actions[name]()
        print(f"[Robot] Action {name} complete")


# ============================ CLI ============================================

def debug_main():
    parser = argparse.ArgumentParser(description='Uniforest Robot Controller')
    parser.add_argument('--port', default=Robot.SERIAL_PORT,
                       help=f'Serial port (default: {Robot.SERIAL_PORT})')
    parser.add_argument('--baud', type=int, default=115200,
                       help='Baud rate (default: 115200)')
    parser.add_argument('--test-ping', action='store_true',
                       help='Test communication with PING/PONG')
    parser.add_argument('--action', type=str, default=None,
                       help='Run action: home, hatch_open, hatch_close, '
                            'grap1, grap2, grap3, build')
    parser.add_argument('--telemetry-only', action='store_true',
                       help='Display telemetry stream')
    parser.add_argument('--keyboard', action='store_true',
                       help='Real-time keyboard teleoperation')
    parser.add_argument('--telem-rate', type=int, default=50,
                       help='Telemetry rate in Hz (default: 50)')
    parser.add_argument('--duration', type=float, default=0,
                       help='Duration in seconds (0=forever)')
    parser.add_argument('--vision', action='store_true',
                       help='Enable cube detection vision')
    default_camera = default_camera_selector() if HAS_VISION else 1
    parser.add_argument('--camera', default=default_camera,
                       help='Camera role, stable path, or diagnostic index '
                            f'(default: {default_camera})')
    parser.add_argument('--vision-gui', action='store_true',
                       help='Show vision debug window')
    parser.add_argument('--vision-exposure', type=float, default=None,
                       help='Manual camera exposure (backend-specific value)')
    parser.add_argument('--vision-gain', type=float, default=None,
                       help='Manual camera gain (backend-specific value)')
    parser.add_argument('--localization', action='store_true',
                       help='Enable AprilTag full-field localization')
    parser.add_argument('--tag-camera', default='tag',
                       help='Tag camera role or stable path (default: tag)')
    parser.add_argument('--localization-gui', action='store_true',
                       help='Show field-localization debug window')
    parser.add_argument('--debug', action='store_true',
                       help='Enable transport debug output')

    args = parser.parse_args()

    robot = Robot(port=args.port, baud=args.baud,
                  enable_vision=args.vision,
                  camera_id=args.camera,
                  vision_gui=args.vision_gui,
                  vision_exposure=args.vision_exposure,
                  vision_gain=args.vision_gain,
                  enable_localization=args.localization,
                  localization_camera=args.tag_camera,
                  localization_gui=args.localization_gui,
                  debug=args.debug)

    try:
        if not robot.connect():
            sys.exit(1)

        robot.start(telem_rate=args.telem_rate)

        if args.test_ping:
            print("[Test] Sending PING...")
            robot.transport.ping()
            time.sleep(1.0)

        elif args.action:
            time.sleep(0.5)  # wait for first telemetry
            robot.run_action(args.action)

        elif args.telemetry_only:
            robot.telemetry_monitor(args.duration)

        elif args.keyboard:
            from control.keyboard_control import KeyboardController

            key_actions = {
                '1': lambda: robot.run_action('grap1'),
                '2': lambda: robot.run_action('grap2'),
                '3': lambda: robot.run_action('grap3'),
                '4': lambda: robot.run_action('build'),
                'h': lambda: robot.run_action('home'),
                'o': lambda: robot.run_action('hatch_open'),
                'p': lambda: robot.run_action('hatch_close'),
                'z': robot.servo.gripper_open,
                'x': robot.servo.gripper_close,
                'r': robot.servo.arm_front_up,
                'f': robot.servo.arm_front_down,
            }
            KeyboardController(
                robot.transport, robot.chassis, robot.actions,
                key_actions).run()

        else:
            # Interactive mode
            print("\nUniforest Robot Controller")
            print("===========================")
            print("Commands:")
            print("  home, hatch_open, hatch_close — Servo actions")
            print("  grap1, grap2, grap3, build  — Full action sequences")
            print("  approach                     — Vision-guided cube approach")
            print("  move DIR MM [SPEED]          — Position move; speed defaults to 500 mm/s")
            print("  telem                        — Print telemetry snapshot")
            print("  vision                       — Print vision detection result")
            print("  stop                         — EMERGENCY STOP")
            print("  exit                         — Quit")
            print()

            while robot._running:
                try:
                    cmd = input("> ").strip().lower()
                    if cmd == 'exit':
                        break
                    elif cmd == 'stop':
                        robot.transport.emergency_stop()
                        print("EMERGENCY STOP")
                    elif cmd == 'telem':
                        robot.print_telem()
                    elif cmd == 'vision':
                        vr = robot.vision_result
                        if vr is None:
                            print("Vision: not available (use --vision flag)")
                        elif vr.is_valid:
                            print(f"[{vr.color_name}] X={vr.x:+.1f} Y={vr.y:+.1f} "
                                  f"Z={vr.z:+.1f}mm Dist={vr.distance:.1f}mm "
                                  f"Conf={vr.confidence:.0f}% "
                                  f"({len(vr.all_blocks)} blocks) FPS={vr.fps:.1f}")
                        else:
                            hint = f" ({len(vr.all_blocks)} seen)" if vr.all_blocks else ""
                            print(f"[SEARCHING]{hint} FPS={vr.fps:.1f}")
                    elif cmd.startswith('move '):
                        parts = cmd.split()
                        if len(parts) not in (3, 4):
                            print('Usage: move forward|backward|left|right MM [MM/S]')
                            continue
                        try:
                            distance = float(parts[2])
                            speed = (float(parts[3]) if len(parts) == 4
                                     else DEFAULT_MOVE_SPEED_MM_S)
                            robot.move_chassis(parts[1], distance, speed)
                        except ValueError as exc:
                            print(f'Invalid move: {exc}')
                    elif cmd in ('home', 'hatch_open', 'hatch_close',
                                'grap1', 'grap2', 'grap3', 'build', 'approach'):
                        robot.run_action(cmd)
                    elif cmd == '':
                        pass
                    else:
                        print(f"Unknown: {cmd}")
                        print("Available: home, hatch_open/close, grap1/2/3, "
                              "build, approach, move, telem, vision, stop, exit")
                except KeyboardInterrupt:
                    break
                except EOFError:
                    break

    except KeyboardInterrupt:
        print("\n[Robot] Interrupted")
    finally:
        robot.stop()


if __name__ == '__main__':
    debug_main()
