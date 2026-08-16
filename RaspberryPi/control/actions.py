"""
Action sequences — ported from actions.c.

All sequences use the protocol layer to control servos and steppers
via the STM32.  Since stepper moves are now non-blocking, we wait
for estimated completion times (based on known speed profiles).

Servo mapping  →  protocol commands:
    SERVO_GRIPPER   (0) = gripper
    SERVO_ARM_FRONT (1) = arm front
    SERVO_HATCH_A   (2) = hatch A
    SERVO_HATCH_B   (3) = hatch B

Stepper mapping →  protocol commands:
    STEPPER_HORIZ (0) = horizontal (forward/back)
    STEPPER_VERT  (1) = vertical (up/down)
"""

import time
import math
import threading
from typing import Optional, Callable

from protocol.transport import Transport
from protocol.commands import TelemBatch
from .servo import (
    Servo, ANGLE_GRIPPER_OPEN, ANGLE_GRIPPER_CLOSE,
    ANGLE_ARM_FRONT_DOWN, ANGLE_ARM_FRONT_UP,
    ANGLE_HATCH_A_CLOSED, ANGLE_HATCH_A_OPEN,
    ANGLE_HATCH_B_CLOSED, ANGLE_HATCH_B_OPEN,
)
from .stepper import (
    Stepper, STEPPER_HORIZ, STEPPER_VERT,
    STEP_DIR_FORWARD, STEP_DIR_REVERSE,
    STEPS_PER_CM, DEFAULT_START_DELAY, DEFAULT_TARGET_DELAY,
)

# ===================== Speed profile parameters ==============================
# Must match stepper.h defaults.

START_DELAY_US  = 1000   # STM32 6/5 scale => about 417 Hz start
TARGET_DELAY_US = 100    # STM32 6/5 scale => about 4167 Hz cruise
ACCEL_STEPS     = 400
STEPPER_BUSY_START_TIMEOUT_MS = 1000
STEPPER_SETTLE_MS             = 100
STEPPER_POLL_MS               = 10

# Estimated step time:  target 5000 Hz → 200 µs/step → 5000 steps/sec
# But with accel/decel ramp, effective speed is slightly lower.
# Conservative estimate: ~4000 steps/sec = 4 steps/ms.
# For STEPS_PER_CM = 400: ~100 steps/cm → ~25 ms/cm.
# Base estimate + 20% margin.

def _est_move_ms(steps: int) -> float:
    """
    Estimate stepper move duration in ms.

    At 5000 Hz cruise (200 µs/step) with 400-step accel/decel ramps,
    effective rate ≈ 4000–4500 steps/sec.
    We use a conservative 3500 steps/sec + fixed overhead.
    """
    if steps == 0:
        return 0.0
    # Ramp overhead: ~2 × accel ramp at avg speed
    ramp_overhead_ms = (ACCEL_STEPS * 2) / 2500.0 * 1000.0  # ~320 ms
    cruise_steps = max(0, int(steps) - 2 * ACCEL_STEPS)
    cruise_ms = cruise_steps / 4167.0 * 1000.0
    return (ramp_overhead_ms + cruise_ms) * 1.15 + 50  # 15% margin + 50ms fixed


class ActionCancelled(RuntimeError):
    """Raised when the operator aborts a composite mechanical action."""


class Actions:
    """
    Composite action sequences: Build(), Grap1–3(), Hatch, ServoHome.

    Usage:
        actions = Actions(servo, stepper)
        actions.servo_home()
        actions.grap3()
    """

    def __init__(self, servo: Servo, stepper: Stepper,
                 telem_getter: Optional[Callable[[], Optional[TelemBatch]]] = None):
        self.servo = servo
        self.stepper = stepper
        self._telem_getter = telem_getter
        self._cancel_event: Optional[threading.Event] = None

    def set_cancel_event(self, event: Optional[threading.Event]):
        """Attach the emergency-stop event used by keyboard control."""
        self._cancel_event = event

    # ==================== Helpers ============================================

    def _wait(self, ms: float):
        if self._cancel_event is None:
            time.sleep(ms / 1000.0)
        elif self._cancel_event.wait(ms / 1000.0):
            raise ActionCancelled("mechanical action cancelled")

    def _check_cancelled(self):
        if self._cancel_event is not None and self._cancel_event.is_set():
            raise ActionCancelled("mechanical action cancelled")

    def _wait_steppers_stopped(self, motor_mask: int, estimate_ms: float):
        """Wait for the requested motors to report BUSY then IDLE and settle."""
        if self._telem_getter is None:
            # Compatibility fallback for standalone use without telemetry.
            self._wait(estimate_ms + STEPPER_SETTLE_MS)
            return

        # Do not accept the cached IDLE sample from before the move command.
        # First require a newer telemetry frame that reports at least one of
        # the commanded motors BUSY.
        initial = self._telem_getter()
        initial_uptime = initial.uptime_ms if initial is not None else None
        start_deadline = time.monotonic() + STEPPER_BUSY_START_TIMEOUT_MS / 1000.0
        saw_busy = False

        while time.monotonic() < start_deadline:
            self._check_cancelled()
            telem = self._telem_getter()
            if (telem is not None and
                    (initial_uptime is None or telem.uptime_ms != initial_uptime) and
                    (telem.stepper_busy & motor_mask)):
                saw_busy = True
                break
            self._wait(STEPPER_POLL_MS)

        if not saw_busy:
            for motor in range(2):
                if motor_mask & (1 << motor):
                    self.stepper.stop(motor)
            raise RuntimeError("stepper BUSY telemetry did not start; action aborted")

        # Allow ample margin over the calculated duration. If telemetry is
        # lost or a motor remains busy, stop the relevant axes instead of
        # letting the gripper start against a moving mechanism.
        stop_timeout_ms = max(3000.0, estimate_ms * 2.0 + 1000.0)
        stop_deadline = time.monotonic() + stop_timeout_ms / 1000.0
        while time.monotonic() < stop_deadline:
            self._check_cancelled()
            telem = self._telem_getter()
            if telem is not None and not (telem.stepper_busy & motor_mask):
                self._wait(STEPPER_SETTLE_MS)
                return
            self._wait(STEPPER_POLL_MS)

        for motor in range(2):
            if motor_mask & (1 << motor):
                self.stepper.stop(motor)
        raise RuntimeError("stepper stop timeout; gripper action aborted")

    def _stepper_move_and_wait(self, motor: int, direction: int, cm: float,
                               start_d: int = START_DELAY_US,
                               target_d: int = TARGET_DELAY_US,
                               accel: int = ACCEL_STEPS):
        """Launch a stepper move and wait for estimated completion."""
        self._check_cancelled()
        steps = int(cm * STEPS_PER_CM)
        if not self.stepper.move(motor, direction, steps,
                                 start_d, target_d, accel):
            raise RuntimeError("failed to send stepper move command")
        self._wait_steppers_stopped(1 << motor, _est_move_ms(steps))

    def _stepper_dual_and_wait(self,
                               m1: int, cm1: float, dir1: int,
                               m2: int, cm2: float, dir2: int,
                               m2_offset_cm: float = 0.0,
                               start_d: int = START_DELAY_US,
                               target_d: int = TARGET_DELAY_US,
                               accel: int = ACCEL_STEPS):
        """Launch dual-motor move and wait for estimated completion."""
        self._check_cancelled()
        steps1 = int(cm1 * STEPS_PER_CM)
        steps2 = int(cm2 * STEPS_PER_CM)
        offset = int(m2_offset_cm * STEPS_PER_CM)
        if not self.stepper.move_dual(m1, steps1, dir1, m2, steps2, dir2,
                                      offset, start_d, target_d, accel):
            raise RuntimeError("failed to send dual-stepper move command")
        # Wait for the longer motor
        estimate_ms = max(_est_move_ms(steps1),
                          _est_move_ms(steps2) + m2_offset_cm * 60)
        self._wait_steppers_stopped((1 << m1) | (1 << m2), estimate_ms)

    def _stepper_dual2_and_wait(self,
                                m_cont: int, cm_cont: float, dir_cont: int,
                                m_ph: int, cm_ph1: float, dir_ph1: int,
                                cm_ph2: float, dir_ph2: int,
                                ph2_offset_cm: float,
                                start_d: int = START_DELAY_US,
                                target_d: int = TARGET_DELAY_US,
                                accel: int = ACCEL_STEPS):
        """Launch dual-motor move with direction change and wait."""
        self._check_cancelled()
        s_cont = int(cm_cont * STEPS_PER_CM)
        s_ph1 = int(cm_ph1 * STEPS_PER_CM)
        s_ph2 = int(cm_ph2 * STEPS_PER_CM)
        off2 = int(ph2_offset_cm * STEPS_PER_CM)
        if not self.stepper.move_dual2(m_cont, s_cont, dir_cont,
                                       m_ph, s_ph1, dir_ph1,
                                       s_ph2, dir_ph2, off2,
                                       start_d, target_d, accel):
            raise RuntimeError("failed to send phased stepper move command")
        estimate_ms = max(_est_move_ms(s_cont),
                          _est_move_ms(s_ph1 + s_ph2))
        self._wait_steppers_stopped((1 << m_cont) | (1 << m_ph), estimate_ms)

    def _arm_front_smooth_up(self):
        """Smooth arm lift (replicates Build steps 5/11/19)."""
        steps = [50, 28, 14, 6, 2, 0]
        for angle in steps:
            self._check_cancelled()
            self.servo.set_angle(1, angle)  # SERVO_ARM_FRONT
            self._wait(90)
        self._wait(50)

    # ==================== Simple Actions ======================================

    def servo_home(self):
        """Home all servos to idle positions."""
        self._check_cancelled()
        self.servo.set_angle(0, ANGLE_GRIPPER_OPEN)
        self.servo.set_angle(1, ANGLE_ARM_FRONT_DOWN)
        self.servo.set_angle(2, ANGLE_HATCH_A_CLOSED)
        self.servo.set_angle(3, ANGLE_HATCH_B_CLOSED)
        self._wait(300)

    def hatch_open(self):
        """Open both hatches."""
        self._check_cancelled()
        self.servo.set_angle(2, ANGLE_HATCH_A_OPEN)
        self.servo.set_angle(3, ANGLE_HATCH_B_OPEN)
        self._wait(500)

    def hatch_close(self):
        """Close both hatches."""
        self._check_cancelled()
        self.servo.set_angle(2, ANGLE_HATCH_A_CLOSED)
        self.servo.set_angle(3, ANGLE_HATCH_B_CLOSED)
        self._wait(500)

    # ==================== Grap1: 22cm horizontal grab =========================

    def grap1(self):
        """7-step simple pick-and-place (22cm reach)."""
        # 1. Servo init
        self.servo_home()

        # 2. Hatch open
        self.hatch_open()

        # 3. fwd 22cm → after 5cm: down 18cm
        self._stepper_dual_and_wait(
            STEPPER_HORIZ, 22, STEP_DIR_FORWARD,
            STEPPER_VERT,  18, STEP_DIR_REVERSE,
            m2_offset_cm=5)

        self._wait(300)

        # 4. Gripper close
        self.servo.gripper_close()
        self._wait(500)

        # 5. up 18cm → after 5cm: back 22cm
        self._stepper_dual_and_wait(
            STEPPER_VERT,  18, STEP_DIR_FORWARD,
            STEPPER_HORIZ, 22, STEP_DIR_REVERSE,
            m2_offset_cm=5)

        self._wait(300)

        # 6. Hatch partial-close + gripper open
        self.servo.set_angle(2, 67)
        self.servo.set_angle(3, 113)
        self._wait(400)
        self.servo.gripper_open()
        self._wait(500)

        # 7. Final servo init
        self.servo_home()
        self._wait(100)

    # ==================== Grap2: 27cm horizontal grab =========================

    def grap2(self):
        """7-step pick-and-place (27cm reach)."""
        # 1. Servo init
        self.servo_home()

        # 2. Hatch open
        self.hatch_open()

        # 3. fwd 27cm → after 5cm: down 18cm
        self._stepper_dual_and_wait(
            STEPPER_HORIZ, 27, STEP_DIR_FORWARD,
            STEPPER_VERT,  18, STEP_DIR_REVERSE,
            m2_offset_cm=5)

        self._wait(300)

        # 4. Gripper close
        self.servo.gripper_close()
        self._wait(500)

        # 5. up 18cm → after 5cm: back 27cm
        self._stepper_dual_and_wait(
            STEPPER_VERT,  18, STEP_DIR_FORWARD,
            STEPPER_HORIZ, 27, STEP_DIR_REVERSE,
            m2_offset_cm=5)

        self._wait(300)

        # 6. Hatch partial-close + gripper open
        self.servo.set_angle(2, 67)
        self.servo.set_angle(3, 113)
        self._wait(400)
        self.servo.gripper_open()
        self._wait(500)

        # 7. Final servo init
        self.servo_home()
        self._wait(100)

    # ==================== Grap3: 22cm+10cm vertical grab ======================

    def grap3(self):
        """9-step pick-place-and-drop (22cm horizontal, 10cm vertical)."""
        # 1. Servo init
        self.servo_home()

        # 2. Arm lower to 45° + hatch open
        self.servo.set_angle(1, 45)
        self.hatch_open()

        # 3. fwd 22cm → after 5cm: down 10cm
        self._stepper_dual_and_wait(
            STEPPER_HORIZ, 22, STEP_DIR_FORWARD,
            STEPPER_VERT,  10, STEP_DIR_REVERSE,
            m2_offset_cm=5)

        # 4. Gripper close
        self.servo.gripper_close()
        self._wait(500)

        # 5. up 10cm → after 5cm: back 22cm
        self._stepper_dual_and_wait(
            STEPPER_VERT,  10, STEP_DIR_FORWARD,
            STEPPER_HORIZ, 22, STEP_DIR_REVERSE,
            m2_offset_cm=5)

        # then down 11cm
        self._stepper_move_and_wait(STEPPER_VERT, STEP_DIR_REVERSE, 11)

        # 6. Hatch partial-close → gripper open → rise 11cm
        self.servo.set_angle(2, 67)
        self.servo.set_angle(3, 113)
        self._wait(400)
        self.servo.gripper_open()
        self._wait(200)

        self._stepper_move_and_wait(STEPPER_VERT, STEP_DIR_FORWARD, 11)

        # 7. Servo init
        self.servo_home()
        self._wait(100)

    # ==================== Build: 24-step pick-and-place ========================

    def build(self):
        """
        24-step pick-and-place merged sequence.

        Balance check:
          Horz: +5+18-23+23-23+23-23 = 0  ✓
          Vert: -1-18+10-2-3+5-12+21-4+4 = 0  ✓
        """
        # 1. Servo init
        self.servo_home()

        # 2. Hatch open
        self.hatch_open()

        # 3. fwd 5cm ‖ down 1cm
        self._stepper_dual_and_wait(
            STEPPER_HORIZ, 5, STEP_DIR_FORWARD,
            STEPPER_VERT,  1, STEP_DIR_REVERSE)
        self._wait(300)

        # 4. Gripper close
        self.servo.gripper_close()
        self._wait(500)

        # 5. Arm front lift (smooth)
        self._arm_front_smooth_up()

        # 6. fwd 18cm → after 3cm: down 18cm
        self._stepper_dual_and_wait(
            STEPPER_HORIZ, 18, STEP_DIR_FORWARD,
            STEPPER_VERT,  18, STEP_DIR_REVERSE,
            m2_offset_cm=3)
        self._wait(300)

        # 7. Gripper open
        self.servo.gripper_open()
        self._wait(500)

        # 8a. up 10cm → after 3cm: back 23cm
        self._stepper_dual_and_wait(
            STEPPER_VERT,  10, STEP_DIR_FORWARD,
            STEPPER_HORIZ, 23, STEP_DIR_REVERSE,
            m2_offset_cm=3)
        self._wait(300)

        # 8b. down 2cm
        self._stepper_move_and_wait(STEPPER_VERT, STEP_DIR_REVERSE, 2)
        self._wait(300)

        # 9. Arm front lower
        self.servo.arm_front_down()
        self._wait(500)

        # 10. Gripper close
        self.servo.gripper_close()
        self._wait(500)

        # 11. Arm front lift (smooth)
        self._arm_front_smooth_up()

        # 12. up2 ‖ fwd23 → after fwd21: down5 (MoveOverlap2)
        self._stepper_dual2_and_wait(
            STEPPER_HORIZ, 23, STEP_DIR_FORWARD,
            STEPPER_VERT,   2, STEP_DIR_FORWARD,
                           5, STEP_DIR_REVERSE,
            ph2_offset_cm=21)
        self._wait(300)

        # 13. Gripper open
        self.servo.gripper_open()
        self._wait(500)

        # 14. up 5cm → after 1cm: back 23cm
        self._stepper_dual_and_wait(
            STEPPER_VERT,   5, STEP_DIR_FORWARD,
            STEPPER_HORIZ, 23, STEP_DIR_REVERSE,
            m2_offset_cm=1)
        self._wait(300)

        # 15. Arm front lower
        self.servo.arm_front_down()
        self._wait(500)

        # 16. down 12cm
        self._stepper_move_and_wait(STEPPER_VERT, STEP_DIR_REVERSE, 12)
        self._wait(300)

        # 17. Gripper close
        self.servo.gripper_close()
        self._wait(500)

        # 18. Rise 21cm
        self._stepper_move_and_wait(STEPPER_VERT, STEP_DIR_FORWARD, 21)
        self._wait(300)

        # 19. Arm front lift (smooth)
        self._arm_front_smooth_up()

        # 20. fwd 23cm → after 22cm: down 4cm
        self._stepper_dual_and_wait(
            STEPPER_HORIZ, 23, STEP_DIR_FORWARD,
            STEPPER_VERT,   4, STEP_DIR_REVERSE,
            m2_offset_cm=22)
        self._wait(300)

        # 21. Gripper open
        self.servo.gripper_open()
        self._wait(500)

        # 22. up 4cm ‖ back 23cm
        self._stepper_dual_and_wait(
            STEPPER_VERT,   4, STEP_DIR_FORWARD,
            STEPPER_HORIZ, 23, STEP_DIR_REVERSE)
        self._wait(300)

        # 23. Arm front lower
        self.servo.arm_front_down()
        self._wait(500)

        # 24. Final servo init (hatch close)
        self.servo_home()
        self._wait(500)
