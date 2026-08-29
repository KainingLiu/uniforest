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
SUCTION_RELEASE_SETTLE_MS = 1000
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

    def _stepper_dual3_and_wait(
            self, m_lead: int, cm_lead1: float, dir_lead1: int,
            cm_lead2: float, dir_lead2: int,
            m_other: int, cm_other: float, dir_other: int,
            other_offset_cm: float, lead2_offset_cm: float,
            start_d: int = START_DELAY_US,
            target_d: int = TARGET_DELAY_US,
            accel: int = ACCEL_STEPS):
        """Launch a cross-triggered three-segment move and wait."""
        self._check_cancelled()
        s_lead1 = int(cm_lead1 * STEPS_PER_CM)
        s_lead2 = int(cm_lead2 * STEPS_PER_CM)
        s_other = int(cm_other * STEPS_PER_CM)
        off_other = int(other_offset_cm * STEPS_PER_CM)
        off_lead2 = int(lead2_offset_cm * STEPS_PER_CM)
        if not self.stepper.move_dual3(
                m_lead, s_lead1, dir_lead1,
                s_lead2, dir_lead2,
                m_other, s_other, dir_other,
                off_other, off_lead2,
                start_d, target_d, accel):
            raise RuntimeError(
                "failed to send three-segment stepper move command")
        estimate_ms = max(
            _est_move_ms(s_lead1 + s_lead2),
            _est_move_ms(s_other) + other_offset_cm * 60)
        self._wait_steppers_stopped(
            (1 << m_lead) | (1 << m_other), estimate_ms)

    def _arm_front_smooth_up(self, target: int = 0):
        """Smoothly lift the arm to target (default 0°)."""
        steps = [50, 28, 14, 6]
        if target not in steps:
            steps.append(target)
        for angle in steps:
            self._check_cancelled()
            self.servo.set_angle(1, angle)  # SERVO_ARM_FRONT
            self._wait(90)
        self._wait(50)

    # ==================== Simple Actions ======================================

    def servo_home(self, settle_ms: float = 300):
        """Home all servos to idle positions."""
        self._check_cancelled()
        # CMD_SERVO_HOME keeps the flip servo at the A-board's exact 97.2°
        # home pulse while preserving the other servo home positions.
        self.servo.home_all()
        if settle_ms > 0:
            self._wait(settle_ms)

    def hatch_open(self, settle_ms: float = 500):
        """Open both hatches."""
        self._check_cancelled()
        self.servo.set_angle(2, ANGLE_HATCH_A_OPEN)
        self.servo.set_angle(3, ANGLE_HATCH_B_OPEN)
        if settle_ms > 0:
            self._wait(settle_ms)

    def hatch_close(self):
        """Close both hatches."""
        self._check_cancelled()
        self.servo.set_angle(2, ANGLE_HATCH_A_CLOSED)
        self.servo.set_angle(3, ANGLE_HATCH_B_CLOSED)
        self._wait(500)

    # ==================== Grap1: 22cm horizontal grab =========================

    def grap1(self, test_mode: bool = False):
        """7-step simple pick-and-place (22cm reach)."""
        # Start suction before any positioning motion and keep it on until
        # gripper_open() releases the cube.
        self.servo.gripper_close()

        # 1. Servo init (servo commands are asynchronous)
        self.servo_home(settle_ms=0)

        # 2. Hatch open (servo commands are asynchronous)
        self.hatch_open(settle_ms=0)

        # 3. fwd 22cm → after 5cm: down 18cm
        self._stepper_dual_and_wait(
            STEPPER_HORIZ, 22, STEP_DIR_FORWARD,
            STEPPER_VERT,  18, STEP_DIR_REVERSE,
            m2_offset_cm=5)

        self._wait(300)

        # 4. up 18cm → after 5cm: back 22cm
        self._stepper_dual_and_wait(
            STEPPER_VERT,  18, STEP_DIR_FORWARD,
            STEPPER_HORIZ, 22, STEP_DIR_REVERSE,
            m2_offset_cm=5)

        self._wait(300)

        # 6. Hatch partial-close + gripper open
        self.servo.set_angle(2, 67)
        self.servo.set_angle(3, 113)
        self.servo.gripper_open()

        # 7. Final servo init
        self.servo_home(settle_ms=0)
        if test_mode:
            self._wait(SUCTION_RELEASE_SETTLE_MS)

    # ==================== Grap2: 27cm horizontal grab =========================

    def grap2(self, test_mode: bool = False):
        """7-step pick-and-place (27cm reach)."""
        # Start suction before any positioning motion and keep it on until
        # gripper_open() releases the cube.
        self.servo.gripper_close()

        # 1. Servo init (servo commands are asynchronous)
        self.servo_home(settle_ms=0)

        # 2. Hatch open (servo commands are asynchronous)
        self.hatch_open(settle_ms=0)

        # 3. fwd 27cm → after 10cm: down 18cm
        self._stepper_dual_and_wait(
            STEPPER_HORIZ, 27, STEP_DIR_FORWARD,
            STEPPER_VERT,  18, STEP_DIR_REVERSE,
            m2_offset_cm=10)

        self._wait(300)

        # 4. up 18cm → after 5cm: back 27cm
        self._stepper_dual_and_wait(
            STEPPER_VERT,  18, STEP_DIR_FORWARD,
            STEPPER_HORIZ, 27, STEP_DIR_REVERSE,
            m2_offset_cm=5)

        self._wait(300)

        # 6. Hatch partial-close + gripper open
        self.servo.set_angle(2, 67)
        self.servo.set_angle(3, 113)
        self.servo.gripper_open()

        # 7. Final servo init
        self.servo_home(settle_ms=0)
        if test_mode:
            self._wait(SUCTION_RELEASE_SETTLE_MS)

    # ==================== Grap3: 22cm+10cm vertical grab ======================

    def grap3(self, test_mode: bool = False):
        """Pick-place-and-drop with 27cm reach and 9cm vertical travel."""
        # Start suction before any positioning motion and keep it on until
        # gripper_open() releases the cube.
        self.servo.gripper_close()

        # 1. Servo init (servo commands are asynchronous)
        self.servo_home(settle_ms=0)

        # 2. Lift the front arm to 45° and flip the suction cup to 52.2°.
        self.servo.set_angle(1, 45)
        self.servo.set_angle(0, 52.2)
        self.hatch_open(settle_ms=0)

        # 3. fwd 27cm → after 17cm: down 9cm
        self._stepper_dual_and_wait(
            STEPPER_HORIZ, 27, STEP_DIR_FORWARD,
            STEPPER_VERT,  9, STEP_DIR_REVERSE,
            m2_offset_cm=17)

        # 4. rise 9cm continuously; after 5cm start retracting 22cm;
        #    after retracting 14cm, descend 9cm
        self._stepper_dual3_and_wait(
            STEPPER_VERT, 9, STEP_DIR_FORWARD,
                          9, STEP_DIR_REVERSE,
            STEPPER_HORIZ, 22, STEP_DIR_REVERSE,
            other_offset_cm=5,
            lead2_offset_cm=14)

        # 6. Hatch partial-close → gripper open → rise 11cm
        self.servo.set_angle(2, 67)
        self.servo.set_angle(3, 113)
        self.servo.gripper_open()

        # 7. Rise 9cm while retracting 5cm horizontally.
        self._stepper_dual_and_wait(
            STEPPER_VERT, 9, STEP_DIR_FORWARD,
            STEPPER_HORIZ, 5, STEP_DIR_REVERSE)

        # 7. Servo init
        self.servo_home(settle_ms=0)
        if test_mode:
            self._wait(SUCTION_RELEASE_SETTLE_MS)

    # ==================== Build: 24-step pick-and-place ========================

    def build(self):
        """
        24-step pick-and-place merged sequence.

        Balance check:
          Horz: +4+19-23+23-23+23-23 = 0  ✓
          Vert: -19+10-2-2+5-11.5+20.5-4+4 = +1 cm
        """
        # 1. Servo init
        self.servo_home()

        # 2. Hatch open
        self.hatch_open()
        # Start suction immediately after the hatch opens and keep it on
        # until the first release below.
        self.servo.gripper_close()

        # 3. fwd 4cm only.
        self._stepper_move_and_wait(
            STEPPER_HORIZ, STEP_DIR_FORWARD, 4)

        # 4. Set pickup angles and settle before the lift.
        self.servo.set_angle(1, 100)
        self.servo.set_angle(0, 102.2)
        self._wait(500)
        # Start cup return immediately before the arm's smooth lift so the
        # two servo movements run concurrently on their independent channels.
        self.servo.set_angle(0, 95.2)
        self._arm_front_smooth_up(target=3)

        # 5. fwd 19cm → after 3cm: down 19cm
        self._stepper_dual_and_wait(
            STEPPER_HORIZ, 19, STEP_DIR_FORWARD,
            STEPPER_VERT,  19, STEP_DIR_REVERSE,
            m2_offset_cm=3)
        self._wait(300)

        # 6. Release the first cube.
        self.servo.gripper_open()

        # 7. Rise 10cm continuously; after 3cm start retracting 23cm;
        #    after retracting 20cm, reverse the vertical axis and descend 2cm
        self._stepper_dual3_and_wait(
            STEPPER_VERT, 10, STEP_DIR_FORWARD,
                          2, STEP_DIR_REVERSE,
            STEPPER_HORIZ, 23, STEP_DIR_REVERSE,
            other_offset_cm=3,
            lead2_offset_cm=20)
        self._wait(300)

        # 8. Start suction for the second cube, set arm/cup angles, then settle.
        self.servo.gripper_close()
        self.servo.set_angle(1, 95)
        self.servo.set_angle(0, 102.2)
        self._wait(500)

        # 9. Return the cup to home while smoothly lifting the arm to 0°.
        self.servo.set_angle(0, 97.2)
        self._arm_front_smooth_up()

        # 12. up2 ‖ fwd23 → after fwd21: down5 (MoveOverlap2)
        self._stepper_dual2_and_wait(
            STEPPER_HORIZ, 23, STEP_DIR_FORWARD,
            STEPPER_VERT,   2, STEP_DIR_FORWARD,
                           5, STEP_DIR_REVERSE,
            ph2_offset_cm=21)
        self._wait(300)

        # 13. Release the second cube.
        self.servo.gripper_open()

        # 14. up 5cm → after 1cm: back 23cm
        self._stepper_dual_and_wait(
            STEPPER_VERT,   5, STEP_DIR_FORWARD,
            STEPPER_HORIZ, 23, STEP_DIR_REVERSE,
            m2_offset_cm=1)
        self._wait(300)

        # 15. Set the second-cube pickup angles.
        self.servo.set_angle(1, 95)
        self.servo.set_angle(0, 102.2)

        # 16. Start suction for the third cube, then descend 11.5cm.
        self.servo.gripper_close()
        self._stepper_move_and_wait(STEPPER_VERT, STEP_DIR_REVERSE, 11.5)
        self._wait(300)

        # 18. Rise 20.5cm
        self._stepper_move_and_wait(STEPPER_VERT, STEP_DIR_FORWARD, 20.5)
        self._wait(300)

        # 19. Arm front smooth lift and return the suction cup to 97.2°.
        self._arm_front_smooth_up()
        self.servo.set_angle(0, 97.2)

        # 20. fwd 23cm → after 22cm: down 4cm
        self._stepper_dual_and_wait(
            STEPPER_HORIZ, 23, STEP_DIR_FORWARD,
            STEPPER_VERT,   4, STEP_DIR_REVERSE,
            m2_offset_cm=22)
        self._wait(300)

        # 21. Release the third cube.
        self.servo.gripper_open()

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
