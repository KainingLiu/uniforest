"""
Servo control via protocol commands.

Maps to servo.h — 4×PWM servos on TIM2/TIM5.
"""

import time
from typing import List

from protocol.transport import Transport
from protocol.commands import (
    SERVO_GRIPPER, SERVO_ARM_FRONT, SERVO_HATCH_A, SERVO_HATCH_B,
)

# ===================== Semantic Angle Aliases =================================

ANGLE_GRIPPER_OPEN   = 85
ANGLE_GRIPPER_CLOSE  = 102
ANGLE_ARM_FRONT_DOWN = 90
ANGLE_ARM_FRONT_UP   = 0
ANGLE_HATCH_A_CLOSED = 63
ANGLE_HATCH_A_OPEN   = 130
ANGLE_HATCH_B_CLOSED = 117
ANGLE_HATCH_B_OPEN   = 50

SERVO_HOME_ANGLES = [85, 90, 63, 117]  # gripper, arm, hatch_a, hatch_b


class Servo:
    """Servo angle control via STM32 protocol."""

    def __init__(self, transport: Transport):
        self._t = transport

    def set_angle(self, servo_id: int, angle_deg: int):
        """Set a single servo to angle_deg (0–180)."""
        self._t.set_servo_angle(servo_id, angle_deg)

    def set_all(self, angles: List[int]):
        """Set all 4 servos at once."""
        self._t.set_servo_all(angles)

    def home_all(self):
        """Home all servos to idle positions."""
        self._t.servo_home_all()

    # ---- Semantic helpers ----

    def gripper_open(self):
        self.set_angle(SERVO_GRIPPER, ANGLE_GRIPPER_OPEN)

    def gripper_close(self):
        self.set_angle(SERVO_GRIPPER, ANGLE_GRIPPER_CLOSE)

    def arm_front_up(self, target: int = 0):
        self.set_angle(SERVO_ARM_FRONT, target)

    def arm_front_down(self):
        self.set_angle(SERVO_ARM_FRONT, ANGLE_ARM_FRONT_DOWN)

    def arm_front_smooth_up(self, steps=None):
        """Smooth deceleration lift (replicates Build() step 5)."""
        if steps is None:
            steps = [50, 28, 14, 6, 2, 0]
        for angle in steps:
            self.set_angle(SERVO_ARM_FRONT, angle)
            time.sleep(0.09)

    def hatch_a_open(self):
        self.set_angle(SERVO_HATCH_A, ANGLE_HATCH_A_OPEN)

    def hatch_a_close(self):
        self.set_angle(SERVO_HATCH_A, ANGLE_HATCH_A_CLOSED)

    def hatch_b_open(self):
        self.set_angle(SERVO_HATCH_B, ANGLE_HATCH_B_OPEN)

    def hatch_b_close(self):
        self.set_angle(SERVO_HATCH_B, ANGLE_HATCH_B_CLOSED)

    def hatch_open(self):
        """Open both hatches (S3→130°, S4→50°)."""
        self.hatch_a_open()
        self.hatch_b_open()

    def hatch_close(self):
        """Close both hatches (S3→63°, S4→117°)."""
        self.hatch_a_close()
        self.hatch_b_close()
