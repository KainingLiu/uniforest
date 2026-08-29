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

ANGLE_GRIPPER_OPEN   = 97
ANGLE_GRIPPER_CLOSE  = 97
ANGLE_SUCTION_FLIP_HOME = 97.2
ANGLE_ARM_FRONT_DOWN = 90
ANGLE_ARM_FRONT_UP   = 0
ANGLE_HATCH_A_CLOSED = 63
ANGLE_HATCH_A_OPEN   = 130
ANGLE_HATCH_B_CLOSED = 117
ANGLE_HATCH_B_OPEN   = 50

SERVO_HOME_ANGLES = [97, 90, 63, 117]  # suction flip, arm, hatch_a, hatch_b


class Servo:
    """Servo angle control via STM32 protocol."""

    def __init__(self, transport: Transport):
        self._t = transport

    def set_angle(self, servo_id: int, angle_deg):
        """Set a servo angle; fractional degrees use 0.1° protocol units."""
        self._t.set_servo_angle(servo_id, angle_deg)

    def set_all(self, angles: List[int]):
        """Set all 4 servos at once."""
        self._t.set_servo_all(angles)

    def home_all(self):
        """Home all servos to idle positions."""
        self._t.servo_home_all()

    # ---- Semantic helpers ----

    def gripper_open(self):
        # Release: pump off + valve on for 1 s; A-board returns immediately.
        self._t.suction(2)

    def gripper_close(self):
        # Pickup: pump on, valve forced off.
        self._t.suction(1)

    def suction_on(self):
        """Start the suction pump."""
        self._t.suction(1)

    def suction_release(self):
        """Stop the pump and open the valve asynchronously for one second."""
        self._t.suction(2)

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
