"""
Stepper motor control via protocol commands.

Maps to stepper.h — 2×TB6600 steppers, non-blocking via STM32 TIM7 ISR.
"""

import time
from typing import Optional

from protocol.transport import Transport
from protocol.commands import (
    STEPPER_HORIZ, STEPPER_VERT,
    STEP_DIR_FORWARD, STEP_DIR_REVERSE,
)

# Steps per cm (400 steps/rev ÷ 10 mm/rev = 400 steps/cm)
STEPS_PER_CM = 400

# Default trapezoidal parameters (µs)
# STM32 applies a small 6/5 half-cycle safety scale, giving approximately
# 417 Hz at start and 4167 Hz at cruise while preserving these wire values.
DEFAULT_START_DELAY  = 1000
DEFAULT_TARGET_DELAY = 100
DEFAULT_ACCEL_STEPS  = 400


def cm_to_steps(cm: float) -> int:
    """Convert centimeters to stepper steps."""
    return int(cm * STEPS_PER_CM)


class Stepper:
    """Stepper motor control via STM32 protocol (non-blocking)."""

    def __init__(self, transport: Transport):
        self._t = transport

    # ---- Low-Level ----

    def move(self, motor: int, direction: int, steps: int,
             start_delay: int = 0, target_delay: int = 0,
             accel_steps: int = 0) -> bool:
        """
        Launch a single-motor trapezoidal move (non-blocking).

        Args:
            motor: STEPPER_HORIZ or STEPPER_VERT
            direction: STEP_DIR_FORWARD or STEP_DIR_REVERSE
            steps: total step count
            start_delay: legacy half-cycle parameter (0=default 1000)
            target_delay: legacy half-cycle parameter (0=default 100)
            accel_steps: ramp length (0=default 400)

        Returns immediately. Use is_busy() to poll completion.
        """
        return self._t.stepper_move(motor, direction, steps,
                                    start_delay, target_delay, accel_steps)

    def stop(self, motor: int):
        """Emergency-stop a stepper motor."""
        self._t.stepper_stop(motor)

    def set_params(self, start_delay: int, target_delay: int,
                   accel_steps: int):
        """Set default trapezoidal speed parameters."""
        self._t.stepper_set_params(start_delay, target_delay, accel_steps)

    def move_dual(self,
                  m1: int, steps1: int, dir1: int,
                  m2: int, steps2: int, dir2: int,
                  m2_offset: int = 0,
                  start_delay: int = None, target_delay: int = None,
                  accel_steps: int = None) -> bool:
        """Launch dual-motor overlapping move (non-blocking)."""
        if start_delay is None:
            start_delay = DEFAULT_START_DELAY
        if target_delay is None:
            target_delay = DEFAULT_TARGET_DELAY
        if accel_steps is None:
            accel_steps = DEFAULT_ACCEL_STEPS
        return self._t.stepper_move_dual(
            m1, steps1, dir1, m2, steps2, dir2,
            m2_offset, start_delay, target_delay, accel_steps)

    def move_dual2(self,
                   m_cont: int, steps_cont: int, dir_cont: int,
                   m_ph: int, steps_ph1: int, dir_ph1: int,
                   steps_ph2: int, dir_ph2: int, ph2_offset: int,
                   start_delay: int = None, target_delay: int = None,
                   accel_steps: int = None) -> bool:
        """Launch dual-motor move with mid-move direction change (non-blocking)."""
        if start_delay is None:
            start_delay = DEFAULT_START_DELAY
        if target_delay is None:
            target_delay = DEFAULT_TARGET_DELAY
        if accel_steps is None:
            accel_steps = DEFAULT_ACCEL_STEPS
        return self._t.stepper_move_dual2(
            m_cont, steps_cont, dir_cont,
            m_ph, steps_ph1, dir_ph1,
            steps_ph2, dir_ph2, ph2_offset,
            start_delay, target_delay, accel_steps)

    def set_position(self, motor: int, pos: int):
        """Set stepper cumulative position."""
        self._t.stepper_set_position(motor, pos)

    # ---- Status (from telemetry) ----

    def is_busy(self, motor: int, telem=None) -> bool:
        """Check if stepper is moving. Requires telem batch."""
        if telem is None:
            return False
        mask = 1 << motor
        return bool(telem.stepper_busy & mask)

    def get_position(self, motor: int, telem=None) -> int:
        """Get stepper cumulative position. Requires telem batch."""
        if telem is None or motor >= 2:
            return 0
        return telem.stepper_pos[motor]

    # ---- High-Level (centimeter-based) ----

    def move_cm(self, motor: int, direction: int, cm: float,
                start_delay: int = 0, target_delay: int = 0,
                accel_steps: int = 0):
        """Move by centimeter distance (non-blocking)."""
        return self.move(motor, direction, cm_to_steps(cm),
                        start_delay, target_delay, accel_steps)

    def wait_done(self, motor: int, telem_getter, poll_ms: int = 10,
                  timeout_ms: int = 30000):
        """Block until stepper is idle. Requires telem callback."""
        t0 = time.time()
        while True:
            telem = telem_getter()
            if not self.is_busy(motor, telem):
                return True
            if (time.time() - t0) * 1000.0 > timeout_ms:
                return False
            time.sleep(poll_ms / 1000.0)
