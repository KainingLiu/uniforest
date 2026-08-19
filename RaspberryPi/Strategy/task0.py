"""Full-mission-only initial positioning move."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import time
from typing import TYPE_CHECKING

from control.chassis import (
    LONG_DISTANCE_FORWARD_ACCEL_MS,
    LONG_DISTANCE_MOVE_SPEED_MM_S,
)

if TYPE_CHECKING:
    from robot import Robot


class Task0State(Enum):
    STARTUP = auto()
    READY = auto()
    INITIAL_MOVE = auto()
    FINISHED = auto()
    FAULT = auto()


@dataclass(frozen=True)
class Task0Config:
    distance_mm: float = 1200.0
    speed_mm_s: float = LONG_DISTANCE_MOVE_SPEED_MM_S
    hold_ms: int = 0
    accel_ms: int = LONG_DISTANCE_FORWARD_ACCEL_MS
    telemetry_wait_s: float = 2.0


class Task0Program:
    """Runs the first-round approach shared only by the full mission."""

    def __init__(self, robot: Robot, config: Task0Config = Task0Config()):
        self.robot = robot
        self.config = config
        self.state = Task0State.STARTUP

    def _preflight(self):
        deadline = time.monotonic() + self.config.telemetry_wait_s
        while self.robot.telem is None and time.monotonic() < deadline:
            time.sleep(0.02)
        if self.robot.telem is None:
            raise RuntimeError('A-board telemetry unavailable')
        if not self.robot.has_vision:
            raise RuntimeError('vision subsystem unavailable')
        if not self.robot.has_field_localization:
            raise RuntimeError('field localization subsystem unavailable')
        self.state = Task0State.READY

    def run(self) -> int:
        try:
            self._preflight()
            cfg = self.config
            self.state = Task0State.INITIAL_MOVE
            print(f'[Task0] Forward {cfg.distance_mm:.0f} mm at '
                  f'{cfg.speed_mm_s:.0f} mm/s')
            result = self.robot.move_chassis(
                'forward', cfg.distance_mm, cfg.speed_mm_s,
                hold_ms=cfg.hold_ms, accel_ms=cfg.accel_ms)
            if result.timed_out or result.cancelled:
                raise RuntimeError('Task0 position move did not complete')
            self.state = Task0State.FINISHED
            return 0
        except Exception:
            self.state = Task0State.FAULT
            self.robot.transport.emergency_stop()
            raise


__all__ = ['Task0Config', 'Task0Program', 'Task0State']
