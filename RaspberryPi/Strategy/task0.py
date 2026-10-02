"""Reusable positioning tasks for strategy starts and transitions."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import time
from typing import TYPE_CHECKING

from control.chassis import (
    LONG_DISTANCE_FORWARD_ACCEL_MS,
    LONG_DISTANCE_MOVE_SPEED_MM_S,
)
from .common import TaskStateReporting, report_visual_fallback, wrap_angle
from .competition import FirstTaskConfig, TaskControl

if TYPE_CHECKING:
    from robot import Robot


class Task0State(Enum):
    STARTUP = auto()
    READY = auto()
    INITIAL_MOVE = auto()
    LATERAL_MOVE = auto()
    FINAL_TURN = auto()
    TURN_TO_START = auto()
    LEFT_WALL_APPROACH = auto()
    FINISHED = auto()
    FAULT = auto()


@dataclass(frozen=True)
class Task0Config:
    distance_mm: float = 1150.0
    speed_mm_s: float = LONG_DISTANCE_MOVE_SPEED_MM_S
    hold_ms: int = 0
    accel_ms: int = LONG_DISTANCE_FORWARD_ACCEL_MS
    telemetry_wait_s: float = 2.0


Task0_1Config = Task0Config


@dataclass(frozen=True)
class Task0_2Config(Task0Config):
    distance_mm: float = 900.0
    lateral_right_mm: float = 2700.0
    final_heading_cw_deg: float = 180.0
    turn_speed_deg_s: float = 120.0


@dataclass(frozen=True)
class Task0_3Config(FirstTaskConfig):
    initial_heading_cw_deg: float = 180.0
    target_heading_cw_deg: float = 0.0
    lateral_left_mm: float = 2600.0
    lateral_speed_mm_s: float = LONG_DISTANCE_MOVE_SPEED_MM_S
    far_wall_speed_mm_s: float = 300.0


class Task0Program(TaskStateReporting):
    """Run the initial approach, independently or at the beginning of a plan."""

    TASK_LABEL = 'task0'

    def __init__(self, robot: Robot, config: Task0Config = Task0Config(), *, context=None):
        self.robot = robot
        self.context = context
        self.config = config
        self.state = Task0State.STARTUP

    def _preflight(self):
        deadline = time.monotonic() + self.config.telemetry_wait_s
        while self.robot.telem is None and time.monotonic() < deadline:
            time.sleep(0.02)
        if self.robot.telem is None:
            raise RuntimeError('A-board telemetry unavailable')
        if not self.robot.has_vision:
            report_visual_fallback(self.robot, self.TASK_LABEL, 'startup',
                                   'cube camera unavailable')
        if not self.robot.has_field_localization:
            report_visual_fallback(self.robot, self.TASK_LABEL, 'startup',
                                   'tag camera unavailable')
        if self.context is not None:
            self.context.check_active()
        self.state = Task0State.READY

    def _move(self, direction, distance_mm):
        cfg = self.config
        if self.context is not None:
            self.context.check_active()
        print(f'[{self.TASK_LABEL}] {direction} {distance_mm:.0f} mm at '
              f'{cfg.speed_mm_s:.0f} mm/s')
        result = self.robot.move_chassis(
            direction, distance_mm, cfg.speed_mm_s,
            hold_ms=cfg.hold_ms, accel_ms=cfg.accel_ms, route_mode=True)
        if result.timed_out or result.cancelled:
            raise RuntimeError(f'{self.TASK_LABEL} position move did not complete')
        if self.context is not None:
            self.context.check_active()

    def _run_route(self):
        self.state = Task0State.INITIAL_MOVE
        self._move('forward', self.config.distance_mm)

    def run(self) -> int:
        try:
            self._preflight()
            self._run_route()
            self.state = Task0State.FINISHED
            return 0
        except Exception:
            self.state = Task0State.FAULT
            self.robot.transport.emergency_stop()
            raise


class Task0_1Program(Task0Program):
    TASK_LABEL = 'task0-1'


class Task0_2Program(Task0Program):
    TASK_LABEL = 'task0-2'

    def __init__(self, robot, config: Task0_2Config = Task0_2Config(), *, context=None):
        super().__init__(robot, config, context=context)

    def _run_route(self):
        heading_zero_deg = self.robot.telem.yaw_deg
        super()._run_route()
        self.state = Task0State.LATERAL_MOVE
        self._move('right', self.config.lateral_right_mm)
        self.state = Task0State.FINAL_TURN
        # Same gyro convention as TaskControl: yaw is CCW-positive, task
        # headings are CW-positive. Correct to the startup frame after travel.
        telem = self.robot.telem
        if telem is None:
            raise RuntimeError('telemetry unavailable before Task0-2 final turn')
        target_yaw = wrap_angle(heading_zero_deg - self.config.final_heading_cw_deg)
        clockwise_delta_deg = -wrap_angle(target_yaw - telem.yaw_deg)
        print(f'[{self.TASK_LABEL}] Turn to {self.config.final_heading_cw_deg:.0f} deg '
              f'from startup (correction {clockwise_delta_deg:+.1f} deg CW)')
        self.robot.chassis.turn(clockwise_delta_deg, self.config.turn_speed_deg_s,
                                hold_ms=0, settle_cycles=1)
        if self.context is not None:
            self.context.check_active()


class Task0_3Program(TaskControl):
    """Enter at 180 degrees, then return to the Task1 start at zero degrees."""

    TASK_LABEL = 'task0-3'

    def __init__(self, robot, config: Task0_3Config = Task0_3Config(), *, context=None):
        super().__init__(robot, config, context=context)
        self.state = Task0State.STARTUP

    def _preflight(self):
        self._wait_ready()
        self._heading_zero_deg = wrap_angle(
            self.robot.telem.yaw_deg + self.config.initial_heading_cw_deg)
        self.state = Task0State.READY

    def run(self) -> int:
        try:
            self._preflight()
            self.state = Task0State.TURN_TO_START
            self._turn_to_heading(self.config.target_heading_cw_deg)
            self.state = Task0State.LATERAL_MOVE
            self._checked_move('left', self.config.lateral_left_mm,
                               self.config.lateral_speed_mm_s)
            self._check_active()
            self.state = Task0State.LEFT_WALL_APPROACH
            self._drive_until_wall(
                timeout_s=self.config.far_wall_timeout_s,
                speed_mm_s=self.config.far_wall_speed_mm_s,
                direction='left', context='Task0-3 left wall approach')
            self._check_active()
            self.state = Task0State.FINISHED
            return 0
        except Exception:
            self.state = Task0State.FAULT
            self.robot.transport.emergency_stop()
            raise


__all__ = ['Task0Config', 'Task0Program', 'Task0State', 'Task0_1Config',
           'Task0_1Program', 'Task0_2Config', 'Task0_2Program',
           'Task0_3Config', 'Task0_3Program']
