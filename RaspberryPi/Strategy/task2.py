"""Task2 strategy and no-motion preflight."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import time
from typing import TYPE_CHECKING, Optional

from .competition import (
    CompetitionProgram,
    FirstTaskConfig,
    TAG_FOV_RETUNE_SCALE,
)

if TYPE_CHECKING:
    from robot import Robot


class Task2State(Enum):
    STARTUP = auto()
    READY = auto()
    INITIAL_MOVE = auto()
    TURN_LEFT = auto()
    TAG_ALIGN = auto()
    WALL_PREMOVE = auto()
    WALL_APPROACH = auto()
    PURPLE_SEARCH = auto()
    PURPLE_ALIGN = auto()
    GRAB = auto()
    POST_GRAB_REVERSE = auto()
    TURN_RIGHT = auto()
    RETURN_MOVE = auto()
    LEFT_WALL_APPROACH = auto()
    FINAL_WALL_APPROACH = auto()
    ORANGE_SEARCH = auto()
    ORANGE_ALIGN = auto()
    ORANGE_GRAB = auto()
    POST_ORANGE_REVERSE = auto()
    POST_ORANGE_LATERAL = auto()
    TAG4_ALIGN = auto()
    FINAL_TURN = auto()
    BUILD_ROUTE = auto()
    TAG6_ALIGN = auto()
    BUILD = auto()
    FINISHED = auto()
    FAULT = auto()


@dataclass(frozen=True)
class Task2Config(FirstTaskConfig):
    initial_distance_mm: float = 2350.0
    initial_speed_mm_s: float = 500.0
    delivery_heading_target_cw_deg: float = -90.0
    delivery_tag_id: int = 3
    delivery_tag_distance_mm: float = 250.0
    delivery_tag_distance_tolerance_mm: float = 30.0 * TAG_FOV_RETUNE_SCALE
    delivery_tag_lateral_tolerance_mm: float = 25.0 * TAG_FOV_RETUNE_SCALE
    delivery_heading_tolerance_deg: float = 3.0
    delivery_tag_fine_gain_scale: float = 1.5
    wall_premove_mm: float = 250.0
    wall_premove_speed_mm_s: float = 300.0
    wall_speed_mm_s: float = 150.0
    wall_timeout_s: float = 4.0
    purple_min_confidence: float = 25.0
    align_min_x_mm: float = -5.0
    align_max_x_mm: float = 5.0
    align_target_x_mm: float = 0.0
    post_grab_reverse_mm: float = 100.0
    post_grab_reverse_speed_mm_s: float = 300.0
    post_grab_heading_target_cw_deg: float = 0.0
    post_grab_forward_base_mm: float = 400.0
    post_grab_forward_speed_mm_s: float = 400.0
    post_grab_wall_speed_mm_s: float = 200.0
    orange_target_count: int = 2
    orange_align_min_x_mm: float = -20.0
    orange_align_max_x_mm: float = 5.0
    orange_align_target_x_mm: float = 0.0
    post_orange_reverse_mm: float = 500.0
    post_orange_reverse_speed_mm_s: float = 300.0
    post_orange_lateral_base_mm: float = 800.0
    post_orange_lateral_speed_mm_s: float = 300.0
    final_tag_id: int = 4
    final_tag_distance_mm: float = 300.0
    final_tag_heading_target_cw_deg: float = 0.0
    final_turn_target_cw_deg: float = 180.0
    build_route_distance_mm: float = 2100.0
    build_route_speed_mm_s: float = 500.0
    build_tag_id: int = 6
    build_tag_distance_mm: float = FirstTaskConfig().delivery_tag_distance_mm
    build_tag_heading_target_cw_deg: float = 180.0
    build_tag_distance_tolerance_mm: float = (
        FirstTaskConfig().delivery_tag_distance_tolerance_mm)
    build_tag_lateral_tolerance_mm: float = (
        FirstTaskConfig().delivery_tag_lateral_tolerance_mm)
    build_tag_heading_tolerance_deg: float = (
        FirstTaskConfig().delivery_heading_tolerance_deg)
    build_tag_fine_gain_scale: float = (
        FirstTaskConfig().delivery_tag_fine_gain_scale)


@dataclass(frozen=True)
class Task2DebugConfig:
    telemetry_wait_s: float = 2.0
    poll_period_s: float = 0.02


class Task2DebugProgram:
    """Report Task2 subsystem readiness without issuing motion commands."""

    def __init__(self, robot: Robot,
                 config: Task2DebugConfig = Task2DebugConfig()):
        self.robot = robot
        self.config = config

    def run(self) -> int:
        deadline = time.monotonic() + self.config.telemetry_wait_s
        while self.robot.telem is None and time.monotonic() < deadline:
            time.sleep(self.config.poll_period_s)
        if self.robot.telem is None:
            raise RuntimeError('A-board telemetry unavailable')

        print('[Task2] Debug preflight only; motion is disabled')
        print(f'[Task2] cube vision: {self.robot.has_vision}')
        print('[Task2] field localization: '
              f'{self.robot.has_field_localization}')
        return 0


class Task2Program(CompetitionProgram):
    """Run the currently implemented first section of Task2."""

    TASK_LABEL = 'Task2'
    TELEMETRY_WAIT_S = 2.0

    def __init__(self, robot: Robot,
                 config: Task2Config = Task2Config()):
        super().__init__(robot, config)
        self.config = config
        self.state = Task2State.STARTUP
        self._heading_zero_deg: Optional[float] = None

    def _preflight(self):
        deadline = time.monotonic() + self.TELEMETRY_WAIT_S
        while self.robot.telem is None and time.monotonic() < deadline:
            time.sleep(0.02)
        if self.robot.telem is None:
            raise RuntimeError('A-board telemetry unavailable')
        if not self.robot.has_field_localization:
            raise RuntimeError('field localization subsystem unavailable')
        if not self.robot.has_vision:
            raise RuntimeError('cube vision subsystem unavailable')

        self._heading_zero_deg = self.robot.telem.yaw_deg
        self.state = Task2State.READY
        print('[Task2] Preflight complete; heading zero='
              f'{self._heading_zero_deg:+.1f} deg')

    def _run_partial_task(self):
        cfg = self.config

        self.state = Task2State.INITIAL_MOVE
        print(f'[Task2] Forward {cfg.initial_distance_mm:.0f} mm at '
              f'{cfg.initial_speed_mm_s:.0f} mm/s')
        self._checked_move(
            'forward', cfg.initial_distance_mm, cfg.initial_speed_mm_s)

        self.state = Task2State.TURN_LEFT
        self._turn_to_heading(cfg.delivery_heading_target_cw_deg)

        self.state = Task2State.TAG_ALIGN
        self.robot.reset_field_localization_filter()
        self._align_delivery_tag()

        self.state = Task2State.WALL_PREMOVE
        print(f'[Task2] Forward {cfg.wall_premove_mm:.0f} mm at '
              f'{cfg.wall_premove_speed_mm_s:.0f} mm/s before wall approach')
        self._checked_move(
            'forward', cfg.wall_premove_mm, cfg.wall_premove_speed_mm_s)

        self.state = Task2State.WALL_APPROACH
        print(f'[Task2] Approach wall at {cfg.wall_speed_mm_s:.0f} mm/s')
        self._drive_until_wall(
            timeout_s=cfg.wall_timeout_s,
            speed_mm_s=cfg.wall_speed_mm_s,
            context='Wall contact',
        )

        self.robot.reset_vision_filter()
        self._search_position_mm = 0.0
        purple_lateral_origin = self._capture_lateral_origin()
        while True:
            self.state = Task2State.PURPLE_SEARCH
            block = self._find_cube(
                color_name='purple',
                min_confidence=cfg.purple_min_confidence,
                search_direction=-1.0,
            )
            self.state = Task2State.PURPLE_ALIGN
            if self._align_cube(
                    block, color_name='purple',
                    min_confidence=cfg.purple_min_confidence):
                break

        self.state = Task2State.WALL_APPROACH
        self._press_wall_before_grab()

        self.state = Task2State.GRAB
        print('[Task2] Purple aligned; running Grap2')
        self.robot.actions.grap2()
        print('[Task2] Grap2 complete')

        purple_lateral_mm = self._measure_lateral_displacement_mm(
            purple_lateral_origin)
        print('[Task2] Encoder-measured purple lateral displacement: '
              f'{purple_lateral_mm:+.0f} mm (right positive)')

        self.state = Task2State.POST_GRAB_REVERSE
        print(f'[Task2] Reverse {cfg.post_grab_reverse_mm:.0f} mm')
        self._checked_move(
            'backward', cfg.post_grab_reverse_mm,
            cfg.post_grab_reverse_speed_mm_s)

        self.state = Task2State.TURN_RIGHT
        self._turn_to_heading(cfg.post_grab_heading_target_cw_deg)

        return_distance_mm = (
            cfg.post_grab_forward_base_mm - purple_lateral_mm)
        if return_distance_mm <= 0.0:
            raise RuntimeError(
                'post-purple forward distance is not positive: '
                f'{cfg.post_grab_forward_base_mm:.0f} - '
                f'({purple_lateral_mm:.0f}) = '
                f'{return_distance_mm:.0f} mm')
        self.state = Task2State.RETURN_MOVE
        print(f'[Task2] Forward {return_distance_mm:.0f} mm at '
              f'{cfg.post_grab_forward_speed_mm_s:.0f} mm/s '
              f'({cfg.post_grab_forward_base_mm:.0f} - encoder lateral '
              f'{purple_lateral_mm:.0f} mm)')
        self._checked_move(
            'forward', return_distance_mm,
            cfg.post_grab_forward_speed_mm_s)

        self.state = Task2State.LEFT_WALL_APPROACH
        print(f'[Task2] Left wall approach at '
              f'{cfg.post_grab_wall_speed_mm_s:.0f} mm/s')
        self._drive_until_wall(
            timeout_s=cfg.wall_timeout_s,
            speed_mm_s=cfg.post_grab_wall_speed_mm_s,
            direction='left',
            context='Left wall contact',
        )

        self.state = Task2State.FINAL_WALL_APPROACH
        print(f'[Task2] Forward wall approach at '
              f'{cfg.post_grab_wall_speed_mm_s:.0f} mm/s')
        self._drive_until_wall(
            timeout_s=cfg.wall_timeout_s,
            speed_mm_s=cfg.post_grab_wall_speed_mm_s,
            direction='forward',
            context='Forward wall contact',
        )
        self._recalibrate_heading_zero()

        self.robot.reset_vision_filter()
        self._search_position_mm = 0.0
        orange_lateral_origin = self._capture_lateral_origin()
        for cube_index in range(1, cfg.orange_target_count + 1):
            print(f'[Task2] Orange cube {cube_index}/'
                  f'{cfg.orange_target_count}')
            while True:
                self.state = Task2State.ORANGE_SEARCH
                block = self._find_cube(
                    color_name='orange',
                    min_confidence=cfg.orange_min_confidence,
                    search_direction=1.0,
                )
                self.state = Task2State.ORANGE_ALIGN
                if self._align_cube(
                        block, color_name='orange',
                        min_confidence=cfg.orange_min_confidence,
                        align_min_x_mm=cfg.orange_align_min_x_mm,
                        align_max_x_mm=cfg.orange_align_max_x_mm,
                        align_target_x_mm=cfg.orange_align_target_x_mm):
                    break

            self.state = Task2State.WALL_APPROACH
            self._press_wall_before_grab(recalibrate_heading_zero=True)

            self.state = Task2State.ORANGE_GRAB
            print(f'[Task2] Orange {cube_index}/'
                  f'{cfg.orange_target_count} aligned; running Grap1')
            self.robot.actions.grap1()
            print(f'[Task2] Grap1 {cube_index}/'
                  f'{cfg.orange_target_count} complete')
            self.robot.reset_vision_filter()
            time.sleep(cfg.post_grab_settle_s)

        orange_lateral_mm = self._measure_lateral_displacement_mm(
            orange_lateral_origin)
        print('[Task2] Encoder-measured orange lateral displacement: '
              f'{orange_lateral_mm:+.0f} mm (right positive)')

        self.state = Task2State.POST_ORANGE_REVERSE
        print(f'[Task2] Reverse {cfg.post_orange_reverse_mm:.0f} mm')
        self._checked_move(
            'backward', cfg.post_orange_reverse_mm,
            cfg.post_orange_reverse_speed_mm_s)

        lateral_distance_mm = (
            cfg.post_orange_lateral_base_mm - orange_lateral_mm)
        if lateral_distance_mm <= 0.0:
            raise RuntimeError(
                'post-orange lateral distance is not positive: '
                f'{cfg.post_orange_lateral_base_mm:.0f} - '
                f'{orange_lateral_mm:.0f} = '
                f'{lateral_distance_mm:.0f} mm')
        self.state = Task2State.POST_ORANGE_LATERAL
        print(f'[Task2] Move right {lateral_distance_mm:.0f} mm at '
              f'{cfg.post_orange_lateral_speed_mm_s:.0f} mm/s '
              f'({cfg.post_orange_lateral_base_mm:.0f} - encoder lateral '
              f'{orange_lateral_mm:.0f} mm)')
        self._checked_move(
            'right', lateral_distance_mm,
            cfg.post_orange_lateral_speed_mm_s)

        self.state = Task2State.TAG4_ALIGN
        self.robot.reset_field_localization_filter()
        self._align_delivery_tag(
            tag_id=cfg.final_tag_id,
            target_distance_mm=cfg.final_tag_distance_mm,
            heading_target_cw_deg=cfg.final_tag_heading_target_cw_deg,
        )

        self.state = Task2State.FINAL_TURN
        self._turn_to_heading(cfg.final_turn_target_cw_deg)

        self.state = Task2State.BUILD_ROUTE
        print(f'[Task2] Forward {cfg.build_route_distance_mm:.0f} mm at '
              f'{cfg.build_route_speed_mm_s:.0f} mm/s before Build')
        self._checked_move(
            'forward', cfg.build_route_distance_mm,
            cfg.build_route_speed_mm_s)

        self.state = Task2State.TAG6_ALIGN
        self.robot.reset_field_localization_filter()
        self._align_delivery_tag(
            tag_id=cfg.build_tag_id,
            target_distance_mm=cfg.build_tag_distance_mm,
            heading_target_cw_deg=cfg.build_tag_heading_target_cw_deg,
            distance_tolerance_mm=cfg.build_tag_distance_tolerance_mm,
            lateral_tolerance_mm=cfg.build_tag_lateral_tolerance_mm,
            heading_tolerance_deg=cfg.build_tag_heading_tolerance_deg,
            fine_gain_scale=cfg.build_tag_fine_gain_scale,
        )

        self.state = Task2State.BUILD
        print('[Task2] Tag 6 aligned; running Build')
        self.robot.actions.build()
        print('[Task2] Build complete')

    def run(self) -> int:
        try:
            self._preflight()
            self._run_partial_task()
            self.state = Task2State.FINISHED
            return 0
        except Exception:
            self.state = Task2State.FAULT
            self.robot.transport.emergency_stop()
            raise


__all__ = [
    'Task2Config',
    'Task2DebugConfig',
    'Task2DebugProgram',
    'Task2Program',
    'Task2State',
]
