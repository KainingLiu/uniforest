"""Task2 strategy and no-motion preflight."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import time
from typing import TYPE_CHECKING, Optional

from .competition import (
    TaskControl,
    FirstTaskConfig,
    SearchRangeExhausted,
    NORMAL_DISTANCE_MOVE_SPEED_MM_S,
)
from .vision_targets import TASK2_ORANGE, TASK2_PURPLE
from .orange_search import OrangeSearchRecovery
from .common import report_visual_fallback

if TYPE_CHECKING:
    from robot import Robot


class Task2State(Enum):
    STARTUP = auto()
    READY = auto()
    INITIAL_MOVE = auto()
    TURN_TO_COLLECTION = auto()
    TAG_ALIGN = auto()
    POST_TAG_LATERAL = auto()
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
    COUNT_CHECK = auto()
    POST_ORANGE_REVERSE = auto()
    POST_ORANGE_LATERAL = auto()
    FINAL_TURN = auto()
    BUILD_ROUTE = auto()
    FINISHED = auto()
    FAULT = auto()


@dataclass(frozen=True)
class Task2Config(FirstTaskConfig):
    initial_heading_cw_deg: float = 180.0
    initial_distance_mm: float = 2500.0
    initial_speed_mm_s: float = 800.0  # Task2 ramp approach, independent of cruise speed.
    delivery_heading_target_cw_deg: float = -90.0
    delivery_tag_id: int = 3
    # Temporary route: skip Tag3 and its post-alignment lateral move together.
    tag3_alignment_enabled: bool = False
    delivery_tag_distance_mm: float = 250.0
    delivery_tag_distance_tolerance_mm: float = 10.0
    delivery_tag_lateral_tolerance_mm: float = 10.0
    delivery_heading_tolerance_deg: float = 3.0
    delivery_tag_fine_gain_scale: float = 1.5
    # Task2 tags use the wide-angle, uncalibrated tag camera.  Keep the
    # chassis stopped briefly on a missed frame and reacquire a fresh pose.
    delivery_tag_vision_stale_s: float = 0.7
    delivery_tag_lost_timeout_s: float = 2.0
    post_tag_lateral_mm: float = 100.0
    post_tag_lateral_speed_mm_s: float = NORMAL_DISTANCE_MOVE_SPEED_MM_S
    wall_premove_mm: float = 250.0
    wall_premove_speed_mm_s: float = NORMAL_DISTANCE_MOVE_SPEED_MM_S
    purple_min_confidence: float = 25.0
    purple_align_timeout_s: float = 5.0
    purple_search_max_distance_mm: float = 750.0
    align_min_x_mm: float = TASK2_PURPLE.align_min_x_mm
    align_max_x_mm: float = TASK2_PURPLE.align_max_x_mm
    # Task2 purple-cube calibration: stable centered sample measured X=-0.5 mm.
    align_target_x_mm: float = TASK2_PURPLE.target_x_mm
    # Task2 orange-cube calibration: choose the candidate nearest camera
    # center, measured at X=-0.1 mm. Preserve the previous relative window.
    orange_fine_min_x_mm: float = TASK2_ORANGE.fine_min_x_mm
    orange_fine_max_x_mm: float = TASK2_ORANGE.fine_max_x_mm
    post_grab_reverse_mm: float = 100.0
    post_grab_reverse_speed_mm_s: float = NORMAL_DISTANCE_MOVE_SPEED_MM_S
    post_grab_heading_target_cw_deg: float = 0.0
    post_grab_forward_base_mm: float = 350.0
    post_grab_forward_speed_mm_s: float = NORMAL_DISTANCE_MOVE_SPEED_MM_S
    compensation_fast_distance_mm: float = 500.0
    compensation_fast_speed_mm_s: float = 800.0
    left_wall_approach_enabled: bool = True
    orange_target_count: int = 2
    orange_target_count_without_purple: int = 3
    orange_align_min_x_mm: float = TASK2_ORANGE.align_min_x_mm
    orange_align_max_x_mm: float = TASK2_ORANGE.align_max_x_mm
    orange_align_target_x_mm: float = TASK2_ORANGE.target_x_mm
    orange_track_ambiguity_margin_mm: float = 18.0
    post_orange_reverse_mm: float = 100.0
    post_orange_reverse_speed_mm_s: float = NORMAL_DISTANCE_MOVE_SPEED_MM_S
    post_orange_lateral_base_mm: float = 550.0
    post_orange_lateral_speed_mm_s: float = NORMAL_DISTANCE_MOVE_SPEED_MM_S
    final_turn_target_cw_deg: float = 180.0
    build_route_distance_mm: float = 2750.0
    build_route_speed_mm_s: float = 800.0  # Task2 second ramp section.
    # Preserve the existing optional Tag3 profile if that route is re-enabled.
    delivery_tag_fast_forward_mm_s: float = 260.0
    delivery_tag_fast_lateral_mm_s: float = 200.0
    delivery_tag_min_linear_mm_s: float = 100.0
    delivery_tag_slowdown_distance_mm: float = 140.0
    delivery_tag_slowdown_lateral_mm: float = 100.0
    delivery_tag_creep_distance_mm: float = 35.0
    delivery_tag_creep_lateral_mm: float = 25.0


@dataclass(frozen=True)
class Task2_2Config(Task2Config):
    initial_distance_mm: float = 2350.0
    purple_search_max_distance_mm: float = 650.0
    post_grab_forward_base_mm: float = 500.0
    post_tag_lateral_mm: float = 0.0


@dataclass(frozen=True)
class Task2_0Config(Task2Config):
    initial_distance_mm: float = 2900.0
    delivery_heading_target_cw_deg: float = 0.0
    initial_lateral_left_mm: float = 400.0
    initial_lateral_speed_mm_s: float = 400.0
    # Number of new Grap1 operations, not a total carried-count target.
    orange_target_count: int = 3

    def __post_init__(self):
        if type(self.orange_target_count) is not int or not 1 <= self.orange_target_count <= 3:
            raise ValueError('task2-0 orange_target_count must be an integer from 1 to 3')


@dataclass(frozen=True)
class Task2DebugConfig:
    telemetry_wait_s: float = 2.0
    poll_period_s: float = 0.02


class Task2DebugProgram:
    """Report Task2 subsystem readiness without issuing motion commands."""

    TASK_LABEL = 'task2-1-preflight'

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

        print(f'[{self.TASK_LABEL}] Debug preflight only; motion is disabled')
        print(f'[{self.TASK_LABEL}] cube vision: {self.robot.has_vision}')
        print(f'[{self.TASK_LABEL}] field localization: '
              f'{self.robot.has_field_localization}')
        return 0


class Task2Program(TaskControl):
    """Collect purple/orange cubes and stop after the 2750 mm approach (steps 1-14)."""

    TASK_LABEL = 'task2-1'
    TELEMETRY_WAIT_S = 2.0

    def __init__(self, robot: Robot,
                 config: Task2Config = Task2Config(), *, context=None):
        super().__init__(robot, config, context=context)
        self.config = config
        self.state = Task2State.STARTUP
        self._heading_zero_deg: Optional[float] = None
        self._purple_search_exhausted = False

    def _preflight(self):
        self._wait_ready()
        # Entry is now at 180 degrees after Task1/Task4; the current yaw must
        # not become heading zero, or every following absolute turn is reversed.
        self._heading_zero_deg = self._wrap_angle(
            self.robot.telem.yaw_deg + self.config.initial_heading_cw_deg)
        self.state = Task2State.READY
        print(f'[{self.TASK_LABEL}] Entry heading '
              f'{self.config.initial_heading_cw_deg:.0f} deg; '
              f'heading zero={self._heading_zero_deg:+.1f} deg')

    def _search_and_align_purple(self) -> bool:
        cfg = self.config
        set_profile = self._set_cube_profile
        try:
            if set_profile is not None:
                set_profile('task2_purple')
            while True:
                self.state = Task2State.PURPLE_SEARCH
                try:
                    block = self._find_cube(
                        color_name='purple',
                        min_confidence=cfg.purple_min_confidence,
                        search_direction=-1.0,
                        max_distance_mm=cfg.purple_search_max_distance_mm,
                    )
                except SearchRangeExhausted:
                    self._check_active()
                    self._purple_search_exhausted = True
                    print(f'[{self.TASK_LABEL}] Purple cube not found within '
                          f'{cfg.purple_search_max_distance_mm:.0f} mm; '
                          'skipping Grap2')
                    return False

                self.state = Task2State.PURPLE_ALIGN
                if self._align_cube(
                        block, color_name='purple',
                        min_confidence=cfg.purple_min_confidence,
                        timeout_s=cfg.purple_align_timeout_s,
                        timeout_is_success=True):
                    return True
        finally:
            if set_profile is not None:
                set_profile('default')

    def _orange_target_count_for_run(self, purple_grabbed: bool) -> int:
        if purple_grabbed:
            return self.config.orange_target_count
        return self.config.orange_target_count_without_purple

    @staticmethod
    def _lateral_correction_command(target_right_mm: float,
                                    measured_right_mm: float):
        correction_mm = target_right_mm - measured_right_mm
        if correction_mm > 0.0:
            return 'right', correction_mm
        if correction_mm < 0.0:
            return 'left', -correction_mm
        return None, 0.0

    def _compensation_motion_parameters(self, distance_mm, short_speed_mm_s):
        cfg = self.config
        if distance_mm >= cfg.compensation_fast_distance_mm:
            return cfg.compensation_fast_speed_mm_s, cfg.long_distance_forward_accel_ms
        return short_speed_mm_s, cfg.delivery_linear_accel_ms

    def _run_post_purple_route(self, purple_lateral_origin):
        cfg = self.config
        print(f'[{self.TASK_LABEL}] Starting post-purple chassis route')
        purple_lateral_mm = self._measure_lateral_displacement_mm(
            purple_lateral_origin)
        print(f'[{self.TASK_LABEL}] Encoder-measured purple lateral displacement: '
              f'{purple_lateral_mm:+.0f} mm (right positive)')

        self.state = Task2State.POST_GRAB_REVERSE
        print(f'[{self.TASK_LABEL}] Reverse {cfg.post_grab_reverse_mm:.0f} mm')
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
        speed_mm_s, accel_ms = self._compensation_motion_parameters(
            return_distance_mm, cfg.post_grab_forward_speed_mm_s)
        print(f'[{self.TASK_LABEL}] Forward {return_distance_mm:.0f} mm at '
              f'{speed_mm_s:.0f} mm/s '
              f'({cfg.post_grab_forward_base_mm:.0f} - encoder lateral '
              f'{purple_lateral_mm:.0f} mm)')
        self._checked_move(
            'forward', return_distance_mm,
            speed_mm_s, accel_ms=accel_ms)

        self._run_post_return_wall_approach()

    def _try_grab_purple(self, *, chassis_followup=None) -> bool:
        if not self._search_and_align_purple():
            return False

        self.state = Task2State.GRAB
        print(f'[{self.TASK_LABEL}] Purple aligned; running Grap2 with short wall press')
        self._grab_with_wall_press(
            self.robot.actions.grap2, chassis_followup=chassis_followup)
        print(f'[{self.TASK_LABEL}] Grap2 complete')
        return True


    def _run_post_tag3_lateral(self):
        cfg = self.config
        if cfg.post_tag_lateral_mm <= 0.0:
            return
        self.state = Task2State.POST_TAG_LATERAL
        print(f'[{self.TASK_LABEL}] Move right '
              f'{cfg.post_tag_lateral_mm:.0f} mm at '
              f'{cfg.post_tag_lateral_speed_mm_s:.0f} mm/s after Tag3')
        self._checked_move(
            'right', cfg.post_tag_lateral_mm,
            cfg.post_tag_lateral_speed_mm_s)

    def _run_post_return_wall_approach(self):
        cfg = self.config
        if cfg.left_wall_approach_enabled:
            self.state = Task2State.LEFT_WALL_APPROACH
            print(f'[{self.TASK_LABEL}] Left wall approach at '
                  f'{cfg.far_wall_speed_mm_s:.0f} mm/s')
            self._drive_until_wall(
                timeout_s=cfg.far_wall_timeout_s,
                speed_mm_s=cfg.far_wall_speed_mm_s,
                direction='left',
                context='Left wall contact',
            )

        self.state = Task2State.FINAL_WALL_APPROACH
        print(f'[{self.TASK_LABEL}] Forward wall approach at '
              f'{cfg.far_wall_speed_mm_s:.0f} mm/s')
        self._drive_until_wall(
            timeout_s=cfg.far_wall_timeout_s,
            speed_mm_s=cfg.far_wall_speed_mm_s,
            direction='forward',
            context='Forward wall contact',
        )
        self._recalibrate_heading_zero()

    def _grab_task2_orange(self):
        cfg = self.config
        while True:
            self.state = Task2State.ORANGE_SEARCH
            block = self._find_cube(
                color_name='orange', min_confidence=cfg.orange_min_confidence,
                search_direction=1.0,
                lock_x_jump_mm=cfg.orange_search_lock_x_jump_mm,
                ambiguity_margin_mm=cfg.orange_track_ambiguity_margin_mm)
            self.state = Task2State.ORANGE_ALIGN
            if self._align_cube(
                    block, color_name='orange',
                    min_confidence=cfg.orange_min_confidence,
                    align_min_x_mm=cfg.orange_align_min_x_mm,
                    align_max_x_mm=cfg.orange_align_max_x_mm,
                    align_target_x_mm=cfg.orange_align_target_x_mm,
                    ambiguity_margin_mm=cfg.orange_track_ambiguity_margin_mm,
                    timeout_s=cfg.orange_coarse_align_timeout_s,
                    timeout_is_success=True):
                if self._last_alignment_timed_out or self._fine_align_orange(block):
                    break
        self.state = Task2State.ORANGE_GRAB
        print(f'[{self.TASK_LABEL}] Orange aligned; running Grap1 with short wall press')
        self._grab_with_wall_press(
            self.robot.actions.grap1, recalibrate_heading_zero=True)
        self.robot.reset_vision_filter()
        time.sleep(cfg.post_grab_settle_s)

    def _run_collection_route(self):
        cfg = self.config

        self.state = Task2State.INITIAL_MOVE
        print(f'[{self.TASK_LABEL}] Backward {cfg.initial_distance_mm:.0f} mm at '
              f'{cfg.initial_speed_mm_s:.0f} mm/s')
        self._checked_move(
            'backward', cfg.initial_distance_mm, cfg.initial_speed_mm_s,
            accel_ms=cfg.long_distance_forward_accel_ms)

        self.state = Task2State.TURN_TO_COLLECTION
        self._turn_to_heading(cfg.delivery_heading_target_cw_deg)

        if cfg.tag3_alignment_enabled:
            self.state = Task2State.TAG_ALIGN
            self.robot.reset_field_localization_filter()
            self._align_delivery_tag_or_continue(fine_align_enabled=False)
            self._run_post_tag3_lateral()
        else:
            print(f'[{self.TASK_LABEL}] Tag3 alignment and lateral move skipped')

        self.state = Task2State.WALL_PREMOVE
        print(f'[{self.TASK_LABEL}] Forward {cfg.wall_premove_mm:.0f} mm at '
              f'{cfg.wall_premove_speed_mm_s:.0f} mm/s before wall approach')
        self._checked_move(
            'forward', cfg.wall_premove_mm, cfg.wall_premove_speed_mm_s)

        self.state = Task2State.WALL_APPROACH
        print(f'[{self.TASK_LABEL}] Approach wall at '
              f'{cfg.far_wall_speed_mm_s:.0f} mm/s')
        self._drive_until_wall(
            timeout_s=cfg.far_wall_timeout_s,
            speed_mm_s=cfg.far_wall_speed_mm_s,
            context='Wall contact',
        )

        self.robot.reset_vision_filter()
        self._search_position_mm = 0.0
        purple_lateral_origin = self._capture_lateral_origin()

        def post_purple_route():
            self._run_post_purple_route(purple_lateral_origin)

        purple_grabbed = self._try_grab_purple(chassis_followup=post_purple_route)
        if not purple_grabbed:
            post_purple_route()

        set_profile = self._set_cube_profile
        if set_profile is not None:
            set_profile('task2_orange')
        orange_lateral_mm = None
        reverse_done = False

        def start_orange_exit():
            nonlocal orange_lateral_mm, reverse_done
            orange_lateral_mm = self._reverse_after_orange(orange_lateral_origin)
            reverse_done = True

        try:
            self.robot.reset_vision_filter()
            self._search_position_mm = 0.0
            orange_lateral_origin = self._capture_lateral_origin()
            # Reuse the post-wall encoder origin across all orange pickups.
            self._orange_recovery = OrangeSearchRecovery(origin=orange_lateral_origin)
            orange_target_count = self._orange_target_count_for_run(
                purple_grabbed)
            self._collect_orange_with_count_check(
                orange_target_count, self._grab_task2_orange,
                chassis_followup=start_orange_exit)
        finally:
            if set_profile is not None:
                set_profile('default')

        def finish_exit():
            if not reverse_done:
                start_orange_exit()
            self._run_post_orange_route(orange_lateral_mm)

        if ((self._orange_search_exhausted or self._purple_search_exhausted)
                and self.TASK_LABEL in ('task2-1', 'task2-2')):
            self._inspect_during_exit(finish_exit)
        else:
            finish_exit()

    def _reverse_after_orange(self, orange_lateral_origin):
        """Measure collection travel before reversing; also used by count callbacks."""
        cfg = self.config
        orange_lateral_mm = self._measure_lateral_displacement_mm(orange_lateral_origin)
        self.state = Task2State.POST_ORANGE_REVERSE
        print(f'[{self.TASK_LABEL}] Reverse {cfg.post_orange_reverse_mm:.0f} mm')
        self._checked_move(
            'backward', cfg.post_orange_reverse_mm,
            cfg.post_orange_reverse_speed_mm_s)
        return orange_lateral_mm

    def _run_post_orange_route(self, orange_lateral_mm):
        """Shared compensated exit and Task3/Task4 handoff for every Task2 variant."""
        cfg = self.config
        print(f'[{self.TASK_LABEL}] Encoder-measured orange lateral displacement: '
              f'{orange_lateral_mm:+.0f} mm (right positive)')

        lateral_direction, lateral_distance_mm = (
            self._lateral_correction_command(
                cfg.post_orange_lateral_base_mm, orange_lateral_mm))
        self.state = Task2State.POST_ORANGE_LATERAL
        if lateral_direction is None:
            print(f'[{self.TASK_LABEL}] Post-orange lateral correction is zero; '
                  'skipping lateral move')
        else:
            speed_mm_s, accel_ms = self._compensation_motion_parameters(
                lateral_distance_mm, cfg.post_orange_lateral_speed_mm_s)
            print(f'[{self.TASK_LABEL}] Move {lateral_direction} '
                  f'{lateral_distance_mm:.0f} mm at '
                  f'{speed_mm_s:.0f} mm/s '
                  f'(target right {cfg.post_orange_lateral_base_mm:.0f} - '
                  f'encoder right {orange_lateral_mm:.0f} mm)')
            self._checked_move(
                lateral_direction, lateral_distance_mm,
                speed_mm_s, accel_ms=accel_ms)

        self.state = Task2State.FINAL_TURN
        self._turn_to_heading(cfg.final_turn_target_cw_deg)

        self.state = Task2State.BUILD_ROUTE
        print(f'[{self.TASK_LABEL}] Forward {cfg.build_route_distance_mm:.0f} mm at '
              f'{cfg.build_route_speed_mm_s:.0f} mm/s before Build')
        self._checked_move(
            'forward', cfg.build_route_distance_mm,
            cfg.build_route_speed_mm_s,
            accel_ms=cfg.long_distance_forward_accel_ms)

        if self.context is not None:
            self.context.publish_build_approach(self._heading_zero_deg)

    def run(self) -> int:
        try:
            self._preflight()
            self._run_collection_route()
            self.state = Task2State.FINISHED
            return 0
        except Exception:
            self.state = Task2State.FAULT
            self.robot.transport.emergency_stop()
            raise


class Task2_2Program(Task2Program):
    TASK_LABEL = 'task2-2'

    def __init__(self, robot,
                 config: Task2_2Config = Task2_2Config(), *, context=None):
        super().__init__(robot, config, context=context)


class Task2_0Program(Task2Program):
    """Collect the requested orange amount, then follow the common Task2 exit."""

    TASK_LABEL = 'task2-0'

    def __init__(self, robot,
                 config: Task2_0Config = Task2_0Config(), *, context=None):
        super().__init__(robot, config, context=context)
        # These describe completed actions only; they do not assert payload count.
        self.completed_grabs = 0
        self.search_exhausted = False

    def _run_collection_route(self):
        cfg = self.config
        self.completed_grabs = 0
        self.search_exhausted = False
        self.state = Task2State.INITIAL_MOVE
        self._checked_move(
            'backward', cfg.initial_distance_mm, cfg.initial_speed_mm_s,
            accel_ms=cfg.long_distance_forward_accel_ms)
        self.state = Task2State.TURN_TO_COLLECTION
        self._turn_to_heading(cfg.delivery_heading_target_cw_deg)
        self.state = Task2State.WALL_PREMOVE
        self._checked_move(
            'left', cfg.initial_lateral_left_mm, cfg.initial_lateral_speed_mm_s,
            accel_ms=cfg.delivery_linear_accel_ms)
        self._run_post_return_wall_approach()

        set_profile = self._set_cube_profile
        try:
            if set_profile is not None:
                set_profile('task2_orange')
            self.robot.reset_vision_filter()
            self._search_position_mm = 0.0
            self._orange_recovery = OrangeSearchRecovery(
                origin=self._capture_lateral_origin())
            for index in range(cfg.orange_target_count):
                self._check_active()
                print(f'[{self.TASK_LABEL}] Orange pickup '
                      f'{index + 1}/{cfg.orange_target_count}')
                try:
                    self._grab_task2_orange()
                except SearchRangeExhausted:
                    self._check_active()
                    self.search_exhausted = True
                    report_visual_fallback(
                        self.robot, self.TASK_LABEL, 'orange collection',
                        'search budget exhausted; continue the Task2 exit route')
                    break
                self._check_active()
                self.completed_grabs += 1
            print(f'[{self.TASK_LABEL}] Grap1 operations completed: '
                  f'{self.completed_grabs}/{cfg.orange_target_count}')
        finally:
            if set_profile is not None:
                set_profile('default')

        def finish_exit():
            orange_lateral_mm = self._reverse_after_orange(self._orange_recovery.origin)
            self._run_post_orange_route(orange_lateral_mm)

        if self._plan_d_cargo_source() is not None:
            # PlanD preserves the original Task1 round/site through this detour.
            # Always inspect after the last pickup, including a zero/partial
            # exhausted exit. Completed Grap1 operations do not establish cargo.
            self._inspect_during_exit(finish_exit, request_refill=False)
        else:
            finish_exit()


# Old import names remain aliases; canonical variant IDs use -0/-1/-2.
Task2Round2Config = Task2_2Config
Task2Round2Program = Task2_2Program

__all__ = ['Task2Config', 'Task2Program', 'Task2State', 'Task2_2Config',
           'Task2_2Program', 'Task2DebugConfig', 'Task2DebugProgram',
           'Task2_0Config', 'Task2_0Program',
           'Task2Round2Config', 'Task2Round2Program']
