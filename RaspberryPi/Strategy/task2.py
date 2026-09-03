"""Task2 strategy and no-motion preflight."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum, auto
from statistics import median
import time
import math
from typing import TYPE_CHECKING, Optional

from .competition import (
    _Pid,
    CompetitionProgram,
    FirstTaskConfig,
    LONG_DISTANCE_MOVE_SPEED_MM_S,
    TAG_FOV_RETUNE_SCALE,
)
from .vision_targets import TASK2_ORANGE, TASK2_PURPLE

if TYPE_CHECKING:
    from robot import Robot


class Task2State(Enum):
    STARTUP = auto()
    READY = auto()
    INITIAL_MOVE = auto()
    TURN_LEFT = auto()
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
    POST_ORANGE_REVERSE = auto()
    POST_ORANGE_LATERAL = auto()
    FINAL_TURN = auto()
    BUILD_ROUTE = auto()
    TAG6_ALIGN = auto()
    POST_TAG6_LATERAL = auto()
    BUILDING_ALIGN = auto()
    BUILD = auto()
    POST_BUILD_REVERSE = auto()
    POST_BUILD_TURN = auto()
    POST_BUILD_ROUTE = auto()
    TAG1_ALIGN = auto()
    FINAL_RIGHT_TURN = auto()
    FINISHED = auto()
    FAULT = auto()


@dataclass(frozen=True)
class Task2Config(FirstTaskConfig):
    initial_distance_mm: float = 2350.0
    initial_speed_mm_s: float = LONG_DISTANCE_MOVE_SPEED_MM_S
    delivery_heading_target_cw_deg: float = -90.0
    delivery_tag_id: int = 3
    delivery_tag_distance_mm: float = 250.0
    delivery_tag_distance_tolerance_mm: float = 30.0 * TAG_FOV_RETUNE_SCALE
    delivery_tag_lateral_tolerance_mm: float = 25.0 * TAG_FOV_RETUNE_SCALE
    delivery_heading_tolerance_deg: float = 3.0
    delivery_tag_fine_gain_scale: float = 1.5
    # Task2 tags use the wide-angle, uncalibrated tag camera.  Keep the
    # chassis stopped briefly on a missed frame and reacquire a fresh pose.
    delivery_tag_vision_stale_s: float = 0.7
    delivery_tag_lost_timeout_s: float = 2.0
    post_tag_lateral_mm: float = 100.0
    post_tag_lateral_speed_mm_s: float = 300.0
    wall_premove_mm: float = 250.0
    wall_premove_speed_mm_s: float = 300.0
    purple_min_confidence: float = 25.0
    purple_search_max_distance_mm: float = 600.0
    align_min_x_mm: float = TASK2_PURPLE.align_min_x_mm
    align_max_x_mm: float = TASK2_PURPLE.align_max_x_mm
    # Task2 purple-cube calibration: stable centered sample measured X=-0.5 mm.
    align_target_x_mm: float = TASK2_PURPLE.target_x_mm
    # Task2 orange-cube calibration: choose the candidate nearest camera
    # center, measured at X=-0.1 mm. Preserve the previous relative window.
    orange_fine_min_x_mm: float = TASK2_ORANGE.fine_min_x_mm
    orange_fine_max_x_mm: float = TASK2_ORANGE.fine_max_x_mm
    post_grab_reverse_mm: float = 100.0
    post_grab_reverse_speed_mm_s: float = 300.0
    post_grab_heading_target_cw_deg: float = 0.0
    post_grab_forward_base_mm: float = 400.0
    post_grab_forward_speed_mm_s: float = 400.0
    left_wall_approach_enabled: bool = True
    orange_target_count: int = 2
    orange_target_count_without_purple: int = 3
    orange_align_min_x_mm: float = TASK2_ORANGE.align_min_x_mm
    orange_align_max_x_mm: float = TASK2_ORANGE.align_max_x_mm
    orange_align_target_x_mm: float = TASK2_ORANGE.target_x_mm
    orange_track_ambiguity_margin_mm: float = 18.0
    post_orange_reverse_mm: float = 500.0
    post_orange_reverse_speed_mm_s: float = 300.0
    post_orange_lateral_base_mm: float = 700.0
    post_orange_lateral_speed_mm_s: float = 300.0
    final_turn_target_cw_deg: float = 180.0
    build_route_distance_mm: float = 2100.0
    build_route_speed_mm_s: float = LONG_DISTANCE_MOVE_SPEED_MM_S
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
    build_tag_vision_stale_s: float = 0.7
    build_tag_lost_timeout_s: float = 2.0
    # Tag6 is approached after a long straight run. Slow the far-field
    # profile and enter deceleration earlier without changing final tolerances.
    delivery_tag_fast_forward_mm_s: float = 260.0
    delivery_tag_fast_lateral_mm_s: float = 200.0
    delivery_tag_slowdown_distance_mm: float = 140.0
    delivery_tag_slowdown_lateral_mm: float = 100.0
    delivery_tag_creep_distance_mm: float = 35.0
    delivery_tag_creep_lateral_mm: float = 25.0
    post_tag6_lateral_right_mm: float = 100.0
    post_tag6_lateral_speed_mm_s: float = 300.0
    building_target_x_mm: float = 0.0
    # Calibrated from the current camera view at the confirmed build position.
    building_target_z_mm: float = 75.0
    # Current confirmed build pose: top-edge vertical pixel coordinate maps to
    # the desired Z distance.  A lower image edge means a nearer building.
    building_top_reference_v_px: float = 82.4
    building_top_camera_cy_px: float = 240.0
    # Building contours vary with occlusion and camera pitch. Keep the
    # geometric gate permissive; position and multi-frame confirmation still
    # reject isolated orange cube candidates.
    building_min_confidence: float = 35.0
    building_min_height_width_ratio: float = 0.35
    building_max_height_width_ratio: float = 2.20
    building_x_tolerance_mm: float = 3.0
    building_z_tolerance_mm: float = 6.0
    building_heading_tolerance_deg: float = 2.4
    building_x_deadband_mm: float = 1.0
    building_z_deadband_mm: float = 2.0
    building_heading_deadband_deg: float = 0.5
    building_confirm_frames: int = 3
    building_median_frames: int = 5
    building_align_timeout_s: float = 7.0
    building_lost_timeout_s: float = 4.0
    building_vision_stale_s: float = 0.7
    building_track_max_x_jump_mm: float = 90.0
    building_track_max_z_jump_mm: float = 140.0
    building_track_lock_frames: int = 2
    building_control_period_s: float = 0.05
    building_forward_kp: float = 1.5
    building_forward_ki: float = 0.01
    building_forward_kd: float = 0.02
    building_lateral_kp: float = 1.8
    building_lateral_ki: float = 0.01
    building_lateral_kd: float = 0.02
    building_heading_kp: float = 1.5
    building_heading_ki: float = 0.02
    building_heading_kd: float = 0.03
    building_linear_integral_limit: float = 150.0
    building_heading_integral_limit: float = 100.0
    building_max_forward_mm_s: float = 250.0
    building_max_lateral_mm_s: float = 250.0
    building_max_yaw_deg_s: float = 30.0
    # Keep the minimum above chassis static friction; do not creep below it.
    building_min_linear_mm_s: float = 100.0
    building_far_linear_mm_s: float = 150.0
    building_min_yaw_deg_s: float = 6.0
    # Slightly soften building alignment command changes without lowering the
    # minimum motion commands needed to overcome chassis static friction.
    building_linear_accel_mm_s2: float = 1000.0
    building_yaw_accel_deg_s2: float = 60.0
    post_build_reverse_mm: float = 200.0
    post_build_reverse_speed_mm_s: float = 300.0
    post_build_turn_target_cw_deg: float = 270.0
    post_build_route_distance_mm: float = 2200.0
    post_build_route_speed_mm_s: float = LONG_DISTANCE_MOVE_SPEED_MM_S
    post_build_tag_id: int = 1
    post_build_tag_distance_mm: float = 200.0
    post_build_tag_distance_tolerance_mm: float = 20.0
    # Tag1 is viewed at close range with the uncalibrated tag camera.  Stop
    # and wait longer for a fresh frame instead of failing on one occlusion.
    post_build_tag_vision_stale_s: float = 0.7
    post_build_tag_lost_timeout_s: float = 2.5
    # At 200 mm the tag-camera lateral noise is larger than the Tag6 region;
    # accept a stable +/-10 mm result instead of driving on a 7-8 mm jitter.
    post_build_tag_lateral_tolerance_mm: float = 10.0
    post_build_tag_heading_tolerance_deg: float = 4.0
    post_build_tag_heading_target_cw_deg: float = 270.0
    final_right_turn_target_cw_deg: float = 360.0
    finish_after_build: bool = False


@dataclass(frozen=True)
class Task2Round2Config(Task2Config):
    post_tag_lateral_mm: float = 0.0
    left_wall_approach_enabled: bool = False
    post_orange_lateral_base_mm: float = 500.0
    post_tag6_lateral_right_mm: float = 400.0
    finish_after_build: bool = True


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

    def _building_from_result(self, result, locked_position=None):
        cfg = self.config
        if (result is None
                or time.time() - result.timestamp
                > cfg.building_vision_stale_s):
            return None
        candidates = [
            block for block in result.all_blocks
            if block.color_name.casefold() == 'orange'
            and block.confidence >= cfg.building_min_confidence
            and cfg.building_min_height_width_ratio
            <= getattr(block, 'height_width_ratio', 0.0)
            <= cfg.building_max_height_width_ratio
        ]
        if not candidates:
            return None
        if locked_position is not None:
            previous_x, previous_z = locked_position
            tracked = [
                block for block in candidates
                if abs(self._building_top_reference(block)[0] - previous_x)
                    <= cfg.building_track_max_x_jump_mm
                and abs(self._building_top_reference(block)[1] - previous_z)
                    <= cfg.building_track_max_z_jump_mm
            ]
            if tracked:
                return min(tracked, key=lambda block:
                           (self._building_top_reference(block)[0] - previous_x) ** 2
                           + (self._building_top_reference(block)[1] - previous_z) ** 2)
            return None
        return min(candidates, key=lambda block:
                   (self._building_top_reference(block)[0]
                    - cfg.building_target_x_mm) ** 2
                   + (self._building_top_reference(block)[1]
                      - cfg.building_target_z_mm) ** 2)

    @staticmethod
    def _building_top_reference(block):
        """Return X/Z measured from the visible upper edge of the contour."""
        quad = getattr(block, 'quad', None)
        if quad is None or len(quad) != 4:
            return block.x, block.z
        # order_corners() puts upper-left and upper-right at indices 0/1.
        top_u = (float(quad[0][0]) + float(quad[1][0])) * 0.5
        top_v = (float(quad[0][1]) + float(quad[1][1])) * 0.5
        cy = 240.0
        ref_v = 82.4
        # Empirical competition-camera calibration: the upper edge moves
        # downward as the robot approaches, so Z must decrease with top_v.
        # Normalize against the confirmed target pixel row.
        z_top = 132.8 * ref_v / max(top_v, 1.0)
        x_top = (top_u - 320.0) * z_top / 331.9
        return x_top, z_top

    @staticmethod
    def _building_motion_command(pid_output, error, *, deadband,
                                 minimum, maximum, kick_error=25.0):
        """Clamp every non-zero correction above chassis static friction."""
        if abs(error) <= deadband:
            return 0.0
        value = max(-maximum, min(maximum, pid_output))
        if abs(value) < minimum:
            value = minimum if value >= 0.0 else -minimum
        return max(-maximum, min(maximum, value))

    def _align_building(self):
        cfg = self.config
        forward_pid = _Pid(
            cfg.building_forward_kp, cfg.building_forward_ki,
            cfg.building_forward_kd, cfg.building_linear_integral_limit,
            cfg.building_max_forward_mm_s)
        lateral_pid = _Pid(
            cfg.building_lateral_kp, cfg.building_lateral_ki,
            cfg.building_lateral_kd, cfg.building_linear_integral_limit,
            cfg.building_max_lateral_mm_s)
        heading_pid = _Pid(
            cfg.building_heading_kp, cfg.building_heading_ki,
            cfg.building_heading_kd, cfg.building_heading_integral_limit,
            cfg.building_max_yaw_deg_s)
        samples = deque(maxlen=cfg.building_median_frames)
        started = time.monotonic()
        last_seen = started
        last_update = started
        first_frame_after = time.time()
        last_frame_timestamp = None
        confirmed = 0
        vx = vy = wz = 0.0
        locked_position = None
        pending_position = None
        lock_frames = 0
        lateral_aligned = False
        print('[Task2] Building visual alignment: '
              f'x={cfg.building_target_x_mm:+.1f} mm, '
              f'z={cfg.building_target_z_mm:.1f} mm')
        try:
            while time.monotonic() - started < cfg.building_align_timeout_s:
                now = time.monotonic()
                result = self.robot.vision_result
                tracking_position = (locked_position
                                     if locked_position is not None
                                     else pending_position)
                block = self._building_from_result(result, tracking_position)
                frame_timestamp = result.timestamp if result is not None else None
                if (frame_timestamp is not None
                        and frame_timestamp <= first_frame_after):
                    block = None
                if (block is not None
                        and frame_timestamp == last_frame_timestamp):
                    time.sleep(cfg.building_control_period_s)
                    continue
                if frame_timestamp is not None:
                    last_frame_timestamp = frame_timestamp

                if block is None:
                    confirmed = 0
                    samples.clear()
                    vx = vy = wz = 0.0
                    for pid in (forward_pid, lateral_pid, heading_pid):
                        pid.reset()
                    self.robot.chassis.set_speeds([0, 0, 0, 0])
                    if now - last_seen >= cfg.building_lost_timeout_s:
                        raise RuntimeError(
                            'three-layer orange building lost before Build')
                    time.sleep(cfg.building_control_period_s)
                    continue

                last_seen = now
                ref_x, ref_z = self._building_top_reference(block)
                if locked_position is None:
                    if pending_position is None:
                        pending_position = (ref_x, ref_z)
                        lock_frames = 1
                    else:
                        pending_position = (ref_x, ref_z)
                        lock_frames += 1
                    if lock_frames >= cfg.building_track_lock_frames:
                        locked_position = pending_position
                        pending_position = None
                else:
                    locked_position = (ref_x, ref_z)
                samples.append((ref_x, ref_z))
                x_mm = median(item[0] for item in samples)
                z_mm = median(item[1] for item in samples)
                x_error = x_mm - cfg.building_target_x_mm
                z_error = z_mm - cfg.building_target_z_mm
                heading_error = self._heading_error(
                    cfg.build_tag_heading_target_cw_deg)
                x_ok = abs(x_error) <= cfg.building_x_tolerance_mm
                z_ok = abs(z_error) <= cfg.building_z_tolerance_mm
                if not lateral_aligned and x_ok:
                    lateral_aligned = True
                    print('[Task2] Building lateral alignment complete; '
                          'starting forward/backward alignment')
                heading_ok = (
                    abs(heading_error)
                    <= cfg.building_heading_tolerance_deg)
                if x_ok and z_ok and heading_ok:
                    self.robot.chassis.set_speeds([0, 0, 0, 0])
                    vx = vy = wz = 0.0
                    confirmed += 1
                    print(f'[Task2] Building aligned '
                          f'{confirmed}/{cfg.building_confirm_frames}: '
                          f'x={x_mm:+.1f} mm, z={z_mm:.1f} mm, '
                          f'gyro={heading_error:+.1f} deg')
                    if confirmed >= cfg.building_confirm_frames:
                        return
                else:
                    confirmed = 0
                    dt = max(0.001, min(0.2, now - last_update))
                    if not lateral_aligned:
                        forward_pid.reset()
                        desired_vx = 0.0
                    elif abs(z_error) <= cfg.building_z_deadband_mm:
                        forward_pid.reset()
                        desired_vx = 0.0
                    else:
                        # Camera Z is positive forward (away from the camera),
                        # matching chassis +vx. Positive error therefore
                        # commands forward motion toward the target distance.
                        desired_vx = forward_pid.update(z_error, dt)
                        if not z_ok:
                            desired_vx = self._building_motion_command(
                                desired_vx, z_error,
                                deadband=cfg.building_z_deadband_mm,
                                minimum=cfg.building_min_linear_mm_s,
                                maximum=cfg.building_max_forward_mm_s)
                    if lateral_aligned:
                        lateral_pid.reset()
                        desired_vy = 0.0
                    elif abs(x_error) <= cfg.building_x_deadband_mm:
                        lateral_pid.reset()
                        desired_vy = 0.0
                    else:
                        desired_vy = lateral_pid.update(x_error, dt)
                        if not x_ok:
                            desired_vy = self._building_motion_command(
                                desired_vy, x_error,
                                deadband=cfg.building_x_deadband_mm,
                                minimum=cfg.building_min_linear_mm_s,
                                maximum=cfg.building_max_lateral_mm_s)
                    if (abs(heading_error)
                            <= cfg.building_heading_deadband_deg):
                        heading_pid.reset()
                        desired_wz = 0.0
                    else:
                        desired_wz = heading_pid.update(heading_error, dt)
                        if not heading_ok:
                            desired_wz = self._minimum_command(
                                desired_wz, cfg.building_min_yaw_deg_s)
                    vx = self._slew_command(
                        desired_vx, vx,
                        cfg.building_linear_accel_mm_s2, dt)
                    vy = self._slew_command(
                        desired_vy, vy,
                        cfg.building_linear_accel_mm_s2, dt)
                    wz = self._slew_command(
                        desired_wz, wz,
                        cfg.building_yaw_accel_deg_s2, dt)
                    # Do not let the slew limiter reduce a required forward
                    # correction below static-friction speed; otherwise the
                    # robot appears to stutter and never closes the Z error.
                    if abs(desired_vx) > cfg.building_z_deadband_mm:
                        if abs(vx) < cfg.building_min_linear_mm_s:
                            vx = (cfg.building_min_linear_mm_s
                                  if desired_vx > 0.0
                                  else -cfg.building_min_linear_mm_s)
                    rpm = self.robot.chassis.mecanum_rpm(
                        vx / 10.0, vy / 10.0, wz)
                    self.robot.chassis.set_speeds(rpm)
                    print(f'[Task2] Building PID: '
                          f'x={x_mm:+.1f} mm, z={z_mm:.1f} mm, '
                          f'gyro={heading_error:+.1f} deg; '
                          f'vx={vx:+.0f}, vy={vy:+.0f} mm/s, '
                          f'wz={wz:+.1f} deg/s')
                last_update = now
                time.sleep(cfg.building_control_period_s)
        finally:
            self.robot.chassis.set_speeds([0, 0, 0, 0])
        raise RuntimeError('building visual alignment timed out')

    def _align_building_or_continue(self) -> bool:
        try:
            self._align_building()
            return True
        except RuntimeError as exc:
            if str(exc) not in {
                    'three-layer orange building lost before Build',
                    'building visual alignment timed out'}:
                raise
            self.robot.chassis.set_speeds([0, 0, 0, 0])
            print(f'[Task2] Warning: {exc}; continuing with Build')
            return False

    def _search_and_align_purple(self) -> bool:
        cfg = self.config
        not_found_error = 'purple cube not found within search range'
        while True:
            self.state = Task2State.PURPLE_SEARCH
            try:
                block = self._find_cube(
                    color_name='purple',
                    min_confidence=cfg.purple_min_confidence,
                    search_direction=-1.0,
                    max_distance_mm=cfg.purple_search_max_distance_mm,
                )
            except RuntimeError as exc:
                if str(exc) != not_found_error:
                    raise
                print('[Task2] Purple cube not found within '
                      f'{cfg.purple_search_max_distance_mm:.0f} mm; '
                      'skipping Grap2')
                return False

            self.state = Task2State.PURPLE_ALIGN
            if self._align_cube(
                    block, color_name='purple',
                    min_confidence=cfg.purple_min_confidence):
                return True

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

    def _try_grab_purple(self) -> bool:
        if not self._search_and_align_purple():
            return False

        self.state = Task2State.WALL_APPROACH
        self._press_wall_before_grab()

        self.state = Task2State.GRAB
        print('[Task2] Purple aligned; running Grap2')
        self.robot.actions.grap2()
        print('[Task2] Grap2 complete')
        return True

    def _run_build_alignment_and_action(self):
        """Run the Tag6-to-Build segment shared with the standalone test."""
        cfg = self.config
        if cfg.post_tag6_lateral_right_mm > 0.0:
            self.state = Task2State.POST_TAG6_LATERAL
            print(f'[{self.TASK_LABEL}] Move right '
                  f'{cfg.post_tag6_lateral_right_mm:.0f} mm at '
                  f'{cfg.post_tag6_lateral_speed_mm_s:.0f} mm/s after Tag6')
            self._checked_move(
                'right', cfg.post_tag6_lateral_right_mm,
                cfg.post_tag6_lateral_speed_mm_s)

        self.state = Task2State.BUILDING_ALIGN
        self.robot.reset_vision_filter()
        building_aligned = self._align_building_or_continue()

        self.state = Task2State.BUILD
        if building_aligned:
            print(f'[{self.TASK_LABEL}] Building aligned; running Build')
        else:
            print(f'[{self.TASK_LABEL}] Building alignment skipped; '
                  'running Build')
        self.robot.actions.build()
        print(f'[{self.TASK_LABEL}] Build complete')

    def _run_build_phase(self):
        cfg = self.config
        self._run_build_alignment_and_action()

        if cfg.finish_after_build:
            print(f'[{self.TASK_LABEL}] Round complete after Build')
            return

        self.state = Task2State.POST_BUILD_REVERSE
        print(f'[Task2] Reverse {cfg.post_build_reverse_mm:.0f} mm '
              'after Build')
        self._checked_move(
            'backward', cfg.post_build_reverse_mm,
            cfg.post_build_reverse_speed_mm_s)

        self.state = Task2State.POST_BUILD_TURN
        self._turn_to_heading(cfg.post_build_turn_target_cw_deg)

        self.state = Task2State.POST_BUILD_ROUTE
        print(f'[Task2] Forward {cfg.post_build_route_distance_mm:.0f} mm '
              f'at {cfg.post_build_route_speed_mm_s:.0f} mm/s toward Tag1')
        self._checked_move(
            'forward', cfg.post_build_route_distance_mm,
            cfg.post_build_route_speed_mm_s)

        self.state = Task2State.TAG1_ALIGN
        self.robot.reset_field_localization_filter()
        self._align_delivery_tag(
            tag_id=cfg.post_build_tag_id,
            target_distance_mm=cfg.post_build_tag_distance_mm,
            heading_target_cw_deg=(
                cfg.post_build_tag_heading_target_cw_deg),
            distance_tolerance_mm=cfg.post_build_tag_distance_tolerance_mm,
            lateral_tolerance_mm=cfg.post_build_tag_lateral_tolerance_mm,
            heading_tolerance_deg=cfg.post_build_tag_heading_tolerance_deg,
            fine_gain_scale=cfg.build_tag_fine_gain_scale,
            vision_stale_s=cfg.post_build_tag_vision_stale_s,
            lost_timeout_s=cfg.post_build_tag_lost_timeout_s,
        )

        self.state = Task2State.FINAL_RIGHT_TURN
        self._turn_to_heading(cfg.final_right_turn_target_cw_deg)

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

        self._run_post_tag3_lateral()

        self.state = Task2State.WALL_PREMOVE
        print(f'[Task2] Forward {cfg.wall_premove_mm:.0f} mm at '
              f'{cfg.wall_premove_speed_mm_s:.0f} mm/s before wall approach')
        self._checked_move(
            'forward', cfg.wall_premove_mm, cfg.wall_premove_speed_mm_s)

        self.state = Task2State.WALL_APPROACH
        print(f'[Task2] Approach wall at '
              f'{cfg.far_wall_speed_mm_s:.0f} mm/s')
        self._drive_until_wall(
            timeout_s=cfg.far_wall_timeout_s,
            speed_mm_s=cfg.far_wall_speed_mm_s,
            context='Wall contact',
        )

        self.robot.reset_vision_filter()
        self._search_position_mm = 0.0
        purple_lateral_origin = self._capture_lateral_origin()
        purple_grabbed = self._try_grab_purple()

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

        self._run_post_return_wall_approach()

        set_profile = getattr(
            self.robot, 'set_cube_detection_profile', None)
        if set_profile is not None:
            set_profile('task2_orange')
        try:
            self.robot.reset_vision_filter()
            self._search_position_mm = 0.0
            orange_lateral_origin = self._capture_lateral_origin()
            orange_target_count = self._orange_target_count_for_run(
                purple_grabbed)
            for cube_index in range(1, orange_target_count + 1):
                print(f'[Task2] Orange cube {cube_index}/'
                      f'{orange_target_count}')
                while True:
                    self.state = Task2State.ORANGE_SEARCH
                    block = self._find_cube(
                        color_name='orange',
                        min_confidence=cfg.orange_min_confidence,
                        search_direction=1.0,
                        lock_x_jump_mm=cfg.orange_search_lock_x_jump_mm,
                        ambiguity_margin_mm=cfg.orange_track_ambiguity_margin_mm,
                    )
                    self.state = Task2State.ORANGE_ALIGN
                    if self._align_cube(
                            block, color_name='orange',
                            min_confidence=cfg.orange_min_confidence,
                            align_min_x_mm=cfg.orange_align_min_x_mm,
                            align_max_x_mm=cfg.orange_align_max_x_mm,
                            align_target_x_mm=cfg.orange_align_target_x_mm,
                            ambiguity_margin_mm=cfg.orange_track_ambiguity_margin_mm):
                        if self._fine_align_orange(block):
                            break

                self.state = Task2State.WALL_APPROACH
                self._press_wall_before_grab(recalibrate_heading_zero=True)

                self.state = Task2State.ORANGE_GRAB
                print(f'[Task2] Orange {cube_index}/'
                      f'{orange_target_count} aligned; running Grap1')
                self.robot.actions.grap1()
                print(f'[Task2] Grap1 {cube_index}/'
                      f'{orange_target_count} complete')
                self.robot.reset_vision_filter()
                time.sleep(cfg.post_grab_settle_s)
        finally:
            if set_profile is not None:
                set_profile('default')

        orange_lateral_mm = self._measure_lateral_displacement_mm(
            orange_lateral_origin)
        print('[Task2] Encoder-measured orange lateral displacement: '
              f'{orange_lateral_mm:+.0f} mm (right positive)')

        self.state = Task2State.POST_ORANGE_REVERSE
        print(f'[Task2] Reverse {cfg.post_orange_reverse_mm:.0f} mm')
        self._checked_move(
            'backward', cfg.post_orange_reverse_mm,
            cfg.post_orange_reverse_speed_mm_s)

        lateral_direction, lateral_distance_mm = (
            self._lateral_correction_command(
                cfg.post_orange_lateral_base_mm, orange_lateral_mm))
        self.state = Task2State.POST_ORANGE_LATERAL
        if lateral_direction is None:
            print('[Task2] Post-orange lateral correction is zero; '
                  'skipping lateral move')
        else:
            print(f'[Task2] Move {lateral_direction} '
                  f'{lateral_distance_mm:.0f} mm at '
                  f'{cfg.post_orange_lateral_speed_mm_s:.0f} mm/s '
                  f'(target right {cfg.post_orange_lateral_base_mm:.0f} - '
                  f'encoder right {orange_lateral_mm:.0f} mm)')
            self._checked_move(
                lateral_direction, lateral_distance_mm,
                cfg.post_orange_lateral_speed_mm_s)

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
            vision_stale_s=cfg.build_tag_vision_stale_s,
            lost_timeout_s=cfg.build_tag_lost_timeout_s,
        )

        self._run_build_phase()

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


class Task2Round2Program(Task2Program):
    TASK_LABEL = 'Task2-R2'

    def __init__(self, robot,
                 config: Task2Round2Config = Task2Round2Config()):
        super().__init__(robot, config)


__all__ = [
    'Task2Config',
    'Task2DebugConfig',
    'Task2DebugProgram',
    'Task2Program',
    'Task2Round2Config',
    'Task2Round2Program',
    'Task2State',
]
