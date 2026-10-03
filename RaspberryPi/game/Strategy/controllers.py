"""Reusable robot control algorithms with no mission sequence ownership."""

from __future__ import annotations

from collections import deque
from enum import Enum, auto
from statistics import median
import time
from typing import TYPE_CHECKING, Optional

from control.chassis import LONG_DISTANCE_MOVE_SPEED_MM_S
from .common import minimum_command, slew_command, wrap_angle
from .common import VisualAlignmentUnavailable, report_visual_fallback
from .tag_alignment import median_translation, translation_jump, missing_tag_details
from .tag_controller import PID as _Pid, TagPidSet, AxisToleranceHold, profiled_command
from .cube_tracker import CubeTargetTracker, select_tracked_block
from .wall_approach import velocity_for_direction
from .wall_controller import StallConfirmation
from .orange_search import OrangeSearchRecovery, find_orange
from .errors import SearchRangeExhausted
from .settings import CommonControlConfig, GroundCollectionConfig, FLAT_ROUTE_SPEED_MM_S
from .building_alignment import BuildingAlignment

if TYPE_CHECKING:
    from robot import Robot
    from vision.cube_detector import BlockInfo
    from vision.field_localizer import TagSolution


class Phase(Enum):
    """Diagnostic progress labels, with no mission transition semantics."""
    STARTUP = auto()
    READY = auto()
    WALL_APPROACH = auto()
    ORANGE_SEARCH = auto()
    ORANGE_ALIGN = auto()
    GRAB = auto()
    COUNT_CHECK = auto()
    DELIVERY_ROUTE = auto()
    DELIVERY_TAG_ALIGN = auto()
    POST_TAG_LATERAL = auto()
    UNLOAD = auto()
    PRE_FINAL_TURN_LATERAL = auto()
    FINISHED = auto()
    FAULT = auto()
    INITIAL_MOVE = auto()
    LATERAL_MOVE = auto()
    FINAL_TURN = auto()
    TURN_TO_START = auto()
    LEFT_WALL_APPROACH = auto()
    TURN_TO_COLLECTION = auto()
    TAG_ALIGN = auto()
    WALL_PREMOVE = auto()
    PURPLE_SEARCH = auto()
    PURPLE_ALIGN = auto()
    POST_GRAB_REVERSE = auto()
    TURN_RIGHT = auto()
    RETURN_MOVE = auto()
    FINAL_WALL_APPROACH = auto()
    ORANGE_GRAB = auto()
    POST_ORANGE_REVERSE = auto()
    POST_ORANGE_LATERAL = auto()
    BUILD_ROUTE = auto()
    TAG6_ALIGN = auto()
    POST_TAG6_LATERAL = auto()
    BUILDING_ALIGN = auto()
    BUILD = auto()
    POST_BUILD_REVERSE = auto()
    POST_BUILD_TURN = auto()
    POST_BUILD_ROUTE = auto()
    POST_BUILD_LEFT_WALL = auto()
    INITIAL_LATERAL = auto()
    POST_WALL_LATERAL = auto()
    FORWARD_WALL_APPROACH = auto()
    FINAL_LATERAL = auto()
    INITIAL_LEFT = auto()
    HATCH_OPEN = auto()
    HATCH_CLOSE = auto()
    BACKWARD = auto()
    BUILD_APPROACH = auto()
    FIRST_BUILD_LEFT = auto()
    SECOND_LOAD_APPROACH = auto()
    FINAL_LEFT = auto()


class RobotController(BuildingAlignment):
    """Bounded motion and alignment operations shared by functional actions.

    A controller owns local visual-search state. The execution context owns
    the heading anchor so constructing another action preserves that anchor.
    """

    TELEMETRY_WAIT_S = 2.0
    CUBE_VISION_PIPELINE_VERSION = 'cube-lock-v3'

    def __init__(self, robot: Robot,
                 config: CommonControlConfig = GroundCollectionConfig(), *,
                 context=None, operation_name: str = 'action'):
        self.robot = robot
        self.context = context
        self.config = config
        self.operation_name = operation_name
        self.state = Phase.STARTUP
        self._search_position_mm = 0.0
        self._orange_recovery = OrangeSearchRecovery()
        self._cube_lateral_displacement_mm: Optional[float] = None
        self._local_heading_zero_deg: Optional[float] = None
        self._last_alignment_block = None
        self._last_alignment_frame_timestamp = None
        self._last_alignment_timed_out = False
        self._alignment_valid_frames = 0

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        self._state = value
        report = getattr(self.robot, 'set_collection_context', None)
        if report is not None:
            report(task=self.operation_name, phase=value.name)

    @property
    def _heading_zero_deg(self):
        context = getattr(self, 'context', None)
        if context is not None:
            return context.heading_zero_deg
        return self._local_heading_zero_deg

    @_heading_zero_deg.setter
    def _heading_zero_deg(self, value):
        context = getattr(self, 'context', None)
        if context is not None:
            context.heading_zero_deg = value
        else:
            self._local_heading_zero_deg = value


    def _check_active(self, *, require_telemetry=True):
        context = getattr(self, "context", None)
        if context is not None:
            context.check_active(require_telemetry=require_telemetry)

    def _wait_ready(self):
        self._check_active(require_telemetry=False)
        deadline = time.monotonic() + self.TELEMETRY_WAIT_S
        while self.robot.telem is None and time.monotonic() < deadline:
            self._check_active(require_telemetry=False)
            time.sleep(0.02)
        if self.robot.telem is None:
            raise RuntimeError('A-board telemetry unavailable')
        if not self.robot.has_vision:
            report_visual_fallback(self.robot, self.operation_name, 'startup',
                                   'cube camera unavailable; use bounded search')
        if not self.robot.has_field_localization:
            report_visual_fallback(self.robot, self.operation_name, 'startup',
                                   'tag camera unavailable; alignment may be skipped')

        self._check_active()

    def _set_cube_profile(self, profile):
        if not getattr(self.robot, 'has_vision', True):
            return
        setter = getattr(self.robot, 'set_cube_detection_profile', None)
        if setter is not None:
            setter(profile)

    @staticmethod
    def _orange_from_result(result, min_confidence: float,
                            max_age_s: float) -> Optional['BlockInfo']:
        return RobotController._block_from_result(
            result, 'orange', min_confidence, max_age_s)

    @staticmethod
    def _block_from_result(result, color_name: str, min_confidence: float,
                           max_age_s: float) -> Optional['BlockInfo']:
        if result is None or time.time() - result.timestamp > max_age_s:
            return None
        candidates = [
            block for block in result.all_blocks
            if block.color_name.casefold() == color_name.casefold()
            and block.confidence >= min_confidence
        ]
        if not candidates:
            return None
        return min(candidates, key=lambda block: block.x * block.x
                   + block.y * block.y + block.z * block.z)

    @staticmethod
    def _tracked_orange_from_result(result, reference_x: float,
                                    cfg: CommonControlConfig):
        return RobotController._tracked_block_from_result(
            result, 'orange', reference_x, cfg.orange_min_confidence, cfg)

    @staticmethod
    def _tracked_block_from_result(result, color_name: str,
                                   reference_x: float,
                                   min_confidence: float,
                                   cfg: CommonControlConfig,
                                   reference_z: Optional[float] = None,
                                   ambiguity_margin_mm: Optional[float] = None):
        return select_tracked_block(
            result, color_name, reference_x, min_confidence, cfg,
            reference_z, ambiguity_margin_mm)

    @staticmethod
    def _stall_sample(telem, cfg: CommonControlConfig,
                      direction: str = 'forward') -> bool:
        # A-board motor order is TL(1), TR(0), BL(2), BR(3); forward contact
        # is confirmed by the rear pair while lateral contact keeps 3/4.
        motors = (telem.motors[2:4] if direction.casefold() == 'forward'
                  else telem.motors)
        required = (2 if direction.casefold() == 'forward'
                    else cfg.stall_motor_count)
        stalled = sum(
            abs(m.speed_rpm) <= cfg.stall_speed_rpm
            and abs(m.torque_current) >= cfg.stall_current_raw
            for m in motors
        )
        return stalled >= required

    def _drive_until_wall(self, *, timeout_s: Optional[float] = None,
                          startup_grace_s: Optional[float] = None,
                          confirm_s: Optional[float] = None,
                          speed_mm_s: Optional[float] = None,
                          direction: str = 'forward',
                          context: str = 'Wall contact',
                          timeout_is_success: Optional[bool] = None):
        cfg = self.config
        timeout_s = (cfg.far_wall_timeout_s
                     if timeout_s is None else timeout_s)
        startup_grace_s = (cfg.stall_startup_grace_s
                           if startup_grace_s is None else startup_grace_s)
        confirm_s = cfg.stall_confirm_s if confirm_s is None else confirm_s
        speed_mm_s = (cfg.far_wall_speed_mm_s
                      if speed_mm_s is None else speed_mm_s)
        timeout_is_success = (
            cfg.wall_timeout_is_success if timeout_is_success is None
            else timeout_is_success)
        vx_cm_s, vy_cm_s = velocity_for_direction(direction, speed_mm_s)
        rpm = self.robot.chassis.mecanum_rpm(vx_cm_s, vy_cm_s, 0.0)
        started = time.monotonic()
        stall_tracker = StallConfirmation()
        last_uptime = None
        last_telem_time = time.monotonic()
        try:
            while time.monotonic() - started < timeout_s:
                self._check_active()
                self.robot.chassis.set_speeds(rpm)
                telem = self.robot.telem
                elapsed = time.monotonic() - started
                is_new = (telem is not None
                          and telem.uptime_ms != last_uptime)
                if not is_new:
                    if time.monotonic() - last_telem_time > cfg.telemetry_stale_s:
                        raise RuntimeError('telemetry lost during wall approach')
                    time.sleep(0.01)
                    continue

                last_uptime = telem.uptime_ms
                last_telem_time = time.monotonic()
                is_stalled = (elapsed >= startup_grace_s
                              and self._stall_sample(telem, cfg, direction))
                if stall_tracker.update(is_stalled, time.monotonic(), confirm_s,
                                        dropout_s=cfg.stall_dropout_s):
                    print(f'[{self.operation_name}] {context} confirmed')
                    return
                time.sleep(0.01)
        finally:
            self.robot.chassis.set_speeds([0, 0, 0, 0])
        if timeout_is_success:
            print(f'[{self.operation_name}] {context} timed out after '
                  f'{timeout_s:.1f}s; accepting wall contact')
            return
        raise RuntimeError('wall contact was not detected before timeout')

    def _recalibrate_heading_zero(self, reference_cw_deg: float = 0.0):
        """Assign the current gyro reading to a known clockwise route heading."""
        telem = self.robot.telem
        if telem is None:
            raise RuntimeError('telemetry unavailable for heading recalibration')
        previous_zero = self._heading_zero_deg
        # Route heading is clockwise-positive; gyro yaw is CCW-positive.
        # yaw(reference) = zero - reference, hence zero = yaw + reference.
        self._heading_zero_deg = self._wrap_angle(
            telem.yaw_deg + reference_cw_deg)
        correction = (0.0 if previous_zero is None else
                      self._wrap_angle(self._heading_zero_deg - previous_zero))
        print(f'[{self.operation_name}] Heading zero recalibrated: '
              f'{self._heading_zero_deg:+.1f} deg '
              f'(reference {reference_cw_deg:.1f} deg CW, '
              f'correction {correction:+.1f} deg)')

    def _chassis_followup(self, route):
        """Adapt a chassis-only route to the action client's cooperative check."""
        def followup(check):
            with self.robot.chassis.monitor_action(check):
                route()
        return followup

    def _grab_press_step(self, *, recalibrate_heading_zero=False):
        """Return the existing nonblocking wall-press phase for an action session."""
        cfg = self.config
        rpm = self.robot.chassis.mecanum_rpm(
            cfg.near_wall_speed_mm_s / 10.0, 0.0, 0.0)
        tracker = StallConfirmation()
        initial = self.robot.telem
        last_uptime = initial.uptime_ms if initial is not None else None
        started = last_telem_time = finished_at = None

        def press_step():
            self._check_active()
            nonlocal started, last_telem_time, last_uptime, finished_at
            now = time.monotonic()
            if finished_at is not None:
                return now - finished_at >= cfg.pre_grab_wall_settle_s
            if started is None:
                started = last_telem_time = now
                print(f'[{self.operation_name}] Grab started; short wall press '
                      f'at {cfg.near_wall_speed_mm_s:.0f} mm/s')
            telem = self.robot.telem
            contact = False
            if telem is not None and telem.uptime_ms != last_uptime:
                last_uptime = telem.uptime_ms
                last_telem_time = now
                stalled = (now - started >= cfg.pre_grab_stall_startup_grace_s
                           and self._stall_sample(telem, cfg))
                contact = tracker.update(
                    stalled, now, cfg.pre_grab_stall_confirm_s,
                    dropout_s=cfg.stall_dropout_s)
            if now - last_telem_time > cfg.telemetry_stale_s:
                raise RuntimeError('telemetry lost during grab wall press')
            timed_out = now - started >= cfg.near_wall_timeout_s
            if contact or timed_out:
                if not self.robot.chassis.set_speeds([0, 0, 0, 0]):
                    raise RuntimeError('failed to stop grab wall press')
                if timed_out and not contact and not cfg.wall_timeout_is_success:
                    raise RuntimeError('grab wall contact timed out')
                finished_at = now
                outcome = 'confirmed' if contact else 'timeout accepted'
                print(f'[{self.operation_name}] Grab wall press {outcome}; '
                      'chassis stopped')
                if recalibrate_heading_zero:
                    self._recalibrate_heading_zero()
                return cfg.pre_grab_wall_settle_s <= 0.0
            if not self.robot.chassis.set_speeds(rpm):
                raise RuntimeError('failed to send grab wall press speed')
            return False

        return press_step

    def _grab_with_wall_press(self, grab, *, recalibrate_heading_zero=False,
                              chassis_followup=None):
        """Blocking compatibility API; phased flows use _grab_press_step directly."""
        cfg = self.config
        press_step = self._grab_press_step(
            recalibrate_heading_zero=recalibrate_heading_zero)
        pose_begin = getattr(self.robot, 'begin_cube_camera_pose_change', None)
        pose_end = getattr(self.robot, 'end_cube_camera_pose_change', None)
        pose_token = pose_begin('grab') if pose_begin is not None else None
        try:
            kwargs = {'parallel_step': press_step}
            if chassis_followup is not None:
                kwargs['chassis_followup'] = self._chassis_followup(chassis_followup)
            grab(**kwargs)
            # Existing whole-action completion and the calibrated settling
            # interval are prerequisites. Faults deliberately leave vision shut.
            if pose_end is not None:
                if pose_end(pose_token, settle_s=cfg.post_grab_settle_s) is False:
                    raise RuntimeError('cube camera pose token changed during grab')
        finally:
            # Zero speed leaves the mechanism alone; Actions handles emergency
            # cancellation on errors. No worker may re-send motion afterward.
            self.robot.chassis.set_speeds([0, 0, 0, 0])

    def _checked_move(self, direction: str, distance_mm: float,
                      speed_mm_s: float, *, accel_ms: Optional[int] = None,
                      ramp_straight: bool = False):
        self._check_active()
        if accel_ms is None:
            accel_ms = self.config.delivery_linear_accel_ms
            if (direction.casefold() in ('forward', 'backward', 'left', 'right')
                    and speed_mm_s >= LONG_DISTANCE_MOVE_SPEED_MM_S):
                accel_ms = max(accel_ms,round(self.config.long_distance_forward_accel_ms
                                            * speed_mm_s / FLAT_ROUTE_SPEED_MM_S))
        result = self.robot.move_chassis(
            direction, distance_mm, speed_mm_s,
            hold_ms=0, accel_ms=accel_ms, route_mode=True)
        if result.cancelled:
            raise RuntimeError(
                f'chassis move failed: {direction} {distance_mm:.0f} mm')
        if result.timed_out:
            if ramp_straight:
                raise RuntimeError('ramp move did not complete; do not turn on the ramp')
            # A move can reach its commanded position just before the
            # controller's timeout while settling. Do not abort the whole
            # strategy when encoder progress proves that the move completed.
            progress = (abs(result.estimated_chassis_distance_mm)
                        / max(abs(distance_mm), 1.0))
            if progress >= 0.90:
                print(f'[{self.operation_name}] chassis timeout accepted: '
                      f'{direction} {result.estimated_chassis_distance_mm:.1f}/'
                      f'{distance_mm:.1f} mm ({progress * 100:.0f}%)')
                return result
            raise RuntimeError(
                f'chassis move failed: {direction} {distance_mm:.0f} mm '
                f'(progress {progress * 100:.0f}%)')
        return result

    def _capture_lateral_origin(self):
        return self.robot.chassis.capture_motor_positions()

    def _unload_cubes(self):
        """Shared hatch unloading: open 300 ms, reverse, close without waiting."""
        cfg = self.config
        self._check_active()
        print(f'[{self.operation_name}] Unload: open hatches')
        self.robot.actions.hatch_open(settle_ms=300)
        self._checked_move(
            'backward', cfg.unload_reverse_mm, cfg.unload_reverse_speed_mm_s)
        self._check_active()
        print(f'[{self.operation_name}] Unload: close hatches')
        self.robot.actions.hatch_close(settle_ms=0)
        self._check_active()

    def _measure_lateral_displacement_mm(self, origin) -> float:
        return self.robot.chassis.lateral_displacement_mm(origin)

    def _find_orange(self) -> 'BlockInfo':
        return self._find_cube(
            color_name='orange', min_confidence=self.config.orange_min_confidence,
            search_direction=1.0,
            lock_x_jump_mm=self.config.orange_search_lock_x_jump_mm)

    def _find_cube(self, *, color_name: str, min_confidence: float,
                   search_direction: float,
                   max_distance_mm: Optional[float] = None,
                   lock_x_jump_mm: Optional[float] = None,
                   ambiguity_margin_mm: Optional[float] = None) -> 'BlockInfo':
        cfg = self.config
        search_limit_mm = (cfg.search_max_distance_mm
                           if max_distance_mm is None else max_distance_mm)
        remaining_mm = search_limit_mm - self._search_position_mm
        if remaining_mm <= 0.0:
            raise SearchRangeExhausted(
                f'{color_name} cube not found within search range')

        rpm = self.robot.chassis.mecanum_rpm(
            0.0, search_direction * cfg.search_speed_mm_s / 10.0, 0.0)
        started = time.monotonic()
        deadline = started + remaining_mm / cfg.search_speed_mm_s
        direction_name = 'right' if search_direction > 0.0 else 'left'
        display_color = color_name.capitalize()
        tracker = CubeTargetTracker(
            max_x_jump_mm=(cfg.orange_search_lock_x_jump_mm
                           if lock_x_jump_mm is None else lock_x_jump_mm),
            max_z_jump_mm=cfg.orange_search_lock_z_jump_mm,
            confirm_frames=cfg.orange_search_confirm_frames,
            lost_frames=cfg.orange_search_lock_lost_frames,
            smoothing_frames=cfg.align_filter_frames,
            ambiguity_margin_mm=(cfg.align_track_ambiguity_margin_mm
                                 if ambiguity_margin_mm is None
                                 else ambiguity_margin_mm),
        )
        if color_name.casefold() == 'orange' and search_direction > 0.0:
            block = find_orange(self, tracker, search_limit_mm)
            if block is None:
                raise SearchRangeExhausted('orange cube not found within search range')
            return block
        print(f'[{self.operation_name}] {display_color} not visible; '
              f'continuous search {direction_name} '
              f'from {self._search_position_mm:.0f}/'
              f'{search_limit_mm:.0f} mm')
        try:
            while time.monotonic() < deadline:
                self._check_active()
                self.robot.chassis.set_speeds(rpm)
                result = self.robot.vision_result
                block = tracker.update(
                    result, color_name=color_name,
                    min_confidence=min_confidence,
                    max_age_s=cfg.vision_stale_s)
                if block is not None:
                    elapsed_s = time.monotonic() - started
                    self._search_position_mm += min(
                        remaining_mm, elapsed_s * cfg.search_speed_mm_s)
                    print(f'[{self.operation_name}] {display_color} acquired: '
                          f'x={block.x:+.0f} mm, '
                          f'confidence={block.confidence:.0f}%, '
                          f'position={self._search_position_mm:.0f} mm')
                    return block
                time.sleep(cfg.search_control_period_s)
        finally:
            self.robot.chassis.set_speeds([0, 0, 0, 0])

        self._search_position_mm = search_limit_mm
        raise SearchRangeExhausted(f'{color_name} cube not found within search range')

    @staticmethod
    def _alignment_speed(x_error: float, integral: float,
                         derivative: float, cfg: CommonControlConfig,
                         minimum_speed_mm_s: Optional[float] = None) -> float:
        speed = (cfg.align_kp * x_error
                 + cfg.align_ki * integral
                 + cfg.align_kd * derivative)
        speed = max(-cfg.align_max_speed_mm_s,
                    min(cfg.align_max_speed_mm_s, speed))
        minimum = (cfg.align_min_speed_mm_s if minimum_speed_mm_s is None
                   else minimum_speed_mm_s)
        if 0.0 < abs(speed) < minimum:
            speed = minimum if speed > 0.0 else -minimum
        return speed

    @staticmethod
    def _slew_alignment_speed(desired: float, previous: float, dt: float,
                              cfg: CommonControlConfig) -> float:
        if desired == 0.0:
            return 0.0
        if previous == 0.0:
            start_speed = min(abs(desired), cfg.align_start_speed_mm_s)
            return start_speed if desired > 0.0 else -start_speed
        if desired * previous < 0.0:
            return 0.0
        max_delta = cfg.align_accel_mm_s2 * dt
        delta = max(-max_delta, min(max_delta, desired - previous))
        return previous + delta

    def _align_orange(self, initial_block: 'BlockInfo') -> bool:
        if not self._align_cube(
            initial_block, color_name='orange',
            min_confidence=self.config.orange_min_confidence,
            timeout_s=self.config.orange_coarse_align_timeout_s,
            timeout_is_success=True):
            return False
        if self._last_alignment_timed_out:
            return True
        return self._fine_align_orange(initial_block)

    def _fine_align_orange(self, initial_block: 'BlockInfo') -> bool:
        """Perform a short, tighter correction before gripping an orange cube."""
        cfg = self.config
        seed_block = (self._last_alignment_block
                      if self._last_alignment_block is not None
                      else initial_block)
        # Mixed-color collection has a separate orange calibration; ground
        # collection uses its general target when no orange override is set.
        orange_target_x = getattr(cfg, 'orange_align_target_x_mm',
                                  cfg.align_target_x_mm)
        aligned = self._align_cube(
            seed_block, color_name='orange',
            min_confidence=cfg.orange_min_confidence,
            align_min_x_mm=cfg.orange_fine_min_x_mm,
            align_max_x_mm=cfg.orange_fine_max_x_mm,
            align_target_x_mm=orange_target_x,
            timeout_s=cfg.orange_fine_align_timeout_s,
            # A fine-align timeout is a perception failure.  Never continue
            # to the gripper after the target has disappeared or jumped.
            timeout_is_success=False,
            timeout_returns_false=True,
            initial_reference_x=seed_block.x)
        if aligned:
            return True

        coarse_min = getattr(cfg, 'orange_align_min_x_mm',
                             cfg.align_min_x_mm)
        coarse_max = getattr(cfg, 'orange_align_max_x_mm',
                             cfg.align_max_x_mm)
        last_block = self._last_alignment_block
        if (self._last_alignment_timed_out
                and self._alignment_valid_frames > 0
                and last_block is not None
                and self._last_alignment_frame_timestamp is not None
                and 0.0 <= time.time() - self._last_alignment_frame_timestamp
                < cfg.vision_stale_s
                and coarse_min <= last_block.x <= coarse_max):
            print(f'[{self.operation_name}] Fine alignment timed out; '
                  f'coarse position x={last_block.x:+.0f} mm is acceptable')
            return True
        return False

    def _align_cube(self, initial_block: 'BlockInfo', *, color_name: str,
                    min_confidence: float,
                    align_min_x_mm: Optional[float] = None,
                    align_max_x_mm: Optional[float] = None,
                    align_target_x_mm: Optional[float] = None,
                     timeout_s: Optional[float] = None,
                     timeout_is_success: bool = False,
                     timeout_returns_false: bool = False,
                     initial_reference_x: Optional[float] = None,
                    ambiguity_margin_mm: Optional[float] = None) -> bool:
        """Continuously center a colored cube; return False after target loss."""
        cfg = self.config
        self._last_alignment_timed_out = False
        self._alignment_valid_frames = 0
        self._last_alignment_frame_timestamp = None
        if initial_reference_x is None:
            # Start a fresh target session for coarse acquisition.  The fine
            # stage deliberately keeps the block recorded by this session.
            self._last_alignment_block = None
        align_min_x_mm = (cfg.align_min_x_mm if align_min_x_mm is None
                          else align_min_x_mm)
        align_max_x_mm = (cfg.align_max_x_mm if align_max_x_mm is None
                          else align_max_x_mm)
        align_target_x_mm = (cfg.align_target_x_mm
                             if align_target_x_mm is None
                             else align_target_x_mm)
        display_color = color_name.capitalize()
        confirmed = 0
        integral = 0.0
        previous_error = None
        last_frame_timestamp = None
        started = time.monotonic()
        last_update = started
        last_seen = started
        reference_x = (initial_block.x if initial_reference_x is None
                       else initial_reference_x)
        reference_z = getattr(initial_block, 'z', None)
        tracker = CubeTargetTracker(
            max_x_jump_mm=cfg.align_track_max_x_jump_mm,
            max_z_jump_mm=cfg.align_track_max_z_jump_mm,
            confirm_frames=1,
            lost_frames=cfg.orange_search_lock_lost_frames,
            smoothing_frames=cfg.align_filter_frames,
            ambiguity_margin_mm=(cfg.align_track_ambiguity_margin_mm
                                 if ambiguity_margin_mm is None
                                 else ambiguity_margin_mm),
        )
        commanded_speed = 0.0
        alignment_window_latched = False
        alignment_edge_since = None
        x_samples = deque(maxlen=max(1, cfg.align_filter_frames))
        try:
            deadline = started + (cfg.align_timeout_s
                                  if timeout_s is None else timeout_s)
            while time.monotonic() - started < (deadline - started):
                self._check_active()
                now = time.monotonic()
                result = self.robot.vision_result
                frame_timestamp = result.timestamp if result is not None else None
                frame_fresh = (frame_timestamp is not None
                               and 0.0 <= time.time() - frame_timestamp
                               < cfg.vision_stale_s)
                repeated_frame = (frame_timestamp is not None
                                  and frame_timestamp == last_frame_timestamp)
                # A duplicate fresh frame neither confirms nor loses a target.
                # Once stale (or no valid target for the loss timeout), it must
                # enter the same stop path as a missing frame.
                if (repeated_frame and frame_fresh
                        and now - last_seen < cfg.align_lost_timeout_s):
                    time.sleep(cfg.align_control_period_s)
                    continue
                block = (tracker.update(
                    result, color_name=color_name,
                    min_confidence=min_confidence,
                    max_age_s=cfg.vision_stale_s,
                    reference_x=reference_x, reference_z=reference_z)
                    if frame_fresh and not repeated_frame else None)
                if frame_timestamp is not None:
                    last_frame_timestamp = frame_timestamp

                if block is None:
                    confirmed = 0
                    integral = 0.0
                    previous_error = None
                    self.robot.chassis.set_speeds([0, 0, 0, 0])
                    commanded_speed = 0.0
                    last_update = now
                    alignment_window_latched = False
                    alignment_edge_since = None
                    x_samples.clear()
                    self._last_alignment_block = None
                    self._last_alignment_frame_timestamp = None
                    if now - last_seen >= cfg.align_lost_timeout_s:
                        print(f'[{self.operation_name}] {display_color} lost; '
                              'resume search')
                        return False
                    time.sleep(cfg.align_control_period_s)
                    continue

                last_seen = now
                self._last_alignment_block = block
                self._last_alignment_frame_timestamp = frame_timestamp
                self._alignment_valid_frames += 1
                x_samples.append(block.x)
                filtered_x = median(x_samples)
                reference_x = filtered_x
                reference_z = block.z
                x_error = filtered_x - align_target_x_mm
                creep_min_speed = (cfg.align_creep_min_speed_mm_s
                                   if abs(x_error) <= cfg.align_creep_start_mm
                                   else cfg.align_min_speed_mm_s)
                # Use the filtered target for both control and acceptance.  A
                # single raw outlier must not authorize a gripper action.
                if align_min_x_mm <= filtered_x <= align_max_x_mm:
                    alignment_window_latched = True
                    alignment_edge_since = None
                    self.robot.chassis.set_speeds([0, 0, 0, 0])
                    commanded_speed = 0.0
                    integral = 0.0
                    previous_error = None
                    confirmed += 1
                    self._last_alignment_block = block
                    print(f'[{self.operation_name}] Alignment sample {confirmed}/'
                          f'{cfg.align_confirm_frames}: x={block.x:+.0f} mm')
                    if confirmed >= cfg.align_confirm_frames:
                        return True
                elif (alignment_window_latched
                      and align_min_x_mm - cfg.align_window_hysteresis_mm
                      <= filtered_x
                      <= align_max_x_mm + cfg.align_window_hysteresis_mm):
                    # Hold still through small edge jitter after entering the
                    # valid window. A subsequent valid frame can continue the
                    # confirmation sequence without a lateral speed kick.
                    if alignment_edge_since is None:
                        alignment_edge_since = now
                    if now - alignment_edge_since < cfg.align_window_hold_s:
                        self.robot.chassis.set_speeds([0, 0, 0, 0])
                        commanded_speed = 0.0
                        integral = 0.0
                        previous_error = None
                        print(f'[{self.operation_name}] Alignment edge hold: '
                          f'x={filtered_x:+.0f} mm')
                    else:
                        alignment_window_latched = False
                        alignment_edge_since = None
                        confirmed = 0
                        dt = max(0.001, min(0.2, now - last_update))
                        integral = max(
                            -cfg.align_integral_limit,
                            min(cfg.align_integral_limit,
                                integral + x_error * dt),
                        )
                        desired_speed = self._alignment_speed(
                            x_error, integral, 0.0, cfg, creep_min_speed)
                        desired_speed = profiled_command(
                            desired_speed, x_error,
                            slowdown_start=cfg.align_slowdown_start_mm,
                            creep_start=cfg.align_creep_start_mm,
                            fast_speed=cfg.align_fast_speed_mm_s,
                            max_speed=cfg.align_fast_speed_mm_s,
                            min_speed=creep_min_speed)
                        commanded_speed = self._slew_alignment_speed(
                            desired_speed, commanded_speed, dt, cfg)
                        rpm = self.robot.chassis.mecanum_rpm(
                            0.0, commanded_speed / 10.0, 0.0)
                        self.robot.chassis.set_speeds(rpm)
                        print(f'[{self.operation_name}] Visual PID recovery: '
                              f'x={filtered_x:+.0f} mm, '
                              f'vy={commanded_speed:+.0f} mm/s')
                else:
                    alignment_window_latched = False
                    alignment_edge_since = None
                    confirmed = 0
                    dt = max(0.001, min(0.2, now - last_update))
                    integral = max(
                        -cfg.align_integral_limit,
                        min(cfg.align_integral_limit,
                            integral + x_error * dt),
                    )
                    derivative = (0.0 if previous_error is None else
                                  (x_error - previous_error) / dt)
                    desired_speed = self._alignment_speed(
                        x_error, integral, derivative, cfg, creep_min_speed)
                    desired_speed = profiled_command(
                        desired_speed, x_error,
                        slowdown_start=cfg.align_slowdown_start_mm,
                        creep_start=cfg.align_creep_start_mm,
                        fast_speed=cfg.align_fast_speed_mm_s,
                        max_speed=cfg.align_fast_speed_mm_s,
                        min_speed=creep_min_speed)
                    commanded_speed = self._slew_alignment_speed(
                        desired_speed, commanded_speed, dt, cfg)
                    rpm = self.robot.chassis.mecanum_rpm(
                        0.0, commanded_speed / 10.0, 0.0)
                    self.robot.chassis.set_speeds(rpm)
                    print(f'[{self.operation_name}] Visual PID: '
                          f'x={filtered_x:+.0f} mm, '
                          f'vy={commanded_speed:+.0f} mm/s')
                    previous_error = x_error
                last_update = now
                time.sleep(cfg.align_control_period_s)
        finally:
            self.robot.chassis.set_speeds([0, 0, 0, 0])
        if timeout_returns_false:
            self._last_alignment_timed_out = True
            print(f'[{self.operation_name}] {display_color} alignment timed out; '
                  'resume search')
            return False
        if timeout_is_success:
            self._last_alignment_timed_out = True
            print(f'[{self.operation_name}] {display_color} alignment '
                  f'time reached; accepting position')
            return True
        raise RuntimeError(f'{color_name} visual alignment timed out')

    @staticmethod
    def _wrap_angle(angle_deg: float) -> float:
        return wrap_angle(angle_deg)

    def _heading_error(self, target_cw_deg: float) -> float:
        """Return gyro heading error, positive in the chassis CCW convention."""
        if self._heading_zero_deg is None:
            raise RuntimeError('startup heading zero is unavailable')
        telem = self.robot.telem
        if telem is None:
            raise RuntimeError('gyro telemetry unavailable')
        target_yaw = self._wrap_angle(
            self._heading_zero_deg - target_cw_deg)
        return self._wrap_angle(target_yaw - telem.yaw_deg)

    def _turn_to_heading(self, target_cw_deg: float,
                         hold_ms: int = 0,
                         settle_cycles: int = 1):
        """Turn to an absolute clockwise heading from the startup zero."""
        self._check_active()
        error_ccw_deg = self._heading_error(target_cw_deg)
        clockwise_delta_deg = -error_ccw_deg
        print(f'[{self.operation_name}] Gyro heading target '
              f'{target_cw_deg:.0f} deg CW from startup '
              f'(correction {clockwise_delta_deg:+.1f} deg CW)')
        self.robot.chassis.turn(
            clockwise_delta_deg,
            self.config.delivery_turn_speed_deg_s,
            hold_ms=hold_ms,
            settle_cycles=settle_cycles,
        )

    @staticmethod
    def _minimum_command(value: float, minimum: float) -> float:
        return minimum_command(value, minimum)

    @staticmethod
    def _slew_command(target: float, current: float,
                      max_rate: float, dt: float) -> float:
        return slew_command(target, current, max_rate, dt)

    @staticmethod
    def _delivery_tag_from_pose(pose, tag_id: int, max_age_s: float):
        if pose is None or time.time() - pose.timestamp > max_age_s:
            return None
        candidates = [item for item in pose.tag_solutions
                      if item.tag_id == tag_id]
        if not candidates:
            return None
        return min(candidates, key=lambda item: item.score)

    def _align_delivery_tag(
            self, *, tag_id: Optional[int] = None,
            target_distance_mm: Optional[float] = None,
            heading_target_cw_deg: Optional[float] = None,
            distance_tolerance_mm: Optional[float] = None,
            lateral_tolerance_mm: Optional[float] = None,
            heading_tolerance_deg: Optional[float] = None,
            fine_gain_scale: Optional[float] = None,
            vision_stale_s: Optional[float] = None,
            lost_timeout_s: Optional[float] = None,
            fine_align_enabled: bool = True,
            stop_axes_in_tolerance: bool = False,
            independent_heading: bool = False):
        """Use a tag for translation while holding startup-relative yaw."""
        if independent_heading and (fine_align_enabled or not stop_axes_in_tolerance):
            raise ValueError('independent heading requires coarse axis-hold alignment')
        cfg = self.config
        tag_id = cfg.delivery_tag_id if tag_id is None else tag_id
        target_distance_mm = (cfg.delivery_tag_distance_mm
                              if target_distance_mm is None
                              else target_distance_mm)
        heading_target_cw_deg = (
            cfg.delivery_heading_target_cw_deg
            if heading_target_cw_deg is None else heading_target_cw_deg)
        distance_tolerance_mm = (
            cfg.delivery_tag_distance_tolerance_mm
            if distance_tolerance_mm is None else distance_tolerance_mm)
        lateral_tolerance_mm = (
            cfg.delivery_tag_lateral_tolerance_mm
            if lateral_tolerance_mm is None else lateral_tolerance_mm)
        heading_tolerance_deg = (
            cfg.delivery_heading_tolerance_deg
            if heading_tolerance_deg is None else heading_tolerance_deg)
        fine_gain_scale = (cfg.delivery_tag_fine_gain_scale
                           if fine_gain_scale is None else fine_gain_scale)
        vision_stale_s = (cfg.delivery_tag_vision_stale_s
                          if vision_stale_s is None else vision_stale_s)
        lost_timeout_s = (cfg.delivery_tag_lost_timeout_s
                          if lost_timeout_s is None else lost_timeout_s)
        distance_pid = _Pid(
            cfg.delivery_tag_distance_kp,
            cfg.delivery_tag_distance_ki,
            cfg.delivery_tag_distance_kd,
            cfg.delivery_tag_linear_integral_limit,
            cfg.delivery_tag_max_forward_mm_s)
        lateral_pid = _Pid(
            cfg.delivery_tag_lateral_kp,
            cfg.delivery_tag_lateral_ki,
            cfg.delivery_tag_lateral_kd,
            cfg.delivery_tag_linear_integral_limit,
            cfg.delivery_tag_max_lateral_mm_s)
        heading_pid = _Pid(
            cfg.tag6_heading_kp if independent_heading else cfg.delivery_heading_kp,
            cfg.tag6_heading_ki if independent_heading else cfg.delivery_heading_ki,
            cfg.tag6_heading_kd if independent_heading else cfg.delivery_heading_kd,
            cfg.delivery_heading_integral_limit,
            cfg.delivery_heading_max_yaw_deg_s)
        pids = TagPidSet(distance_pid, lateral_pid, heading_pid)
        axis_holds = [AxisToleranceHold() for _ in range(3)]
        hold_axes = stop_axes_in_tolerance and not fine_align_enabled
        started = time.monotonic()
        first_valid_frame_after = time.time()
        last_seen = started
        last_update = started
        last_heading_update = started
        next_translation_update = started
        translation_ready = False
        control_period = (cfg.tag6_heading_control_period_s if independent_heading
                          else cfg.delivery_tag_control_period_s)
        last_frame_timestamp = None
        last_translation = None
        relock_candidate = None
        relock_count = 0
        translation_samples = deque(
            maxlen=cfg.delivery_tag_translation_median_frames)
        fine_started = None
        confirmed = 0
        vx = vy = wz = 0.0

        def update_heading(now):
            # Same command owner as translation; no background motor writer.
            nonlocal wz, last_heading_update, confirmed
            error = self._heading_error(heading_target_cw_deg)
            if abs(error) > heading_tolerance_deg:
                confirmed = 0
            heading_dt = max(0.001, min(0.2, now - last_heading_update))
            if abs(error) <= cfg.delivery_heading_deadband_deg:
                heading_pid.reset()
                wz = 0.0
            else:
                wz = self._slew_command(
                    heading_pid.update(error, heading_dt), wz,
                    cfg.delivery_heading_yaw_accel_deg_s2, heading_dt)
            last_heading_update = now
            self.robot.chassis.set_speeds(self.robot.chassis.mecanum_rpm(
                vx / 10.0, vy / 10.0, wz))

        print(f'[{self.operation_name}] Align tag {tag_id} at '
              f'{target_distance_mm:.0f} mm')
        try:
            while time.monotonic() - started < cfg.delivery_tag_align_timeout_s:
                self._check_active()
                now = time.monotonic()
                pose = self.robot.field_pose
                observation = self._delivery_tag_from_pose(
                    pose, tag_id,
                    vision_stale_s)
                if (pose is not None
                        and pose.timestamp <= first_valid_frame_after):
                    observation = None
                frame_timestamp = pose.timestamp if pose is not None else None
                if (observation is not None and (
                        frame_timestamp == last_frame_timestamp
                        or (independent_heading and now < next_translation_update))):
                    # Cached visual data never advances confirmation or the
                    # translation filter. Fresh IMU data still corrects yaw.
                    # Rejected frames must stay stopped until visual relock.
                    if independent_heading and translation_ready:
                        update_heading(now)
                    time.sleep(control_period)
                    continue
                if frame_timestamp is not None:
                    last_frame_timestamp = frame_timestamp

                if observation is None:
                    translation_ready = False
                    last_heading_update = now
                    confirmed = 0
                    fine_started = None
                    for axis in axis_holds:
                        axis.interrupt_confirmation()
                    if hold_axes:
                        last_update = now
                    translation_samples.clear()
                    vx = vy = wz = 0.0
                    pids.reset()
                    self.robot.chassis.set_speeds([0, 0, 0, 0])
                    if now - last_seen >= lost_timeout_s:
                        details = missing_tag_details(
                            pose, tag_id, now=time.time(),
                            max_age_s=vision_stale_s,
                            not_before=first_valid_frame_after)
                        raise VisualAlignmentUnavailable(
                            f'tag {tag_id} lost during delivery alignment; {details}')
                    time.sleep(control_period)
                    continue

                last_seen = now
                if independent_heading:
                    # Keep the existing 50 ms visual-control cadence; the
                    # 20 ms scheduler may quantize individual update times.
                    while next_translation_update <= now:
                        next_translation_update += cfg.delivery_tag_control_period_s
                dt = max(0.001, min(0.2, now - last_update))
                raw_distance_mm = observation.distance_m * 1000.0
                raw_lateral_mm = observation.lateral_m * 1000.0
                if last_translation is not None:
                    jump = translation_jump(
                        last_translation,
                        (raw_distance_mm, raw_lateral_mm),
                        cfg.delivery_tag_max_distance_jump_mm,
                        cfg.delivery_tag_max_lateral_jump_mm)
                    if jump is not None:
                        translation_ready = False
                        last_heading_update = now
                        distance_jump, lateral_jump = jump
                        for axis in axis_holds:
                            axis.interrupt_confirmation()
                        if hold_axes:
                            last_update = now
                        confirmed = 0
                        vx = vy = wz = 0.0
                        pids.reset()
                        self.robot.chassis.set_speeds([0, 0, 0, 0])
                        current = (raw_distance_mm, raw_lateral_mm)
                        if (relock_candidate is not None
                                and abs(current[0] - relock_candidate[0])
                                <= cfg.delivery_tag_max_distance_jump_mm
                                and abs(current[1] - relock_candidate[1])
                                <= cfg.delivery_tag_max_lateral_jump_mm):
                            relock_count += 1
                        else:
                            relock_candidate = current
                            relock_count = 1
                        print(f'[{self.operation_name}] Reject tag jump: '
                              f'd={distance_jump:.0f} mm, '
                              f'x={lateral_jump:.0f} mm '
                              f'(relock {relock_count}/3)')
                        if relock_count >= 3:
                            last_translation = current
                            translation_samples.clear()
                            relock_candidate = None
                            relock_count = 0
                            print(f'[{self.operation_name}] '
                                  'Tag translation relocked')
                        time.sleep(control_period)
                        continue
                last_translation = (raw_distance_mm, raw_lateral_mm)
                relock_candidate = None
                relock_count = 0
                translation_samples.append(
                    (raw_distance_mm, raw_lateral_mm))
                distance_mm, lateral_mm = median_translation(
                    translation_samples)
                translation_ready = True
                heading_error_deg = self._heading_error(
                    heading_target_cw_deg)
                distance_error = distance_mm - target_distance_mm
                distance_ok = (abs(distance_error)
                               <= distance_tolerance_mm)
                lateral_ok = (abs(lateral_mm)
                              <= lateral_tolerance_mm)
                heading_ok = (abs(heading_error_deg)
                              <= heading_tolerance_deg)

                stop_distance = stop_lateral = stop_heading = False
                if hold_axes:
                    # Only accepted, distinct frames reach this point. Keep the
                    # final confirmation based on measured tolerances, not holds.
                    stop_distance = axis_holds[0].update(distance_ok)
                    stop_lateral = axis_holds[1].update(lateral_ok)
                    if not independent_heading:
                        stop_heading = axis_holds[2].update(heading_ok)
                    if stop_distance:
                        vx = 0.0
                    if stop_lateral:
                        vy = 0.0
                    if stop_heading:
                        wz = 0.0

                within_tolerance = distance_ok and lateral_ok and heading_ok
                if within_tolerance and not fine_align_enabled:
                    # Tag6 keeps yaw hold active throughout confirmation.
                    # Legacy tag alignment still stops all axes here.
                    if not independent_heading:
                        self.robot.chassis.set_speeds([0, 0, 0, 0])
                        vx = vy = wz = 0.0
                        pids.reset()
                    confirmed += 1
                    print(f'[{self.operation_name}] Tag {tag_id} aligned '
                          f'{confirmed}/{cfg.delivery_tag_confirm_frames}: '
                          f'd={distance_mm:.0f} mm, x={lateral_mm:+.0f} mm, '
                          f'gyro={heading_error_deg:+.1f} deg')
                    if confirmed >= cfg.delivery_tag_confirm_frames:
                        return
                    if not independent_heading:
                        last_update = now
                        time.sleep(control_period)
                        continue
                precision_ok = (
                    abs(distance_error)
                    <= cfg.delivery_tag_distance_deadband_mm
                    and abs(lateral_mm)
                    <= cfg.delivery_tag_lateral_deadband_mm
                    and abs(heading_error_deg)
                    <= cfg.delivery_heading_deadband_deg)
                if within_tolerance and fine_align_enabled:
                    if fine_started is None:
                        fine_started = now
                        print(f'[{self.operation_name}] Tag within tolerance; '
                              'fine alignment')
                    if precision_ok:
                        confirmed += 1
                        print(f'[{self.operation_name}] Tag '
                              f'{tag_id} precision '
                              f'{confirmed}/{cfg.delivery_tag_confirm_frames}: '
                              f'd={distance_mm:.0f} mm, '
                              f'x={lateral_mm:+.0f} mm, '
                              f'gyro={heading_error_deg:+.1f} deg')
                    else:
                        confirmed = 0
                    if (confirmed >= cfg.delivery_tag_confirm_frames
                            or now - fine_started
                            >= cfg.delivery_tag_fine_align_timeout_s):
                        self.robot.chassis.set_speeds([0, 0, 0, 0])
                        if not precision_ok:
                            print(f'[{self.operation_name}] '
                                  'Fine alignment time reached; '
                                  'accepting position within tolerance')
                        return
                elif not within_tolerance:
                    confirmed = 0

                if (stop_distance or (not fine_align_enabled and distance_ok)
                        or abs(distance_error) <= cfg.delivery_tag_distance_deadband_mm):
                    distance_pid.reset()
                    desired_vx = 0.0
                else:
                    desired_vx = distance_pid.update(distance_error, dt)
                    if not distance_ok:
                        desired_vx = self._minimum_command(
                            desired_vx, cfg.delivery_tag_min_linear_mm_s)
                    else:
                        desired_vx *= fine_gain_scale
                    desired_vx = profiled_command(
                        desired_vx, distance_error,
                        slowdown_start=cfg.delivery_tag_slowdown_distance_mm,
                        creep_start=cfg.delivery_tag_creep_distance_mm,
                        fast_speed=cfg.delivery_tag_fast_forward_mm_s,
                        max_speed=cfg.delivery_tag_fast_forward_mm_s,
                        min_speed=cfg.delivery_tag_min_linear_mm_s)
                if (stop_lateral or (not fine_align_enabled and lateral_ok)
                        or abs(lateral_mm) <= cfg.delivery_tag_lateral_deadband_mm):
                    lateral_pid.reset()
                    desired_vy = 0.0
                else:
                    desired_vy = lateral_pid.update(lateral_mm, dt)
                    if not lateral_ok:
                        desired_vy = self._minimum_command(
                            desired_vy, cfg.delivery_tag_min_linear_mm_s)
                    else:
                        desired_vy *= fine_gain_scale
                    desired_vy = profiled_command(
                        desired_vy, lateral_mm,
                        slowdown_start=cfg.delivery_tag_slowdown_lateral_mm,
                        creep_start=cfg.delivery_tag_creep_lateral_mm,
                        fast_speed=cfg.delivery_tag_fast_lateral_mm_s,
                        max_speed=cfg.delivery_tag_fast_lateral_mm_s,
                        min_speed=cfg.delivery_tag_min_linear_mm_s)
                if not independent_heading:
                    if (stop_heading or (not fine_align_enabled and heading_ok)
                            or abs(heading_error_deg) <= cfg.delivery_heading_deadband_deg):
                        heading_pid.reset()
                        desired_wz = 0.0
                    else:
                        desired_wz = heading_pid.update(heading_error_deg, dt)
                        if not heading_ok:
                            desired_wz = self._minimum_command(
                                desired_wz, cfg.delivery_heading_min_yaw_deg_s)
                        else:
                            desired_wz *= fine_gain_scale
                vx = self._slew_command(
                    desired_vx, vx,
                    cfg.delivery_tag_linear_accel_mm_s2, dt)
                vy = self._slew_command(
                    desired_vy, vy,
                    cfg.delivery_tag_linear_accel_mm_s2, dt)
                if independent_heading:
                    update_heading(now)
                else:
                    wz = self._slew_command(
                        desired_wz, wz,
                        cfg.delivery_heading_yaw_accel_deg_s2, dt)
                    rpm = self.robot.chassis.mecanum_rpm(
                        vx / 10.0, vy / 10.0, wz)
                    self.robot.chassis.set_speeds(rpm)
                print(f'[{self.operation_name}] Tag PID: '
                      f'd={distance_mm:.0f} mm, '
                      f'x={lateral_mm:+.0f} mm, '
                      f'gyro={heading_error_deg:+.1f} deg; '
                      f'vx={vx:+.0f}, vy={vy:+.0f} mm/s, '
                      f'wz={wz:+.1f} deg/s')
                last_update = now
                time.sleep(control_period)
        finally:
            self.robot.chassis.set_speeds([0, 0, 0, 0])
        raise VisualAlignmentUnavailable(f'tag {tag_id} alignment timed out')

    def _align_delivery_tag_or_continue(self, **kwargs):
        try:
            self._align_delivery_tag(**kwargs)
            return True
        except VisualAlignmentUnavailable as exc:
            self._check_active()
            report_visual_fallback(self.robot, self.operation_name, 'tag alignment', exc)
            return False
