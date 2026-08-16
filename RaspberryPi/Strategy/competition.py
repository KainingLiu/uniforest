"""Top-level RoboGame competition state machine."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum, auto
from statistics import median
import time
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from robot import Robot
    from vision.cube_detector import BlockInfo
    from vision.field_localizer import TagSolution


# Preserve the physical stopping points tuned with the old 60-degree tag
# camera model after adopting the specified 125-degree horizontal FOV.
TAG_FOV_RETUNE_SCALE = 0.300549527


class _Pid:
    def __init__(self, kp: float, ki: float, kd: float,
                 integral_limit: float, output_limit: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.integral = 0.0
        self.previous_error = None

    def reset(self):
        self.integral = 0.0
        self.previous_error = None

    def update(self, error: float, dt: float) -> float:
        self.integral = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral + error * dt))
        derivative = (0.0 if self.previous_error is None else
                      (error - self.previous_error) / max(dt, 1e-6))
        self.previous_error = error
        output = (self.kp * error + self.ki * self.integral
                  + self.kd * derivative)
        return max(-self.output_limit, min(self.output_limit, output))


class CompetitionState(Enum):
    STARTUP = auto()
    READY = auto()
    INITIAL_MOVE = auto()
    WALL_APPROACH = auto()
    ORANGE_SEARCH = auto()
    ORANGE_ALIGN = auto()
    GRAB = auto()
    DELIVERY_ROUTE = auto()
    DELIVERY_TAG_ALIGN = auto()
    UNLOAD = auto()
    FINISHED = auto()
    FAULT = auto()


@dataclass(frozen=True)
class FirstTaskConfig:
    target_cube_count: int = 3
    initial_distance_mm: float = 1200.0
    initial_speed_mm_s: float = 500.0
    initial_hold_ms: int = 0
    wall_speed_mm_s: float = 150.0
    wall_timeout_s: float = 4.0
    wall_timeout_is_success: bool = True
    wall_settle_s: float = 0.3
    stall_startup_grace_s: float = 0.5
    stall_confirm_s: float = 0.3
    pre_grab_wall_timeout_s: float = 1.5
    pre_grab_stall_startup_grace_s: float = 0.1
    pre_grab_stall_confirm_s: float = 0.15
    pre_grab_wall_settle_s: float = 0.1
    telemetry_stale_s: float = 0.3
    stall_speed_rpm: int = 35
    stall_current_raw: int = 2500
    stall_motor_count: int = 3
    search_speed_mm_s: float = 180.0
    search_max_distance_mm: float = 1500.0
    search_control_period_s: float = 0.02
    vision_observe_s: float = 0.35
    vision_stale_s: float = 0.5
    orange_min_confidence: float = 25.0
    align_min_x_mm: float = -20.0
    align_max_x_mm: float = 5.0
    align_target_x_mm: float = 0.0
    align_confirm_frames: int = 3
    align_kp: float = 1.5
    align_ki: float = 0.10
    align_kd: float = 0.0
    align_integral_limit: float = 300.0
    align_min_speed_mm_s: float = 100.0
    align_max_speed_mm_s: float = 250.0
    align_accel_mm_s2: float = 300.0
    align_track_max_x_jump_mm: float = 80.0
    align_control_period_s: float = 0.05
    align_lost_timeout_s: float = 0.5
    align_timeout_s: float = 10.0
    post_grab_settle_s: float = 0.3
    delivery_reverse_mm: float = 400.0
    delivery_reverse_speed_mm_s: float = 300.0
    delivery_turn_deg: float = 90.0
    delivery_turn_speed_deg_s: float = 90.0
    delivery_turn_heading_hold_ms: int = 500
    delivery_forward_base_mm: float = 2800.0
    delivery_forward_speed_mm_s: float = 500.0
    delivery_tag_id: int = 6
    delivery_tag_distance_mm: float = 425.0
    delivery_tag_distance_tolerance_mm: float = 24.0 * TAG_FOV_RETUNE_SCALE
    delivery_tag_lateral_tolerance_mm: float = 20.0 * TAG_FOV_RETUNE_SCALE
    delivery_tag_distance_deadband_mm: float = 5.0 * TAG_FOV_RETUNE_SCALE
    delivery_tag_lateral_deadband_mm: float = 5.0 * TAG_FOV_RETUNE_SCALE
    delivery_heading_target_cw_deg: float = 180.0
    delivery_heading_tolerance_deg: float = 2.4
    delivery_heading_deadband_deg: float = 0.5
    delivery_tag_confirm_frames: int = 4
    delivery_tag_fine_align_timeout_s: float = 2.0
    delivery_tag_fine_gain_scale: float = 1.5
    delivery_tag_vision_stale_s: float = 0.3
    delivery_tag_lost_timeout_s: float = 1.0
    delivery_tag_align_timeout_s: float = 12.0
    delivery_tag_control_period_s: float = 0.05
    delivery_tag_translation_median_frames: int = 5
    delivery_tag_max_distance_jump_mm: float = 250.0 * TAG_FOV_RETUNE_SCALE
    delivery_tag_max_lateral_jump_mm: float = 250.0 * TAG_FOV_RETUNE_SCALE
    delivery_tag_distance_kp: float = 0.8 / TAG_FOV_RETUNE_SCALE
    delivery_tag_distance_ki: float = 0.02 / TAG_FOV_RETUNE_SCALE
    delivery_tag_distance_kd: float = 0.03 / TAG_FOV_RETUNE_SCALE
    delivery_tag_lateral_kp: float = 1.2 / TAG_FOV_RETUNE_SCALE
    delivery_tag_lateral_ki: float = 0.02 / TAG_FOV_RETUNE_SCALE
    delivery_tag_lateral_kd: float = 0.03 / TAG_FOV_RETUNE_SCALE
    delivery_heading_kp: float = 1.5
    delivery_heading_ki: float = 0.02
    delivery_heading_kd: float = 0.03
    delivery_tag_linear_integral_limit: float = (
        500.0 * TAG_FOV_RETUNE_SCALE)
    delivery_heading_integral_limit: float = 100.0
    delivery_tag_max_forward_mm_s: float = 250.0
    delivery_tag_max_lateral_mm_s: float = 200.0
    delivery_heading_max_yaw_deg_s: float = 45.0
    delivery_tag_min_linear_mm_s: float = 50.0
    delivery_heading_min_yaw_deg_s: float = 8.0
    delivery_tag_linear_accel_mm_s2: float = 300.0
    delivery_heading_yaw_accel_deg_s2: float = 90.0
    unload_wall_speed_mm_s: float = 200.0
    unload_wall_timeout_s: float = 4.0
    unload_reverse_mm: float = 300.0
    unload_reverse_speed_mm_s: float = 300.0
    unload_final_turn_cw_deg: float = 180.0
    unload_final_heading_hold_ms: int = 500
    delivery_linear_accel_ms: int = 200


class CompetitionProgram:
    """Owns competition flow; hardware details stay in Robot/control modules."""

    TELEMETRY_WAIT_S = 2.0
    TASK_LABEL = 'Task1'

    def __init__(self, robot: Robot,
                 config: FirstTaskConfig = FirstTaskConfig()):
        self.robot = robot
        self.config = config
        self.state = CompetitionState.STARTUP
        self._search_position_mm = 0.0
        self._cube_lateral_displacement_mm: Optional[float] = None
        self._heading_zero_deg: Optional[float] = None

    def _preflight(self):
        deadline = time.monotonic() + self.TELEMETRY_WAIT_S
        while self.robot.telem is None and time.monotonic() < deadline:
            time.sleep(0.02)
        if self.robot.telem is None:
            raise RuntimeError('A-board telemetry unavailable')
        if not self.robot.has_vision:
            raise RuntimeError('vision subsystem unavailable')
        if not self.robot.has_field_localization:
            raise RuntimeError('field localization subsystem unavailable')

        self._heading_zero_deg = self.robot.telem.yaw_deg
        self.state = CompetitionState.READY
        print('[Competition] Preflight complete; heading zero='
              f'{self._heading_zero_deg:+.1f} deg')

    @staticmethod
    def _orange_from_result(result, min_confidence: float,
                            max_age_s: float) -> Optional['BlockInfo']:
        return CompetitionProgram._block_from_result(
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
                                    cfg: FirstTaskConfig):
        return CompetitionProgram._tracked_block_from_result(
            result, 'orange', reference_x, cfg.orange_min_confidence, cfg)

    @staticmethod
    def _tracked_block_from_result(result, color_name: str,
                                   reference_x: float,
                                   min_confidence: float,
                                   cfg: FirstTaskConfig):
        if result is None or time.time() - result.timestamp > cfg.vision_stale_s:
            return None
        candidates = [
            block for block in result.all_blocks
            if block.color_name.casefold() == color_name.casefold()
            and block.confidence >= min_confidence
            and abs(block.x - reference_x) <= cfg.align_track_max_x_jump_mm
        ]
        if not candidates:
            return None
        return min(candidates, key=lambda block: abs(block.x - reference_x))

    @staticmethod
    def _stall_sample(telem, cfg: FirstTaskConfig) -> bool:
        stalled = sum(
            abs(m.speed_rpm) <= cfg.stall_speed_rpm
            and abs(m.torque_current) >= cfg.stall_current_raw
            for m in telem.motors
        )
        return stalled >= cfg.stall_motor_count

    def _drive_until_wall(self, *, timeout_s: Optional[float] = None,
                          startup_grace_s: Optional[float] = None,
                          confirm_s: Optional[float] = None,
                          speed_mm_s: Optional[float] = None,
                          direction: str = 'forward',
                          context: str = 'Wall contact',
                          timeout_is_success: Optional[bool] = None):
        cfg = self.config
        timeout_s = cfg.wall_timeout_s if timeout_s is None else timeout_s
        startup_grace_s = (cfg.stall_startup_grace_s
                           if startup_grace_s is None else startup_grace_s)
        confirm_s = cfg.stall_confirm_s if confirm_s is None else confirm_s
        speed_mm_s = cfg.wall_speed_mm_s if speed_mm_s is None else speed_mm_s
        timeout_is_success = (
            cfg.wall_timeout_is_success if timeout_is_success is None
            else timeout_is_success)
        speed_cm_s = speed_mm_s / 10.0
        velocity = {
            'forward': (speed_cm_s, 0.0),
            'backward': (-speed_cm_s, 0.0),
            'left': (0.0, -speed_cm_s),
            'right': (0.0, speed_cm_s),
        }
        if direction not in velocity:
            raise ValueError(
                'wall direction must be forward, backward, left, or right')
        vx_cm_s, vy_cm_s = velocity[direction]
        rpm = self.robot.chassis.mecanum_rpm(vx_cm_s, vy_cm_s, 0.0)
        started = time.monotonic()
        stall_since = None
        last_uptime = None
        last_telem_time = time.monotonic()
        try:
            while time.monotonic() - started < timeout_s:
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
                              and self._stall_sample(telem, cfg))
                if is_stalled:
                    if stall_since is None:
                        stall_since = time.monotonic()
                    elif time.monotonic() - stall_since >= confirm_s:
                        print(f'[{self.TASK_LABEL}] {context} confirmed')
                        return
                else:
                    stall_since = None
                time.sleep(0.01)
        finally:
            self.robot.chassis.set_speeds([0, 0, 0, 0])
        if timeout_is_success:
            print(f'[{self.TASK_LABEL}] {context} timed out after '
                  f'{timeout_s:.1f}s; accepting wall contact')
            return
        raise RuntimeError('wall contact was not detected before timeout')

    def _recalibrate_heading_zero(self):
        telem = self.robot.telem
        if telem is None:
            raise RuntimeError('telemetry unavailable for heading recalibration')
        previous_zero = self._heading_zero_deg
        self._heading_zero_deg = telem.yaw_deg
        correction = (0.0 if previous_zero is None else
                      self._wrap_angle(self._heading_zero_deg - previous_zero))
        print(f'[{self.TASK_LABEL}] Heading zero recalibrated: '
              f'{self._heading_zero_deg:+.1f} deg '
              f'(correction {correction:+.1f} deg)')

    def _press_wall_before_grab(self, *, recalibrate_heading_zero=False):
        cfg = self.config
        print(f'[{self.TASK_LABEL}] Press wall before grab')
        self._drive_until_wall(
            timeout_s=cfg.pre_grab_wall_timeout_s,
            startup_grace_s=cfg.pre_grab_stall_startup_grace_s,
            confirm_s=cfg.pre_grab_stall_confirm_s,
            context='Pre-grab wall contact',
        )
        time.sleep(cfg.pre_grab_wall_settle_s)
        if recalibrate_heading_zero:
            self._recalibrate_heading_zero()

    def _checked_move(self, direction: str, distance_mm: float,
                      speed_mm_s: float):
        result = self.robot.move_chassis(
            direction, distance_mm, speed_mm_s,
            hold_ms=0, accel_ms=self.config.delivery_linear_accel_ms)
        if result.timed_out or result.cancelled:
            raise RuntimeError(
                f'chassis move failed: {direction} {distance_mm:.0f} mm')
        return result

    def _capture_lateral_origin(self):
        return self.robot.chassis.capture_motor_positions()

    def _measure_lateral_displacement_mm(self, origin) -> float:
        return self.robot.chassis.lateral_displacement_mm(origin)

    def _observe_orange(self) -> Optional['BlockInfo']:
        cfg = self.config
        deadline = time.monotonic() + cfg.vision_observe_s
        best = None
        while time.monotonic() < deadline:
            block = self._orange_from_result(
                self.robot.vision_result,
                cfg.orange_min_confidence,
                cfg.vision_stale_s,
            )
            if block is not None and (best is None
                                      or block.confidence > best.confidence):
                best = block
            time.sleep(0.02)
        return best

    def _find_orange(self) -> 'BlockInfo':
        return self._find_cube(
            color_name='orange', min_confidence=self.config.orange_min_confidence,
            search_direction=1.0)

    def _find_cube(self, *, color_name: str, min_confidence: float,
                   search_direction: float) -> 'BlockInfo':
        cfg = self.config
        remaining_mm = cfg.search_max_distance_mm - self._search_position_mm
        if remaining_mm <= 0.0:
            raise RuntimeError(
                f'{color_name} cube not found within search range')

        rpm = self.robot.chassis.mecanum_rpm(
            0.0, search_direction * cfg.search_speed_mm_s / 10.0, 0.0)
        started = time.monotonic()
        deadline = started + remaining_mm / cfg.search_speed_mm_s
        direction_name = 'right' if search_direction > 0.0 else 'left'
        display_color = color_name.capitalize()
        print(f'[{self.TASK_LABEL}] {display_color} not visible; '
              f'continuous search {direction_name} '
              f'from {self._search_position_mm:.0f}/'
              f'{cfg.search_max_distance_mm:.0f} mm')
        try:
            while time.monotonic() < deadline:
                self.robot.chassis.set_speeds(rpm)
                block = self._block_from_result(
                    self.robot.vision_result, color_name,
                    min_confidence, cfg.vision_stale_s)
                if block is not None:
                    elapsed_s = time.monotonic() - started
                    self._search_position_mm += min(
                        remaining_mm, elapsed_s * cfg.search_speed_mm_s)
                    print(f'[{self.TASK_LABEL}] {display_color} acquired: '
                          f'x={block.x:+.0f} mm, '
                          f'confidence={block.confidence:.0f}%, '
                          f'position={self._search_position_mm:.0f} mm')
                    return block
                time.sleep(cfg.search_control_period_s)
        finally:
            self.robot.chassis.set_speeds([0, 0, 0, 0])

        self._search_position_mm = cfg.search_max_distance_mm
        raise RuntimeError(f'{color_name} cube not found within search range')

    @staticmethod
    def _alignment_speed(x_error: float, integral: float,
                         derivative: float, cfg: FirstTaskConfig) -> float:
        speed = (cfg.align_kp * x_error
                 + cfg.align_ki * integral
                 + cfg.align_kd * derivative)
        speed = max(-cfg.align_max_speed_mm_s,
                    min(cfg.align_max_speed_mm_s, speed))
        if 0.0 < abs(speed) < cfg.align_min_speed_mm_s:
            speed = cfg.align_min_speed_mm_s if speed > 0.0 else -cfg.align_min_speed_mm_s
        return speed

    @staticmethod
    def _slew_alignment_speed(desired: float, previous: float, dt: float,
                              cfg: FirstTaskConfig) -> float:
        if desired == 0.0:
            return 0.0
        if previous == 0.0:
            return (cfg.align_min_speed_mm_s
                    if desired > 0.0 else -cfg.align_min_speed_mm_s)
        if desired * previous < 0.0:
            return 0.0
        max_delta = cfg.align_accel_mm_s2 * dt
        delta = max(-max_delta, min(max_delta, desired - previous))
        return previous + delta

    def _alignment_observation(self, reference_x: float):
        result = self.robot.vision_result
        block = self._tracked_orange_from_result(
            result, reference_x, self.config)
        timestamp = result.timestamp if result is not None else None
        return block, timestamp

    def _align_orange(self, initial_block: 'BlockInfo') -> bool:
        return self._align_cube(
            initial_block, color_name='orange',
            min_confidence=self.config.orange_min_confidence)

    def _align_cube(self, initial_block: 'BlockInfo', *, color_name: str,
                    min_confidence: float,
                    align_min_x_mm: Optional[float] = None,
                    align_max_x_mm: Optional[float] = None,
                    align_target_x_mm: Optional[float] = None) -> bool:
        """Continuously center a colored cube; return False after target loss."""
        cfg = self.config
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
        reference_x = initial_block.x
        commanded_speed = 0.0
        try:
            while time.monotonic() - started < cfg.align_timeout_s:
                now = time.monotonic()
                result = self.robot.vision_result
                block = self._tracked_block_from_result(
                    result, color_name, reference_x, min_confidence, cfg)
                frame_timestamp = result.timestamp if result is not None else None

                if (block is not None
                        and frame_timestamp is not None
                        and frame_timestamp == last_frame_timestamp):
                    time.sleep(cfg.align_control_period_s)
                    continue
                if frame_timestamp is not None:
                    last_frame_timestamp = frame_timestamp

                if block is None:
                    confirmed = 0
                    integral = 0.0
                    previous_error = None
                    self.robot.chassis.set_speeds([0, 0, 0, 0])
                    if now - last_seen >= cfg.align_lost_timeout_s:
                        print(f'[{self.TASK_LABEL}] {display_color} lost; '
                              'resume search')
                        return False
                    time.sleep(cfg.align_control_period_s)
                    continue

                last_seen = now
                reference_x = block.x
                x_error = block.x - align_target_x_mm
                if align_min_x_mm <= block.x <= align_max_x_mm:
                    self.robot.chassis.set_speeds([0, 0, 0, 0])
                    commanded_speed = 0.0
                    integral = 0.0
                    previous_error = None
                    confirmed += 1
                    print(f'[{self.TASK_LABEL}] Alignment sample {confirmed}/'
                          f'{cfg.align_confirm_frames}: x={block.x:+.0f} mm')
                    if confirmed >= cfg.align_confirm_frames:
                        return True
                else:
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
                        x_error, integral, derivative, cfg)
                    commanded_speed = self._slew_alignment_speed(
                        desired_speed, commanded_speed, dt, cfg)
                    rpm = self.robot.chassis.mecanum_rpm(
                        0.0, commanded_speed / 10.0, 0.0)
                    self.robot.chassis.set_speeds(rpm)
                    print(f'[{self.TASK_LABEL}] Visual PID: '
                          f'x={block.x:+.0f} mm, '
                          f'vy={commanded_speed:+.0f} mm/s')
                    previous_error = x_error
                last_update = now
                time.sleep(cfg.align_control_period_s)
        finally:
            self.robot.chassis.set_speeds([0, 0, 0, 0])
        raise RuntimeError(f'{color_name} visual alignment timed out')

    def _run_first_task(self):
        cfg = self.config
        self.state = CompetitionState.INITIAL_MOVE
        print('[Task1] Initial forward move')
        result = self.robot.move_chassis(
            'forward', cfg.initial_distance_mm, cfg.initial_speed_mm_s,
            hold_ms=cfg.initial_hold_ms)
        if result.timed_out or result.cancelled:
            raise RuntimeError('initial position move did not complete')

        self.state = CompetitionState.WALL_APPROACH
        print('[Task1] Slow approach until motor stall')
        self._drive_until_wall()
        time.sleep(cfg.wall_settle_s)
        self._recalibrate_heading_zero()
        self.robot.reset_vision_filter()

        self._search_position_mm = 0.0
        lateral_origin = self._capture_lateral_origin()
        for cube_index in range(1, cfg.target_cube_count + 1):
            print(f'[Task1] Cube {cube_index}/{cfg.target_cube_count}')
            while True:
                self.state = CompetitionState.ORANGE_SEARCH
                block = self._find_orange()
                self.state = CompetitionState.ORANGE_ALIGN
                if self._align_orange(block):
                    break

            self.state = CompetitionState.WALL_APPROACH
            self._press_wall_before_grab(recalibrate_heading_zero=True)

            self.state = CompetitionState.GRAB
            print(f'[Task1] Orange {cube_index}/{cfg.target_cube_count} '
                  f'aligned; running Grap3')
            self.robot.actions.grap3()
            print(f'[Task1] Grap3 {cube_index}/{cfg.target_cube_count} '
                  f'complete')
            self.robot.reset_vision_filter()
            time.sleep(cfg.post_grab_settle_s)

        self._cube_lateral_displacement_mm = (
            self._measure_lateral_displacement_mm(lateral_origin))
        print('[Task1] Encoder-measured cube lateral displacement: '
              f'{self._cube_lateral_displacement_mm:+.0f} mm '
              '(right positive)')

    @staticmethod
    def _wrap_angle(angle_deg: float) -> float:
        return (angle_deg + 180.0) % 360.0 - 180.0

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
        error_ccw_deg = self._heading_error(target_cw_deg)
        clockwise_delta_deg = -error_ccw_deg
        print(f'[{self.TASK_LABEL}] Gyro heading target '
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
        if value == 0.0 or abs(value) >= minimum:
            return value
        return minimum if value > 0.0 else -minimum

    @staticmethod
    def _slew_command(target: float, current: float,
                      max_rate: float, dt: float) -> float:
        max_delta = max_rate * dt
        delta = max(-max_delta, min(max_delta, target - current))
        return current + delta

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
            fine_gain_scale: Optional[float] = None):
        """Use a tag for translation while holding startup-relative yaw."""
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
            cfg.delivery_heading_kp,
            cfg.delivery_heading_ki,
            cfg.delivery_heading_kd,
            cfg.delivery_heading_integral_limit,
            cfg.delivery_heading_max_yaw_deg_s)
        pids = (distance_pid, lateral_pid, heading_pid)
        started = time.monotonic()
        first_valid_frame_after = time.time()
        last_seen = started
        last_update = started
        last_frame_timestamp = None
        last_translation = None
        relock_candidate = None
        relock_count = 0
        translation_samples = deque(
            maxlen=cfg.delivery_tag_translation_median_frames)
        fine_started = None
        confirmed = 0
        vx = vy = wz = 0.0
        print(f'[{self.TASK_LABEL}] Align tag {tag_id} at '
              f'{target_distance_mm:.0f} mm')
        try:
            while time.monotonic() - started < cfg.delivery_tag_align_timeout_s:
                now = time.monotonic()
                pose = self.robot.field_pose
                observation = self._delivery_tag_from_pose(
                    pose, tag_id,
                    cfg.delivery_tag_vision_stale_s)
                if (pose is not None
                        and pose.timestamp <= first_valid_frame_after):
                    observation = None
                frame_timestamp = pose.timestamp if pose is not None else None
                if (observation is not None
                        and frame_timestamp is not None
                        and frame_timestamp == last_frame_timestamp):
                    time.sleep(cfg.delivery_tag_control_period_s)
                    continue
                if frame_timestamp is not None:
                    last_frame_timestamp = frame_timestamp

                if observation is None:
                    confirmed = 0
                    fine_started = None
                    translation_samples.clear()
                    vx = vy = wz = 0.0
                    for pid in pids:
                        pid.reset()
                    self.robot.chassis.set_speeds([0, 0, 0, 0])
                    if now - last_seen >= cfg.delivery_tag_lost_timeout_s:
                        raise RuntimeError(
                            f'tag {tag_id} lost during delivery alignment')
                    time.sleep(cfg.delivery_tag_control_period_s)
                    continue

                last_seen = now
                dt = max(0.001, min(0.2, now - last_update))
                raw_distance_mm = observation.distance_m * 1000.0
                raw_lateral_mm = observation.lateral_m * 1000.0
                if last_translation is not None:
                    distance_jump = abs(
                        raw_distance_mm - last_translation[0])
                    lateral_jump = abs(
                        raw_lateral_mm - last_translation[1])
                    if (distance_jump > cfg.delivery_tag_max_distance_jump_mm
                            or lateral_jump
                            > cfg.delivery_tag_max_lateral_jump_mm):
                        confirmed = 0
                        vx = vy = wz = 0.0
                        for pid in pids:
                            pid.reset()
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
                        print(f'[{self.TASK_LABEL}] Reject tag jump: '
                              f'd={distance_jump:.0f} mm, '
                              f'x={lateral_jump:.0f} mm '
                              f'(relock {relock_count}/3)')
                        if relock_count >= 3:
                            last_translation = current
                            translation_samples.clear()
                            relock_candidate = None
                            relock_count = 0
                            print(f'[{self.TASK_LABEL}] '
                                  'Tag translation relocked')
                        time.sleep(cfg.delivery_tag_control_period_s)
                        continue
                last_translation = (raw_distance_mm, raw_lateral_mm)
                relock_candidate = None
                relock_count = 0
                translation_samples.append(
                    (raw_distance_mm, raw_lateral_mm))
                distance_mm = median(
                    item[0] for item in translation_samples)
                lateral_mm = median(
                    item[1] for item in translation_samples)
                heading_error_deg = self._heading_error(
                    heading_target_cw_deg)
                distance_error = distance_mm - target_distance_mm
                distance_ok = (abs(distance_error)
                               <= distance_tolerance_mm)
                lateral_ok = (abs(lateral_mm)
                              <= lateral_tolerance_mm)
                heading_ok = (abs(heading_error_deg)
                              <= heading_tolerance_deg)

                within_tolerance = distance_ok and lateral_ok and heading_ok
                precision_ok = (
                    abs(distance_error)
                    <= cfg.delivery_tag_distance_deadband_mm
                    and abs(lateral_mm)
                    <= cfg.delivery_tag_lateral_deadband_mm
                    and abs(heading_error_deg)
                    <= cfg.delivery_heading_deadband_deg)
                if within_tolerance:
                    if fine_started is None:
                        fine_started = now
                        print(f'[{self.TASK_LABEL}] Tag within tolerance; '
                              'fine alignment')
                    if precision_ok:
                        confirmed += 1
                        print(f'[{self.TASK_LABEL}] Tag '
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
                            print(f'[{self.TASK_LABEL}] '
                                  'Fine alignment time reached; '
                                  'accepting position within tolerance')
                        return
                else:
                    confirmed = 0

                if (abs(distance_error)
                        <= cfg.delivery_tag_distance_deadband_mm):
                    distance_pid.reset()
                    desired_vx = 0.0
                else:
                    desired_vx = distance_pid.update(distance_error, dt)
                    if not distance_ok:
                        desired_vx = self._minimum_command(
                            desired_vx, cfg.delivery_tag_min_linear_mm_s)
                    else:
                        desired_vx *= fine_gain_scale
                if (abs(lateral_mm)
                        <= cfg.delivery_tag_lateral_deadband_mm):
                    lateral_pid.reset()
                    desired_vy = 0.0
                else:
                    desired_vy = lateral_pid.update(lateral_mm, dt)
                    if not lateral_ok:
                        desired_vy = self._minimum_command(
                            desired_vy, cfg.delivery_tag_min_linear_mm_s)
                    else:
                        desired_vy *= fine_gain_scale
                if (abs(heading_error_deg)
                        <= cfg.delivery_heading_deadband_deg):
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
                wz = self._slew_command(
                    desired_wz, wz,
                    cfg.delivery_heading_yaw_accel_deg_s2, dt)
                rpm = self.robot.chassis.mecanum_rpm(
                    vx / 10.0, vy / 10.0, wz)
                self.robot.chassis.set_speeds(rpm)
                print(f'[{self.TASK_LABEL}] Tag PID: '
                      f'd={distance_mm:.0f} mm, '
                      f'x={lateral_mm:+.0f} mm, '
                      f'gyro={heading_error_deg:+.1f} deg; '
                      f'vx={vx:+.0f}, vy={vy:+.0f} mm/s, '
                      f'wz={wz:+.1f} deg/s')
                last_update = now
                time.sleep(cfg.delivery_tag_control_period_s)
        finally:
            self.robot.chassis.set_speeds([0, 0, 0, 0])
        raise RuntimeError(f'tag {tag_id} alignment timed out')

    def _run_delivery_route(self):
        cfg = self.config
        self.state = CompetitionState.DELIVERY_ROUTE
        if self._cube_lateral_displacement_mm is None:
            raise RuntimeError('cube lateral encoder measurement unavailable')
        delivery_forward_mm = (
            cfg.delivery_forward_base_mm
            - self._cube_lateral_displacement_mm)
        if delivery_forward_mm <= 0.0:
            raise RuntimeError(
                'delivery forward distance must be positive: '
                f'{cfg.delivery_forward_base_mm:.0f} - '
                f'{self._cube_lateral_displacement_mm:.0f} = '
                f'{delivery_forward_mm:.0f} mm')

        print(f'[Task1] Delivery: reverse {cfg.delivery_reverse_mm:.0f} mm')
        self._checked_move(
            'backward', cfg.delivery_reverse_mm,
            cfg.delivery_reverse_speed_mm_s)

        self._turn_to_heading(
            cfg.delivery_turn_deg,
            hold_ms=cfg.delivery_turn_heading_hold_ms)

        print(f'[Task1] Delivery: forward {delivery_forward_mm:.0f} mm '
              f'({cfg.delivery_forward_base_mm:.0f} calibration base - '
              f'encoder lateral '
              f'{self._cube_lateral_displacement_mm:.0f} mm)')
        self._checked_move(
            'forward', delivery_forward_mm,
            cfg.delivery_forward_speed_mm_s)

        self._turn_to_heading(cfg.delivery_turn_deg * 2.0)

        self.state = CompetitionState.DELIVERY_TAG_ALIGN
        self.robot.reset_field_localization_filter()
        self._align_delivery_tag()

        self.state = CompetitionState.WALL_APPROACH
        print('[Task1] Delivery: approach unload wall')
        self._drive_until_wall(
            timeout_s=cfg.unload_wall_timeout_s,
            speed_mm_s=cfg.unload_wall_speed_mm_s,
            context='Unload wall contact',
        )

        self.state = CompetitionState.UNLOAD
        print('[Task1] Unload: open hatches')
        self.robot.actions.hatch_open()
        self._checked_move(
            'backward', cfg.unload_reverse_mm,
            cfg.unload_reverse_speed_mm_s)
        print('[Task1] Unload: close hatches')
        self.robot.actions.hatch_close()
        self._turn_to_heading(
            cfg.delivery_heading_target_cw_deg
            + cfg.unload_final_turn_cw_deg,
            hold_ms=cfg.unload_final_heading_hold_ms)

    def _run_mission(self):
        self._run_first_task()
        self._run_delivery_route()

    def run(self) -> int:
        try:
            self._preflight()
            self._run_mission()
            self.state = CompetitionState.FINISHED
            return 0
        except Exception:
            self.state = CompetitionState.FAULT
            self.robot.transport.emergency_stop()
            raise
