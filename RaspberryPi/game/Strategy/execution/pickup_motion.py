"""Continue an orange acquisition from the measured arm-return velocity.

The caller retains the chassis owner and the collection's encoder origin. This
controller never restarts the search budget or invalidates the restored camera.
Only an explicitly validated motion profile enables the moving handoff. Existing
camera-X targets remain the source of alignment calibration.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
import math
import time

from control.chassis import MECANUM_RPM_PER_CM_S
from ..common import wrap_angle
from ..cube_tracker import CubeTargetTracker
from .blind import BlindMotionFault, _finite


@dataclass(frozen=True)
class PickupMotionProfile:
    acceleration_mm_s2: float
    braking_mm_s2: float
    max_distance_mm: float
    max_duration_s: float
    braking_margin_mm: float
    telemetry_timeout_s: float
    frame_timeout_s: float
    tick_s: float
    max_command_delay_s: float
    settled_speed_mm_s: float
    validated: bool = False
    trial_enabled: bool = False

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            if name not in ('validated','trial_enabled') and (
                    not _finite(getattr(self, name)) or getattr(self, name) <= 0):
                raise ValueError(f'{name} must be finite and positive')
        if self.braking_margin_mm >= self.max_distance_mm:
            raise ValueError('braking margin must leave positive travel distance')
        if (self.tick_s + self.max_command_delay_s >= self.max_duration_s
                or self.tick_s > self.telemetry_timeout_s):
            raise ValueError('control tick must fit duration and telemetry limits')
        if type(self.validated) is not bool:
            raise ValueError('validated must be an explicit boolean')
        if type(self.trial_enabled) is not bool:
            raise ValueError('trial_enabled must be an explicit boolean')


def measured_lateral_speed(robot):
    """Encoder wheel speed projected onto the calibrated chassis lateral axis."""
    telem = robot.telem
    if telem is None or len(telem.motors) != 4:
        raise BlindMotionFault('pickup telemetry unavailable')
    rpm = [motor.speed_rpm for motor in telem.motors]
    scale = robot.chassis.lateral_distance_scale
    if not all(_finite(value) for value in (*rpm, scale)) or scale <= 0:
        raise BlindMotionFault('invalid pickup wheel velocity')
    return (rpm[0] + rpm[1] - rpm[2] - rpm[3]) / 4 * 10 / MECANUM_RPM_PER_CM_S / scale


def _slew(desired, previous, dt, profile):
    # A direction change goes through zero; braking and acceleration may differ.
    if desired * previous < 0:
        desired = 0.0
    slowing = abs(desired) < abs(previous)
    rate = profile.braking_mm_s2 if slowing else profile.acceleration_mm_s2
    change = max(-rate * dt, min(rate * dt, desired - previous))
    return previous + change


def _velocity_cap(remaining, age, profile):
    usable = remaining - profile.braking_margin_mm
    if usable <= 0:
        return 0.0
    latency = age + profile.tick_s + profile.max_command_delay_s
    brake = profile.braking_mm_s2
    return math.sqrt((brake * latency) ** 2 + 2 * brake * usable) - brake * latency


def acquire_after_blind(control, *, initial_speed_mm_s, guard, pose_epoch,
                        arm_reset_at_s, phase_origin, profile,
                        heading_yaw_deg=None, command_speed=None, stop=None,
                        clock=time.monotonic, sleep=time.sleep, alignment=None,
                        neighbor_observer=None):
    """Acquire and align one orange cube, starting at the handed-off velocity.

    Return True only after fresh target confirmations at measured low speed.
    False means a bounded perception/search failure and the chassis has received
    a stop. Transport, stale telemetry, cancellation, invalid motion and command
    failures raise after stopping. These exceptions must abort the parent flow.

    ``max_distance_mm`` is the absolute rightward limit measured from the
    collection's original encoder origin, including the preceding blind move.
    Optional command/stop callbacks allow a blind adapter to retain the same
    velocity/heading controller without inserting a zero between controllers.
    They must cooperatively honor ``guard``. Frame timestamps are host camera
    acquisition times, never detector processing-completion times.

    Left-edge recovery remains in the normal acquisition path. A lost locked
    target causes bounded braking/reacquisition, then returns False to that path
    with its OrangeSearchRecovery state and cumulative budgets preserved.
    """
    if not isinstance(profile, PickupMotionProfile) or not (profile.validated or profile.trial_enabled):
        raise ValueError('pickup motion requires a field-validated profile')
    if not _finite(initial_speed_mm_s):
        raise ValueError('initial speed must be finite')
    if type(pose_epoch) is not int or pose_epoch < 0 or not _finite(arm_reset_at_s):
        raise ValueError('restored camera pose requires epoch and monotonic time')
    for callback in (guard, clock, sleep):
        if not callable(callback):
            raise TypeError('guard, clock and sleep must be callable')
    if command_speed is not None and not callable(command_speed):
        raise TypeError('command_speed must be callable')
    if stop is not None and not callable(stop):
        raise TypeError('stop must be callable')

    if alignment is not None:
        from ..optimizations.fast_alignment import align_cube
        return align_cube(control, color_name='orange', phase_origin=phase_origin,
                          profile=alignment, guard=guard, initial_speed_mm_s=initial_speed_mm_s,
                          pose_epoch=pose_epoch, arm_reset_at_s=arm_reset_at_s,
                          search_profile=profile, heading_yaw_deg=heading_yaw_deg,
                          command_speed=command_speed, stop=stop, clock=clock, sleep=sleep,
                          neighbor_observer=neighbor_observer)

    robot, cfg, recovery = control.robot, control.config, control._orange_recovery
    stop = stop or (lambda: robot.chassis.set_speeds([0, 0, 0, 0]))
    primary_error = None
    control._last_alignment_block = None
    control._last_alignment_frame_timestamp = None
    control._last_alignment_timed_out = False
    control._alignment_valid_frames = 0

    def check():
        control._check_active()
        if guard() is False:
            raise BlindMotionFault('pickup motion guard rejected execution')

    try:
        check()
        started = previous_time = clock()
        if not _finite(started) or arm_reset_at_s > started:
            raise BlindMotionFault('invalid pickup monotonic clock or arm reset time')
        telem = robot.telem
        if telem is None:
            raise BlindMotionFault('pickup telemetry unavailable')
        if heading_yaw_deg is None:
            heading_yaw_deg = telem.yaw_deg
        if not _finite(heading_yaw_deg):
            raise BlindMotionFault('pickup heading unavailable')
        generation = robot.transport.emergency_stop_generation
        last_uptime, last_telem_at = telem.uptime_ms, started
        position = previous_position = control._measure_lateral_displacement_mm(phase_origin)
        limit = min(profile.max_distance_mm, cfg.search_max_distance_mm)
        control._search_position_mm = max(control._search_position_mm, position)
        phase_time_limit = (cfg.search_max_distance_mm / cfg.search_speed_mm_s
                            + cfg.orange_edge_timeout_s
                            + cfg.target_cube_count * cfg.vision_observe_s)
        speed = initial_speed_mm_s
        tracker = CubeTargetTracker(
            max_x_jump_mm=cfg.align_track_max_x_jump_mm,
            max_z_jump_mm=cfg.align_track_max_z_jump_mm,
            confirm_frames=cfg.orange_search_confirm_frames,
            lost_frames=cfg.orange_search_lock_lost_frames,
            smoothing_frames=cfg.align_filter_frames,
            ambiguity_margin_mm=getattr(cfg, 'orange_track_ambiguity_margin_mm',
                                        cfg.align_track_ambiguity_margin_mm))
        target_x = getattr(cfg, 'orange_align_target_x_mm', cfg.align_target_x_mm)
        low_x, high_x = cfg.orange_fine_min_x_mm, cfg.orange_fine_max_x_mm
        last_frame = None
        last_seen = None
        last_usable_frame = started
        target_block = None
        confirmed = 0
        integral = 0.0
        previous_error = None
        desired = initial_speed_mm_s

        def send(value):
            if command_speed is not None:
                return command_speed(value)
            yaw = robot.telem.yaw_deg
            if not _finite(yaw):
                raise BlindMotionFault('invalid pickup heading telemetry')
            correction = max(-cfg.delivery_heading_max_yaw_deg_s,
                             min(cfg.delivery_heading_max_yaw_deg_s,
                                 cfg.tag6_heading_kp * wrap_angle(heading_yaw_deg - yaw)))
            return robot.chassis.set_speeds(
                robot.chassis.mecanum_rpm(0.0, value / 10.0, correction))

        while True:
            check()
            now = clock()
            if not _finite(now) or now < previous_time:
                raise BlindMotionFault('pickup monotonic clock regressed')
            dt = now - previous_time
            if not robot.transport.connected:
                raise BlindMotionFault('communication lost during pickup motion')
            if robot.transport.emergency_stop_generation != generation:
                raise BlindMotionFault('pickup motion cancelled by emergency stop')
            telem = robot.telem
            if telem is None:
                raise BlindMotionFault('pickup telemetry unavailable')
            if telem.uptime_ms != last_uptime:
                # uint32 rollover is permitted; a reboot invalidates continuity.
                advance = (telem.uptime_ms - last_uptime) & 0xffffffff
                if advance >= 0x80000000:
                    raise BlindMotionFault('pickup telemetry uptime regressed')
                last_uptime, last_telem_at = telem.uptime_ms, now
            age = now - last_telem_at
            if age >= profile.telemetry_timeout_s:
                raise BlindMotionFault('stale telemetry during pickup motion')
            measured_speed = measured_lateral_speed(robot)
            position = control._measure_lateral_displacement_mm(phase_origin)
            if not _finite(position) or position < 0:
                raise BlindMotionFault('pickup moved outside original collection origin')
            control._search_position_mm += max(0.0, position - previous_position)
            recovery.search_elapsed_s += dt
            previous_position = position
            if (now - started >= profile.max_duration_s
                    or recovery.search_elapsed_s >= phase_time_limit):
                control._last_alignment_timed_out = True
                return False
            right_remaining = min(limit - position, limit - control._search_position_mm)
            left_remaining = position
            positive_cap = _velocity_cap(right_remaining, age, profile)
            negative_cap = _velocity_cap(left_remaining, age, profile)
            bound_speed = max(abs(measured_speed), abs(speed))
            remaining = right_remaining if measured_speed >= 0 else left_remaining
            if (remaining <= profile.braking_margin_mm
                    or bound_speed ** 2 / (2 * profile.braking_mm_s2)
                    + bound_speed * (age + profile.max_command_delay_s)
                    >= remaining - profile.braking_margin_mm):
                return False

            result = robot.vision_result
            captured = getattr(result, 'captured_monotonic', None)
            fresh = (result is not None and type(getattr(result, 'pose_epoch', None)) is int
                     and result.pose_epoch == pose_epoch and _finite(captured)
                     and arm_reset_at_s < captured <= now
                     and now - captured <= profile.frame_timeout_s)
            new_frame = fresh and (last_frame is None or captured > last_frame)
            if new_frame:
                last_frame = captured
                last_usable_frame = now
                # Tracker uses timestamp only for uniqueness/staleness. Supply
                # the already-validated capture time and disable wall-clock age.
                snapshot = copy.copy(result)
                snapshot.timestamp = captured
                snapshot.all_blocks = [b for b in result.all_blocks
                                       if all(_finite(getattr(b, key, None))
                                              for key in ('x', 'y', 'z', 'confidence'))]
                block = tracker.update(snapshot, color_name='orange',
                                       min_confidence=cfg.orange_min_confidence,
                                       max_age_s=float('inf'),
                                       reference_x=(None if target_block is None else target_block.x),
                                       reference_z=(None if target_block is None else target_block.z))
                if block is not None:
                    last_seen = now
                    target_block = block
                    control._last_alignment_block = block
                    control._last_alignment_frame_timestamp = result.timestamp
                    control._alignment_valid_frames += 1
                    error = block.x - target_x
                    if low_x <= block.x <= high_x:
                        desired = 0.0
                        integral = 0.0
                        previous_error = None
                        if abs(measured_speed) <= profile.settled_speed_mm_s:
                            confirmed += 1
                        else:
                            confirmed = 0
                        if confirmed >= cfg.align_confirm_frames:
                            return True
                    else:
                        confirmed = 0
                        integral = max(-cfg.align_integral_limit,
                                       min(cfg.align_integral_limit, integral + error * dt))
                        derivative = (0.0 if previous_error is None or dt <= 0
                                      else (error - previous_error) / dt)
                        previous_error = error
                        minimum = (cfg.align_creep_min_speed_mm_s
                                   if abs(error) <= cfg.align_creep_start_mm
                                   else cfg.align_min_speed_mm_s)
                        desired = control._alignment_speed(error, integral, derivative,
                                                           cfg, minimum)
                        # Account for image age before approaching the acceptance
                        # window. This prevents minimum-speed clipping from
                        # defeating the measured braking envelope near the cube.
                        error_distance = max(0.0, min(abs(block.x - low_x),
                                                       abs(block.x - high_x)))
                        visual_latency = now - captured + profile.tick_s + profile.max_command_delay_s
                        brake = profile.braking_mm_s2
                        target_cap = (math.sqrt((brake * visual_latency) ** 2
                                               + 2 * brake * error_distance)
                                      - brake * visual_latency)
                        desired = math.copysign(min(abs(desired), target_cap), desired)
                else:
                    confirmed = 0
                    integral = 0.0
                    previous_error = None
                    # Do not move using a stale locked position. Fresh empty
                    # frames before acquisition may keep bounded right search.
                    desired = (cfg.search_speed_mm_s if last_seen is None
                               and not tracker.locked else 0.0)
            elif not fresh:
                confirmed = 0
                desired = 0.0
            if now - last_usable_frame >= cfg.align_lost_timeout_s:
                return False
            if last_seen is not None and now - last_seen >= cfg.align_lost_timeout_s:
                return False
            desired = max(-negative_cap, min(positive_cap, desired))
            desired = math.copysign(min(abs(desired), profile.braking_mm_s2 * max(
                0.0, profile.max_duration_s - (now - started)
                - profile.tick_s - profile.max_command_delay_s)), desired)
            next_speed = _slew(desired, speed, min(dt, profile.tick_s), profile)
            if next_speed > positive_cap + 1e-9 or -next_speed > negative_cap + 1e-9:
                return False
            check()
            if send(next_speed) is False:
                raise BlindMotionFault('chassis rejected pickup velocity command')
            check()
            after_command = clock()
            if not _finite(after_command) or after_command < now:
                raise BlindMotionFault('invalid pickup clock after command')
            if after_command - now > profile.max_command_delay_s:
                raise BlindMotionFault('pickup velocity command exceeded delay budget')
            speed, previous_time = next_speed, now
            sleep(max(0.0, profile.tick_s - (after_command - now)))
    except BaseException as error:
        primary_error = error
        raise
    finally:
        try:
            if stop() is False:
                raise BlindMotionFault('chassis rejected pickup stop command')
        except BaseException as stop_error:
            if primary_error is None:
                raise
            if hasattr(primary_error, 'add_note'):
                primary_error.add_note(f'pickup stop also failed: {type(stop_error).__name__}')


__all__ = ['PickupMotionProfile', 'acquire_after_blind', 'measured_lateral_speed']
