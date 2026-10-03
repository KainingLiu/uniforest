"""Capture a cube inside a calibrated window instead of centering it twice.

Both ordinary acquisition and a moving blind handoff use align_cube(). The
window policy has no hardware dependencies. Timings/limits are calibration,
never evidence of mechanical clearance or a successful suction pickup.
"""

from dataclasses import dataclass, fields, replace
import copy
import math
from numbers import Real
import time

from ..common import wrap_angle
from ..cube_tracker import CubeTargetTracker
from ..execution.pickup_motion import measured_lateral_speed


def _finite(value):
    return isinstance(value,Real) and not isinstance(value,bool) and math.isfinite(value)


class AlignmentFault(RuntimeError):
    """Hardware/observation continuity failure; never retry as a visual miss."""


@dataclass(frozen=True)
class FastAlignmentProfile:
    capture_tolerance_mm: float
    mechanical_tolerance_mm: float
    position_uncertainty_mm: float
    max_speed_mm_s: float
    acceleration_mm_s2: float
    braking_mm_s2: float
    settled_speed_mm_s: float
    max_duration_s: float
    max_travel_mm: float
    frame_timeout_s: float
    telemetry_timeout_s: float
    tick_s: float
    max_command_delay_s: float
    confirm_frames: int = 2
    max_reversals: int = 1
    max_attempts: int = 2
    validated: bool = False
    trial_enabled: bool = False
    min_correction_speed_mm_s: float = 0.0

    def __post_init__(self):
        for item in fields(self):
            if item.name in ('validated', 'trial_enabled', 'confirm_frames', 'max_reversals', 'max_attempts'):
                continue
            value = getattr(self, item.name)
            if not _finite(value) or value < 0 or value == 0 and item.name not in ('position_uncertainty_mm', 'min_correction_speed_mm_s'):
                raise ValueError(f'{item.name} must be finite and positive (uncertainty may be zero)')
        if type(self.validated) is not bool:
            raise ValueError('validated must be boolean')
        if type(self.trial_enabled) is not bool:
            raise ValueError('trial_enabled must be boolean')
        if type(self.confirm_frames) is not int or self.confirm_frames < 2:
            raise ValueError('confirmation requires at least two distinct frames')
        if type(self.max_reversals) is not int or not 0 <= self.max_reversals <= 2:
            raise ValueError('max_reversals must be an integer in 0..2')
        if type(self.max_attempts) is not int or not 1 <= self.max_attempts <= 3:
            raise ValueError('max_attempts must be an integer in 1..3')
        if self.capture_tolerance_mm + self.position_uncertainty_mm > self.mechanical_tolerance_mm:
            raise ValueError('capture window plus uncertainty exceeds mechanical tolerance')
        if self.settled_speed_mm_s >= self.max_speed_mm_s:
            raise ValueError('settled speed must be below approach speed')
        if self.min_correction_speed_mm_s > self.max_speed_mm_s:
            raise ValueError('correction speed exceeds approach limit')
        if (self.tick_s > min(.05, self.telemetry_timeout_s, self.frame_timeout_s)
                or self.telemetry_timeout_s > .5 or self.frame_timeout_s > .5
                or self.tick_s + self.max_command_delay_s >= self.max_duration_s):
            raise ValueError('invalid control period/freshness/duration budget')


@dataclass(frozen=True)
class Decision:
    speed_mm_s: float
    complete: bool = False
    exhausted: bool = False


class CaptureWindow:
    """One approach, braking latch, fresh-frame confirmation, bounded reversal."""

    def __init__(self, profile, initial_speed=0.0):
        if not (profile.validated or profile.trial_enabled) or not _finite(initial_speed):
            raise ValueError('fast alignment requires validated limits and finite speed')
        if abs(initial_speed) > profile.max_speed_mm_s:
            raise ValueError('entry speed exceeds fast alignment envelope')
        self.profile = profile
        self.command = initial_speed
        self.latched = False
        self.confirmed = self.outside = self.reversals = 0
        self.last_direction = 1 if initial_speed > 0 else -1 if initial_speed < 0 else 0
        self.last_confirmation = None
        self.predicted_error_mm = None
        self.quiet_for_s = 0.0

    def update(self, error_mm, measured_speed, age_s, dt, *, evidence=None):
        """error=None brakes. evidence is a distinct (capture, telemetry) pair.

        Error is relative to the existing calibrated camera-X target. Predict
        latency drift and stopping travel with calibrated deceleration; never
        authorize pickup from position alone while the chassis is still moving.
        """
        p = self.profile
        if (not all(_finite(v) for v in (measured_speed, age_s, dt))
                or age_s < 0 or dt < 0 or error_mm is not None and not _finite(error_mm)):
            raise AlignmentFault('invalid capture-window feedback')
        complete = False
        quiet = abs(measured_speed) <= p.settled_speed_mm_s and self.command == 0.0
        self.quiet_for_s = self.quiet_for_s + dt if quiet else 0.0
        if error_mm is None:
            self.confirmed = self.outside = 0
            self.last_confirmation = None
            desired = 0.0
        else:
            current = error_mm - measured_speed * age_s
            # Include the queued command's speed in the stop envelope.
            velocity = max((measured_speed, self.command), key=abs)
            drift = velocity * p.max_command_delay_s + velocity * abs(velocity) / (2*p.braking_mm_s2)
            landing = current - drift
            self.predicted_error_mm = landing
            # A frame captured after the stationary interval began no longer
            # needs the acceleration reserve from a previous moving exposure.
            moving_age = 0.0 if quiet and self.quiet_for_s >= age_s else age_s
            uncertainty = p.position_uncertainty_mm + .5*max(p.acceleration_mm_s2,p.braking_mm_s2)*moving_age**2
            in_window = (abs(current) <= p.capture_tolerance_mm
                         and abs(current)+p.position_uncertainty_mm <= p.mechanical_tolerance_mm)
            # A queued setpoint is a braking bound, not evidence of motion.
            # Do not stop a correction merely because its first small command
            # predicts entry while the physical chassis is still stationary.
            approaching = current*measured_speed > 0 and abs(measured_speed) > p.settled_speed_mm_s
            if in_window or approaching and abs(landing) <= p.capture_tolerance_mm:
                self.latched = True
            new_evidence = (evidence is not None and
                            (self.last_confirmation is None or
                             evidence[0] != self.last_confirmation[0] and
                             evidence[1] != self.last_confirmation[1]))
            inside = (abs(error_mm) <= p.capture_tolerance_mm
                      and abs(current) <= p.capture_tolerance_mm and
                      abs(landing) + uncertainty <= p.mechanical_tolerance_mm)
            if new_evidence:
                self.last_confirmation = evidence
                self.confirmed = self.confirmed + 1 if quiet and inside else 0
                self.outside = self.outside + 1 if quiet and not in_window else 0
                complete = self.confirmed >= p.confirm_frames
                # Wait for multiple stopped observations before correcting a
                # miss. A single edge jitter never kicks the chassis sideways.
                if self.outside >= p.confirm_frames:
                    self.latched = False
                    self.outside = 0
            if self.latched or complete:
                desired = 0.0
            else:
                # Aim inside the window, without requiring zero position error.
                distance = max(0.0, abs(current) - p.capture_tolerance_mm/2)
                delay = p.tick_s + p.max_command_delay_s
                cap = math.sqrt((p.braking_mm_s2*delay)**2 + 2*p.braking_mm_s2*distance)
                cap -= p.braking_mm_s2*delay
                # Reuse the configured starting speed to overcome low-speed
                # deadband, bounded by travel to the FAR edge of the window.
                # It remains subordinate to acceleration and stopping limits.
                safe_distance = max(0.0, abs(current)+p.capture_tolerance_mm-uncertainty)
                safe_cap = math.sqrt((p.braking_mm_s2*delay)**2 + 2*p.braking_mm_s2*safe_distance)
                safe_cap -= p.braking_mm_s2*delay
                desired = math.copysign(min(p.max_speed_mm_s, safe_cap,
                    max(cap, p.min_correction_speed_mm_s)), current)
        # Stop before reversing either a setpoint or measured physical motion.
        if desired*self.command < 0 or desired*measured_speed < 0 and abs(measured_speed) > p.settled_speed_mm_s:
            desired = 0.0
        rate = p.braking_mm_s2 if abs(desired) < abs(self.command) else p.acceleration_mm_s2
        change = max(-rate*dt, min(rate*dt, desired-self.command))
        command = self.command + change
        if abs(command-desired) < 1e-9:
            command = desired
        direction = 1 if command > 0 else -1 if command < 0 else 0
        if direction and self.last_direction and direction != self.last_direction:
            self.reversals += 1
            if self.reversals > p.max_reversals:
                return Decision(0.0, exhausted=True)
        if direction:
            self.last_direction = direction
        self.command = command
        return Decision(command, complete)


def align_cube(control, initial_block=None, *, color_name, phase_origin, profile,
               guard=None, initial_speed_mm_s=None, pose_epoch=None,
               arm_reset_at_s=None, search_profile=None, heading_yaw_deg=None,
               command_speed=None, stop=None, clock=time.monotonic, sleep=time.sleep,
               neighbor_observer=None):
    """True authorizes pickup; False stops and lets the caller resume searching.

    Hard faults raise after stopping. With search_profile, continue the incoming
    blind velocity while looking for the first orange target. Otherwise retain
    the seed from ordinary search. No legacy timeout-success fallback is used.
    """
    if not isinstance(profile, FastAlignmentProfile) or not (profile.validated or profile.trial_enabled):
        raise ValueError('fast alignment requires an explicitly validated profile')
    if color_name not in ('orange', 'purple'):
        raise ValueError('unsupported pickup color')
    if search_profile is not None:
        if not (search_profile.validated or search_profile.trial_enabled) or color_name != 'orange':
            raise ValueError('moving search requires a validated orange motion profile')
        # Both independently calibrated envelopes remain binding.
        profile = replace(profile, **{name:min(getattr(profile,name),getattr(search_profile,name))
            for name in ('acceleration_mm_s2','braking_mm_s2','settled_speed_mm_s',
                         'max_duration_s','frame_timeout_s','telemetry_timeout_s',
                         'tick_s','max_command_delay_s')})
    robot, cfg = control.robot, control.config
    stop = stop or (lambda: robot.chassis.set_speeds([0,0,0,0]))
    generation = robot.transport.emergency_stop_generation
    started = previous_time = clock()
    reason = 'timeout'
    primary_error = None
    policy = None
    position = remaining = stop_distance = None
    error = target_x = speed = None
    control._last_alignment_timed_out = False
    control._last_alignment_block = None
    control._last_alignment_frame_timestamp = None
    control._alignment_valid_frames = 0

    def check():
        control._check_active()
        if guard is not None and guard() is False:
            raise AlignmentFault('alignment guard rejected execution')
        if (not robot.transport.connected or
                robot.transport.emergency_stop_generation != generation):
            raise AlignmentFault('alignment communication/cancellation fault')

    try:
        check()
        initial_frame = robot.vision_result
        if pose_epoch is None:
            pose_epoch = getattr(initial_frame, 'pose_epoch', None)
        if type(pose_epoch) is not int or pose_epoch < 0:
            reason = 'camera_pose_unavailable'
            return False
        if arm_reset_at_s is not None and (not _finite(arm_reset_at_s) or arm_reset_at_s > started):
            raise AlignmentFault('invalid camera restoration time')
        initial = measured_lateral_speed(robot) if initial_speed_mm_s is None else initial_speed_mm_s
        policy = CaptureWindow(profile, initial)
        heading = robot.telem.yaw_deg if heading_yaw_deg is None else heading_yaw_deg
        if not _finite(heading):
            raise AlignmentFault('invalid alignment heading')
        position = previous_position = control._measure_lateral_displacement_mm(phase_origin)
        if not _finite(position):
            raise AlignmentFault('invalid alignment encoder position')
        direction = 1 if color_name == 'orange' else -1
        limit = cfg.search_max_distance_mm if direction == 1 else cfg.purple_search_max_distance_mm
        if search_profile is not None:
            limit = min(limit, search_profile.max_distance_mm)
        travel = 0.0
        phase_timeout = limit/cfg.search_speed_mm_s + cfg.orange_edge_timeout_s + cfg.target_cube_count*cfg.vision_observe_s
        target_x = (getattr(cfg,'orange_align_target_x_mm',cfg.align_target_x_mm)
                    if color_name == 'orange' else cfg.align_target_x_mm)
        tracker = CubeTargetTracker(max_x_jump_mm=cfg.align_track_max_x_jump_mm,
            max_z_jump_mm=cfg.align_track_max_z_jump_mm, confirm_frames=1,
            lost_frames=cfg.orange_search_lock_lost_frames, smoothing_frames=1,
            ambiguity_margin_mm=(getattr(cfg,'orange_track_ambiguity_margin_mm',cfg.align_track_ambiguity_margin_mm)
                                 if color_name == 'orange' else cfg.align_track_ambiguity_margin_mm))
        reference = initial_block
        target = None
        capture = None
        last_frame = None
        last_seen = started
        last_uptime = last_received = link_epoch = None
        progressed_at = started

        def send(speed):
            check()
            if command_speed is not None:
                return command_speed(speed)
            yaw = robot.telem.yaw_deg
            scale = robot.chassis.lateral_distance_scale
            if not _finite(yaw) or not _finite(scale) or scale <= 0:
                raise AlignmentFault('invalid alignment chassis geometry')
            correction = max(-cfg.delivery_heading_max_yaw_deg_s,min(cfg.delivery_heading_max_yaw_deg_s,
                cfg.tag6_heading_kp*wrap_angle(heading-yaw)))
            return robot.chassis.set_speeds(robot.chassis.mecanum_rpm(0,speed*scale/10,correction))

        while True:
            check()
            # Snapshot before reading the clock: background acquisition can
            # publish between calls, so a pre-snapshot 'now' can be older than
            # perfectly valid telemetry or camera capture timestamps.
            telem, received, _, epoch = robot.inspection_link_snapshot()
            result = robot.vision_result
            now = clock()
            if not _finite(now) or now < previous_time:
                raise AlignmentFault('alignment clock regressed')
            dt = now-previous_time
            if (telem is None or not _finite(received) or not 0 <= now-received <= profile.telemetry_timeout_s
                    or last_received is not None and received < last_received
                    or link_epoch is not None and epoch != link_epoch):
                raise AlignmentFault('alignment telemetry stale or link changed')
            if last_uptime is not None and ((telem.uptime_ms-last_uptime)&0xffffffff) >= 0x80000000:
                raise AlignmentFault('A-board restarted during alignment')
            if last_uptime != telem.uptime_ms:
                progressed_at = now
            if now-progressed_at > profile.telemetry_timeout_s:
                raise AlignmentFault('A-board alignment telemetry clock stopped')
            last_uptime, last_received, link_epoch = telem.uptime_ms, received, epoch
            speed = measured_lateral_speed(robot)
            position = control._measure_lateral_displacement_mm(phase_origin)
            if not _finite(position):
                raise AlignmentFault('invalid alignment encoder position')
            delta = position-previous_position
            travel += abs(delta)
            previous_position = position
            # Search progress/time belong to the no-target search only. Once
            # locked, preserve legacy bidirectional visual alignment, governed
            # by this invocation's own travel, time and stopping limits.
            if reference is None:
                control._search_position_mm += max(0.0,direction*delta)
                if color_name == 'orange':
                    control._orange_recovery.search_elapsed_s += dt
            if now-started >= profile.max_duration_s or travel >= profile.max_travel_mm:
                control._last_alignment_timed_out = True
                reason = 'alignment_budget'
                return False
            captured = getattr(result,'captured_monotonic',None)
            result_epoch = getattr(result,'pose_epoch',None)
            if result is not None and result_epoch != pose_epoch:
                raise AlignmentFault('camera pose changed during alignment')
            fresh = (result is not None and _finite(captured) and 0 <= now-captured <= profile.frame_timeout_s
                     and (arm_reset_at_s is None or captured > arm_reset_at_s))
            if fresh and last_frame is not None and captured < last_frame:
                raise AlignmentFault('camera capture time regressed')
            evidence = None
            if fresh and captured != last_frame:
                last_frame = captured
                snapshot = copy.copy(result)
                snapshot.timestamp = captured
                snapshot.all_blocks = [b for b in result.all_blocks
                    if all(_finite(getattr(b,k,None)) for k in ('x','y','z','confidence'))]
                target = tracker.update(snapshot,color_name=color_name,
                    min_confidence=cfg.orange_min_confidence if color_name=='orange' else cfg.purple_min_confidence,
                    max_age_s=float('inf'), reference_x=None if reference is None else reference.x,
                    reference_z=None if reference is None else reference.z)
                capture = captured
                if target is not None:
                    reference = target
                    last_seen = now
                    evidence = (captured,telem.uptime_ms)
                    control._last_alignment_block = target
                    control._last_alignment_frame_timestamp = result.timestamp
                    control._alignment_valid_frames += 1
            elif not fresh:
                target = None
            if now-last_seen >= cfg.align_lost_timeout_s and (reference is not None or not fresh):
                reason = 'target_lost'
                return False
            if reference is None:
                if color_name == 'orange' and control._orange_recovery.search_elapsed_s >= phase_timeout:
                    reason = 'phase_time_budget'
                    return False
                if control._search_position_mm >= limit:
                    reason = 'search_boundary'
                    return False
            error = target.x-target_x if target is not None else None
            previous_command = policy.command
            decision = policy.update(error,speed,0 if capture is None else max(0,now-capture),
                                     min(dt,profile.tick_s),evidence=evidence)
            if neighbor_observer is not None:
                neighbor_observer.observe(result, target if fresh else None,
                    position_mm=position, speed_mm_s=speed, target_x_mm=target_x,
                    yaw_deg=telem.yaw_deg, now=now, link_epoch=epoch,
                    stop_generation=generation, telemetry_id=telem.uptime_ms,
                    stopped=previous_command == 0.0,
                    acceleration_mm_s2=profile.acceleration_mm_s2)
            if decision.exhausted:
                reason = 'reversal_budget'
                return False
            if decision.complete:
                reason = 'captured'
                return True
            command = decision.speed_mm_s
            if search_profile is not None and reference is None and fresh:
                # Initial no-target search preserves the handed-off direction.
                command = min(profile.max_speed_mm_s,cfg.search_speed_mm_s,
                              previous_command+profile.acceleration_mm_s2*min(dt,profile.tick_s))
                policy.command = command
            remaining = profile.max_travel_mm-travel
            if reference is None:
                projected = direction*position
                motion = speed if speed != 0 else command
                search_remaining = (min(limit-projected,limit-control._search_position_mm)
                                    if motion*direction >= 0 else projected)
                remaining = min(remaining, search_remaining)
            if search_profile is not None:
                remaining -= search_profile.braking_margin_mm
            braking_speed = max(abs(speed),abs(command))
            stop_distance = braking_speed*(now-received+profile.tick_s+profile.max_command_delay_s)
            stop_distance += braking_speed**2/(2*profile.braking_mm_s2)
            if remaining < 0 or braking_speed != 0 and stop_distance >= remaining:
                reason = 'search_boundary' if reference is None else 'alignment_travel'
                return False
            if send(command) is not True:
                raise AlignmentFault('alignment speed send failed')
            check()
            sent_at = clock()
            if sent_at < now or not _finite(sent_at) or sent_at-now > profile.max_command_delay_s:
                raise AlignmentFault('alignment command delay exceeded')
            previous_time = now
            sleep(max(0,profile.tick_s-(sent_at-now)))
    except BaseException as exc:
        primary_error = exc
        reason = type(exc).__name__
        raise
    finally:
        try:
            if stop() is False:
                raise AlignmentFault('alignment stop send failed')
        except BaseException:
            if primary_error is None:
                raise
        diagnostics = getattr(robot,'diagnostics',None)
        if diagnostics is not None:
            try:
                diagnostics.write('fast_alignment',color=color_name,reason=reason,
                    operation=getattr(control,'operation_name',''),
                    elapsed_s=clock()-started,reversals=0 if policy is None else policy.reversals,
                    predicted_error_mm=None if policy is None else policy.predicted_error_mm,
                    raw_error_mm=error,target_x_mm=target_x,measured_speed_mm_s=speed,
                    position_mm=position, remaining_mm=remaining, stop_distance_mm=stop_distance)
            except Exception:
                pass  # Diagnostics must never turn a hardware fault into a retry.
