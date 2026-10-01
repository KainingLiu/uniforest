"""Opt-in bounded arm-return motion, using measured motion and fresh camera data.

No profile is enabled here. Field-validated parameters and milestone producers
must be supplied by a caller that retains sole chassis ownership. All times use
the same monotonic clock; displacement is signed encoder progress from entry.
"""

from dataclasses import dataclass
from enum import Enum
import math
import time


class BlindMotionFault(RuntimeError):
    """Invalid hardware continuity or observation; never a soft retry result."""


def _finite(value):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


@dataclass(frozen=True)
class BlindMotionProfile:
    direction: int
    cruise_speed_mm_s: float
    acceleration_mm_s2: float
    braking_mm_s2: float
    max_distance_mm: float
    max_duration_s: float
    braking_margin_mm: float
    telemetry_timeout_s: float
    frame_timeout_s: float
    tick_s: float
    max_command_delay_s: float
    validated: bool = False

    def __post_init__(self):
        if type(self.direction) is not int or self.direction not in (-1, 1):
            raise ValueError('blind direction must be +1 or -1')
        for name in ('cruise_speed_mm_s', 'acceleration_mm_s2', 'braking_mm_s2',
                     'max_distance_mm', 'max_duration_s', 'braking_margin_mm',
                     'telemetry_timeout_s', 'frame_timeout_s', 'tick_s', 'max_command_delay_s'):
            value = getattr(self, name)
            if not _finite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.braking_margin_mm >= self.max_distance_mm:
            raise ValueError('braking margin must leave positive travel distance')
        if (self.tick_s + self.max_command_delay_s >= self.max_duration_s
                or self.tick_s > self.telemetry_timeout_s):
            raise ValueError('tick must fit duration and telemetry freshness limits')
        if type(self.validated) is not bool:
            raise ValueError('validated must be an explicit boolean')


@dataclass(frozen=True)
class BlindSample:
    token: int
    epoch: int
    timestamp_s: float
    displacement_mm: float
    velocity_mm_s: float
    lifted: bool
    arm_reset: bool
    arm_reset_at_s: float | None = None
    frame_captured_at_s: float | None = None
    frame_epoch: int | None = None


class BlindStatus(str, Enum):
    HANDED_OFF = 'handed_off'
    CAMERA_READY_STOPPED = 'camera_ready_stopped'
    CAMERA_UNAVAILABLE = 'camera_unavailable'
    BOUND_REACHED = 'bound_reached'


@dataclass(frozen=True)
class BlindResult:
    status: BlindStatus
    displacement_mm: float
    velocity_mm_s: float
    duration_s: float
    reason: str = ''


def run_blind_transition(profile, *, token, epoch, read_sample, command_speed,
                         stop, guard, handoff=None, clock=time.monotonic,
                         sleep=time.sleep):
    """Move only after full lift; hand off only after full reset and a new frame.

    ``command_speed`` receives signed mm/s in the profile's known direction;
    adapters convert this to existing chassis controls. ``handoff`` receives
    actual measured signed velocity and continues under the same owner/thread.
    It must accept ownership before returning (False rejects it).

    Duration includes waiting for lift. The distance envelope uses measured
    progress, speed, observation age, one future control tick, and a configured
    braking margin. Late samples or exceeded braking envelopes trigger a stop;
    a software stop request cannot guarantee physical stopping distance.

    Non-handoff returns always issue stop. Their velocity is the last measured
    value, not a claim of physical standstill. Hardware continuity errors and
    cancellations raise after attempting stop; they must abort the outer flow.
    Long injected callbacks must cooperatively honor the same hardware guard.
    """
    if not isinstance(profile, BlindMotionProfile):
        raise TypeError('profile must be a BlindMotionProfile')
    if not profile.validated:
        raise ValueError('blind motion requires an explicitly field-validated profile')
    if type(token) is not int or not 0 < token <= 0xffffffff:
        raise ValueError('token must be a nonzero uint32 action token')
    if type(epoch) is not int or epoch < 0:
        raise ValueError('epoch must be a nonnegative generation')
    for name, callback in (('read_sample', read_sample), ('command_speed', command_speed),
                           ('stop', stop), ('guard', guard), ('clock', clock), ('sleep', sleep)):
        if not callable(callback):
            raise TypeError(f'{name} must be callable')
    if handoff is not None and not callable(handoff):
        raise TypeError('handoff must be callable')

    handed_off = False
    primary_error = None
    sample = None

    def check():
        if guard() is False:
            raise BlindMotionFault('blind motion guard rejected the current state')

    try:
        check()
        started = previous_time = clock()
        if not _finite(started):
            raise BlindMotionFault('invalid monotonic clock')
        previous_command = None
        was_lifted = was_reset = False
        reset_time = None
        while True:
            check()
            sample = read_sample()
            check()
            now = clock()
            if not _finite(now) or now < previous_time:
                raise BlindMotionFault('monotonic clock moved backward or became invalid')
            elapsed = now - started
            if not isinstance(sample, BlindSample):
                raise BlindMotionFault('blind telemetry unavailable')
            if (type(sample.token) is not int or type(sample.epoch) is not int
                    or sample.token != token or sample.epoch != epoch):
                raise BlindMotionFault('blind observation token/epoch does not match execution')
            if not all(_finite(value) for value in
                       (sample.timestamp_s, sample.displacement_mm, sample.velocity_mm_s)):
                raise BlindMotionFault('blind telemetry contains invalid numeric values')
            age = now - sample.timestamp_s
            if age < 0 or age > profile.telemetry_timeout_s:
                raise BlindMotionFault('blind telemetry stale or from a future clock')
            if type(sample.lifted) is not bool or type(sample.arm_reset) is not bool:
                raise BlindMotionFault('blind milestones must be explicit boolean states')
            if was_lifted and not sample.lifted:
                raise BlindMotionFault('full-lift milestone regressed')
            if was_reset and not sample.arm_reset:
                raise BlindMotionFault('arm-reset milestone regressed')
            if sample.arm_reset:
                if (not sample.lifted or not _finite(sample.arm_reset_at_s)
                        or sample.arm_reset_at_s > now):
                    raise BlindMotionFault('arm reset lacks a correlated completion time')
                if reset_time is not None and sample.arm_reset_at_s != reset_time:
                    raise BlindMotionFault('arm-reset completion time changed within one epoch')
                reset_time = sample.arm_reset_at_s
            was_lifted, was_reset = sample.lifted, sample.arm_reset
            progress = sample.displacement_mm * profile.direction
            velocity = sample.velocity_mm_s * profile.direction
            if progress < 0 or velocity < 0:
                raise BlindMotionFault('measured movement opposes the registered direction')
            if not sample.lifted and (progress > 0 or velocity > 0):
                raise BlindMotionFault('chassis moved before full lift')

            def result(status, reason=''):
                return BlindResult(status, sample.displacement_mm, sample.velocity_mm_s,
                                   elapsed, reason)

            if elapsed >= profile.max_duration_s:
                return result(BlindStatus.CAMERA_UNAVAILABLE, 'duration_limit')
            remaining = profile.max_distance_mm - progress
            # A repeated but still fresh telemetry packet may precede a higher
            # speed setpoint. Include that setpoint in the unseen-distance bound.
            envelope_velocity = max(velocity, previous_command or 0.0)
            if remaining <= profile.braking_margin_mm + envelope_velocity * age:
                return result(BlindStatus.BOUND_REACHED, 'distance_limit')

            frame_time = sample.frame_captured_at_s
            if frame_time is not None and (not _finite(frame_time) or frame_time > now):
                raise BlindMotionFault('camera capture timestamp is invalid')
            camera_ready = (
                sample.arm_reset and type(sample.frame_epoch) is int
                and sample.frame_epoch == epoch and frame_time is not None
                and frame_time > reset_time and now - frame_time <= profile.frame_timeout_s)
            if camera_ready:
                if handoff is None:
                    return result(BlindStatus.CAMERA_READY_STOPPED)
                check()
                if handoff(sample.velocity_mm_s) is False:
                    raise BlindMotionFault('vision controller rejected velocity handoff')
                check()
                handed_off = True
                return result(BlindStatus.HANDED_OFF)

            dt = min(profile.tick_s, max(0.0, now - previous_time))
            # The first setpoint covers the first tick; subsequent ramp increments
            # use elapsed time, never a large catch-up step after a scheduling delay.
            if previous_command is None:
                previous_command = velocity
                dt = profile.tick_s
            if not sample.lifted:
                speed = 0.0
            else:
                latency = age + profile.tick_s + profile.max_command_delay_s
                stopping_distance = envelope_velocity ** 2 / (2.0 * profile.braking_mm_s2)
                usable = (remaining - profile.braking_margin_mm
                          - max(0.0, envelope_velocity - velocity) * age)
                if stopping_distance + velocity * age + envelope_velocity * profile.max_command_delay_s >= usable:
                    return result(BlindStatus.BOUND_REACHED, 'measured_braking_envelope')
                distance_cap = (math.sqrt((profile.braking_mm_s2 * latency) ** 2
                                + 2.0 * profile.braking_mm_s2 * usable)
                                - profile.braking_mm_s2 * latency)
                time_cap = profile.braking_mm_s2 * max(
                    0.0, profile.max_duration_s - elapsed
                    - profile.tick_s - profile.max_command_delay_s)
                target = min(profile.cruise_speed_mm_s, distance_cap, time_cap)
                if target >= previous_command:
                    speed = min(target, previous_command + profile.acceleration_mm_s2 * dt)
                else:
                    speed = max(target, previous_command - profile.braking_mm_s2 * dt)
                if speed > distance_cap + 1e-9 or speed > time_cap + 1e-9:
                    return result(BlindStatus.BOUND_REACHED, 'command_braking_envelope')
            check()
            if command_speed(profile.direction * speed) is False:
                raise BlindMotionFault('chassis rejected blind velocity command')
            check()
            after_command = clock()
            if not _finite(after_command) or after_command < now:
                raise BlindMotionFault('invalid clock after blind velocity command')
            if after_command - now > profile.max_command_delay_s:
                raise BlindMotionFault('blind velocity command exceeded its delay budget')
            previous_command, previous_time = speed, now
            sleep(max(0.0, min(profile.tick_s - (after_command - now),
                               profile.max_duration_s - (after_command - started))))
    except BaseException as error:
        primary_error = error
        raise
    finally:
        if not handed_off:
            try:
                if stop() is False:
                    raise BlindMotionFault('chassis rejected blind stop command')
            except BaseException as stop_error:
                if primary_error is None:
                    raise
                if hasattr(primary_error, 'add_note'):
                    primary_error.add_note(f'blind stop also failed: {type(stop_error).__name__}')
