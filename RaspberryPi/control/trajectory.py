"""Opt-in local cubic routes. Units: mm, seconds, unwrapped clockwise degrees.

Profiles and points must be field validated before a route can use this follower.
This is encoder/IMU tracking, with no obstacle avoidance or absolute localization.
Acceleration limits constrain body-frame velocity command increments; traction and
load limits still require measurement. No zero command is inserted at waypoints.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
import bisect
import math
import time


@dataclass(frozen=True)
class Waypoint:
    x_mm: float
    y_mm: float
    yaw_deg: float


@dataclass(frozen=True)
class BodyVelocity:
    vx_mm_s: float = 0.0
    vy_mm_s: float = 0.0
    yaw_deg_s: float = 0.0


@dataclass(frozen=True)
class PoseSample:
    x_mm: float
    y_mm: float
    yaw_deg: float
    velocity: BodyVelocity
    received_at: float


@dataclass(frozen=True)
class TrajectoryProfile:
    validated: bool
    max_speed_mm_s: float
    max_accel_mm_s2: float
    max_yaw_speed_deg_s: float
    max_yaw_accel_deg_s2: float
    max_wheel_rpm: float
    position_gain_s: float
    yaw_gain_s: float
    position_tolerance_mm: float
    yaw_tolerance_deg: float
    settle_speed_mm_s: float
    settle_yaw_speed_deg_s: float
    settle_time_s: float
    max_tracking_error_mm: float
    max_tracking_yaw_error_deg: float
    timeout_s: float
    control_period_s: float
    max_telemetry_age_s: float

    def validate(self):
        if self.validated is not True:
            raise ValueError('continuous trajectory profile requires field validation')
        for field in fields(self):
            if field.name != 'validated':
                value = getattr(self, field.name)
                if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                    raise ValueError(f'{field.name} must be finite and positive')
        if self.control_period_s > 0.05 or self.max_telemetry_age_s > 0.5:
            raise ValueError('trajectory period/stale limit exceeds supervision limit')
        if not 1 <= self.max_wheel_rpm <= 32767:
            raise ValueError('wheel RPM exceeds signed protocol range')
        if self.max_yaw_speed_deg_s*self.max_telemetry_age_s >= 180:
            raise ValueError('yaw speed and telemetry age make IMU unwrapping ambiguous')
        if (self.position_tolerance_mm >= self.max_tracking_error_mm
                or self.yaw_tolerance_deg >= self.max_tracking_yaw_error_deg):
            raise ValueError('arrival tolerances must be smaller than tracking limits')


@dataclass(frozen=True)
class TrajectoryResult:
    elapsed_s: float
    final_pose: PoseSample
    points_passed: int


def _components(value):
    return tuple(getattr(value, f.name) for f in fields(value))


def _finite(values):
    return all(not isinstance(v, bool) and math.isfinite(v) for v in values)


class CubicRoute:
    """C1 Hermite interpolation, including prescribed yaw (no shortest-turn wrap).

    Segment durations are conservative seeds derived from explicit limits. The
    runtime also limits wheel speed and acceleration; excessive lag aborts rather
    than skipping to a later route point. Curves can leave the waypoint polygon;
    the entire swept corridor must be checked when validating a route.
    """
    def __init__(self, points, profile, initial_velocity=BodyVelocity()):
        profile.validate()
        self.points = tuple(points)
        if len(self.points) < 2 or any(
                not isinstance(p, Waypoint) or not _finite(_components(p))
                for p in self.points):
            raise ValueError('route requires at least two finite Waypoints')
        if self.points[0] != Waypoint(0.0, 0.0, 0.0):
            raise ValueError('local route must start at (0, 0, 0)')
        if not _finite(_components(initial_velocity)):
            raise ValueError('initial velocity must be finite')
        self.knots = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            distance = math.hypot(b.x_mm - a.x_mm, b.y_mm - a.y_mm)
            angle = abs(b.yaw_deg - a.yaw_deg)
            if distance == 0 and angle == 0:
                raise ValueError('adjacent duplicate waypoints are invalid')
            duration = max(1.5 * distance / profile.max_speed_mm_s,
                           1.5 * angle / profile.max_yaw_speed_deg_s,
                           2 * math.sqrt(distance / profile.max_accel_mm_s2),
                           2 * math.sqrt(angle / profile.max_yaw_accel_deg_s2),
                           profile.control_period_s)
            self.knots.append(self.knots[-1] + duration)
        self.tangents = [_components(initial_velocity)]
        for i in range(1, len(self.points) - 1):
            before, after = map(_components, (self.points[i-1], self.points[i+1]))
            dt = self.knots[i+1] - self.knots[i-1]
            self.tangents.append(tuple((b-a)/dt for a, b in zip(before, after)))
        self.tangents.append((0.0, 0.0, 0.0))
        self.duration_s = self.knots[-1]
        if self.duration_s + profile.settle_time_s >= profile.timeout_s:
            raise ValueError('timeout too short for trajectory and final settle')

    def sample(self, elapsed_s):
        t = min(self.duration_s, max(0.0, elapsed_s))
        i = min(len(self.points) - 2, bisect.bisect_right(self.knots, t) - 1)
        dt = self.knots[i+1] - self.knots[i]
        u = (t - self.knots[i]) / dt
        a, b = map(_components, (self.points[i], self.points[i+1]))
        ma, mb = self.tangents[i:i+2]
        position, velocity = [], []
        for start, end, va, vb in zip(a, b, ma, mb):
            position.append((2*u**3-3*u*u+1)*start + (u**3-2*u*u+u)*dt*va
                            + (-2*u**3+3*u*u)*end + (u**3-u*u)*dt*vb)
            velocity.append(((6*u*u-6*u)*start + (-6*u*u+6*u)*end)/dt
                            + (3*u*u-4*u+1)*va + (3*u*u-2*u)*vb)
        return Waypoint(*position), BodyVelocity(*velocity)


def _limited(velocity, previous, profile, dt, wheel_rpm):
    """Project into a convex speed envelope, then slew inside that envelope."""
    vx, vy, wz = _components(velocity)
    ratio = max(1.0, math.hypot(vx, vy) / profile.max_speed_mm_s,
                abs(wz) / profile.max_yaw_speed_deg_s)
    requested = BodyVelocity(vx/ratio, vy/ratio, wz/ratio)
    rpms = tuple(wheel_rpm(requested))
    if len(rpms) != 4 or not _finite(rpms):
        raise RuntimeError('invalid wheel kinematics')
    ratio = max(1.0, max(map(abs, rpms)) / math.floor(profile.max_wheel_rpm))
    requested = BodyVelocity(*(v/ratio for v in _components(requested)))
    deltas = tuple(b-a for a, b in zip(_components(previous), _components(requested)))
    ratio = max(1.0, math.hypot(*deltas[:2]) / (profile.max_accel_mm_s2 * dt),
                abs(deltas[2]) / (profile.max_yaw_accel_deg_s2 * dt))
    return BodyVelocity(*(a+d/ratio for a, d in zip(_components(previous), deltas)))


def follow_trajectory(points, profile, *, read_pose, send_velocity, wheel_rpm,
                      check, emergency_stop, initial_velocity=None,
                      clock=time.monotonic, sleep=time.sleep):
    """Track a single route; faults raise and latch the caller's emergency stop.

    ``check`` must reject cancellation, reconnect generation changes and emergency
    stops. It is invoked before every sample and output. send_velocity must return
    True on success. Pose velocity is measured in the current body frame.
    """
    profile.validate()  # Invalid configuration must never emit motor commands.
    points = tuple(points)
    CubicRoute(points, profile)
    started = clock()
    try:
        check()
        initial = read_pose()
        previous = initial.velocity if initial_velocity is None else initial_velocity
        route = CubicRoute(points, profile, previous)
        capped = _limited(previous, previous, profile, profile.control_period_s, wheel_rpm)
        if any(abs(a-b) > 1e-6 for a, b in zip(_components(capped), _components(previous))):
            raise RuntimeError('entry velocity exceeds validated trajectory envelope')
        last_tick, settled_at = started, None
        last_stamp = None
        while True:
            now = clock()
            check()
            pose = read_pose()
            if (not _finite((pose.x_mm, pose.y_mm, pose.yaw_deg, pose.received_at,
                             *_components(pose.velocity)))
                    or not 0 <= now-pose.received_at <= profile.max_telemetry_age_s):
                raise RuntimeError('trajectory telemetry invalid or stale')
            if last_stamp is not None and pose.received_at < last_stamp:
                raise RuntimeError('trajectory telemetry continuity lost')
            last_stamp = pose.received_at
            elapsed = now - started
            if elapsed >= profile.timeout_s:
                raise TimeoutError('continuous trajectory did not settle before timeout')
            dt = min(profile.control_period_s, max(0.0, now-last_tick))
            if dt == 0:
                dt = profile.control_period_s
            last_tick = now
            target, feedforward = route.sample(elapsed)
            dx, dy = target.x_mm-pose.x_mm, target.y_mm-pose.y_mm
            yaw_error = target.yaw_deg-pose.yaw_deg
            if (math.hypot(dx, dy) > profile.max_tracking_error_mm
                    or abs(yaw_error) > profile.max_tracking_yaw_error_deg):
                raise RuntimeError('trajectory tracking error exceeded validated envelope')
            at_end = (elapsed >= route.duration_s
                      and math.hypot(dx, dy) <= profile.position_tolerance_mm
                      and abs(yaw_error) <= profile.yaw_tolerance_deg)
            if at_end:
                desired = BodyVelocity()
            else:
                vx = feedforward.vx_mm_s + profile.position_gain_s*dx
                vy = feedforward.vy_mm_s + profile.position_gain_s*dy
                angle = math.radians(pose.yaw_deg)
                desired = BodyVelocity(math.cos(angle)*vx + math.sin(angle)*vy,
                                       -math.sin(angle)*vx + math.cos(angle)*vy,
                                       feedforward.yaw_deg_s + profile.yaw_gain_s*yaw_error)
            command = _limited(desired, previous, profile, dt, wheel_rpm)
            check()
            if send_velocity(command) is not True:
                raise RuntimeError('trajectory velocity send failed')
            previous = command
            settled = (at_end and command == BodyVelocity()
                       and math.hypot(pose.velocity.vx_mm_s, pose.velocity.vy_mm_s)
                       <= profile.settle_speed_mm_s
                       and abs(pose.velocity.yaw_deg_s) <= profile.settle_yaw_speed_deg_s)
            if settled:
                if settled_at is None:
                    settled_at = pose.received_at
                if pose.received_at-settled_at >= profile.settle_time_s:
                    check()
                    return TrajectoryResult(elapsed, pose, len(route.points)-1)
            else:
                settled_at = None
            sleep(max(0.0, profile.control_period_s-(clock()-now)))
    except BaseException as original:
        try:
            emergency_stop()
        except BaseException as stop_error:
            if hasattr(original, 'add_note'):
                original.add_note(f'trajectory emergency stop also failed: {stop_error}')
        raise
