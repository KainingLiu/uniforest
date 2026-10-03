"""Opt-in local cubic routes. Units: mm, seconds, unwrapped clockwise degrees.

Profiles are field validated or explicitly generated from existing route commands.
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
    route_derived: bool = False
    segment_speeds_mm_s: tuple = ()
    corner_tangent_deviation_mm: float = 40.0
    monotone_xy: bool = False

    def validate(self):
        if any(type(v) is not bool for v in (self.route_derived,self.validated,self.monotone_xy)):
            raise ValueError('trajectory provenance flags must be boolean')
        if self.validated is not True and not self.route_derived:
            raise ValueError('continuous trajectory profile requires field validation')
        for field in fields(self):
            if field.name not in ('validated', 'route_derived', 'segment_speeds_mm_s', 'monotone_xy'):
                value = getattr(self, field.name)
                if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                    raise ValueError(f'{field.name} must be finite and positive')
        if not isinstance(self.segment_speeds_mm_s,(tuple,list)) or any(
                isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not 0<v<=self.max_speed_mm_s
                for v in self.segment_speeds_mm_s):
            raise ValueError('invalid per-segment speed limits')
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
        self.profile = profile
        self.initial_velocity = initial_velocity
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
        self.speed_limits=tuple(profile.segment_speeds_mm_s) or (profile.max_speed_mm_s,)*(len(self.points)-1)
        if len(self.speed_limits)!=len(self.points)-1:
            raise ValueError('one speed limit per route segment is required')
        for a, b, speed_limit in zip(self.points, self.points[1:],self.speed_limits):
            distance = math.hypot(b.x_mm - a.x_mm, b.y_mm - a.y_mm)
            angle = abs(b.yaw_deg - a.yaw_deg)
            if distance == 0 and angle == 0:
                raise ValueError('adjacent duplicate waypoints are invalid')
            duration = max(1.5 * distance / speed_limit,
                           1.5 * angle / profile.max_yaw_speed_deg_s,
                           # A cubic zero-endpoint-velocity segment peaks at
                           # 6*d/T^2 acceleration. The former 4*d/T^2 estimate
                           # outran the controller when cruise rose to 2 m/s.
                           math.sqrt(6 * distance / profile.max_accel_mm_s2),
                           math.sqrt(6 * angle / profile.max_yaw_accel_deg_s2),
                           profile.control_period_s)
            self.knots.append(self.knots[-1] + duration)
        self.tangents = [_components(initial_velocity)]
        for i in range(1, len(self.points) - 1):
            before, after = map(_components, (self.points[i-1], self.points[i+1]))
            dt = self.knots[i+1] - self.knots[i-1]
            tangent = tuple((b-a)/dt for a, b in zip(before, after))
            if profile.segment_speeds_mm_s:
                speed_limit=min(self.speed_limits[i-1:i+1])
                translation_scale=min(1.,speed_limit/max(math.hypot(*tangent[:2]),1e-9))
                tangent=(tangent[0]*translation_scale,tangent[1]*translation_scale,tangent[2])
            if profile.route_derived:
                # Bound the generated path's departure from its original line
                # segments. This is a software smoothing limit, not surveyed
                # obstacle clearance. Clearance retreats remain separate runs.
                adjacent = max(self.knots[i]-self.knots[i-1], self.knots[i+1]-self.knots[i])
                left,right=self.points[i-1],self.points[i+1]
                current=self.points[i]
                ax,ay=current.x_mm-left.x_mm,current.y_mm-left.y_mm
                bx,by=right.x_mm-current.x_mm,right.y_mm-current.y_mm
                straight=ax*bx+ay*by>.99999*math.hypot(ax,ay)*math.hypot(bx,by)
                limit=(3*min(math.hypot(ax,ay)/(self.knots[i]-self.knots[i-1]),
                             math.hypot(bx,by)/(self.knots[i+1]-self.knots[i])) if straight else
                       3*profile.corner_tangent_deviation_mm/adjacent)
                scale = min(1.0, limit/max(math.hypot(*tangent[:2]), 1e-9))
                tangent = (tangent[0]*scale, tangent[1]*scale, tangent[2])
                # A constant-heading entry span must stay constant. Central
                # differences otherwise bend yaw the wrong way before a turn.
                before_yaw=(current.yaw_deg-left.yaw_deg)/(self.knots[i]-self.knots[i-1])
                after_yaw=(right.yaw_deg-current.yaw_deg)/(self.knots[i+1]-self.knots[i])
                yaw_tangent=(0. if before_yaw*after_yaw<=0 else
                             math.copysign(min(abs(tangent[2]),3*min(abs(before_yaw),abs(after_yaw))),before_yaw))
                tangent=(tangent[0],tangent[1],yaw_tangent)
            if profile.monotone_xy:
                # Shape-preserving Hermite slopes: each coordinate stays
                # between its segment endpoints, eliminating outward lobes.
                current = _components(self.points[i])
                limited = []
                for axis in (0,1):
                    incoming = (current[axis]-before[axis])/(self.knots[i]-self.knots[i-1])
                    outgoing = (after[axis]-current[axis])/(self.knots[i+1]-self.knots[i])
                    limited.append(0. if incoming*outgoing <= 0 else math.copysign(
                        min(abs(tangent[axis]),3*min(abs(incoming),abs(outgoing))),incoming))
                tangent = (*limited,tangent[2])
            self.tangents.append(tangent)
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

    def speed_limit_at(self, elapsed_s):
        i=max(0,min(len(self.speed_limits)-1,bisect.bisect_right(self.knots,elapsed_s)-1))
        return self.speed_limits[i]


def _limited(velocity, previous, profile, dt, wheel_rpm, *, speed_limit=None):
    """Project into a convex speed envelope, then slew inside that envelope."""
    vx, vy, wz = _components(velocity)
    speed_limit=profile.max_speed_mm_s if speed_limit is None else min(speed_limit,profile.max_speed_mm_s)
    ratio = max(1.0, math.hypot(vx, vy) / speed_limit,
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
                      clock=time.monotonic, sleep=time.sleep, guidance=None):
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
        reference_time = 0.0
        while True:
            check()
            pose = read_pose()
            now = clock()
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
            sample_time = reference_time if profile.route_derived else elapsed
            target, feedforward = route.sample(sample_time)
            reference_done = sample_time >= route.duration_s
            completion_ready = True
            speed_limit = route.speed_limit_at(sample_time)
            if guidance is not None:
                reference = guidance.update(route, sample_time, pose, now)
                target, feedforward = reference.target, reference.velocity
                reference_done, completion_ready = reference.finished, reference.ready
                speed_limit = min(speed_limit, reference.speed_limit_mm_s)
                if not _finite((*_components(target), *_components(feedforward),speed_limit)) or speed_limit<=0:
                    raise RuntimeError('invalid visual trajectory reference')
            dx, dy = target.x_mm-pose.x_mm, target.y_mm-pose.y_mm
            yaw_error = target.yaw_deg-pose.yaw_deg
            if (math.hypot(dx, dy) > profile.max_tracking_error_mm
                    or abs(yaw_error) > profile.max_tracking_yaw_error_deg):
                raise RuntimeError('trajectory tracking error exceeded validated envelope')
            at_end = (reference_done
                      and math.hypot(dx, dy) <= profile.position_tolerance_mm
                      and abs(yaw_error) <= profile.yaw_tolerance_deg)
            if at_end:
                desired = BodyVelocity()
            else:
                if profile.route_derived:
                    # A slower corner/visual envelope slows the reference too,
                    # preventing a fast nominal clock from running away.
                    rate=min(1.,max(0.,1-math.hypot(dx,dy)/profile.max_tracking_error_mm),
                        max(0.,1-abs(yaw_error)/profile.max_tracking_yaw_error_deg),
                        speed_limit/max(speed_limit,math.hypot(feedforward.vx_mm_s,feedforward.vy_mm_s)))
                    reference_time += dt*rate
                    feedforward=BodyVelocity(*(v*rate for v in _components(feedforward)))
                vx = feedforward.vx_mm_s + profile.position_gain_s*dx
                vy = feedforward.vy_mm_s + profile.position_gain_s*dy
                angle = math.radians(pose.yaw_deg)
                desired = BodyVelocity(math.cos(angle)*vx + math.sin(angle)*vy,
                                       -math.sin(angle)*vx + math.cos(angle)*vy,
                                       feedforward.yaw_deg_s + profile.yaw_gain_s*yaw_error)
            command = _limited(desired, previous, profile, dt, wheel_rpm,
                               speed_limit=speed_limit)
            check()
            if send_velocity(command) is not True:
                raise RuntimeError('trajectory velocity send failed')
            previous = command
            settled = (at_end and completion_ready and command == BodyVelocity()
                       and math.hypot(pose.velocity.vx_mm_s, pose.velocity.vy_mm_s)
                       <= profile.settle_speed_mm_s
                       and abs(pose.velocity.yaw_deg_s) <= profile.settle_yaw_speed_deg_s)
            if settled:
                if settled_at is None:
                    settled_at = pose.received_at
                if pose.received_at-settled_at >= profile.settle_time_s:
                    check()
                    if guidance is not None:
                        guidance.finish()
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
