"""Virtual-time mecanum plant used with the production trajectory controller.

All dynamics/limits in this module are simulation assumptions, not field tuning.
No Robot, Transport, serial port, camera, or firmware connection is constructed.
"""
from dataclasses import asdict, dataclass
import math

from control.chassis import (Chassis, COUNTS_PER_CM, ENCODER_COUNTS_PER_REV,
                             LATERAL_DISTANCE_SCALE, MECANUM_RPM_PER_CM_S,
                             TURN_DEG_S_TO_RPM)
from control.trajectory import (BodyVelocity, CubicRoute, PoseSample,
                                TrajectoryProfile, Waypoint, follow_trajectory)


@dataclass(frozen=True)
class SimulationSettings:
    """Shared hypothetical limits and actuator response for every compared mode."""
    wheel_response_s: float = .08
    lateral_traction: float = 1.0
    max_speed_mm_s: float = 600.0
    max_accel_mm_s2: float = 800.0
    max_yaw_speed_deg_s: float = 90.0
    max_yaw_accel_deg_s2: float = 140.0
    max_wheel_rpm: float = 3000.0
    control_period_s: float = .02
    timeout_s: float = 60.0
    max_tracking_error_mm: float = 300.0
    max_tracking_yaw_error_deg: float = 60.0
    fault_after_s: float | None = None

    def __post_init__(self):
        for name, value in asdict(self).items():
            if name == 'fault_after_s' and value is None:
                continue
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name}: simulation values must be finite and positive')
        self.controller_profile().validate()

    def controller_profile(self):
        # The real follower requires this gate. It is asserted only on an
        # in-memory profile bound to this simulated plant; never export it as
        # a competition TransitionConfig or a claimed calibration record.
        return TrajectoryProfile(
            validated=True, max_speed_mm_s=self.max_speed_mm_s,
            max_accel_mm_s2=self.max_accel_mm_s2,
            max_yaw_speed_deg_s=self.max_yaw_speed_deg_s,
            max_yaw_accel_deg_s2=self.max_yaw_accel_deg_s2,
            max_wheel_rpm=self.max_wheel_rpm, position_gain_s=3.0, yaw_gain_s=3.0,
            position_tolerance_mm=5.0, yaw_tolerance_deg=1.0,
            settle_speed_mm_s=5.0, settle_yaw_speed_deg_s=1.0,
            settle_time_s=.1, max_tracking_error_mm=self.max_tracking_error_mm,
            max_tracking_yaw_error_deg=self.max_tracking_yaw_error_deg,
            timeout_s=self.timeout_s, control_period_s=self.control_period_s,
            max_telemetry_age_s=.2)


def as_point(point):
    return {'x_mm': round(point.x_mm, 6), 'y_mm': round(point.y_mm, 6),
            'yaw_deg': round(point.yaw_deg, 6)}


def wheel_rpm(velocity):
    return Chassis.mecanum_rpm(velocity.vx_mm_s/10.0,
                              velocity.vy_mm_s*LATERAL_DISTANCE_SCALE/10.0,
                              -velocity.yaw_deg_s)


def body_velocity(rpms):
    tr, tl, bl, br = rpms
    return BodyVelocity((-tr+tl+bl-br)*10.0/(4*MECANUM_RPM_PER_CM_S),
                        (tr+tl-bl-br)*10.0/(4*MECANUM_RPM_PER_CM_S*LATERAL_DISTANCE_SCALE),
                        sum(rpms)/(4*TURN_DEG_S_TO_RPM))


class MecanumPlant:
    """Four first-order wheel motors, encoder quantization and ideal wrapped IMU.

    Wheel rotation drives an independent physical pose and an encoder/IMU pose
    estimate. ``lateral_traction`` affects physical motion only, demonstrating
    that encoder feedback cannot observe lateral slip. No collision forces,
    pitch/ramps, static friction, payload, servo, or camera dynamics are modeled.
    """
    def __init__(self, settings=None):
        self.settings = settings or SimulationSettings()
        self.profile = self.settings.controller_profile()
        self.now = 0.0
        self.x = self.y = self.yaw = 0.0
        self.ox = self.oy = self.oyaw = 0.0
        self.rpms = [0.0]*4
        self.targets = [0.0]*4
        self.counts = [0.0]*4
        self._last_counts = [0]*4
        self._last_imu = 0.0
        self._origin = Waypoint(0, 0, 0)
        self._reference = None
        self._reference_started = 0.0
        self.stopped = False
        self.emergency_stops = 0
        self.controller_runs = 0
        self.trace = []
        self.references = []
        self.barriers = []
        self.path_length = 0.0
        self.max_speed = self.max_yaw_speed = self.max_wheel = 0.0
        self.max_tracking_error = self.max_yaw_error = self.max_odometry_error = 0.0
        self._record()

    def guard(self):
        if self.stopped:
            raise RuntimeError('simulation controller was stopped')
        if self.settings.fault_after_s is not None and self.now >= self.settings.fault_after_s:
            raise RuntimeError('injected simulation communication fault')

    def measured_velocity(self):
        return body_velocity(tuple(round(r) for r in self.rpms))

    def read_pose(self):
        theta = math.radians(self._origin.yaw_deg)
        dx, dy = self.ox-self._origin.x_mm, self.oy-self._origin.y_mm
        sample = PoseSample(math.cos(theta)*dx+math.sin(theta)*dy,
                            -math.sin(theta)*dx+math.cos(theta)*dy,
                            self.oyaw-self._origin.yaw_deg,
                            self.measured_velocity(), self.now)
        if self._reference is not None:
            target, _ = self._reference.sample(self.now-self._reference_started)
            self.max_tracking_error = max(self.max_tracking_error,
                                          math.hypot(target.x_mm-sample.x_mm,
                                                     target.y_mm-sample.y_mm))
            self.max_yaw_error = max(self.max_yaw_error, abs(target.yaw_deg-sample.yaw_deg))
        return sample

    def send_velocity(self, command):
        self.guard()
        self.targets = [round(r) for r in wheel_rpm(command)]
        self.max_wheel = max(self.max_wheel, max(map(abs, self.targets)))
        return True

    def emergency_stop(self):
        self.emergency_stops += 1
        self.stopped = True
        self.targets = [0.0]*4
        return True

    def _telemetry(self):
        rounded = [round(c) for c in self.counts]
        tr, tl, bl, br = [new-old for new, old in zip(rounded, self._last_counts)]
        dx = (-tr+tl+bl-br)*10.0/(4*COUNTS_PER_CM)
        dy = (tr+tl-bl-br)*10.0/(4*COUNTS_PER_CM*LATERAL_DISTANCE_SCALE)
        imu = (-self.yaw+180.0) % 360.0-180.0
        delta = -((imu-self._last_imu+180.0) % 360.0-180.0)
        theta = math.radians(self.oyaw+delta/2)
        self.ox += math.cos(theta)*dx-math.sin(theta)*dy
        self.oy += math.sin(theta)*dx+math.cos(theta)*dy
        self.oyaw += delta
        self._last_counts, self._last_imu = rounded, imu

    def sleep(self, duration):
        if not math.isfinite(duration) or duration < 0:
            raise ValueError('invalid virtual time advance')
        remaining = duration
        while remaining > 1e-10:
            dt = min(.005, remaining)
            decay = math.exp(-dt/self.settings.wheel_response_s)
            averaged = [target+(rpm-target)*self.settings.wheel_response_s*(1-decay)/dt
                        for rpm, target in zip(self.rpms, self.targets)]
            self.rpms = [target+(rpm-target)*decay for rpm, target in zip(self.rpms, self.targets)]
            velocity = body_velocity(averaged)
            vy = velocity.vy_mm_s*self.settings.lateral_traction
            theta = math.radians(self.yaw+velocity.yaw_deg_s*dt/2)
            dx = (math.cos(theta)*velocity.vx_mm_s-math.sin(theta)*vy)*dt
            dy = (math.sin(theta)*velocity.vx_mm_s+math.cos(theta)*vy)*dt
            self.x += dx
            self.y += dy
            self.yaw += velocity.yaw_deg_s*dt
            self.path_length += math.hypot(dx, dy)
            self.counts = [c+r*ENCODER_COUNTS_PER_REV/60.0*dt
                           for c, r in zip(self.counts, averaged)]
            self.now += dt
            remaining -= dt
        self._telemetry()
        self._record()

    def _record(self):
        velocity = body_velocity(self.rpms)
        vy = velocity.vy_mm_s*self.settings.lateral_traction
        self.max_speed = max(self.max_speed, math.hypot(velocity.vx_mm_s, vy))
        self.max_yaw_speed = max(self.max_yaw_speed, abs(velocity.yaw_deg_s))
        self.max_odometry_error = max(self.max_odometry_error, math.hypot(self.x-self.ox, self.y-self.oy))
        self.trace.append(dict(t=self.now, x=self.x, y=self.y, yaw=self.yaw,
                               vx=velocity.vx_mm_s, vy=vy, wz=velocity.yaw_deg_s))

    def follow_local(self, points, *, profile=None, initial_velocity=None):
        self.guard()
        profile = profile or self.profile
        points = tuple(points)
        self._origin = Waypoint(self.ox, self.oy, self.oyaw)
        initial_velocity = initial_velocity or self.measured_velocity()
        self._reference = CubicRoute(points, profile, initial_velocity)
        self._reference_started = self.now
        self.controller_runs += 1
        self.references.append({'started_at_s': self.now, 'origin': as_point(self._origin),
                                'points': [as_point(p) for p in points]})
        try:
            return follow_trajectory(points, profile, read_pose=self.read_pose,
                                     send_velocity=self.send_velocity, wheel_rpm=wheel_rpm,
                                     check=self.guard, emergency_stop=self.emergency_stop,
                                     initial_velocity=initial_velocity,
                                     clock=lambda: self.now, sleep=self.sleep)
        finally:
            self._reference = None

    def barrier(self, kind, **details):
        self.barriers.append(dict(kind=kind, t=round(self.now, 6),
                                  x=round(self.x, 6), y=round(self.y, 6),
                                  simulated_motion=False, **details))

    def metrics(self):
        return {k: round(v, 3) for k, v in {
            'elapsed_s': self.now, 'path_length_mm': self.path_length,
            'max_speed_mm_s': self.max_speed, 'max_yaw_speed_deg_s': self.max_yaw_speed,
            'max_wheel_rpm': self.max_wheel, 'max_tracking_error_mm': self.max_tracking_error,
            'max_tracking_yaw_error_deg': self.max_yaw_error,
            'max_odometry_error_mm': self.max_odometry_error,
            'controller_runs': self.controller_runs, 'emergency_stops': self.emergency_stops,
        }.items()}

    def export_trace(self, maximum=100):
        if type(maximum) is not int or maximum < 2:
            raise ValueError('trace limit must be an integer of at least two')
        count = min(maximum, len(self.trace))
        indices = sorted({round(i*(len(self.trace)-1)/(count-1)) for i in range(count)}) if count > 1 else [0]
        return [{k: round(v, 3) for k, v in self.trace[i].items()} for i in indices]
