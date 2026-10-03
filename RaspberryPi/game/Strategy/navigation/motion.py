"""Competition-sourced motion settings; added curve limits remain explicit.

The old controller owns its feedback gains. A continuous path uses a spatial
velocity envelope, with the same cubic start ramp and legacy braking distance.
This does not claim identical closed-loop timing or measured curve capability.
"""
from dataclasses import asdict, dataclass
import math
import numpy as np
from control.chassis import (COUNTS_PER_CM, MECANUM_RPM_PER_CM_S,
    FWD_BASE_DECEL_DIST, FWD_BASE_SPEED_RPM, TURN_ACCEL_MS,
    FWD_BASE_PID_LIMIT, DEFAULT_POS_KP, FWD_SETTLE_SPEED_RPM, LATERAL_DISTANCE_SCALE, TURN_BASE_SPEED_DEG_S,
    NORMAL_DISTANCE_MOVE_SPEED_MM_S, NORMAL_DISTANCE_MOVE_ACCEL_MS,
    ROUTE_ARRIVAL_TOLERANCE_MM, FWD_SETTLE_MS)
from control.motion_law import smoothstep
from ..settings import PROFILES


@dataclass(frozen=True)
class CompetitionMotion:
    cruise_mm_s: float
    short_mm_s: float
    ramp_mm_s: float
    wall_mm_s: float
    approach_mm_s: float
    accel_s: float
    short_accel_s: float
    yaw_deg_s: float
    yaw_accel_s: float
    short_distance_mm: float
    arrival_mm: float = ROUTE_ARRIVAL_TOLERANCE_MM
    completion_reserve_mm: float = 1.0
    settle_s: float = FWD_SETTLE_MS / 1000
    # Additional continuous-curve constraints, not inherited field measurements.
    curve_accel_mm_s2: float = 800.0
    cruise_multiplier: float = 1.10
    wheel_rpm: float | None = None
    curve_yaw_deg_s: float = TURN_BASE_SPEED_DEG_S
    feedforward_lead_s: float = .10  # .08 s model response + .02 s control tick
    response_allowance_s: float = .18
    transit_margin_mm: float = 100.0

    def __post_init__(self):
        if self.wheel_rpm is None:
            object.__setattr__(self,'wheel_rpm',self.command_mm_s*MECANUM_RPM_PER_CM_S/10*LATERAL_DISTANCE_SCALE)
        for key, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int,float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'invalid motion setting: {key}')
        if not self.approach_mm_s <= self.wall_mm_s <= self.cruise_mm_s:
            raise ValueError('inconsistent approach/wall/cruise speeds')
        if self.cruise_multiplier>1.2:
            raise ValueError('cruise multiplier above 1.20 requires a reviewed motion profile')
        if self.completion_reserve_mm>=self.arrival_mm:
            raise ValueError('completion reserve must fit inside the arrival window')

    @classmethod
    def competition(cls):
        departure, ground, highland = (PROFILES[k] for k in ('depart-b','ground-1','highland-1'))
        return cls(departure.speed_mm_s, NORMAL_DISTANCE_MOVE_SPEED_MM_S,
                   min(highland.initial_speed_mm_s, highland.build_route_speed_mm_s),
                   ground.search_speed_mm_s, ground.near_wall_speed_mm_s,
                   departure.accel_ms/1000, NORMAL_DISTANCE_MOVE_ACCEL_MS/1000,
                   departure.turn_speed_deg_s, TURN_ACCEL_MS/1000,
                   highland.compensation_fast_distance_mm)

    @property
    def acceleration(self):
        # Peak derivative of cubic smoothstep, not speed/time treated as a constant.
        return max((1.5*self.travel_mm_s+self.correction_limit(self.travel_mm_s))/self.accel_s,
                   (1.5*self.short_mm_s+self.correction_limit(self.short_mm_s))/self.short_accel_s)

    @staticmethod
    def correction_limit(speed): return speed*FWD_BASE_PID_LIMIT/FWD_BASE_SPEED_RPM

    @property
    def travel_mm_s(self): return self.cruise_mm_s*self.cruise_multiplier

    @property
    def command_mm_s(self): return self.travel_mm_s+self.correction_limit(self.travel_mm_s)

    @property
    def settled_mm_s(self): return FWD_SETTLE_SPEED_RPM*10/MECANUM_RPM_PER_CM_S

    @property
    def jerk(self):
        return max(6*self.travel_mm_s/self.accel_s**2, 6*self.short_mm_s/self.short_accel_s**2)

    @property
    def yaw_acceleration(self): return 1.5*self.yaw_deg_s/self.yaw_accel_s

    @property
    def yaw_jerk(self): return 6*self.yaw_deg_s/self.yaw_accel_s**2

    def braking_distance(self, speed):
        # Same conversion as Chassis._move_linear; wheel scaling cancels for a
        # pure lateral move because it applies to distance AND command speed.
        return FWD_BASE_DECEL_DIST * (speed*MECANUM_RPM_PER_CM_S/10/FWD_BASE_SPEED_RPM) * 10/COUNTS_PER_CM

    @property
    def braking_acceleration(self):
        return self.travel_mm_s**2 / (2*self.braking_distance(self.travel_mm_s))

    def clearance_speed(self, clearance):
        # Tangential motion near a wall can retain search speed. Inward motion
        # is independently limited by the executor's stopping envelope.
        return np.where(np.asarray(clearance)>=self.transit_margin_mm+25, self.command_mm_s, self.wall_mm_s*.9)

    def describe(self):
        return dict(values=asdict(self), source='Strategy.settings.PROFILES + control.chassis',
                    derived=dict(acceleration_mm_s2=self.acceleration, jerk_mm_s3=self.jerk,
                                 cruise_setpoint_mm_s=self.travel_mm_s,command_ceiling_mm_s=self.command_mm_s,
                                 cruise_braking_distance_mm=self.braking_distance(self.travel_mm_s)),
                    curve_limits_field_validated=False)


def spatial_ramp(distance, speed, duration, correction=0):
    """Integrate the existing cubic v(t), then invert s(t) monotonically."""
    u=np.linspace(0,1,1001)
    position=duration*(speed*(u**3-.5*u**4)+.5*correction*u**2)
    return np.interp(distance,position,speed*smoothstep(u)+correction*u)


def route_timing(points, motion=None):
    motion=motion or CompetitionMotion.competition()
    if len(points)==1: return np.array([0.]),np.array([0.])
    arc=points[:,3]; ds=np.diff(arc); v=points[:,5].copy()
    speed=motion.short_mm_s if arc[-1]<motion.short_distance_mm else motion.travel_mm_s
    duration=motion.short_accel_s if arc[-1]<motion.short_distance_mm else motion.accel_s
    correction=motion.correction_limit(speed)
    v=np.minimum(v,speed+correction)
    v=np.minimum(v,spatial_ramp(arc,speed,duration,correction))
    # Classic distance-S braking plus bounded proportional position correction.
    # Integral/yaw feedback is still controller-specific, not replayed as truth.
    remaining=arc[-1]-arc
    brake_ratio=np.clip(remaining/motion.braking_distance(speed),0,1)
    proportional=DEFAULT_POS_KP*COUNTS_PER_CM/MECANUM_RPM_PER_CM_S*remaining
    v=np.minimum(v,speed*smoothstep(brake_ratio)+np.minimum(correction,proportional))
    yaw_slope=np.abs(np.gradient(points[:,2],arc))
    accel=np.minimum(motion.acceleration,.5*motion.yaw_acceleration/np.maximum(yaw_slope,1e-6))
    brake=np.minimum(motion.braking_acceleration,.5*motion.yaw_acceleration/np.maximum(yaw_slope,1e-6))
    v[0]=v[-1]=0
    for i,d in enumerate(ds): v[i+1]=min(v[i+1],math.sqrt(v[i]**2+2*min(accel[i:i+2])*d))
    for i in range(len(ds)-1,-1,-1): v[i]=min(v[i],math.sqrt(v[i+1]**2+2*min(brake[i:i+2])*ds[i]))
    return np.r_[0,np.cumsum(2*ds/np.maximum(v[:-1]+v[1:],1e-5))],v
