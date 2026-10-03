"""World-frame position PID, independent of route generation and hardware.

Derivative uses reference minus measured velocity, so a position observation
correction cannot create a numerical derivative impulse. Integral is bounded,
separated from large errors, and conditionally rolled back after output limits.
All gains are initial navigation settings, not new hardware calibration facts.
"""
from dataclasses import asdict, dataclass
import math
import numpy as np


@dataclass(frozen=True)
class TrackingGains:
    xy_kp: float = 2.5
    xy_ki: float = .12
    xy_kd: float = .08
    yaw_kp: float = 2.5
    yaw_ki: float = .10
    yaw_kd: float = .05
    xy_integral_limit_mm_s: float = 40.0
    yaw_integral_limit_deg_s: float = 8.0
    xy_output_limit_mm_s: float = 350.0
    yaw_output_limit_deg_s: float = 65.0
    xy_integral_zone_mm: float = 40.0
    yaw_integral_zone_deg: float = 5.0
    derivative_filter_s: float = .06

    def __post_init__(self):
        for key,value in asdict(self).items():
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<0:
                raise ValueError(f'invalid tracking gain: {key}')
            if not key.endswith(('_kp','_ki','_kd')) and value==0:
                raise ValueError(f'tracking bound must be positive: {key}')


class PositionTracker:
    """step returns [world vx mm/s, world vy mm/s, CW yaw deg/s] correction."""
    def __init__(self,gains=None):
        self.gains=gains or TrackingGains()
        g=self.gains
        self.kp=np.array([g.xy_kp,g.xy_kp,g.yaw_kp])
        self.ki=np.array([g.xy_ki,g.xy_ki,g.yaw_ki])
        self.kd=np.array([g.xy_kd,g.xy_kd,g.yaw_kd])
        self.i_limit=np.array([g.xy_integral_limit_mm_s]*2+[g.yaw_integral_limit_deg_s])
        self.limit=np.array([g.xy_output_limit_mm_s]*2+[g.yaw_output_limit_deg_s])
        self.zone=np.array([g.xy_integral_zone_mm]*2+[g.yaw_integral_zone_deg])
        self.reset()

    def reset(self):
        self.integral=np.zeros(3); self.derivative=np.zeros(3)
        self.previous_integral=np.zeros(3); self.requested=np.zeros(3)

    def step(self,error,velocity_error,dt,*,integrate=True):
        e=np.asarray(error,dtype=float); dv=np.asarray(velocity_error,dtype=float)
        if e.shape!=(3,) or dv.shape!=(3,) or not np.isfinite(e).all() or not np.isfinite(dv).all() or not math.isfinite(dt) or not 0<dt<=.1:
            raise ValueError('invalid position PID input')
        e=e.copy(); e[2]=(e[2]+180)%360-180
        alpha=dt/(self.gains.derivative_filter_s+dt)
        self.derivative+=alpha*(dv-self.derivative)
        self.previous_integral=self.integral.copy()
        if integrate:
            delta=np.where(np.abs(e)<=self.zone,self.ki*e*dt,0.)
            self.integral=np.clip(self.integral+delta,-self.i_limit,self.i_limit)
        raw=self.kp*e+self.integral+self.kd*self.derivative
        # Local saturation cannot accumulate more integral in the blocked direction.
        rejected=(raw-np.clip(raw,-self.limit,self.limit))*(self.integral-self.previous_integral)>0
        self.integral[rejected]=self.previous_integral[rejected]
        self.requested=np.clip(self.kp*e+self.integral+self.kd*self.derivative,-self.limit,self.limit)
        return self.requested.copy()

    def applied(self,correction):
        """Call once with the correction left after wheel/wall/acceleration caps."""
        actual=np.asarray(correction,dtype=float)
        if actual.shape!=(3,) or not np.isfinite(actual).all(): raise ValueError('invalid applied PID correction')
        rejected=(self.requested-actual)*(self.integral-self.previous_integral)>1e-9
        self.integral[rejected]=self.previous_integral[rejected]
