"""Read the original route functions without sending hardware commands.

The local optimizer records original commands without hardware side effects.
Distances, ordered turns, measured search compensation and barriers stay owned
by the original route functions; no field map or assumed start is involved.
"""
import math
from types import SimpleNamespace
from control.chassis import NORMAL_DISTANCE_MOVE_ACCEL_MS, LONG_DISTANCE_MOVE_SPEED_MM_S
from control.trajectory import Waypoint
from ..common import wrap_angle
from ..settings import FLAT_ROUTE_SPEED_MM_S

CLEARANCE_RETREATS = {'ground_to_delivery','purple_to_orange','orange_to_build',
                     'build_return','staged_to_build','staged_return_first','staged_return_final'}


class RouteRecorder:
    """Read-only stand-in for the original route's hardware-facing methods."""
    def __init__(self, env, profile, *, heading_cw_deg=None):
        self.env, self.profile = env, profile
        self.source = env.control(profile)
        self.config = self.source.config
        self.context = env.context
        self.data = dict(env.data)
        self.heading = self.source._heading_error(0) if heading_cw_deg is None else heading_cw_deg
        self.pose = Waypoint(0.,0.,0.)
        self.events = []
        self.robot = SimpleNamespace(telem=SimpleNamespace(yaw_deg=wrap_angle(-self.heading)),
            chassis=SimpleNamespace(turn=self.turn), move_chassis=self._checked_move,
            reset_field_localization_filter=lambda:self.events.append(('tag_reset',self.pose,(),{})))

    def control(self, profile):
        if profile != self.profile: raise ValueError('route changed its profile while recording')
        return self

    def _check_active(self): self.context.check_active()
    def phase(self, *args): self._check_active()
    _wrap_angle = staticmethod(wrap_angle)
    def _heading_error(self, target): return wrap_angle(self.heading+self.pose.yaw_deg-target)
    def _measure_lateral_displacement_mm(self, origin): return self.source._measure_lateral_displacement_mm(origin)

    def _checked_move(self, direction, distance_mm, speed_mm_s, **kwargs):
        self._check_active()
        if not all(math.isfinite(v) and v >= 0 for v in (distance_mm,speed_mm_s)) or speed_mm_s == 0:
            raise ValueError('invalid source route motion')
        if kwargs.get('accel_ms') is None:
            kwargs['accel_ms'] = getattr(self.config,'delivery_linear_accel_ms',NORMAL_DISTANCE_MOVE_ACCEL_MS)
            if speed_mm_s>=LONG_DISTANCE_MOVE_SPEED_MM_S:
                kwargs['accel_ms']=max(kwargs['accel_ms'],round(
                    self.config.long_distance_forward_accel_ms*speed_mm_s/FLAT_ROUTE_SPEED_MM_S))
        dx,dy = {'forward':(distance_mm,0), 'backward':(-distance_mm,0),
                 'right':(0,distance_mm), 'left':(0,-distance_mm)}[direction]
        angle = math.radians(self.pose.yaw_deg)
        self.pose = Waypoint(self.pose.x_mm+math.cos(angle)*dx-math.sin(angle)*dy,
                             self.pose.y_mm+math.sin(angle)*dx+math.cos(angle)*dy,self.pose.yaw_deg)
        self.events.append(('move', self.pose, (direction,distance_mm,speed_mm_s), kwargs))
        return SimpleNamespace(timed_out=False,cancelled=False)

    def turn(self, degrees, speed, **kwargs):
        self._check_active()
        self.pose = Waypoint(self.pose.x_mm,self.pose.y_mm,self.pose.yaw_deg+degrees)
        self.robot.telem.yaw_deg = wrap_angle(-self.heading-self.pose.yaw_deg)
        self.events.append(('turn',self.pose,(degrees,speed),kwargs))

    def _turn_to_heading(self, target, **kwargs):
        self.turn(-self._heading_error(target),self.config.delivery_turn_speed_deg_s,**kwargs)

    def _drive_until_wall(self, **kwargs):
        self.events.append(('wall',self.pose,(),kwargs))

    def _recalibrate_heading_zero(self, *args):
        self.events.append(('rebase',self.pose,args,{}))

    def _align_delivery_tag_or_continue(self, **kwargs):
        self.events.append(('tag_align',self.pose,(),kwargs))

