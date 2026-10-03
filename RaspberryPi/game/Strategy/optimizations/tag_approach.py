"""Moving Tag6 guidance in the existing route's local frame.

Uses the same camera-to-Tag distance/lateral measurements as the existing Tag
alignment. No field-map pose or nominal robot start is required. New blending
limits are software trial bounds, not surveyed obstacle clearance.
"""
from collections import deque
from dataclasses import dataclass, replace
import math

from control.trajectory import BodyVelocity, Waypoint, CubicRoute
from ..common import wrap_angle, VisualAlignmentUnavailable


@dataclass(frozen=True)
class TagApproachProfile:
    building: bool = False
    capture_remaining_mm: float = 900.0
    entry_heading_deg: float = 25.0
    goal_jump_mm: float = 120.0
    goal_slew_mm_s: float = 200.0
    confirm_goal_frames: int = 2

    @property
    def correction_limit_mm(self):
        return 400.0 if self.building else 200.0


@dataclass(frozen=True)
class GuidedReference:
    target: Waypoint
    velocity: BodyVelocity
    finished: bool
    ready: bool
    speed_limit_mm_s: float


class TagApproach:
    def __init__(self, control, nominal_goal, prefix_index, offset_mm, purpose, profile):
        self.control, self.profile = control, profile
        self.nominal_goal, self.prefix_index, self.offset_mm = nominal_goal, prefix_index, offset_mm
        cfg = control.config
        build = purpose == 'build'
        self.distance = cfg.build_tag_distance_mm if build else cfg.delivery_tag_distance_mm
        self.distance_tolerance = cfg.build_tag_distance_tolerance_mm if build else cfg.delivery_tag_distance_tolerance_mm
        self.lateral_tolerance = cfg.build_tag_lateral_tolerance_mm if build else cfg.delivery_tag_lateral_tolerance_mm
        self.heading_tolerance = cfg.build_tag_heading_tolerance_deg if build else cfg.delivery_heading_tolerance_deg
        self.frame_age = cfg.build_tag_vision_stale_s if build else cfg.delivery_tag_vision_stale_s
        self.lost_timeout = cfg.build_tag_lost_timeout_s if build else cfg.delivery_tag_lost_timeout_s
        self.confirm_frames = cfg.delivery_tag_confirm_frames
        self.approach_speed = max(cfg.delivery_tag_fast_forward_mm_s,cfg.delivery_tag_fast_lateral_mm_s)
        target_heading = cfg.build_tag_heading_target_cw_deg if build else cfg.delivery_heading_target_cw_deg
        self.heading_matches = abs(wrap_angle(-control._heading_error(target_heading)-nominal_goal.yaw_deg)) <= 1.0
        self.history = deque()
        self.last_frame = self.last_good = self.last_tick = None
        self.pending_goal = self.observed_goal = None
        self.pending_count = self.confirmed = self.accepted_frames = 0
        self.shift_x = self.shift_y = 0.0
        self.active = self.completed = False
        self.reason = 'no_confirmed_tag6'
        self.base_route = None
        self.blend_started = None

    def _at_capture(self, stamp):
        if not self.history or not self.history[0][0] <= stamp <= self.history[-1][0]:
            return None
        values = list(self.history)
        for (a,p),(b,q) in zip(values,values[1:]):
            if a <= stamp <= b:
                u=(stamp-a)/max(b-a,1e-9)
                return Waypoint(p.x_mm+u*(q.x_mm-p.x_mm),p.y_mm+u*(q.y_mm-p.y_mm),
                                p.yaw_deg+u*(q.yaw_deg-p.yaw_deg))
        return values[-1][1]

    def _observe(self, pose, now, prefix_goal):
        p = self.profile
        if (not self.heading_matches or (not self.active and math.hypot(pose.x_mm-prefix_goal.x_mm,pose.y_mm-prefix_goal.y_mm)
                > p.capture_remaining_mm) or abs(wrap_angle(pose.yaw_deg-self.nominal_goal.yaw_deg))>p.entry_heading_deg):
            return
        frame = getattr(self.control.robot,'field_pose',None)
        stamp = getattr(frame,'captured_monotonic',None)
        if (frame is None or getattr(frame,'valid',False) is not True or not isinstance(stamp,(float,int))
                or not math.isfinite(stamp) or stamp<=0 or not 0<=now-stamp<=self.frame_age
                or self.last_frame is not None and stamp<=self.last_frame):
            return
        captured = self._at_capture(stamp)
        if captured is None:
            return
        if self.last_frame is not None and stamp-self.last_frame>self.frame_age:
            self.pending_count=self.confirmed=0
            self.pending_goal=None
        self.last_frame=stamp
        tags=[t for t in getattr(frame,'tag_solutions',()) if getattr(t,'tag_id',None)==6
              and all(isinstance(getattr(t,k,None),(int,float)) and math.isfinite(getattr(t,k))
                      for k in ('distance_m','lateral_m','score')) and t.distance_m>0]
        if not tags:
            self.pending_count=self.confirmed=0
            return
        tag=min(tags,key=lambda t:t.score)
        # Tag translation is expressed along its plane/normal. Rotate using the
        # required facing direction, not an intermediate robot turn angle.
        a=math.radians(self.nominal_goal.yaw_deg)
        dx,dy=tag.distance_m*1000-self.distance,tag.lateral_m*1000+self.offset_mm
        goal=Waypoint(captured.x_mm+math.cos(a)*dx-math.sin(a)*dy,
                      captured.y_mm+math.sin(a)*dx+math.cos(a)*dy,self.nominal_goal.yaw_deg)
        if math.hypot(goal.x_mm-self.nominal_goal.x_mm,goal.y_mm-self.nominal_goal.y_mm)>p.correction_limit_mm:
            self.pending_goal=None
            self.pending_count=self.confirmed=0
            self.reason='tag6_outside_local_correction_bound'
            return
        if self.pending_goal is not None and math.hypot(goal.x_mm-self.pending_goal.x_mm,
                goal.y_mm-self.pending_goal.y_mm)<=p.goal_jump_mm:
            self.pending_count+=1
        else:
            self.pending_count=1
            self.confirmed=0
        self.pending_goal=goal
        if self.pending_count<p.confirm_goal_frames:
            return
        self.active=True
        self.last_good=stamp
        self.observed_goal=goal
        self.accepted_frames+=1
        ex,ey=goal.x_mm-pose.x_mm,goal.y_mm-pose.y_mm
        forward,right=math.cos(a)*ex+math.sin(a)*ey,-math.sin(a)*ex+math.cos(a)*ey
        in_window=(abs(forward)<=self.distance_tolerance and abs(right)<=self.lateral_tolerance
                   and abs(wrap_angle(pose.yaw_deg-goal.yaw_deg))<=self.heading_tolerance)
        self.confirmed=self.confirmed+1 if in_window else 0
        self.reason='moving_tag6'

    def update(self, route, elapsed, pose, now):
        if self.base_route is None:
            settings=replace(route.profile,segment_speeds_mm_s=tuple(route.speed_limits[:self.prefix_index]))
            self.base_route=CubicRoute(route.points[:self.prefix_index+1],settings,route.initial_velocity)
        if not self.history or pose.received_at>self.history[-1][0]:
            self.history.append((pose.received_at,Waypoint(pose.x_mm,pose.y_mm,pose.yaw_deg)))
        while len(self.history)>2 and self.history[1][0]<now-max(1.0,self.frame_age+.1):
            self.history.popleft()
        prefix_goal=self.base_route.points[-1]
        self._observe(pose,now,prefix_goal)
        dt=0. if self.last_tick is None else min(.05,max(0.,now-self.last_tick))
        self.last_tick=now
        if not self.active:
            # Never execute the appended offset before Tag evidence. Stop at
            # the original route end and retain the original visual/offset steps.
            end=self.base_route.duration_s
            target,velocity=self.base_route.sample(elapsed)
            return GuidedReference(target,velocity,elapsed>=end,True,self.base_route.speed_limit_at(elapsed))
        if now-self.last_good>self.lost_timeout:
            raise VisualAlignmentUnavailable('Tag6 lost after moving approach handoff; stop without replay')
        if now-self.last_good>self.frame_age:
            self.confirmed=0
        gx=self.observed_goal.x_mm-self.nominal_goal.x_mm
        gy=self.observed_goal.y_mm-self.nominal_goal.y_mm
        dx,dy=gx-self.shift_x,gy-self.shift_y
        ratio=min(1.,self.profile.goal_slew_mm_s*dt/max(math.hypot(dx,dy),1e-9))
        self.shift_x+=dx*ratio
        self.shift_y+=dy*ratio
        if self.blend_started is None:
            self.blend_started=elapsed
        # Blend the existing offset into the remaining approach, rather than
        # insisting on first visiting the Tag centre and then reversing sideways.
        duration=max(.5,1.5*abs(self.offset_mm)/self.approach_speed,
                     self.base_route.duration_s-self.blend_started)
        u=max(0.,min(1.,(elapsed-self.blend_started)/duration))
        blend=u*u*(3-2*u)
        blend_rate=6*u*(1-u)/duration
        ox=self.nominal_goal.x_mm-prefix_goal.x_mm
        oy=self.nominal_goal.y_mm-prefix_goal.y_mm
        target,velocity=self.base_route.sample(elapsed)
        target=Waypoint(target.x_mm+blend*ox+self.shift_x,target.y_mm+blend*oy+self.shift_y,target.yaw_deg)
        velocity=BodyVelocity(velocity.vx_mm_s+blend_rate*ox,velocity.vy_mm_s+blend_rate*oy,velocity.yaw_deg_s)
        remaining=math.hypot(self.observed_goal.x_mm-pose.x_mm,self.observed_goal.y_mm-pose.y_mm)
        cruise=max(route.speed_limits)
        cap=min(cruise,self.approach_speed+max(0.,cruise-self.approach_speed)*min(1.,remaining/self.profile.capture_remaining_mm))
        ready=(self.confirmed>=self.confirm_frames and now-self.last_good<=self.frame_age
               and math.hypot(gx-self.shift_x,gy-self.shift_y)<=1.0)
        finished=elapsed>=max(self.base_route.duration_s,self.blend_started+duration)
        return GuidedReference(target,velocity,finished,ready,cap)

    def finish(self):
        self.completed=self.active and self.confirmed>=self.confirm_frames
        if self.completed:
            self.reason='tag6_and_offset_confirmed'
