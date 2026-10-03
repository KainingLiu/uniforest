"""Explicitly calibrated adapter for the real Robot interfaces.

CompetitionRoutes connects this adapter to explicitly enabled competition routes.
Field/robot geometry, camera extrinsics and capture delay must be measured before
enabling it. Building observations are produced by the existing cube geometry.
"""
from collections import deque
from dataclasses import dataclass
import math
import time
from control.chassis import COUNTS_PER_CM
from .planner import Pose, wrap, NavigationError
from .control import MotionFeedback, execute_route
from .localization import Observation


@dataclass(frozen=True)
class NavigationCalibration:
    verified: bool
    record: str
    capture_delay_s: float
    frame: str = 'field_mm_cw'

    def validate(self):
        if self.verified is not True or not isinstance(self.record,str) or not self.record.strip() or self.frame!='field_mm_cw':
            raise ValueError('measured field, robot envelope and camera calibration record required')
        if not math.isfinite(self.capture_delay_s) or not 0<=self.capture_delay_s<=.2:
            raise ValueError('measured camera acquisition delay required')


class RobotNavigation:
    """Connect encoder/IMU and raw Tag6 solutions to the shared navigator.

    The owner holds the robot-wide strategy lock. A successful building result
    is a precondition for a later Build; navigation never starts the mechanism.
    """
    def __init__(self,robot,context,planner,calibration,*,building_controller=None):
        if not isinstance(calibration, NavigationCalibration):
            raise ValueError('navigation requires measured Tag calibration')
        calibration.validate()
        self.robot,self.context,self.planner=robot,context,planner
        self.calibration,self.building_controller=calibration,building_controller
        self.pose=None; self.previous=None; self.history=deque()
        self.locked_building=None

    def _feedback(self):
        self.context.check_active()
        telem,received,_,_=self.robot.inspection_link_snapshot()
        counts=tuple(m.cumulative_pos for m in telem.motors)
        if len(counts)!=4 or not math.isfinite(telem.yaw_deg):
            raise NavigationError('invalid navigation encoder/IMU data')
        if self.previous is not None:
            old,yaw=self.previous
            delta=[(b-a+0x80000000)%0x100000000-0x80000000 for a,b in zip(old,counts)]
            tr,tl,bl,br=delta
            dx=(-tr+tl+bl-br)*10/(4*COUNTS_PER_CM)
            dy=(tr+tl-bl-br)*10/(4*COUNTS_PER_CM*self.robot.chassis.lateral_distance_scale)
            da=-wrap(telem.yaw_deg-yaw)
            theta=math.radians(self.pose.yaw+da/2)
            self.pose=Pose(self.pose.x+math.cos(theta)*dx-math.sin(theta)*dy,
                           self.pose.y+math.sin(theta)*dx+math.cos(theta)*dy,self.pose.yaw+da)
        self.previous=counts,telem.yaw_deg
        now=time.monotonic(); self.history.append((now,self.pose))
        while len(self.history)>2 and self.history[1][0]<now-1: self.history.popleft()
        return MotionFeedback(self.pose,self.robot.chassis.measured_body_velocity(),received)

    def _observations(self,goal):
        result=[]; field=self.robot.field_pose
        if field is not None and field.valid and field.calibrated and field.captured_monotonic>0:
            tag=next((t for t in field.tag_solutions if t.tag_id==6),None)
            if tag is not None and tag.area_px>=200:
                # Per-tag unsmoothed solution retains a well-defined capture
                # instant; the old EMA FieldPose is intentionally not fused.
                result.append(Observation(Pose(2400+1000*tag.x_m,3600-1000*tag.y_m,-tag.yaw_deg),
                                          field.captured_monotonic-self.calibration.capture_delay_s,
                                          reprojection_px=tag.reprojection_error_px))
        c=self.building_controller
        if c is not None and math.hypot(self.pose.x-goal.x,self.pose.y-goal.y)<500:
            c._set_cube_profile('building')
            frame=self.robot.vision_result
            captured=getattr(frame,'captured_monotonic',0)
            block=c._building_from_result(frame,self.locked_building)
            if captured>0 and block is not None and self.history and self.history[0][0]<=captured:
                x,z=c._building_top_reference(block)
                self.locked_building=(x,z)
                at=min(self.history,key=lambda p:abs(p[0]-captured))[1]
                if abs(wrap(at.yaw-goal.yaw))<=20:
                    forward=z-c.config.building_target_z_mm
                    right=x-c.config.building_target_x_mm
                    angle=math.radians(at.yaw)
                    observed=Pose(goal.x-math.cos(angle)*forward+math.sin(angle)*right,
                                  goal.y-math.sin(angle)*forward-math.cos(angle)*right,at.yaw)
                    result.append(Observation(observed,captured,'building',3))
        return result

    def prepare(self,source,target,*,actual_start=None):
        """Validate and plan without sending a motion command."""
        self.calibration.validate(); self.context.check_active()
        telem,_,_,_=self.robot.inspection_link_snapshot()
        if telem.stepper_busy or self.robot.actions._action_lock.locked():
            raise ValueError('operation-position navigation requires a completed mechanism action')
        route=self.planner.plan(source,target,actual_start=actual_start)
        if route.needs_tag6 and self.building_controller is None:
            raise ValueError('building navigation requires the calibrated building controller')
        # This interface starts at an operation pose. Future moving-to-moving
        # goals must provide their own velocity boundary contract.
        velocity=self.robot.chassis.measured_body_velocity()
        if math.hypot(velocity.vx_mm_s,velocity.vy_mm_s)>6 or abs(velocity.yaw_deg_s)>1:
            raise ValueError('operation-position navigation requires a stationary entry')
        return route

    def navigate(self,source,target,*,actual_start=None):
        return self.execute(self.prepare(source,target,actual_start=actual_start))

    def prepare_between(self, start, goal):
        """Plan a recipe endpoint from an observed field pose without movement."""
        self.calibration.validate()
        self.context.check_active()
        telem,_,_,_=self.robot.inspection_link_snapshot()
        velocity=self.robot.chassis.measured_body_velocity()
        if telem.stepper_busy or self.robot.actions._action_lock.locked():
            raise ValueError('competition navigation requires a completed mechanism action')
        if math.hypot(velocity.vx_mm_s,velocity.vy_mm_s)>6 or abs(velocity.yaw_deg_s)>1:
            raise ValueError('competition navigation requires a stationary entry')
        return self.planner.plan_between(start,goal)

    def execute(self,route):
        # Planning may take time. Recheck mechanism, stationary entry and link
        # immediately before using its result; never continue a stale session.
        self.calibration.validate()
        self.context.check_active()
        telem,_,_,_=self.robot.inspection_link_snapshot()
        velocity=self.robot.chassis.measured_body_velocity()
        if (telem.stepper_busy or self.robot.actions._action_lock.locked()
                or math.hypot(velocity.vx_mm_s,velocity.vy_mm_s)>6 or abs(velocity.yaw_deg_s)>1):
            raise ValueError('navigation entry changed during planning')
        self.pose=route.start; self.previous=None; self.history.clear(); self.locked_building=None
        def send(v):
            self.context.check_active()
            return self.robot.chassis.set_speeds(self.robot.chassis.mecanum_rpm(
                v.vx_mm_s/10,v.vy_mm_s*self.robot.chassis.lateral_distance_scale/10,-v.yaw_deg_s))
        def stop():
            return self.robot.chassis.set_speeds([0,0,0,0])
        try:
            return execute_route(self.planner,route,read_feedback=self._feedback,
                                 read_observations=lambda:self._observations(route.goal),
                                 send_velocity=send,check=self.context.check_active,stop=stop,
                                 clock=time.monotonic,sleep=time.sleep)
        except BaseException as original:
            self.context.close()
            try:
                self.robot.transport.emergency_stop()
            except BaseException as stop_error:
                if hasattr(original,'add_note'):
                    original.add_note(f'navigation emergency stop also failed: {stop_error}')
            raise
