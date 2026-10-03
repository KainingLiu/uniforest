"""Execute a field route with one chassis owner and continuous visual correction.

Dependencies are injected. The same executor drives the virtual mecanum plant
and an explicitly calibrated real robot adapter. It never commands a Build.
"""
from dataclasses import dataclass
import math
import numpy as np
from control.trajectory import BodyVelocity
from control.chassis import Chassis, LATERAL_DISTANCE_SCALE
from .localization import VisualOdometry, compose
from .planner import Pose, NavigationError, wrap, route_timing
from .motion import CompetitionMotion
from .tracking import PositionTracker


@dataclass(frozen=True)
class MotionFeedback:
    pose: Pose
    velocity: BodyVelocity
    received_at: float


class TimedRoute:
    def __init__(self, route):
        self.route=route
        points=route.samples
        if len(points)==1:
            self.times=np.array([0.]); self.speeds=np.array([0.]); self.duration=0.
            return
        self.times,self.speeds=route_timing(points,route.motion)
        self.duration=float(self.times[-1])

    def sample(self,t):
        rows=self.route.samples
        s=float(np.interp(t,self.times,rows[:,3])); speed=float(np.interp(t,self.times,self.speeds))
        point=self.route.at(s)
        if t>=self.duration or len(rows)==1: return point,np.zeros(3),s
        lo=max(0,s-2); hi=min(self.route.nominal_length_mm,s+2)
        derivative=(self.route.at(hi)[:3]-self.route.at(lo)[:3])/max(hi-lo,1e-5)
        return point,derivative*speed,s


def execute_route(planner,route,*,read_feedback,read_observations,send_velocity,
                  check,stop,clock,sleep,trace=None,timeout_s=120):
    """Return only after actual arrival and required visual confirmations.

    Hard faults attempt stop and propagate. Missing visual evidence also stops
    and fails explicitly; a caller may request a NEW observation-area route only
    after distinguishing that perception outcome from communication faults.
    """
    reference=TimedRoute(route); estimator=VisualOdometry()
    motion=route.motion or CompetitionMotion.competition()
    tracker=PositionTracker(route.tracking)
    wall_speed=.9*motion.wall_mm_s
    margin=motion.transit_margin_mm
    command_limit=min(motion.command_mm_s,max(float(route.samples[:,5].max()),motion.approach_mm_s))
    started=clock(); last=started; virtual=0.; settled=None; clock_rate=1.
    command=np.zeros(3); acceleration=np.zeros(3)
    perception_wait=None
    wall_latched=False; rising_since=None; previous_clear=None; clearance_rate=0.
    try:
        while True:
            if check() is False: raise NavigationError('navigation state guard rejected execution')
            feedback=read_feedback()
            now=clock()
            if now-started>max(timeout_s,reference.duration+15): raise NavigationError('navigation timeout')
            if not 0<=now-feedback.received_at<=.15: raise NavigationError('navigation telemetry stale')
            v=feedback.velocity
            if not all(math.isfinite(x) for x in (v.vx_mm_s,v.vy_mm_s,v.yaw_deg_s)):
                raise NavigationError('invalid measured velocity')
            estimator.predict(now,feedback.pose)
            corrected=False
            for observation in read_observations():
                corrected=estimator.update(observation,now) or corrected
            pose=compose(estimator.offset,feedback.pose)
            if len(route.samples)>1:
                # A new visual fix may reveal we have already passed the old
                # time reference. Continue from the corrected along-path pose,
                # rather than drive backwards to recover an obsolete point.
                rows=route.samples
                progress=float(np.interp(virtual,reference.times,rows[:,3]))
                angle_error=(rows[:,2]-pose.yaw+180)%360-180
                score=np.sum((rows[:,:2]-[pose.x,pose.y])**2,axis=1)+(angle_error*4)**2
                score[(rows[:,3]<progress)|(rows[:,3]>progress+220)]=float('inf')
                index=int(np.argmin(score))
                advanced=rows[index,3]-progress
                if math.isfinite(score[index]) and score[index]<80**2 and (corrected or advanced>20):
                    virtual=max(virtual,float(reference.times[index]))
            clear=float(planner.clearance([(pose.x,pose.y,pose.yaw)])[0])
            if clear < -2: raise NavigationError('estimated body left collision-free corridor')
            dt=max(.001,min(.02,now-last)); last=now
            target,feed,s=reference.sample(virtual)
            error=np.array([target[0]-pose.x,target[1]-pose.y,wrap(target[2]-pose.yaw)])
            if np.linalg.norm(error[:2])>140 or abs(error[2])>25:
                raise NavigationError(f'tracking error exceeds navigation envelope: {np.linalg.norm(error[:2]):.1f} mm, {error[2]:.1f} deg')
            distance=math.hypot(route.goal.x-pose.x,route.goal.y-pose.y)
            # Retiming is part of tracking: when a turn or a clearance envelope
            # slows the real robot, do not let the reference run away in time.
            pa=math.radians(pose.yaw)
            # Include acceleration reversal under the jerk limit, wheel lag,
            # and deceleration from cruise before entering a close-wall zone.
            horizon=max(.4, math.hypot(v.vx_mm_s,v.vy_mm_s)/motion.braking_acceleration
                        + motion.acceleration/motion.jerk + motion.response_allowance_s)
            ahead=np.linspace(.02,horizon,28)
            future=np.c_[pose.x+(math.cos(pa)*v.vx_mm_s-math.sin(pa)*v.vy_mm_s)*ahead,
                         pose.y+(math.sin(pa)*v.vx_mm_s+math.cos(pa)*v.vy_mm_s)*ahead,
                         pose.yaw+v.yaw_deg_s*ahead]
            # Check the whole predicted sweep: a long front corner can be
            # closest to a wall midway through a turn, then clear its endpoint.
            future_clear=float(planner.clearance(future).min())
            if previous_clear is not None:
                clearance_rate=.8*clearance_rate+.2*(clear-previous_clear)/dt
            previous_clear=clear
            if math.hypot(v.vx_mm_s,v.vy_mm_s)>wall_speed and (future_clear<margin+25 or clear<margin+25):
                wall_latched=True; rising_since=None
            elif wall_latched:
                if future_clear>=margin+25 and clear>=margin+30 and clearance_rate>=-5:
                    rising_since=now if rising_since is None else rising_since
                    if now-rising_since>=.25: wall_latched=False
                else:
                    rising_since=None
            available=motion.command_mm_s if clear>=margin+30 and future_clear>=margin+25 else wall_speed
            if wall_latched: available=min(available,wall_speed)
            rate=min(1.,max(0.,1-np.linalg.norm(error[:2])/80),max(0.,1-abs(error[2])/12),
                     available/max(np.linalg.norm(feed[:2]),available))
            clock_rate=min(rate,clock_rate+2*dt)
            feed*=clock_rate
            # Observe during the entire approach. There is no fixed Tag PID
            # reference point. Only the final precision zone requires evidence.
            wait_visual=(route.needs_tag6 and not estimator.tag_locked and distance<60
                         and abs(wrap(route.goal.yaw-pose.yaw))<4)
            if wait_visual:
                feed[:]=0
                perception_wait=now if perception_wait is None else perception_wait
                if now-perception_wait>3:
                    raise NavigationError('Tag6 unavailable: reacquire in observation area before building')
            else:
                perception_wait=None
                virtual=min(reference.duration,virtual+dt*clock_rate)
            angle=math.radians(pose.yaw); c,sn=math.cos(angle),math.sin(angle)
            # Compensate the model's wheel response during reference braking.
            # Bounded feedforward lead does not amplify visual position jumps.
            _,next_feed,_=reference.sample(virtual)
            feed_accel=(next_feed*clock_rate-feed)/.02
            feed_accel[:2]*=min(1,motion.acceleration/max(np.linalg.norm(feed_accel[:2]),1e-6))
            feed_accel[2]=max(-motion.yaw_acceleration,min(motion.yaw_acceleration,feed_accel[2]))
            led_feed=feed+motion.feedforward_lead_s*feed_accel
            led_feed[2]=feed[2]
            measured_world=np.array([c*v.vx_mm_s-sn*v.vy_mm_s,
                                     sn*v.vx_mm_s+c*v.vy_mm_s,v.yaw_deg_s])
            correction=tracker.step(error,feed-measured_world,dt,integrate=not wait_visual)
            desired_world=led_feed[:2]+correction[:2]
            desired=np.array([c*desired_world[0]+sn*desired_world[1],
                              -sn*desired_world[0]+c*desired_world[1],led_feed[2]+correction[2]])
            # Clearance-dependent envelope is applied to feedback corrections
            # as well as feedforward. No high-speed near-wall correction.
            cap=command_limit if clear>=margin+20 and future_clear>=margin+10 else wall_speed
            if wall_latched: cap=min(cap,wall_speed)
            ratio=max(1,np.linalg.norm(desired[:2])/cap,abs(desired[2])/motion.yaw_deg_s)
            desired/=ratio
            if clear<80:
                probe=Pose(pose.x+(c*desired[0]-sn*desired[1])*.1,
                           pose.y+(sn*desired[0]+c*desired[1])*.1,pose.yaw+desired[2]*.1)
                if planner.clearance([(probe.x,probe.y,probe.yaw)])[0]<clear:
                    # Include actuator lag and one future sample in stopping
                    # distance, leaving an odometry/contact tolerance reserve.
                    braking=motion.braking_acceleration
                    lag=motion.response_allowance_s
                    inward=min(motion.approach_mm_s, math.sqrt((braking*lag)**2+2*braking*max(0,clear-1.5))-braking*lag)
                    desired/=max(1,np.linalg.norm(desired[:2])/max(inward,.01),
                                 abs(math.radians(desired[2]))*planner.radius/max(inward,.01))
            rpms=Chassis.mecanum_rpm(desired[0]/10,desired[1]*LATERAL_DISTANCE_SCALE/10,-desired[2])
            desired/=max(1,max(map(abs,rpms))/motion.wheel_rpm)
            # Acceleration and jerk-limited command changes; normal arrival
            # joins zero continuously. Emergency stops intentionally bypass it.
            wanted=(desired-command)/dt
            wanted[:2]*=min(1,motion.acceleration/max(np.linalg.norm(wanted[:2]),1e-6))
            wanted[2]=max(-motion.yaw_acceleration,min(motion.yaw_acceleration,wanted[2]))
            delta=wanted-acceleration
            delta[:2]*=min(1,motion.jerk*dt/max(np.linalg.norm(delta[:2]),1e-6))
            delta[2]=max(-motion.yaw_jerk*dt,min(motion.yaw_jerk*dt,delta[2]))
            acceleration+=delta
            command+=acceleration*dt
            rpms=Chassis.mecanum_rpm(command[0]/10,command[1]*LATERAL_DISTANCE_SCALE/10,-command[2])
            command/=max(1,np.linalg.norm(command[:2])/command_limit,abs(command[2])/motion.yaw_deg_s,max(map(abs,rpms))/motion.wheel_rpm)
            # Integrator momentum must not overshoot a local wall limit after
            # desired was clipped. Safety limits dominate the comfort limiter.
            if np.linalg.norm(command[:2])>cap:
                command[:2]*=cap/np.linalg.norm(command[:2])
                acceleration[:2]=0
            # Avoid driving through a wall at terminal contact: test the next
            # actual braking sweep, reducing inward commands before output.
            if clear<45:
                forecast=Pose(pose.x+(c*command[0]-sn*command[1])*.16,
                              pose.y+(sn*command[0]+c*command[1])*.16,
                              pose.yaw+command[2]*.16)
                if planner.clearance([(forecast.x,forecast.y,forecast.yaw)])[0]<0:
                    command[:]*=.5; acceleration[:]=0
            position_window=motion.arrival_mm-motion.completion_reserve_mm
            tolerance=5 if route.needs_tag6 else min(position_window,route.nominal_length_mm/2) if route.nominal_length_mm else position_window
            settled_speed=6 if route.needs_tag6 else motion.settled_mm_s
            arrived=(distance<tolerance and abs(wrap(route.goal.yaw-pose.yaw))<1
                     and math.hypot(v.vx_mm_s,v.vy_mm_s)<settled_speed and abs(v.yaw_deg_s)<1)
            if arrived:
                # Like classic route arrival, hold zero inside the low-speed
                # window. Retained jerk integrator state must not restart/reverse
                # a chassis that already met its completion conditions.
                command[:]=0; acceleration[:]=0
                tracker.reset()
            else:
                applied_world=np.array([c*command[0]-sn*command[1],sn*command[0]+c*command[1],command[2]])
                tracker.applied(applied_world-led_feed)
            if send_velocity(BodyVelocity(*map(float,command))) is not True:
                raise NavigationError('navigation wheel command rejected')
            precision=(not route.needs_tag6 or estimator.terminal_ready(now))
            if arrived and precision:
                settled=now if settled is None else settled
                if now-settled>=(.2 if route.needs_tag6 else motion.settle_s):
                    if stop() is False: raise NavigationError('navigation final stop rejected')
                    return dict(status='arrived',elapsed_s=now-started,pose=pose.__dict__,
                                build_ready=route.needs_tag6 and precision,
                                visual_updates=estimator.accepted,visual_events=estimator.events,
                                reference_duration_s=reference.duration)
            else: settled=None
            if arrived and not precision and now-started>reference.duration+4:
                raise NavigationError('building precision observation unavailable; Build inhibited')
            if trace:
                trace(dict(t=now-started,x=pose.x,y=pose.y,yaw=pose.yaw,
                           vx=v.vx_mm_s,vy=v.vy_mm_s,wz=v.yaw_deg_s,clearance=clear,
                           tag_locked=estimator.tag_locked,building_ready=route.needs_tag6 and precision,
                           target_x=float(target[0]),target_y=float(target[1]),target_yaw=float(target[2]),
                           error_x=float(error[0]),error_y=float(error[1]),error_yaw=float(error[2]),
                           pid_x=float(correction[0]),pid_y=float(correction[1]),pid_yaw=float(correction[2]),
                           integral_x=float(tracker.integral[0]),integral_y=float(tracker.integral[1]),
                           progress=min(1,virtual/max(reference.duration,.001))))
            sleep(max(0,.02-(clock()-now)))
    except BaseException:
        try: stop()
        except BaseException: pass
        raise
