"""Fuse the departure-to-purple transport chain in its original local corridor.

Only a purely logical rebase may disappear. Contact, vision and mechanism
operations remain barriers; route geometry and calibrated speeds are recorded
from the original functions. No global starting position is assumed.
"""
import math
from control.trajectory import Waypoint
from ..common import wrap_angle
from ..flows.route_source import RouteRecorder
from ..flows.routes import ROUTES


def grouped_steps(steps, *, enabled):
    index=0
    while index<len(steps):
        group=steps[index:index+3]
        if enabled and len(group)==3:
            first,rebase,last=group
            if (first.kind=='navigate' and first.parameters.get('route')=='depart_b'
                    and first.profile=='depart-b' and first.requires_anchor=='start'
                    and first.ends_at==rebase.requires_anchor=='delivery_exit'
                    and rebase.kind=='rebase_heading' and last.kind=='navigate'
                    and last.parameters.get('route')=='to_purple'
                    and rebase.profile==last.profile and not last.parameters.get('after_build')):
                yield tuple(group)
                index+=3
                continue
        yield (steps[index],)
        index+=1


def record_chain(env, steps):
    first,rebase,last=steps
    departure=RouteRecorder(env,first.profile)
    ROUTES[first.parameters['route']](departure,first.profile)
    if any(e[0] not in ('move','turn') for e in departure.events):
        raise ValueError('departure chain contains a physical or visual boundary')
    reference=rebase.parameters['reference_cw_deg']
    onward=RouteRecorder(env,last.profile,heading_cw_deg=reference)
    ROUTES[last.parameters['route']](onward,last.profile)
    join=departure.pose
    a=math.radians(join.yaw_deg)
    def transform(p):
        return Waypoint(join.x_mm+math.cos(a)*p.x_mm-math.sin(a)*p.y_mm,
                        join.y_mm+math.sin(a)*p.x_mm+math.cos(a)*p.y_mm,
                        join.yaw_deg+p.yaw_deg)
    events=departure.events+[(kind,transform(p),args,kw) for kind,p,args,kw in onward.events]
    result=[];group=[];yaw=0.
    def flush():
        nonlocal yaw,group
        # Intermediate yaw requirements only served the old handoff. Keep the
        # last facing requirement before the next real boundary, shortest turn.
        last_turn=next((i for i in range(len(group)-1,-1,-1) if group[i][0]=='turn'),None)
        for i,(kind,p,args,kw) in enumerate(group):
            if kind=='turn':
                if i!=last_turn:continue
                delta=wrap_angle(p.yaw_deg-yaw)
                yaw+=delta
                args=(delta,*args[1:])
                if abs(delta)<1e-8:continue
            result.append((kind,Waypoint(p.x_mm,p.y_mm,yaw),args,kw))
        group=[]
    for event in events:
        if event[0] in ('move','turn'):
            group.append(event)
        else:
            flush()
            kind,p,args,kw=event
            result.append((kind,Waypoint(p.x_mm,p.y_mm,yaw),args,kw))
    flush()
    onward.events=result
    onward.pose=result[-1][1]
    # zero = entry gyro - original join rotation + logical reference.
    return onward,reference-join.yaw_deg


def prepare_chain(planner, env, steps):
    from .motion_planning import PreparedMotion
    recorder,reference_delta=record_chain(env,steps)
    profile=steps[-1].profile
    motion=planner.prepare_recorded(env,'plan_b_first',profile,recorder)
    def execute():
        env.context.check_active()
        telem=env.robot.telem
        if telem is None or not math.isfinite(telem.yaw_deg):
            raise RuntimeError('telemetry unavailable for fused departure reference')
        c=env.control(profile)
        previous=c._heading_zero_deg
        c._heading_zero_deg=wrap_angle(telem.yaw_deg+reference_delta)
        try:
            result=motion.execute()
            env.context.check_active()
        except BaseException:
            c._heading_zero_deg=previous
            raise
        diagnostics=getattr(env.robot,'diagnostics',None)
        if diagnostics is not None:
            diagnostics.write('route_chain_completed',actions=[s.name for s in steps],
                final_reference_cw_deg=wrap_angle(recorder.pose.yaw_deg-reference_delta))
        return result
    return PreparedMotion('local_route:fused_departure',execute)
