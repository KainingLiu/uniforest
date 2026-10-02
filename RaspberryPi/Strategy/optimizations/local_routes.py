"""Generate local trajectories by recording the existing route functions.

All endpoints, dynamic search corrections and contact tails come from ROUTES.
No global position, field geometry, camera calibration or simulated start is used.
"""
from dataclasses import replace
import math

from control.chassis import (MECANUM_RPM_PER_CM_S, LATERAL_DISTANCE_SCALE,
    TURN_DEG_S_TO_RPM, TURN_BASE_SPEED_DEG_S, FWD_BASE_PID_LIMIT, FWD_BASE_SPEED_RPM,
    TURN_ACCEL_MS, TURN_SETTLE_DEG, FWD_SETTLE_MS, FWD_SETTLE_SPEED_RPM,
    ROUTE_ARRIVAL_TOLERANCE_MM, NORMAL_DISTANCE_MOVE_ACCEL_MS, LONG_DISTANCE_MOVE_SPEED_MM_S)
from control.trajectory import Waypoint, TrajectoryProfile, CubicRoute
from ..flows.curves import CURVE_ROUTES
from ..flows.route_source import RouteRecorder, CLEARANCE_RETREATS

# Route-level exclusion used to discard every move around a visual/contact
# operation. Only those operations themselves are execution boundaries.
LOCAL_ROUTES = CURVE_ROUTES | {'to_purple','unload_approach',
                             'ground_delivery_reverse','orange_depart_reverse'}



def local_points(events, origin, *, approach_turn_mm=None):
    angle = math.radians(origin.yaw_deg)
    points = [Waypoint(0.,0.,0.)]
    for kind, pose, _, _ in events:
        dx,dy = pose.x_mm-origin.x_mm,pose.y_mm-origin.y_mm
        point = Waypoint(math.cos(angle)*dx+math.sin(angle)*dy,
                         -math.sin(angle)*dx+math.cos(angle)*dy,pose.yaw_deg-origin.yaw_deg)
        if point == points[-1]: continue
        if point.x_mm == points[-1].x_mm and point.y_mm == points[-1].y_mm:
            # Holonomic translation can carry the requested heading change.
            # Preserve the original initial heading, including pure-turn routes.
            if len(points) > 1: points[-1] = point
            else: points.append(point)
        else:
            if len(points)==2 and points[1].x_mm==points[1].y_mm==0:
                points.pop()
            points.append(point)
    # Intermediate headings describe how the old axis-aligned script obtained
    # its displacement. Only the final heading is a requirement for this span.
    if len(points)>2:
        lengths=[math.hypot(b.x_mm-a.x_mm,b.y_mm-a.y_mm) for a,b in zip(points,points[1:])]
        total=sum(lengths); arc=0.
        if total:
            final=points[-1].yaw_deg
            for i,length in enumerate(lengths,1):
                arc+=length
                points[i]=replace(points[i],yaw_deg=final*arc/total)
    # The long uphill leg is not a clearance retreat. Keep its entry heading,
    # then turn while moving in the final approach interval rather than parking
    # at the end for a standalone turn. The interval is derived from the existing
    # platform premove distance, not asserted as surveyed ramp clearance.
    if approach_turn_mm and len(points)>=2:
        first=points[1]; distance=math.hypot(first.x_mm,first.y_mm)
        if distance>approach_turn_mm and abs(points[-1].yaw_deg)>.001:
            u=1-approach_turn_mm/distance
            points.insert(1,Waypoint(first.x_mm*u,first.y_mm*u,0.))
    return tuple(points)


def route_profile(events, points):
    moves = [e for e in events if e[0]=='move' and e[2][1]>0]
    turns = [e for e in events if e[0]=='turn' and e[2][0]!=0]
    speed = max((e[2][2] for e in moves),default=400.)
    # Project the original move budgets onto each generated geometric segment.
    # A short 400 mm/s terminal approach must not cap a 2500 mm ramp transit.
    budgets=[]; cursor=0.
    for event in moves:
        length=event[2][1];budgets.append((cursor,cursor+length,event[2][2]));cursor+=length
    limits=[]; cursor=0.
    for a,b in zip(points,points[1:]):
        end=cursor+math.hypot(b.x_mm-a.x_mm,b.y_mm-a.y_mm)
        limits.append(min((v for lo,hi,v in budgets if lo<end-1e-6 and hi>cursor+1e-6),default=speed))
        cursor=end
    yaw_speed = min((e[2][1] for e in turns),default=TURN_BASE_SPEED_DEG_S)
    accel_s = max((e[3].get('accel_ms',NORMAL_DISTANCE_MOVE_ACCEL_MS)/1000 for e in moves),default=.3)
    accel_s = max(.02,accel_s)
    profile = TrajectoryProfile(validated=False,route_derived=True,
        max_speed_mm_s=speed,max_accel_mm_s2=1.5*speed/accel_s,
        max_yaw_speed_deg_s=yaw_speed,max_yaw_accel_deg_s2=1.5*yaw_speed/(TURN_ACCEL_MS/1000),
        max_wheel_rpm=max(speed*(1+FWD_BASE_PID_LIMIT/FWD_BASE_SPEED_RPM)*MECANUM_RPM_PER_CM_S/10*LATERAL_DISTANCE_SCALE,
                          yaw_speed*TURN_DEG_S_TO_RPM),
        # Local pose gains reuse the existing trajectory-controller defaults;
        # wheel-position PID gains use different units and cannot be copied.
        position_gain_s=3.0,yaw_gain_s=3.0,
        position_tolerance_mm=ROUTE_ARRIVAL_TOLERANCE_MM,yaw_tolerance_deg=TURN_SETTLE_DEG,
        settle_speed_mm_s=FWD_SETTLE_SPEED_RPM*10/MECANUM_RPM_PER_CM_S,
        settle_yaw_speed_deg_s=1.0,settle_time_s=FWD_SETTLE_MS/1000,
        max_tracking_error_mm=150.,max_tracking_yaw_error_deg=45.,
        timeout_s=3600.,control_period_s=.02,max_telemetry_age_s=.2,segment_speeds_mm_s=tuple(limits))
    duration = CubicRoute(points,profile).duration_s
    return replace(profile,timeout_s=duration+max(5.,duration*.5))


class LocalRoutes:
    def prepare(self, env, route, profile):
        from .motion_planning import PreparedMotion
        from ..flows.routes import ROUTES
        if route not in LOCAL_ROUTES:
            return PreparedMotion('classic:contact_or_visual_barrier',lambda: ROUTES[route](env,profile))
        recorder = RouteRecorder(env,profile)
        ROUTES[route](recorder,profile)
        prepared=[]; actions=[]; group=[]
        origin=Waypoint(0.,0.,0.)
        def flush():
            nonlocal origin,group
            if not group:return
            points=local_points(group,origin,approach_turn_mm=(recorder.config.wall_premove_mm
                               if route=='to_purple' and not prepared else None))
            if len(points)>1:
                item=(points,route_profile(group,points))
                prepared.append(item); actions.append(('trajectory',item))
            origin=group[-1][1]; group=[]
        for index,event in enumerate(recorder.events):
            if event[0] in ('move','turn'):
                group.append(event)
                if index==0 and route in CLEARANCE_RETREATS and event[0]=='move' and event[2][0]=='backward':
                    flush()
            else:
                flush(); actions.append(('boundary',event)); origin=event[1]
        flush()
        changed={key:recorder.data[key] for key in ('ground_lateral_mm','orange_lateral_mm','purple_lateral_mm',
            'ground_reverse_done','orange_reverse_done','purple_route_done') if key in recorder.data}

        def execute():
            result=None
            diagnostics=getattr(env.robot,'diagnostics',None)
            if diagnostics is not None:
                diagnostics.write('planned_local_route',route=route,profile=profile,
                    source='existing_recipe',field_validated=False,
                    source_move_count=sum(e[0]=='move' for e in recorder.events),
                    source_turn_count=sum(e[0]=='turn' for e in recorder.events),
                    direct_classic_turn_calls=0,
                    standalone_turn_count=sum(math.hypot(b.x_mm-a.x_mm,b.y_mm-a.y_mm)<1e-6
                        and abs(b.yaw_deg-a.yaw_deg)>1e-6
                        for points,_ in prepared for a,b in zip(points,points[1:])),
                    boundaries=[item[0] for kind,item in actions if kind=='boundary'],
                    groups=[dict(points=[vars(point) for point in points],
                                 speed_mm_s=settings.max_speed_mm_s,accel_mm_s2=settings.max_accel_mm_s2,
                                 yaw_deg_s=settings.max_yaw_speed_deg_s,
                                 segment_speeds_mm_s=settings.segment_speeds_mm_s) for points,settings in prepared])
            c=env.control(profile)
            for kind,item in actions:
                env.context.check_active()
                if kind=='trajectory':
                    points,settings=item
                    result=env.robot.chassis.follow_trajectory(points,settings,check=env.context.check_active,
                        initial_velocity=env.robot.chassis.measured_body_velocity())
                else:
                    boundary,_,args,kwargs=item
                    if boundary=='wall':c._drive_until_wall(**kwargs)
                    elif boundary=='rebase':c._recalibrate_heading_zero(*args)
                    elif boundary=='tag_reset':env.robot.reset_field_localization_filter()
                    elif boundary=='tag_align':c._align_delivery_tag_or_continue(**kwargs)
            env.context.check_active()
            env.data.update(changed)
            env.record_transition('local_route',route,'complete')
            return result
        return PreparedMotion('local_route:existing_recipe',execute)
