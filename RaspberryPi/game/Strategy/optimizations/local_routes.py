"""Smooth existing local action sequences, retaining their destinations/barriers.

All endpoints, dynamic search corrections and contact tails come from ROUTES.
No global position, field geometry, camera calibration or simulated start is used.
"""
from dataclasses import replace
import math

from control.chassis import (MECANUM_RPM_PER_CM_S, LATERAL_DISTANCE_SCALE,
    TURN_DEG_S_TO_RPM, TURN_BASE_SPEED_DEG_S, FWD_BASE_PID_LIMIT, FWD_BASE_SPEED_RPM,
    TURN_ACCEL_MS, TURN_SETTLE_DEG, FWD_SETTLE_MS, FWD_SETTLE_SPEED_RPM,
    ROUTE_ARRIVAL_TOLERANCE_MM, NORMAL_DISTANCE_MOVE_ACCEL_MS, LONG_DISTANCE_MOVE_SPEED_MM_S)
from control.trajectory import Waypoint, TrajectoryProfile, CubicRoute, BodyVelocity
from ..flows.curves import CURVE_ROUTES
from ..flows.route_source import RouteRecorder, CLEARANCE_RETREATS

# Route-level exclusion used to discard every move around a visual/contact
# operation. Only those operations themselves are execution boundaries.
LOCAL_ROUTES = CURVE_ROUTES | {'to_purple','unload_approach',
                             'ground_delivery_reverse','orange_depart_reverse'}

CORNER_CUTBACK_MM = 750.0
CORNER_CORRIDOR_MM = 250.0


def round_corners(points, settings):
    """Cut inside translation corners, retaining endpoints and segment limits.

    Certify each Hermite segment's Bezier hull inside a capsule around an
    original leg. A capsule is convex, so its whole curve stays in the corridor.
    This bounds reference geometry only; it is not measured obstacle clearance.
    """
    def mix(a,b,u):
        return Waypoint(*(x+(y-x)*u for x,y in zip(
            (a.x_mm,a.y_mm,a.yaw_deg),(b.x_mm,b.y_mm,b.yaw_deg))))
    def distance(p,a,b):
        dx,dy=b.x_mm-a.x_mm,b.y_mm-a.y_mm
        u=max(0.,min(1.,((p[0]-a.x_mm)*dx+(p[1]-a.y_mm)*dy)/max(dx*dx+dy*dy,1e-12)))
        return math.hypot(p[0]-a.x_mm-u*dx,p[1]-a.y_mm-u*dy)
    def in_corridor(curve):
        for i,(a,b) in enumerate(zip(curve.points,curve.points[1:])):
            dt=(curve.knots[i+1]-curve.knots[i])/3
            ta,tb=curve.tangents[i:i+2]
            hull=((a.x_mm,a.y_mm),(a.x_mm+dt*ta[0],a.y_mm+dt*ta[1]),
                  (b.x_mm-dt*tb[0],b.y_mm-dt*tb[1]),(b.x_mm,b.y_mm))
            if not any(all(distance(p,left,right)<=CORNER_CORRIDOR_MM for p in hull)
                       for left,right in zip(points,points[1:])):
                return False
        return True
    for attempt in range(7):
        rounded=[points[0]];limits=[]
        for i in range(1,len(points)-1):
            a,b,c=points[i-1:i+2]
            ax,ay=b.x_mm-a.x_mm,b.y_mm-a.y_mm
            bx,by=c.x_mm-b.x_mm,c.y_mm-b.y_mm
            la,lb=math.hypot(ax,ay),math.hypot(bx,by)
            cosine=(ax*bx+ay*by)/max(la*lb,1e-12)
            incoming,outgoing=settings.segment_speeds_mm_s[i-1:i+1]
            if la<1e-6 or lb<1e-6 or not -.8<cosine<.999:
                rounded.append(b);limits.append(incoming);continue
            # End legs have only one neighbouring corner; interior legs reserve
            # disjoint halves so successive blends cannot overlap or reverse.
            cut=min(CORNER_CUTBACK_MM/(2**attempt),
                    (.85 if i==1 else .45)*la,
                    (.85 if i==len(points)-2 else .45)*lb)
            entry,exit=mix(b,a,cut/la),mix(b,c,cut/lb)
            middle=mix(mix(entry,b,.5),mix(b,exit,.5),.5)
            rounded.extend((entry,middle,exit))
            limits.extend((incoming,min(incoming,outgoing),min(incoming,outgoing)))
        rounded.append(points[-1]);limits.append(settings.segment_speeds_mm_s[-1])
        candidate=replace(settings,segment_speeds_mm_s=tuple(limits),timeout_s=3600.,
                          corner_tangent_deviation_mm=CORNER_CORRIDOR_MM)
        curve=CubicRoute(rounded,candidate)
        if in_corridor(curve):
            return tuple(rounded),replace(candidate,timeout_s=curve.duration_s+max(5.,curve.duration_s*.5))
    return points,settings



def local_points(events, origin, *, approach_turn_mm=None, terminal_tag_window_mm=None):
    """Keep translation corners and concentrate yaw changes near original turns."""
    angle = math.radians(origin.yaw_deg)
    points = [Waypoint(0.,0.,0.)]
    pending_yaw = None
    turn_span = approach_turn_mm or 250.0
    def append(point):
        if point != points[-1]:
            points.append(point)
    def between(a,b,u,yaw):
        return Waypoint(a.x_mm+(b.x_mm-a.x_mm)*u,a.y_mm+(b.y_mm-a.y_mm)*u,yaw)
    for index,(kind,pose,_,_) in enumerate(events):
        dx,dy = pose.x_mm-origin.x_mm,pose.y_mm-origin.y_mm
        point = Waypoint(math.cos(angle)*dx+math.sin(angle)*dy,
                         -math.sin(angle)*dx+math.cos(angle)*dy,pose.yaw_deg-origin.yaw_deg)
        if kind == 'move':
            left = points[-1]
            distance = math.hypot(point.x_mm-left.x_mm,point.y_mm-left.y_mm)
            if distance <= 1e-6:
                continue
            if pending_yaw is not None:
                # Initial turns finish near their original position, rather
                # than rotating throughout a long ramp/transport straight.
                append(between(left,point,min(1.,turn_span/distance),pending_yaw))
                pending_yaw = None
            append(point)
        elif kind == 'turn':
            if len(points) == 1:
                pending_yaw = point.yaw_deg
                continue
            end = points.pop()
            left = points[-1]
            distance = math.hypot(end.x_mm-left.x_mm,end.y_mm-left.y_mm)
            if distance <= 1e-6:
                append(point)
                continue
            terminal = terminal_tag_window_mm and index == len(events)-1
            span = min(distance,terminal_tag_window_mm if terminal else turn_span)
            append(between(left,end,1-span/distance,end.yaw_deg))
            if terminal:
                # Face Tag6 early in the open terminal stretch, keeping the
                # centre-line endpoints and the original final yaw unchanged.
                append(between(left,end,1-.65*span/distance,point.yaw_deg))
            append(point)
    if pending_yaw is not None:
        append(Waypoint(0.,0.,pending_yaw))
    return tuple(points)


def route_profile(events, points):
    moves = [e for e in events if e[0]=='move' and e[2][1]>0]
    turns = [e for e in events if e[0]=='turn' and e[2][0]!=0]
    speed = max((e[2][2] for e in moves),default=400.)
    yaw_speed = min((e[2][1] for e in turns),default=TURN_BASE_SPEED_DEG_S)
    # Project the original move budgets onto each generated geometric segment.
    # A short 400 mm/s terminal approach must not cap a 2500 mm ramp transit.
    budgets=[]; cursor=0.
    for event in moves:
        length=event[2][1];budgets.append((cursor,cursor+length,event[2][2]));cursor+=length
    limits=[]; cursor=0.
    for a,b in zip(points,points[1:]):
        end=cursor+math.hypot(b.x_mm-a.x_mm,b.y_mm-a.y_mm)
        limit=min((v for lo,hi,v in budgets if lo<end-1e-6 and hi>cursor+1e-6),default=speed)
        angle=abs(b.yaw_deg-a.yaw_deg)
        if angle>1e-6 and end-cursor>1e-6:
            # Slow before a moving turn rather than arriving faster than the
            # required yaw change can follow and nearly stopping to catch up.
            limit=min(limit,(end-cursor)*yaw_speed/angle)
        limits.append(limit)
        cursor=end
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
    def __init__(self):
        self.approaches = {}
        self.completed_actions = set()

    def configure(self, plan, *, moving_tag6_enabled=False):
        """Optionally attach Tag6 guidance; plain smoothing keeps original goals."""
        self.approaches.clear()
        self.completed_actions.clear()
        if not moving_tag6_enabled:
            return
        for source, tag, offset in zip(plan.steps, plan.steps[1:], plan.steps[2:]):
            if (source.kind == 'navigate' and tag.kind == 'align_tag'
                    and offset.kind == 'navigate' and tag.profile == offset.profile
                    and source.parameters['route'] in ('ground_to_delivery','orange_to_build')
                    and offset.parameters['route'] in ('ground_tag_offset','build_offset')):
                self.approaches[source.name] = (source.parameters['route'], tag, offset)

    def consume_completed(self, name):
        if name not in self.completed_actions:
            return False
        self.completed_actions.remove(name)
        return True

    def prepare(self, env, route, profile):
        from Strategy.optimizations.motion_planning import PreparedMotion
        from Strategy.flows.routes import ROUTES
        if route not in LOCAL_ROUTES:
            return PreparedMotion('classic:contact_or_visual_barrier',lambda: ROUTES[route](env,profile))
        recorder = RouteRecorder(env,profile)
        ROUTES[route](recorder,profile)
        approach = self.approaches.get(env.context.current_action)
        if approach is not None and approach[0] != route:
            approach = None  # An inspection may run its retreat under the next action's name.
        return self.prepare_recorded(env,route,profile,recorder,approach)

    def prepare_recorded(self, env, route, profile, recorder, approach=None):
        from Strategy.optimizations.motion_planning import PreparedMotion
        from Strategy.flows.routes import ROUTES
        # A plain move, including strict depart_a, keeps the tested controller
        # and its timeout/arrival semantics exactly. Tag approaches opt in below.
        if not approach and len(recorder.events) == 1 and route in ROUTES:
            return PreparedMotion('classic:single_action',lambda: ROUTES[route](env,profile))
        prepared=[]; actions=[]; group=[]
        origin=Waypoint(0.,0.,0.)
        def flush():
            nonlocal origin,group
            if not group:return
            points=local_points(group,origin,approach_turn_mm=(recorder.config.wall_premove_mm
                               if route=='to_purple' and not prepared else None),
                terminal_tag_window_mm=900. if approach else None)
            if len(points)>1:
                settings=route_profile(group,points)
                if any(e[3].get('bounded_connector') for e in group):
                    settings=replace(settings,monotone_xy=True)
                item=((points,settings) if any(e[3].get('curve_shaped') for e in group)
                      else round_corners(points,settings))
                if (len(group) == 1 and group[0][0] == 'move'
                        and not group[0][3].get('trajectory_only')):
                    actions.append(('straight',group[0]))
                elif all(e[0] == 'turn' for e in group):
                    actions.extend(('turn',e) for e in group)
                else:
                    prepared.append(item); actions.append(('trajectory',item))
            origin=group[-1][1]; group=[]
        for index,event in enumerate(recorder.events):
            if event[0] in ('move','turn'):
                if event[0]=='move' and event[3].get('ramp_straight'):
                    # Neither adjacent corner rounding nor moving yaw/vision
                    # may modify the protected ramp. Complete and settle each
                    # side independently before changing travel direction.
                    flush()
                    points=local_points([event],origin)
                    item=(points,route_profile([event],points))
                    prepared.append(item);actions.append(('ramp',item))
                    origin=event[1]
                    continue
                group.append(event)
                if index==0 and route in CLEARANCE_RETREATS and event[0]=='move' and event[2][0]=='backward':
                    flush()
            else:
                flush(); actions.append(('boundary',event)); origin=event[1]
        flush()
        if approach and actions and actions[-1][0] in ('trajectory','straight'):
            from .tag_approach import TagApproachProfile
            _, tag_spec, offset_spec = approach
            tag_control = env.control(tag_spec.profile)
            cfg = tag_control.config
            is_build = tag_spec.parameters['purpose'] == 'build'
            if (cfg.build_tag_id if is_build else cfg.delivery_tag_id) == 6:
                kind, last = actions[-1]
                if kind == 'straight':
                    # This last move has its own local origin after any turn.
                    _, endpoint, (direction,distance,speed), kw = last
                    dx,dy = {'forward':(distance,0),'backward':(-distance,0),
                             'right':(0,distance),'left':(0,-distance)}[direction]
                    points = (Waypoint(0,0,0),Waypoint(dx,dy,0))
                    settings = route_profile([last],points)
                else:
                    points, settings = last
                offset_mm = (cfg.post_tag6_lateral_right_mm if is_build else cfg.post_tag_lateral_right_mm)
                direction = cfg.post_tag6_lateral_direction if is_build else cfg.post_tag_lateral_direction
                offset_mm *= 1 if direction == 'right' else -1
                endpoint = points[-1]
                angle = math.radians(endpoint.yaw_deg)
                prefix = len(points)-1
                if offset_mm:
                    points = (*points,Waypoint(endpoint.x_mm-math.sin(angle)*offset_mm,
                        endpoint.y_mm+math.cos(angle)*offset_mm,endpoint.yaw_deg))
                    settings = replace(settings,segment_speeds_mm_s=(*settings.segment_speeds_mm_s,
                        min(settings.max_speed_mm_s,
                            cfg.post_tag6_lateral_speed_mm_s if is_build else cfg.post_tag_lateral_speed_mm_s)))
                limits = TagApproachProfile(building=is_build)
                settings = replace(settings,timeout_s=settings.timeout_s+cfg.delivery_tag_align_timeout_s)
                # Construct at execution time, after preceding turns/retreats.
                actions[-1] = ('guided',(points,settings,prefix,offset_mm,tag_spec,offset_spec,limits))
        changed={key:recorder.data[key] for key in ('ground_lateral_mm','orange_lateral_mm','purple_lateral_mm',
            'ground_reverse_done','orange_reverse_done','purple_route_done') if key in recorder.data}

        def execute():
            result=None
            completed_guide=None
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
                    protected_ramp_count=sum(kind=='ramp' for kind,_ in actions),
                    groups=[dict(points=[vars(point) for point in points],
                                 speed_mm_s=settings.max_speed_mm_s,accel_mm_s2=settings.max_accel_mm_s2,
                                 yaw_deg_s=settings.max_yaw_speed_deg_s,
                                 segment_speeds_mm_s=settings.segment_speeds_mm_s) for points,settings in prepared])
            c=env.control(profile)
            for kind,item in actions:
                env.context.check_active()
                if kind in ('trajectory','ramp'):
                    points,settings=item
                    initial_velocity=env.robot.chassis.measured_body_velocity()
                    if kind=='ramp':
                        if (not all(math.isfinite(v) for v in vars(initial_velocity).values())
                                or math.hypot(initial_velocity.vx_mm_s,initial_velocity.vy_mm_s)>settings.settle_speed_mm_s
                                or abs(initial_velocity.yaw_deg_s)>settings.settle_yaw_speed_deg_s):
                            env.robot.transport.emergency_stop()
                            raise RuntimeError('ramp entry is not settled; refuse curved entry')
                        # Exact zero transverse/yaw reference; residual measured
                        # motion is handled by feedback, never a curved tangent.
                        initial_velocity=BodyVelocity()
                    result=env.robot.chassis.follow_trajectory(points,settings,check=env.context.check_active,
                        initial_velocity=initial_velocity)
                elif kind=='straight':
                    _,_,args,kwargs=item
                    # Original checked move owns the straight segment.
                    c._checked_move(*args,accel_ms=kwargs.get('accel_ms'))
                elif kind=='turn':
                    _,_,args,kwargs=item
                    env.robot.chassis.turn(*args,**kwargs)
                elif kind=='guided':
                    from .tag_approach import TagApproach
                    points,settings,prefix,offset_mm,tag_spec,offset_spec,limits=item
                    guide=TagApproach(env.control(tag_spec.profile),points[-1],prefix,
                        offset_mm,tag_spec.parameters['purpose'],limits)
                    result=env.robot.chassis.follow_trajectory(points,settings,check=env.context.check_active,
                        initial_velocity=env.robot.chassis.measured_body_velocity(),guidance=guide)
                    if guide.completed:
                        completed_guide=(tag_spec.name,offset_spec.name)
                    if diagnostics is not None:
                        diagnostics.write('moving_tag6_approach',route=route,confirmed=guide.completed,
                            fused=guide.active,accepted_frames=guide.accepted_frames,
                            offset_mm=offset_mm,reason=guide.reason)
                else:
                    boundary,_,args,kwargs=item
                    if boundary=='wall':c._drive_until_wall(**kwargs)
                    elif boundary=='rebase':c._recalibrate_heading_zero(*args)
                    elif boundary=='tag_reset':env.robot.reset_field_localization_filter()
                    elif boundary=='tag_align':c._align_delivery_tag_or_continue(**kwargs)
            env.context.check_active()
            env.data.update(changed)
            if completed_guide:
                self.completed_actions.update(completed_guide)
            env.record_transition('local_route',route,'complete')
            return result
        return PreparedMotion('local_route:existing_recipe',execute)
