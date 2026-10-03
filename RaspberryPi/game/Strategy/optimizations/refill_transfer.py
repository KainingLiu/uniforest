"""Connect orange areas directly, bypassing delivery/Tag6 positioning.

Record the old relative moves to derive the same nominal ramp and destination
coordinates. The retreat is a flat corner control; the straight ramp and
destination contact remain constraints. Intermediate contacts have unknown displacement; their
corrections are deliberately NOT fabricated as odometry or executed in transit.
"""

import math
from types import SimpleNamespace

from control.trajectory import Waypoint
from ..common import wrap_angle
from ..flows.route_source import RouteRecorder
from ..flows.routes import ROUTES
from .local_routes import CORNER_CORRIDOR_MM


def _tangent_points(start, end, incoming, outgoing):
    """Inside connector with collinear leads that match neighbouring travel.

    Transition windows must provide positive progress in both end directions.
    Unlike the old outward handles, these consume available flat displacement.
    """
    dx,dy = end.x_mm-start.x_mm,end.y_mm-start.y_mm
    available = (dx*incoming[0]+dy*incoming[1],dx*outgoing[0]+dy*outgoing[1])
    if min(available) <= 1e-6:
        raise ValueError('transition window cannot support an inward tangent')
    if abs(incoming[0]*outgoing[0]+incoming[1]*outgoing[1]) < 1e-6:
        # Concentrate a perpendicular turn near the join. A long S-curve
        # spread over the entire return otherwise leaves a tiny-looking hook.
        radius = min(*available,CORNER_CORRIDOR_MM)
        lead = radius/100.
        radius -= 2*lead
        def at(x,y):
            return Waypoint(start.x_mm+incoming[0]*x+outgoing[0]*y,
                            start.y_mm+incoming[1]*x+outgoing[1]*y,start.yaw_deg)
        offset = available[0]-radius
        a,b = at(offset,0.),at(available[0],radius)
        k = 4*(math.sqrt(2)-1)/3
        c,d = at(offset+k*radius,0.),at(available[0],(1-k)*radius)
        points=[at(offset/2.,0.),a]
        for i in range(1,13):
            u=i/12.
            weights=((1-u)**3,3*(1-u)**2*u,3*(1-u)*u*u,u**3)
            points.append(Waypoint(sum(w*p.x_mm for w,p in zip(weights,(a,c,d,b))),
                                  sum(w*p.y_mm for w,p in zip(weights,(a,c,d,b))),start.yaw_deg))
        remaining=available[1]-radius
        points.extend((at(available[0],radius+remaining/3.),
                       at(available[0],radius+2*remaining/3.),
                       Waypoint(end.x_mm,end.y_mm,start.yaw_deg)))
        return tuple(points)
    handle = min(*[v/6. for v in available],math.hypot(dx,dy)/6.,CORNER_CORRIDOR_MM/2.)
    lead = handle/2.
    def shift(p,d,n):
        return Waypoint(p.x_mm+d[0]*n,p.y_mm+d[1]*n,start.yaw_deg)
    a,b = shift(start,incoming,2*lead),shift(end,outgoing,-2*lead)
    c,d = shift(a,incoming,handle),shift(b,outgoing,-handle)
    points = [shift(start,incoming,lead),a]
    for i in range(1,9):
        u=i/8.
        weights=((1-u)**3,3*(1-u)**2*u,3*(1-u)*u*u,u**3)
        points.append(Waypoint(sum(w*p.x_mm for w,p in zip(weights,(a,c,d,b))),
            sum(w*p.y_mm for w,p in zip(weights,(a,c,d,b))),start.yaw_deg))
    return (*points,shift(end,outgoing,-lead),Waypoint(end.x_mm,end.y_mm,start.yaw_deg))


def _arrival_points(start, end):
    """An inward cubic approach with a short, forward-facing contact tail."""
    dx, dy = end.x_mm-start.x_mm, end.y_mm-start.y_mm
    angle = math.radians(end.yaw_deg)
    ux, uy = math.cos(angle), math.sin(angle)
    forward = dx*ux+dy*uy
    lead = min(forward/6., math.hypot(dx,dy)/12., CORNER_CORRIDOR_MM/2.)
    if lead <= 1e-6:
        return (end,)
    def shift(distance):
        return Waypoint(end.x_mm-distance*ux,end.y_mm-distance*uy,start.yaw_deg)
    p1 = Waypoint(start.x_mm+dx/3.,start.y_mm+dy/3.,start.yaw_deg)
    p2, tail = shift(4*lead), shift(2*lead)
    points = []
    for i in range(1,9):
        u = i/8.
        weights = ((1-u)**3,3*(1-u)**2*u,3*(1-u)*u*u,u**3)
        points.append(Waypoint(
            sum(w*p.x_mm for w,p in zip(weights,(start,p1,p2,tail))),
            sum(w*p.y_mm for w,p in zip(weights,(start,p1,p2,tail))),start.yaw_deg))
    return (*points,shift(lead),Waypoint(end.x_mm,end.y_mm,start.yaw_deg))


class _NominalRecipe(RouteRecorder):
    """Read original distances in one virtual frame, with no hardware writes."""

    def __init__(self, env, profile):
        super().__init__(env, profile)
        self.context = SimpleNamespace(check_active=env.context.check_active)
        self.virtual_origin = object()

    def control(self, profile):
        self.profile = profile
        self.source = self.env.control(profile)
        self.config = self.source.config
        return self

    def route(self, name, profile):
        self.control(profile)
        start = len(self.events)
        ROUTES[name](self, profile)
        return start, len(self.events)

    def _recalibrate_heading_zero(self, reference_cw_deg=0.):
        super()._recalibrate_heading_zero(reference_cw_deg)
        self.heading = reference_cw_deg - self.pose.yaw_deg

    def _measure_lateral_displacement_mm(self, origin):
        if origin is self.virtual_origin:
            return 0.  # No purple search occurs during an orange transfer.
        return super()._measure_lateral_displacement_mm(origin)


def _record(env, source, ground_profile, highland_profile):
    profile = ground_profile if source == 'ground' else highland_profile
    r = _NominalRecipe(env, profile)
    if source == 'ground':
        _, clearance_end = r.route('ground_delivery_reverse', ground_profile)
        r.route('ground_to_delivery', ground_profile)
        r.route('ground_tag_offset', ground_profile)
        c = r.control(ground_profile)
        c._checked_move('backward', c.config.unload_reverse_mm,
                        c.config.unload_reverse_speed_mm_s)
        r.route('ground_delivery_depart', ground_profile)
        r._recalibrate_heading_zero(180.)
        ramp, _ = r.route('to_purple', highland_profile)
        r.data.update(purple_origin=r.virtual_origin, purple_route_done=False)
        r.route('purple_to_orange', highland_profile)
    else:
        _, clearance_end = r.route('orange_depart_reverse', highland_profile)
        _, end = r.route('orange_to_build', highland_profile)
        ramp = end - 1  # This route ends with the calibrated straight descent.
        unload = 'unload-' + highland_profile.rsplit('-', 1)[1]
        r.route('unload_approach', unload)
        r._recalibrate_heading_zero(180.)
        c = r.control(unload)
        c._checked_move('backward', c.config.unload_reverse_mm,
                        c.config.unload_reverse_speed_mm_s)
        r.route('unload_depart', unload)
        r._recalibrate_heading_zero(180.)
        r.route('return_orange', 'return-orange')
        r._drive_until_wall(direction='forward', timeout_is_success=False,
                            context='Cross-region refill anchor')
        r._recalibrate_heading_zero(0.)
    return r, clearance_end, ramp


def record_direct_transfer(env, source, destination, *, ground_profile, highland_profile):
    """Keep derived endpoints, removing obsolete intermediate waypoint goals."""
    if (source, destination) not in (('ground', 'highland'), ('highland', 'ground')):
        raise ValueError('refill transfer must connect different orange regions')
    r, clearance_end, ramp_index = _record(env, source, ground_profile, highland_profile)
    old = r.events
    ramp = old[ramp_index]
    if ramp[0] != 'move':
        raise ValueError('refill recipe has no straight ramp')
    entry = old[ramp_index - 1][1]
    goal = r.pose
    yaw = wrap_angle(goal.yaw_deg)
    # Face the target orange wall throughout transit. Downhill is traversed
    # backwards; no 180-degree delivery handoff turn is needed in either case.
    def facing(p):
        return Waypoint(p.x_mm, p.y_mm, yaw)
    ramp_entry, ramp_exit = facing(entry), facing(ramp[1])
    a = math.radians(yaw)
    dx, dy = ramp_exit.x_mm-ramp_entry.x_mm, ramp_exit.y_mm-ramp_entry.y_mm
    lateral = -math.sin(a)*dx + math.cos(a)*dy
    if abs(lateral) > 1e-6:
        raise ValueError('refill ramp must align with destination heading')
    length = math.hypot(dx,dy)
    flat_lead = (env.control(highland_profile).config.refill_downhill_flat_lead_mm
                 if source == 'highland' else 0.)
    flat_tail = (env.control(highland_profile).config.refill_downhill_flat_tail_mm
                 if source == 'highland' else 0.)
    if isinstance(flat_lead,bool) or not math.isfinite(flat_lead) or not 0 <= flat_lead < length:
        raise ValueError('invalid refill downhill flat lead')
    if (isinstance(flat_tail,bool) or not math.isfinite(flat_tail) or flat_tail < 0
            or flat_lead+flat_tail >= length):
        raise ValueError('invalid refill downhill flat tail')
    ramp_direction = (dx/length,dy/length)
    ramp_entry = Waypoint(ramp_entry.x_mm+dx*flat_lead/length,
                          ramp_entry.y_mm+dy*flat_lead/length,yaw)
    ramp_exit = Waypoint(ramp_exit.x_mm-dx*flat_tail/length,
                         ramp_exit.y_mm-dy*flat_tail/length,yaw)
    events = list(old[:clearance_end])
    cursor = events[-1][1] if events else Waypoint(0., 0., 0.)

    def connect(end, originals, *, arrival=False, tangents=None):
        nonlocal cursor
        moves = [e for e in originals if e[0] == 'move' and e[2][1] > 0]
        turns = [e for e in originals if e[0] == 'turn']
        distance = math.hypot(end.x_mm-cursor.x_mm, end.y_mm-cursor.y_mm)
        if distance > 1e-6:
            if not moves:
                raise ValueError('direct refill connector has no source speed budget')
            speed = min(e[2][2] for e in moves)
            accel = max(e[3]['accel_ms'] for e in moves)
            points = (_tangent_points(cursor,end,*tangents) if tangents else
                      _arrival_points(cursor,end) if arrival else
                      (Waypoint(end.x_mm,end.y_mm,cursor.yaw_deg),))
            previous = cursor
            for point in points:
                step = math.hypot(point.x_mm-previous.x_mm,point.y_mm-previous.y_mm)
                events.append(('move',point,('forward',step,speed),dict(accel_ms=accel,
                    trajectory_only=True,bounded_connector=True,curve_shaped=bool(arrival or tangents))))
                previous = point
        delta = wrap_angle(end.yaw_deg-cursor.yaw_deg)
        if abs(delta) > 1e-6:
            speed = min((e[2][1] for e in turns), default=r.config.delivery_turn_speed_deg_s)
            events.append(('turn', end, (delta, speed), {}))
        cursor = end

    # With an explicit flat window, match the ramp's spatial tangent using
    # that window. Zero trim retains the previous independent-stop fallback.
    approach_events = old[clearance_end:ramp_index]
    if flat_lead:
        # Include the old downhill budget so the extended flat approach never
        # inherits a faster speed than that part of the original recipe.
        approach_events = [*approach_events,ramp]
    connect(ramp_entry, approach_events,
            tangents=((-1.,0.),ramp_direction) if flat_lead else None)
    direction = 'forward' if math.cos(a)*dx + math.sin(a)*dy >= 0 else 'backward'
    events.append(('move', ramp_exit, (direction, length-flat_lead-flat_tail, ramp[2][2]),
                   dict(ramp[3], ramp_straight=True)))
    cursor = ramp_exit
    last_move = max(i for i, e in enumerate(old) if e[0] == 'move')
    # The first destination contact is the left wall. Finish travelling left
    # toward it, avoiding a compulsory forward hook at the goal.
    connect(facing(goal), old[ramp_index+1:last_move+1], arrival=True,
            tangents=(ramp_direction,(math.sin(a),-math.cos(a))) if flat_tail else None)
    # Keep only destination side/front contact. Discard purple and delivery
    # contacts, Tag observations and logical rebases from the skipped route.
    for kind, _, args, kw in old[last_move+1:]:
        if kind == 'wall':
            events.append((kind, facing(goal), args, dict(kw, timeout_is_success=False)))
    events.append(('rebase', facing(goal), (0.,), {}))
    r.events, r.pose = events, facing(goal)
    return r


def run_transfer(planner, env, source, destination, **profiles):
    env.context.check_active()
    env.stop()
    env.context.anchor = 'refill_transit'
    private = dict(env.data)
    execution = SimpleNamespace(context=env.context, robot=env.robot, data=private,
        control=env.control, record_transition=env.record_transition)
    try:
        recorder = record_direct_transfer(execution, source, destination, **profiles)
        profile = profiles['highland_profile' if destination == 'highland' else 'ground_profile']
        motion = planner.prepare_recorded(execution, 'refill_transfer', profile, recorder)
        motion.execute()
        env.stop()
        env.context.check_active()
    except BaseException:
        # No replay after partial travel; normal runtime owns stop/cleanup.
        env.context.anchor = 'refill_transit'
        raise
    env.data.update(private)
    env.context.anchor = 'upper_orange_area' if destination == 'highland' else 'ground_area'
    env.record_transition('refill_transfer', f'{source}_to_{destination}', 'complete')
