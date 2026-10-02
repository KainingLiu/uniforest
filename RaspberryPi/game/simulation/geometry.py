"""Planar footprint checks for illustrative, uncalibrated field replays.

Distances are millimetres. Local x is forward and local y is right; positive
yaw is clockwise on the displayed world xy plane (world +y points down).
Height never implies a wall: only shapes explicitly marked collision=True are
obstacles. In particular, ramps and traversable platforms remain traversable.

Trace interpolation is sampled, not a certification of continuous physical
clearance. A conservative envelope between adjacent samples also reports
possible contact, including corner sweep during rotation. This cannot recover
curvature lost by downsampling an arbitrary source trajectory.
"""

from __future__ import annotations

import math


_EPS = 1e-8


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    return float(value)


def _polygon(points):
    result = tuple((_number(p[0], 'polygon x'), _number(p[1], 'polygon y')) for p in points)
    if len(result) > 1 and result[0] == result[-1]:
        result = result[:-1]
    if len(result) < 3 or abs(sum(a[0]*b[1]-a[1]*b[0]
                                  for a, b in _edges(result))) <= _EPS:
        raise ValueError('polygon must have at least three non-collinear vertices')
    return result


def _edges(polygon):
    return zip(polygon, polygon[1:] + polygon[:1])


def _cross(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])


def _on_segment(point, a, b):
    return (abs(_cross(a, b, point)) <= _EPS
            and min(a[0], b[0])-_EPS <= point[0] <= max(a[0], b[0])+_EPS
            and min(a[1], b[1])-_EPS <= point[1] <= max(a[1], b[1])+_EPS)


def _intersects(a, b, c, d):
    ab_c, ab_d, cd_a, cd_b = _cross(a,b,c), _cross(a,b,d), _cross(c,d,a), _cross(c,d,b)
    if ((ab_c > _EPS and ab_d < -_EPS or ab_c < -_EPS and ab_d > _EPS)
            and (cd_a > _EPS and cd_b < -_EPS or cd_a < -_EPS and cd_b > _EPS)):
        return True
    return any((_on_segment(c,a,b), _on_segment(d,a,b),
                _on_segment(a,c,d), _on_segment(b,c,d)))


def _inside(point, polygon):
    inside = False
    x, y = point
    for a, b in _edges(polygon):
        if _on_segment(point, a, b):
            return True
        if (a[1] > y) != (b[1] > y):
            crossing = a[0] + (y-a[1])*(b[0]-a[0])/(b[1]-a[1])
            if x < crossing:
                inside = not inside
    return inside


def polygons_overlap(a, b):
    """Intersect simple polygons, including concave polygons and touching edges."""
    a, b = _polygon(a), _polygon(b)
    return (any(_intersects(p,q,r,s) for p,q in _edges(a) for r,s in _edges(b))
            or _inside(a[0],b) or _inside(b[0],a))


def _point_distance(point, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    length2 = dx*dx + dy*dy
    fraction = (0.0 if length2 == 0 else
                max(0.0, min(1.0, ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length2)))
    return math.hypot(point[0]-a[0]-fraction*dx, point[1]-a[1]-fraction*dy)


def polygon_clearance(a, b):
    """Shortest nonnegative edge clearance; overlap or contact returns zero."""
    a, b = _polygon(a), _polygon(b)
    if polygons_overlap(a,b):
        return 0.0
    return min(min(_point_distance(p,r,s), _point_distance(q,r,s),
                   _point_distance(r,p,q), _point_distance(s,p,q))
               for p,q in _edges(a) for r,s in _edges(b))


clearance = polygon_clearance


def _rotate(x, y, yaw):
    radians = math.radians(yaw)
    c, s = math.cos(radians), math.sin(radians)
    return c*x-s*y, s*x+c*y


def robot_polygon(x, y, yaw, length, width, front_extension=0):
    """Rectangle about the chassis centre, with optional extra forward reach."""
    x,y,yaw,length,width,front_extension = (
        _number(value,name) for value,name in zip(
            (x,y,yaw,length,width,front_extension),
            ('x','y','yaw','length','width','front_extension')))
    if length <= 0 or width <= 0 or front_extension < 0:
        raise ValueError('body dimensions must be positive and extension nonnegative')
    points = ((-length/2,-width/2), (length/2+front_extension,-width/2),
              (length/2+front_extension,width/2), (-length/2,width/2))
    return tuple((x+dx,y+dy) for dx,dy in (_rotate(px,py,yaw) for px,py in points))


def _hull(points):
    points = sorted(set(points))
    def half(values):
        result = []
        for point in values:
            while len(result) >= 2 and _cross(result[-2],result[-1],point) <= 0:
                result.pop()
            result.append(point)
        return result
    return tuple(half(points)[:-1] + half(reversed(points))[:-1])


def _pose(point):
    return tuple(_number(point.get(long,point.get(short,0.0)),long)
                 for long,short in (('x_mm','x'),('y_mm','y'),('yaw_deg','yaw')))


def assess_trace(trace, start_pose, field, robot, *, max_step_mm=20.0,
                 max_step_deg=2.0, max_events=40):
    """Assess local cumulative trace poses transformed by a world start pose.

    Points accept x/y/yaw/t or x_mm/y_mm/yaw_deg/time_s. Yaw is unwrapped:
    0 -> 270 explicitly turns clockwise by 270 degrees. For an exact circular
    centre path, a destination point may carry ``arc`` with local centre_x_mm,
    centre_y_mm and sweep_deg; its endpoint must agree with that circle.

    Returned collisions are contact episodes (one per contiguous contact with
    each shape), capped by max_events; collision_count counts all episodes.
    Evidence distinguishes an actual interpolated footprint overlap from a
    conservative swept-envelope contact. All are planar-model assumptions.
    """
    max_step_mm, max_step_deg = _number(max_step_mm,'max_step_mm'), _number(max_step_deg,'max_step_deg')
    if max_step_mm <= 0 or not 0 < max_step_deg <= 90:
        raise ValueError('sampling steps must be positive, with angle at most 90 degrees')
    if type(max_events) is not int or max_events < 0:
        raise ValueError('max_events must be a nonnegative integer')
    centre_shift = 0.0
    if 'front_extent_mm' in robot or 'rear_extent_mm' in robot:
        front = _number(robot['front_extent_mm'],'front_extent_mm')
        rear = _number(robot['rear_extent_mm'],'rear_extent_mm')
        if front <= 0 or rear <= 0:
            raise ValueError('front and rear extents must be positive')
        length,centre_shift = front+rear,(front-rear)/2
    else:
        length = _number(robot['length_mm'],'length_mm')
    width = _number(robot['width_mm'],'width_mm')
    extension = _number(robot.get('front_extension_mm',0.0),'front_extension_mm')
    robot_polygon(0,0,0,length,width,extension)
    radius = math.hypot(max(length/2+centre_shift+extension,length/2-centre_shift),width/2)
    start_x,start_y,start_yaw = _pose(start_pose)
    field_width,field_height = _number(field['width_mm'],'field width'), _number(field['height_mm'],'field height')
    if field_width <= 0 or field_height <= 0:
        raise ValueError('field dimensions must be positive')
    warnings = [
        '离散采样与保守平面包络检查；无法证明连续实机轨迹无碰撞。',
        '全局起点及局部路线映射须单独标定；演示锚点不构成场地路径认证。',
        '仅检查 collision=true 的平面障碍；高度、坡道通行与机构三维净空另行验证。',
        '缺少精确弧段信息时按相邻位姿插值，无法恢复轨迹下采样丢失的曲率。',
    ]
    obstacles = []
    ids = {'__boundary__'}
    for index,shape in enumerate(field.get('shapes',())):
        collision = shape.get('collision',False)
        if type(collision) is not bool:
            raise ValueError('shape collision must be an explicit boolean')
        if not collision:
            continue
        identity = str(shape.get('id',f'shape_{index}'))
        if identity in ids:
            raise ValueError(f'duplicate or reserved shape id: {identity}')
        ids.add(identity)
        obstacles.append((identity,shape,_polygon(shape['polygon'])))
    points = list(trace)
    if not points:
        return dict(collisions=[],collision_count=0,colliding_samples=0,min_clearance_mm=None,
                    samples_checked=0,warnings=warnings+['空轨迹：未执行几何检查。'])
    poses = [_pose(point) for point in points]
    times = [_number(point.get('t',point.get('time_s',i)),'trace time') for i,point in enumerate(points)]
    if any(b < a for a,b in zip(times,times[1:])):
        raise ValueError('trace times must be nondecreasing')
    collisions, active = [], set()
    samples_checked = colliding_samples = collision_count = 0
    minimum = math.inf
    previous_polygon = None

    def inspect(pose, timestamp, segment, pad=0.0):
        nonlocal previous_polygon,minimum,samples_checked,colliding_samples,collision_count,active
        local_x,local_y,yaw = pose
        dx,dy = _rotate(local_x,local_y,start_yaw)
        world_x,world_y = start_x+dx,start_y+dy
        shift_x,shift_y = _rotate(centre_shift,0,start_yaw+yaw)
        polygon = robot_polygon(world_x+shift_x,world_y+shift_y,start_yaw+yaw,length,width,extension)
        envelope = polygon if previous_polygon is None else _hull(previous_polygon+polygon)
        touches = {}
        boundary_clear = min(min(x,field_width-x,y,field_height-y) for x,y in polygon)
        envelope_clear = min(min(x,field_width-x,y,field_height-y) for x,y in envelope)-pad
        minimum = min(minimum,max(0.0,boundary_clear),max(0.0,envelope_clear))
        if boundary_clear <= _EPS or envelope_clear <= _EPS:
            touches['__boundary__'] = ({'label':'场地边界','certainty':'planar_assumption'},
                'sampled_overlap' if boundary_clear <= _EPS else 'swept_envelope')
        for identity,shape,obstacle in obstacles:
            distance = polygon_clearance(polygon,obstacle)
            swept_distance = polygon_clearance(envelope,obstacle)-pad
            minimum = min(minimum,distance,max(0.0,swept_distance))
            if distance <= _EPS:
                touches[identity] = (shape,'sampled_overlap')
            elif previous_polygon is not None and swept_distance <= _EPS:
                touches[identity] = (shape,'swept_envelope')
        if touches:
            colliding_samples += 1
        for identity,(shape,evidence) in touches.items():
            if identity in active:
                continue
            collision_count += 1
            if len(collisions) < max_events:
                collisions.append(dict(shape_id=identity,label=shape.get('label',identity),t=timestamp,
                    x_mm=world_x,y_mm=world_y,yaw_deg=start_yaw+yaw,segment_index=segment,
                    evidence=evidence,certainty=shape.get('certainty','unspecified')))
        active = set(touches)
        previous_polygon = polygon
        samples_checked += 1

    inspect(poses[0],times[0],0)
    for index,(a,b) in enumerate(zip(poses,poses[1:]),1):
        arc = points[index].get('arc')
        distance = math.hypot(b[0]-a[0],b[1]-a[1])
        sweep = arc_radius = 0.0
        if arc is not None:
            cx = _number(arc['center_x_mm'],'arc center x')
            cy = _number(arc['center_y_mm'],'arc center y')
            sweep = _number(arc['sweep_deg'],'arc sweep')
            rx,ry = a[0]-cx,a[1]-cy
            arc_radius = math.hypot(rx,ry)
            ex,ey = _rotate(rx,ry,sweep)
            if arc_radius <= _EPS or math.hypot(cx+ex-b[0],cy+ey-b[1]) > 1e-5:
                raise ValueError('arc endpoint must match its centre and sweep')
            distance = arc_radius*abs(math.radians(sweep))
        steps = max(1,math.ceil(distance/max_step_mm),math.ceil(abs(b[2]-a[2])/max_step_deg),
                    math.ceil(abs(sweep)/max_step_deg))
        # Standard interpolation-error bound from the maximum second derivative.
        pad = (radius*math.radians((b[2]-a[2])/steps)**2
               + arc_radius*math.radians(sweep/steps)**2)/8
        for step in range(1,steps+1):
            fraction = step/steps
            x,y,yaw = (p+(q-p)*fraction for p,q in zip(a,b))
            if arc is not None:
                dx,dy = _rotate(rx,ry,sweep*fraction)
                x,y = cx+dx,cy+dy
            inspect((x,y,yaw),times[index-1]+(times[index]-times[index-1])*fraction,index,pad)
    if collision_count > len(collisions):
        warnings.append(f'仅列出前 {len(collisions)} 段接触；总接触段数 {collision_count}。')
    return dict(collisions=collisions,collision_count=collision_count,colliding_samples=colliding_samples,
                min_clearance_mm=minimum,samples_checked=samples_checked,warnings=warnings)


__all__ = ['robot_polygon','polygons_overlap','polygon_clearance','clearance','assess_trace']
