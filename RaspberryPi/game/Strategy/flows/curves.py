"""Calibrated continuous route prefixes; wall contacts remain explicit barriers."""

import math
from dataclasses import dataclass
from control.trajectory import Waypoint


CURVE_ROUTES = {
    'depart_a','depart_b','return_orange','ground_to_delivery',
    'purple_to_orange','orange_to_build','build_return','staged_initial',
    'staged_to_build','staged_return_first','staged_return_final',
    'ground_tag_offset','ground_delivery_depart','build_offset','unload_depart',
}


def _turn(c, target):
    return -c._heading_error(target)


def _forward_end(x, y, distance, heading):
    radians=math.radians(heading)
    return x+distance*math.cos(radians), y+distance*math.sin(radians)


@dataclass(frozen=True)
class RouteGeometry:
    endpoint: Waypoint
    complete: object


def prepare_route_geometry(env, route, profile):
    """Resolve the existing recipe endpoint; defer contacts/commits until arrival."""
    c=env.control(profile)
    cfg=c.config
    data=env.data
    chassis=env.robot.chassis
    env.context.check_active()
    x=y=yaw=0.0
    tail=None
    commit=lambda:None

    def wall(direction):
        c._drive_until_wall(direction=direction,timeout_s=cfg.far_wall_timeout_s,
                            speed_mm_s=cfg.far_wall_speed_mm_s,context=f'{route} curve wall')

    if route in ('depart_a','depart_b'):
        x=cfg.distance_mm
        if route=='depart_b':
            y=cfg.lateral_right_mm
            yaw=cfg.final_heading_cw_deg
    elif route=='return_orange':
        yaw=_turn(c,cfg.target_heading_cw_deg)
        x,y=_forward_end(0,0,cfg.lateral_left_mm,yaw-90)
        tail=lambda:wall('left')
    elif route=='ground_to_delivery':
        lateral=data.get('ground_lateral_mm')
        if lateral is None:
            lateral=c._measure_lateral_displacement_mm(data['ground_origin'])
        distance=cfg.delivery_forward_base_mm-lateral
        if distance<=0:
            raise RuntimeError('ground delivery displacement is invalid')
        data['ground_lateral_mm']=lateral
        x=0 if data.get('ground_reverse_done') else -cfg.delivery_reverse_mm
        first_turn=_turn(c,cfg.delivery_turn_deg)
        x,y=_forward_end(x,0,distance,first_turn)
        yaw=first_turn+c._wrap_angle(_turn(c,cfg.delivery_turn_deg*2)-first_turn)
        commit=lambda:data.__setitem__('ground_reverse_done',True)
    elif route=='purple_to_orange':
        if data.get('purple_route_done'):
            return
        lateral=c._measure_lateral_displacement_mm(data['purple_origin'])
        data['purple_lateral_mm']=lateral
        distance=cfg.post_grab_forward_base_mm-lateral
        if distance<=0:
            raise RuntimeError('purple return displacement is invalid')
        yaw=_turn(c,cfg.post_grab_heading_target_cw_deg)
        x,y=_forward_end(-cfg.post_grab_reverse_mm,0,distance,yaw)
        def tail():
            if cfg.left_wall_approach_enabled:wall('left')
            wall('forward')
            c._recalibrate_heading_zero()
        commit=lambda:data.__setitem__('purple_route_done',True)
    elif route=='orange_to_build':
        lateral=data.get('orange_lateral_mm')
        if lateral is None:
            lateral=c._measure_lateral_displacement_mm(data['orange_origin'])
        data['orange_lateral_mm']=lateral
        x=0 if data.get('orange_reverse_done') else -cfg.post_orange_reverse_mm
        y=cfg.post_orange_lateral_base_mm-lateral
        yaw=_turn(c,cfg.final_turn_target_cw_deg)
        x,y=_forward_end(x,y,cfg.build_route_distance_mm,yaw)
        commit=lambda:data.__setitem__('orange_reverse_done',True)
    elif route=='build_return':
        yaw=cfg.post_build_turn_cw_deg
        x,y=_forward_end(-cfg.post_build_reverse_mm,0,cfg.post_build_route_distance_mm,yaw-90)
        tail=lambda:wall('left')
    elif route=='staged_initial':
        y=-cfg.initial_left_mm
        tail=lambda:wall('left')
    elif route=='staged_to_build':
        x,y=-cfg.load_reverse_mm,cfg.building_approach_right_mm
    elif route in ('staged_return_first','staged_return_final'):
        x=-cfg.post_build_reverse_mm
        y=-(cfg.first_build_left_mm if route=='staged_return_first' else cfg.final_left_mm)
        if route=='staged_return_first':
            def tail():
                wall('left')
                c._checked_move('right',cfg.second_load_right_mm,cfg.route_speed_mm_s)
    elif route=='ground_tag_offset':
        y=cfg.post_tag_lateral_right_mm*(1 if cfg.post_tag_lateral_direction=='right' else -1)
    elif route=='ground_delivery_depart':
        y=cfg.pre_final_turn_lateral_left_mm*(1 if cfg.pre_final_turn_lateral_direction=='right' else -1)
    elif route=='build_offset':
        y=cfg.post_tag6_lateral_right_mm*(1 if cfg.post_tag6_lateral_direction=='right' else -1)
    elif route=='unload_depart':
        y=cfg.final_lateral_right_mm
    else:
        raise ValueError(f'no continuous route contract for {route}')

    def complete():
        env.context.check_active()
        if tail:tail()
        env.context.check_active()
        commit()

    return RouteGeometry(Waypoint(x,y,yaw),complete)


def run_curve(env, route, profile, calibration):
    """Run a calibrated curve, then preserve the recipe's contact barriers."""
    geometry=prepare_route_geometry(env,route,profile)
    if geometry is None:
        return
    x,y,yaw=geometry.endpoint.x_mm,geometry.endpoint.y_mm,geometry.endpoint.yaw_deg
    chassis=env.robot.chassis
    points=tuple(Waypoint(p['x_mm']+p['dx_scale']*x,
                          p['y_mm']+p['dy_scale']*y,
                          p['yaw_deg']+p['dyaw_scale']*yaw) for p in calibration.points)
    end=points[-1]
    if (abs(end.x_mm-x)>1e-6 or abs(end.y_mm-y)>1e-6 or abs(end.yaw_deg-yaw)>1e-6):
        raise ValueError(f'{route}: curve endpoint does not match calibrated route destination')
    result=chassis.follow_trajectory(points,calibration.profile,
                                    check=env.context.check_active,
                                    initial_velocity=chassis.measured_body_velocity())
    geometry.complete()
    env.record_transition('continuous_route',route,'complete')
    return result
