"""Named, parameterized routes used by functional action plans.

Routes contain the legs needed to travel between local field anchors. They
do not select missions, collect cubes, or invoke another action executor.
Distances and motion profiles come from the selected calibration profile.
"""

from ..common import wrap_angle


def _phase(env, control, name):
    control._check_active()
    env.phase(control, name)


def _wall(env, control, direction, label):
    cfg = control.config
    _phase(env, control, label)
    control._drive_until_wall(
        timeout_s=cfg.far_wall_timeout_s,
        speed_mm_s=cfg.far_wall_speed_mm_s,
        direction=direction,
        context=label,
    )


def _strict_depart_move(env, control, direction, distance_mm):
    """The departure calibration requires full completion, including timeout."""
    cfg = control.config
    control._check_active()
    result = env.robot.move_chassis(
        direction, distance_mm, cfg.speed_mm_s,
        hold_ms=cfg.hold_ms, accel_ms=cfg.accel_ms, route_mode=True,
    )
    if result.timed_out or result.cancelled:
        raise RuntimeError('departure position move did not complete')
    control._check_active()


def _collection_origin(env, name):
    origin = env.data.get(name)
    if origin is None:
        origin = env.data.get('collection', {}).get('origin')
    if origin is None:
        raise RuntimeError(f'collection encoder origin unavailable: {name}')
    return origin


def _compensation_parameters(cfg, distance_mm, short_speed_mm_s):
    if distance_mm >= cfg.compensation_fast_distance_mm:
        return cfg.compensation_fast_speed_mm_s, cfg.long_distance_forward_accel_ms
    return short_speed_mm_s, cfg.delivery_linear_accel_ms


def depart_a(env, profile):
    c = env.control(profile)
    _phase(env, c, 'INITIAL_MOVE')
    _strict_depart_move(env, c, 'forward', c.config.distance_mm)


def depart_b(env, profile):
    c = env.control(profile)
    cfg = c.config
    c._check_active()
    # Preserve the departure yaw through both straight legs.
    heading_zero_deg = env.robot.telem.yaw_deg
    _phase(env, c, 'INITIAL_MOVE')
    _strict_depart_move(env, c, 'forward', cfg.distance_mm)
    _phase(env, c, 'LATERAL_MOVE')
    _strict_depart_move(env, c, 'right', cfg.lateral_right_mm)
    _phase(env, c, 'FINAL_TURN')
    telem = env.robot.telem
    if telem is None:
        raise RuntimeError('telemetry unavailable before departure final turn')
    target_yaw = wrap_angle(heading_zero_deg - cfg.final_heading_cw_deg)
    clockwise_delta_deg = -wrap_angle(target_yaw - telem.yaw_deg)
    env.robot.chassis.turn(
        clockwise_delta_deg, cfg.turn_speed_deg_s, hold_ms=0, settle_cycles=1,
    )
    c._check_active()


def return_orange(env, profile):
    c = env.control(profile)
    cfg = c.config
    _phase(env, c, 'TURN_TO_START')
    c._turn_to_heading(cfg.target_heading_cw_deg)
    _phase(env, c, 'LATERAL_MOVE')
    c._checked_move('left', cfg.lateral_left_mm, cfg.lateral_speed_mm_s)
    _wall(env, c, 'left', 'LEFT_WALL_APPROACH')


def ground_delivery_reverse(env, profile):
    c = env.control(profile)
    c._check_active()
    if env.data.get('ground_reverse_done', False):
        return
    cfg = c.config
    # Freeze search displacement before moving away from the pickup wall.
    lateral_mm = c._measure_lateral_displacement_mm(
        _collection_origin(env, 'ground_origin'))
    if cfg.delivery_forward_base_mm <= lateral_mm:
        raise RuntimeError('delivery forward distance must be positive')
    env.data['ground_lateral_mm'] = lateral_mm
    _phase(env, c, 'DELIVERY_ROUTE')
    c._checked_move('backward', cfg.delivery_reverse_mm,
                    cfg.delivery_reverse_speed_mm_s)
    env.data['ground_reverse_done'] = True


def ground_to_delivery(env, profile):
    c = env.control(profile)
    cfg = c.config
    c._check_active()
    if not env.data.get('ground_reverse_done', False):
        ground_delivery_reverse(env, profile)
    lateral_mm = env.data.get('ground_lateral_mm')
    if lateral_mm is None:
        raise RuntimeError('ground collection displacement unavailable')
    forward_mm = cfg.delivery_forward_base_mm - lateral_mm
    if forward_mm <= 0.0:
        raise RuntimeError('delivery forward distance must be positive')
    _phase(env, c, 'DELIVERY_ROUTE')
    c._turn_to_heading(cfg.delivery_turn_deg,
                       hold_ms=cfg.delivery_turn_heading_hold_ms)
    c._checked_move('forward', forward_mm, cfg.delivery_forward_speed_mm_s)
    c._turn_to_heading(cfg.delivery_turn_deg * 2.0)


def ground_tag_offset(env, profile):
    c = env.control(profile)
    cfg = c.config
    _phase(env, c, 'POST_TAG_LATERAL')
    if cfg.post_tag_lateral_right_mm > 0.0:
        c._checked_move(cfg.post_tag_lateral_direction,
                        cfg.post_tag_lateral_right_mm,
                        cfg.post_tag_lateral_speed_mm_s)


def ground_delivery_depart(env, profile):
    c = env.control(profile)
    cfg = c.config
    _phase(env, c, 'PRE_FINAL_TURN_LATERAL')
    if cfg.pre_final_turn_lateral_left_mm > 0.0:
        c._checked_move(cfg.pre_final_turn_lateral_direction,
                        cfg.pre_final_turn_lateral_left_mm,
                        cfg.pre_final_turn_lateral_speed_mm_s)
    c._check_active()


def to_purple(env, profile):
    c = env.control(profile)
    cfg = c.config
    _phase(env, c, 'INITIAL_MOVE')
    c._checked_move('backward', cfg.initial_distance_mm, cfg.initial_speed_mm_s,
                    accel_ms=cfg.long_distance_forward_accel_ms)
    _phase(env, c, 'TURN_TO_COLLECTION')
    c._turn_to_heading(cfg.delivery_heading_target_cw_deg)
    if cfg.tag3_alignment_enabled:
        _phase(env, c, 'TAG_ALIGN')
        env.robot.reset_field_localization_filter()
        c._align_delivery_tag_or_continue(fine_align_enabled=False)
        if cfg.post_tag_lateral_mm > 0.0:
            _phase(env, c, 'POST_TAG_LATERAL')
            c._checked_move('right', cfg.post_tag_lateral_mm,
                            cfg.post_tag_lateral_speed_mm_s)
    _phase(env, c, 'WALL_PREMOVE')
    c._checked_move('forward', cfg.wall_premove_mm, cfg.wall_premove_speed_mm_s)


def purple_to_orange(env, profile):
    c = env.control(profile)
    c._check_active()
    if env.data.get('purple_route_done', False):
        return
    cfg = c.config
    lateral_mm = c._measure_lateral_displacement_mm(
        _collection_origin(env, 'purple_origin'))
    env.data['purple_lateral_mm'] = lateral_mm
    _phase(env, c, 'POST_GRAB_REVERSE')
    c._checked_move('backward', cfg.post_grab_reverse_mm,
                    cfg.post_grab_reverse_speed_mm_s)
    _phase(env, c, 'TURN_RIGHT')
    c._turn_to_heading(cfg.post_grab_heading_target_cw_deg)
    forward_mm = cfg.post_grab_forward_base_mm - lateral_mm
    if forward_mm <= 0.0:
        raise RuntimeError('post-purple forward distance must be positive')
    speed, accel = _compensation_parameters(
        cfg, forward_mm, cfg.post_grab_forward_speed_mm_s)
    _phase(env, c, 'RETURN_MOVE')
    c._checked_move('forward', forward_mm, speed, accel_ms=accel)
    if cfg.left_wall_approach_enabled:
        _wall(env, c, 'left', 'LEFT_WALL_APPROACH')
    _wall(env, c, 'forward', 'FINAL_WALL_APPROACH')
    c._recalibrate_heading_zero()
    env.data['purple_route_done'] = True


def orange_depart_reverse(env, profile):
    c = env.control(profile)
    c._check_active()
    if env.data.get('orange_reverse_done', False):
        return
    cfg = c.config
    env.data['orange_lateral_mm'] = c._measure_lateral_displacement_mm(
        _collection_origin(env, 'orange_origin'))
    _phase(env, c, 'POST_ORANGE_REVERSE')
    c._checked_move('backward', cfg.post_orange_reverse_mm,
                    cfg.post_orange_reverse_speed_mm_s)
    env.data['orange_reverse_done'] = True


def orange_to_build(env, profile):
    c = env.control(profile)
    cfg = c.config
    c._check_active()
    if not env.data.get('orange_reverse_done', False):
        orange_depart_reverse(env, profile)
    lateral_mm = env.data.get('orange_lateral_mm')
    if lateral_mm is None:
        raise RuntimeError('orange collection displacement unavailable')
    correction_mm = cfg.post_orange_lateral_base_mm - lateral_mm
    _phase(env, c, 'POST_ORANGE_LATERAL')
    if correction_mm != 0.0:
        direction = 'right' if correction_mm > 0.0 else 'left'
        distance_mm = abs(correction_mm)
        speed, accel = _compensation_parameters(
            cfg, distance_mm, cfg.post_orange_lateral_speed_mm_s)
        c._checked_move(direction, distance_mm, speed, accel_ms=accel)
    _phase(env, c, 'FINAL_TURN')
    c._turn_to_heading(cfg.final_turn_target_cw_deg)
    _phase(env, c, 'BUILD_ROUTE')
    c._checked_move('forward', cfg.build_route_distance_mm,
                    cfg.build_route_speed_mm_s,
                    accel_ms=cfg.long_distance_forward_accel_ms)


def build_offset(env, profile):
    c = env.control(profile)
    cfg = c.config
    _phase(env, c, 'POST_TAG6_LATERAL')
    if cfg.post_tag6_lateral_right_mm > 0.0:
        c._checked_move(cfg.post_tag6_lateral_direction,
                        cfg.post_tag6_lateral_right_mm,
                        cfg.post_tag6_lateral_speed_mm_s)


def build_return(env, profile):
    c = env.control(profile)
    cfg = c.config
    _phase(env, c, 'POST_BUILD_REVERSE')
    c._checked_move('backward', cfg.post_build_reverse_mm,
                    cfg.post_build_reverse_speed_mm_s)
    _phase(env, c, 'POST_BUILD_TURN')
    # This calibrated leg deliberately uses a relative clockwise half-turn.
    env.robot.chassis.turn(cfg.post_build_turn_cw_deg,
                           cfg.delivery_turn_speed_deg_s,
                           hold_ms=0, settle_cycles=1)
    _phase(env, c, 'POST_BUILD_ROUTE')
    c._checked_move('left', cfg.post_build_route_distance_mm,
                    cfg.post_build_route_speed_mm_s)
    _wall(env, c, 'left', 'POST_BUILD_LEFT_WALL')


def unload_approach(env, profile):
    c = env.control(profile)
    cfg = c.config
    _phase(env, c, 'INITIAL_LATERAL')
    c._checked_move('left', cfg.initial_lateral_left_mm,
                    cfg.long_route_speed_mm_s,
                    accel_ms=cfg.long_distance_forward_accel_ms)
    _wall(env, c, 'left', 'LEFT_WALL_APPROACH')
    if cfg.post_wall_lateral_right_mm > 0.0:
        _phase(env, c, 'POST_WALL_LATERAL')
        c._checked_move('right', cfg.post_wall_lateral_right_mm,
                        cfg.lateral_speed_mm_s)
    _wall(env, c, 'forward', 'FORWARD_WALL_APPROACH')


def unload_depart(env, profile):
    c = env.control(profile)
    cfg = c.config
    _phase(env, c, 'FINAL_LATERAL')
    c._checked_move('right', cfg.final_lateral_right_mm,
                    cfg.long_route_speed_mm_s,
                    accel_ms=cfg.long_distance_forward_accel_ms)


def staged_initial(env, profile):
    c = env.control(profile)
    cfg = c.config
    _phase(env, c, 'INITIAL_LEFT')
    c._checked_move('left', cfg.initial_left_mm, cfg.long_route_speed_mm_s,
                    accel_ms=cfg.long_distance_forward_accel_ms)
    _wall(env, c, 'left', 'LEFT_WALL_APPROACH')


def staged_to_build(env, profile):
    c = env.control(profile)
    cfg = c.config
    _phase(env, c, 'BACKWARD')
    c._checked_move('backward', cfg.load_reverse_mm, cfg.route_speed_mm_s)
    _phase(env, c, 'BUILD_APPROACH')
    c._checked_move('right', cfg.building_approach_right_mm,
                    cfg.long_route_speed_mm_s,
                    accel_ms=cfg.long_distance_forward_accel_ms)


def staged_return_first(env, profile):
    c = env.control(profile)
    cfg = c.config
    _phase(env, c, 'POST_BUILD_REVERSE')
    c._checked_move('backward', cfg.post_build_reverse_mm,
                    cfg.post_build_reverse_speed_mm_s)
    _phase(env, c, 'FIRST_BUILD_LEFT')
    c._checked_move('left', cfg.first_build_left_mm,
                    cfg.build_followup_speed_mm_s,
                    accel_ms=cfg.long_distance_forward_accel_ms)
    _wall(env, c, 'left', 'LEFT_WALL_APPROACH')
    _phase(env, c, 'SECOND_LOAD_APPROACH')
    c._checked_move('right', cfg.second_load_right_mm, cfg.route_speed_mm_s)


def staged_return_final(env, profile):
    c = env.control(profile)
    cfg = c.config
    _phase(env, c, 'POST_BUILD_REVERSE')
    c._checked_move('backward', cfg.post_build_reverse_mm,
                    cfg.post_build_reverse_speed_mm_s)
    _phase(env, c, 'FINAL_LEFT')
    c._checked_move('left', cfg.final_left_mm, cfg.build_followup_speed_mm_s,
                    accel_ms=cfg.long_distance_forward_accel_ms)


ROUTES = {
    'depart_a': depart_a,
    'depart_b': depart_b,
    'return_orange': return_orange,
    'ground_delivery_reverse': ground_delivery_reverse,
    'ground_to_delivery': ground_to_delivery,
    'ground_tag_offset': ground_tag_offset,
    'ground_delivery_depart': ground_delivery_depart,
    'to_purple': to_purple,
    'purple_to_orange': purple_to_orange,
    'orange_depart_reverse': orange_depart_reverse,
    'orange_to_build': orange_to_build,
    'build_offset': build_offset,
    'build_return': build_return,
    'unload_approach': unload_approach,
    'unload_depart': unload_depart,
    'staged_initial': staged_initial,
    'staged_to_build': staged_to_build,
    'staged_return_first': staged_return_first,
    'staged_return_final': staged_return_final,
}
