"""Loaded orange-area transfers using main's existing Task1/2/4/0 calibrations.

No task run method or unloading mechanism is invoked. Configuration changes
are scoped to these serial route legs; heading corrections survive the trip.
"""


def _wall(c, direction='forward', reference=None):
    c._check_active()
    c._drive_until_wall(direction=direction, timeout_s=c.config.far_wall_timeout_s,
                        speed_mm_s=c.config.far_wall_speed_mm_s,
                        timeout_is_success=False, context='Orange refill transfer')
    if reference is not None:
        c._recalibrate_heading_zero(reference)


def _compensate(c, direction, distance, short_speed):
    if distance >= c.config.compensation_fast_distance_mm:
        c._checked_move(direction, distance, c.config.compensation_fast_speed_mm_s,
                        accel_ms=c.config.long_distance_forward_accel_ms)
    else:
        c._checked_move(direction, distance, short_speed)


def transfer(c, source, destination, *, ground_config, highland_config, unload_config):
    from .task0 import Task0_3Config
    if (source, destination) not in (('ground', 'highland'), ('highland', 'ground')):
        raise ValueError('orange refill transfer needs different regions')
    c._check_active()
    saved_config = c.config
    lateral = c._measure_lateral_displacement_mm(c._orange_recovery.origin)
    try:
        if source == 'ground':
            c.config = cfg = ground_config
            forward = cfg.delivery_forward_base_mm - lateral
            if forward <= 0:
                raise RuntimeError('ground refill transfer distance must be positive')
            c._checked_move('backward', cfg.delivery_reverse_mm, cfg.delivery_reverse_speed_mm_s)
            c._turn_to_heading(cfg.delivery_turn_deg, hold_ms=cfg.delivery_turn_heading_hold_ms)
            c._checked_move('forward', forward, cfg.delivery_forward_speed_mm_s)
            c._turn_to_heading(cfg.delivery_turn_deg * 2)
            c.robot.reset_field_localization_filter()
            c._align_delivery_tag(fine_align_enabled=False, stop_axes_in_tolerance=True,
                                  independent_heading=True)
            if cfg.post_tag_lateral_right_mm:
                c._checked_move(cfg.post_tag_lateral_direction, cfg.post_tag_lateral_right_mm,
                                cfg.post_tag_lateral_speed_mm_s)
            _wall(c, reference=180.)
            # Retain the unloading route geometry with hatches closed.
            c._checked_move('backward', cfg.unload_reverse_mm, cfg.unload_reverse_speed_mm_s)
            if cfg.pre_final_turn_lateral_left_mm:
                c._checked_move(cfg.pre_final_turn_lateral_direction,
                                cfg.pre_final_turn_lateral_left_mm, cfg.pre_final_turn_lateral_speed_mm_s)
            c.config = cfg = highland_config
            c._recalibrate_heading_zero(180.)
            c._checked_move('backward', cfg.initial_distance_mm, cfg.initial_speed_mm_s,
                            accel_ms=cfg.long_distance_forward_accel_ms)
            c._turn_to_heading(cfg.delivery_heading_target_cw_deg)
            if cfg.tag3_alignment_enabled:
                c.robot.reset_field_localization_filter()
                c._align_delivery_tag(fine_align_enabled=False)
                if cfg.post_tag_lateral_mm:
                    c._checked_move('right', cfg.post_tag_lateral_mm, cfg.post_tag_lateral_speed_mm_s)
            c._checked_move('forward', cfg.wall_premove_mm, cfg.wall_premove_speed_mm_s)
            _wall(c)
            # Roof area is a waypoint only: no purple search or Grap2.
            c._checked_move('backward', cfg.post_grab_reverse_mm, cfg.post_grab_reverse_speed_mm_s)
            c._turn_to_heading(cfg.post_grab_heading_target_cw_deg)
            _compensate(c, 'forward', cfg.post_grab_forward_base_mm, cfg.post_grab_forward_speed_mm_s)
            if cfg.left_wall_approach_enabled:
                _wall(c, 'left')
            _wall(c, reference=0.)
        else:
            c.config = cfg = highland_config
            c._checked_move('backward', cfg.post_orange_reverse_mm, cfg.post_orange_reverse_speed_mm_s)
            correction = cfg.post_orange_lateral_base_mm - lateral
            if correction:
                _compensate(c, 'right' if correction > 0 else 'left', abs(correction),
                            cfg.post_orange_lateral_speed_mm_s)
            c._turn_to_heading(cfg.final_turn_target_cw_deg)
            c._checked_move('forward', cfg.build_route_distance_mm, cfg.build_route_speed_mm_s,
                            accel_ms=cfg.long_distance_forward_accel_ms)
            c.config = cfg = unload_config
            c._checked_move('left', cfg.initial_lateral_left_mm, cfg.long_route_speed_mm_s,
                            accel_ms=cfg.long_distance_forward_accel_ms)
            _wall(c, 'left')
            if cfg.post_wall_lateral_right_mm:
                c._checked_move('right', cfg.post_wall_lateral_right_mm, cfg.lateral_speed_mm_s)
            _wall(c, reference=180.)
            c._checked_move('backward', cfg.unload_reverse_mm, cfg.unload_reverse_speed_mm_s)
            c._checked_move('right', cfg.final_lateral_right_mm, cfg.long_route_speed_mm_s,
                            accel_ms=cfg.long_distance_forward_accel_ms)
            c.config = cfg = Task0_3Config()
            c._turn_to_heading(cfg.target_heading_cw_deg)
            c._checked_move('left', cfg.lateral_left_mm, cfg.lateral_speed_mm_s)
            _wall(c, 'left')
            c.config = ground_config
            _wall(c, reference=0.)
        c._check_active()
        if not c.robot.chassis.set_speeds([0, 0, 0, 0]):
            raise RuntimeError('failed to stop orange transfer')
    finally:
        c.config = saved_config
