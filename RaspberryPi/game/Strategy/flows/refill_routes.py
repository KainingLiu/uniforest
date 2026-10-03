"""Carry cargo between the two orange areas using existing route distances.

With motion planning enabled, connect directly via the original ramp corridor
and destination goal, skipping Tag6 and intermediate unloading contacts.
The classic fallback retains its original stops. Both keep cargo aboard;
the direct combination still requires loaded field validation.
"""

from .routes import ROUTES


def _route(env, name, profile):
    env.context.check_active()
    env.stop()
    ROUTES[name](env, profile)
    env.context.check_active()
    env.stop()


def _wall(env, profile, reference=None):
    c = env.control(profile)
    env.phase(c, 'WALL_APPROACH')
    c._drive_until_wall(direction='forward', timeout_is_success=False,
                        context='Cross-region refill anchor')
    if reference is not None:
        c._recalibrate_heading_zero(reference)


def _keep_cargo_departure(env, profile):
    """Execute the unloading route's retreat without opening either hatch."""
    c = env.control(profile)
    c._checked_move('backward', c.config.unload_reverse_mm,
                    c.config.unload_reverse_speed_mm_s)
    c._check_active()


def transfer(env, source, destination, *, ground_profile, highland_profile):
    if getattr(env.transition_config, 'motion_planning_enabled', False):
        return env.motion_planning.run_refill(env, source, destination,
            ground_profile=ground_profile, highland_profile=highland_profile)
    return classic_transfer(env, source, destination, ground_profile=ground_profile,
                            highland_profile=highland_profile)


def classic_transfer(env, source, destination, *, ground_profile, highland_profile):
    if (source, destination) not in (('ground', 'highland'), ('highland', 'ground')):
        raise ValueError('refill transfer must connect different orange regions')
    env.context.check_active()
    env.context.anchor = 'refill_transit'
    if source == 'ground':
        _route(env, 'ground_to_delivery', ground_profile)
        c = env.control(ground_profile)
        env.phase(c, 'TAG_ALIGN')
        env.robot.reset_field_localization_filter()
        # A missed observation cannot establish this new transit's anchor.
        c._align_delivery_tag(fine_align_enabled=False, stop_axes_in_tolerance=True,
                              independent_heading=True)
        _route(env, 'ground_tag_offset', ground_profile)
        _wall(env, ground_profile, 180.0)
        _keep_cargo_departure(env, ground_profile)
        _route(env, 'ground_delivery_depart', ground_profile)
        c = env.control(highland_profile)
        c._recalibrate_heading_zero(180.0)
        _route(env, 'to_purple', highland_profile)
        _wall(env, highland_profile)
        # Purple is only a route waypoint; do not search for or grab a roof.
        env.data['purple_origin'] = c._capture_lateral_origin()
        env.data['purple_route_done'] = False
        _route(env, 'purple_to_orange', highland_profile)
        _wall(env, highland_profile, 0.0)
        env.context.anchor = 'upper_orange_area'
    else:
        unload_profile = 'unload-' + highland_profile.rsplit('-', 1)[1]
        _route(env, 'orange_to_build', highland_profile)
        _route(env, 'unload_approach', unload_profile)
        _wall(env, unload_profile, 180.0)
        _keep_cargo_departure(env, unload_profile)
        _route(env, 'unload_depart', unload_profile)
        env.control('return-orange')._recalibrate_heading_zero(180.0)
        _route(env, 'return_orange', 'return-orange')
        _wall(env, ground_profile, 0.0)
        env.context.anchor = 'ground_area'
    env.stop()
    env.context.check_active()
