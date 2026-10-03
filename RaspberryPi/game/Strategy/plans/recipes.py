"""Pure expansion of competition objectives into named functional actions.

These helpers return data only. Hardware code cannot select a round or run a
mission; all choices/counts/routes are visible in the expanded plan.
"""

from ..flows.model import ActionSpec


def step(kind, name, profile, *, requires=None, ends=None, **parameters):
    return ActionSpec(kind, name, profile, parameters, requires, ends)


def navigate(name, profile, route, *, requires=None, ends=None, after_build=False):
    return step('navigate', name, profile, requires=requires, ends=ends,
                route=route, after_build=after_build)


def orange_pickups(prefix, profile, method, *, conditional=False):
    actions = [step('begin_collection', f'{prefix}.begin', profile, color='orange')]
    for index in range(1, 4):
        actions.extend((
            step('acquire_cube', f'{prefix}.find.{index}', profile,
                 index=index, conditional_on_purple=conditional),
            step('grab_cube', f'{prefix}.grab.{index}', profile,
                 index=index, conditional_on_purple=conditional, method=method),
        ))
    return actions


def ground_delivery(prefix, profile):
    return (
        step('anchor_wall', f'{prefix}.anchor', profile, requires='ground_area',
             direction='forward', recalibrate=True),
        *orange_pickups(f'{prefix}.orange', profile, 'grap3'),
        step('inspect_cargo', f'{prefix}.inspect', profile, method='grap3',
             exit_route='ground_delivery_reverse', cross_region_refill=True),
        navigate(f'{prefix}.delivery', profile, 'ground_to_delivery', ends='delivery_tag'),
        step('align_tag', f'{prefix}.tag', profile, purpose='delivery'),
        navigate(f'{prefix}.offset', profile, 'ground_tag_offset'),
        step('anchor_wall', f'{prefix}.unload_anchor', profile,
             direction='forward', recalibrate=True, reference_cw_deg=180.0, ends='unload_wall'),
        step('unload', f'{prefix}.unload', profile),
        navigate(f'{prefix}.leave', profile, 'ground_delivery_depart', ends='delivery_exit'),
    )


def mixed_collection(prefix, profile):
    return (
        step('rebase_heading', f'{prefix}.heading', profile, requires='delivery_exit',
             reference_cw_deg=180.0),
        navigate(f'{prefix}.purple_route', profile, 'to_purple', ends='purple_area'),
        step('anchor_wall', f'{prefix}.purple_anchor', profile, direction='forward'),
        step('begin_collection', f'{prefix}.purple.begin', profile, color='purple'),
        step('acquire_cube', f'{prefix}.purple.find', profile),
        step('grab_cube', f'{prefix}.purple.grab', profile, method='grap2',
             followup_route='purple_to_orange'),
        navigate(f'{prefix}.orange_route', profile, 'purple_to_orange', ends='upper_orange_area'),
        *orange_pickups(f'{prefix}.orange', profile, 'grap1', conditional=True),
        step('inspect_cargo', f'{prefix}.inspect', profile, method='grap1',
             exit_route='orange_depart_reverse', cross_region_refill=True),
        navigate(f'{prefix}.build_route', profile, 'orange_to_build', ends='build_approach'),
    )


def build_and_return(prefix, profile):
    return (
        step('align_tag', f'{prefix}.tag', profile, requires='build_approach', purpose='build'),
        navigate(f'{prefix}.offset', profile, 'build_offset'),
        step('align_building', f'{prefix}.align', profile),
        step('build', f'{prefix}.build', profile),
        navigate(f'{prefix}.return', profile, 'build_return', ends='ground_area', after_build=True),
    )


def wall_unload(prefix, profile):
    return (
        navigate(f'{prefix}.approach', profile, 'unload_approach',
                 requires='build_approach', ends='unload_wall'),
        step('unload', f'{prefix}.unload', profile),
        navigate(f'{prefix}.leave', profile, 'unload_depart', ends='delivery_exit'),
    )


def staged_building(prefix):
    profile = 'staged-building'
    actions = [step('rebase_heading', f'{prefix}.heading', profile,
                    requires='delivery_exit', reference_cw_deg=180.0),
               navigate(f'{prefix}.approach', profile, 'staged_initial', ends='staged_materials')]
    for index, route in ((1, 'staged_return_first'), (2, 'staged_return_final')):
        actions.extend((
            step('load_staged', f'{prefix}.load.{index}', profile),
            navigate(f'{prefix}.approach_build.{index}', profile, 'staged_to_build'),
            step('align_building', f'{prefix}.align.{index}', profile),
            step('build', f'{prefix}.build.{index}', profile),
            navigate(f'{prefix}.return.{index}', profile, route, after_build=True,
                     ends='staged_materials' if index == 1 else 'finished'),
        ))
    return tuple(actions)


def return_orange(prefix):
    return (step('rebase_heading', f'{prefix}.heading', 'return-orange',
                 requires='delivery_exit', reference_cw_deg=180.0),
            navigate(prefix, 'return-orange', 'return_orange',
                     requires='delivery_exit', ends='ground_area'))
