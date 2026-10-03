"""One bounded alternate-area visit when the current orange search is spent.

Fresh inspected counts decide the deficit. Completed mechanism sequences do
not prove captured cargo. Both-area exhaustion has no automatic next task.
"""

from dataclasses import replace

from . import operations, refill_routes
from ..refill_policy import BothOrangeAreasExhausted, checked_count, missing


def _report(env, spec, status, **details):
    print(f'[{spec.name}] Cross-region refill: {status}; {details}')
    diagnostics = getattr(env.robot, 'diagnostics', None)
    if diagnostics is not None:
        diagnostics.write('cross_region_refill', action=spec.name,
                          status=status, **details)


def _count(env, spec):
    env.context.check_active()
    env.stop()
    env.data['carried_count'] = None
    env.phase(env.control(spec.profile), 'COUNT_CHECK')
    count = env.robot.check_carried_cube_count(
        allow_visual_failure=False, allow_idle=True)
    env.context.check_active()
    count = checked_count(count)
    env.data['carried_count'] = count
    return count


def on_exhausted(env, spec):
    """Called only after serializing/closing the original inspection session."""
    collection = operations._collection(env, spec)
    if not spec.parameters.get('cross_region_refill', False):
        return None
    if collection['color'] != 'orange' or not collection['exhausted']:
        return None
    env.context.check_active()
    source = 'ground' if spec.profile.startswith('ground-') else 'highland'
    target = 'highland' if source == 'ground' else 'ground'
    round_index = int(spec.profile.rsplit('-', 1)[1])
    ground_profile = spec.profile if source == 'ground' else f'ground-{round_index}'
    highland_profile = (spec.profile if source == 'highland'
                        else f'highland-{min(round_index, 2)}')
    target_profile = highland_profile if target == 'highland' else ground_profile
    target_method = 'grap1' if target == 'highland' else 'grap3'

    # Early departure may already have moved away from the collection wall.
    # Close camera ownership before returning or doing a new inspection.
    env.transitions.finish_inspection(spec)
    env.transitions._return_for_refill()
    count = _count(env, spec)
    if count == 3:
        _report(env, spec, 'already_full', count=count)
        return count
    if collection.get('alternate_visited', False):
        raise BothOrangeAreasExhausted(count)
    collection['alternate_visited'] = True

    _report(env, spec, 'depart', source=source, target=target, count=count,
            missing=missing(count))
    source_search = env.control(spec.profile)._search_position_mm
    old_action, old_anchor = env.context.current_action, env.context.anchor
    # An independent action name also prevents consuming planned Tag handoffs.
    env.context.current_action = spec.name + '.cross-region'
    try:
        refill_routes.transfer(env, source, target, ground_profile=ground_profile,
                               highland_profile=highland_profile)
        target_spec = replace(spec, name=spec.name+'.alternate', profile=target_profile,
                              requires_anchor=None, ends_at=None,
                              parameters={'color': 'orange'})
        operations.begin_collection(env, replace(target_spec, kind='begin_collection'))
        count = _count(env, target_spec)
        while missing(count):
            env.context.check_active()
            pickup = replace(target_spec, kind='acquire_cube',
                             name=f'{target_spec.name}.next',
                             parameters={'index': 1, 'method': target_method})
            if not operations.acquire_cube(env, pickup):
                count = _count(env, target_spec)
                if count == 3:
                    break
                if env.data['collection']['exhausted']:
                    env.data['collection']['alternate_visited'] = True
                    _report(env, spec, 'both_areas_exhausted_pending', count=count)
                    raise BothOrangeAreasExhausted(count)
                raise RuntimeError('orange alignment unresolved; area exhaustion not confirmed')
            if not operations.grab_cube(env, replace(pickup, kind='grab_cube')):
                raise RuntimeError('cross-region pickup lost its acquisition before gripping')
            count = _count(env, target_spec)
        pickups = collection['pickups'] + env.data['collection']['pickups']
        refill_routes.transfer(env, target, source, ground_profile=ground_profile,
                               highland_profile=highland_profile)
        # Reanchoring changed the route origin. Rebuild its odometry bookkeeping
        # instead of reusing encoder deltas measured before the round trip.
        operations.begin_collection(env, replace(spec, kind='begin_collection',
                                    parameters={'color': 'orange'}))
        returned = env.data['collection']
        returned.update(exhausted=True, alternate_visited=True,
                        pickups=pickups)
        count = _count(env, spec)
        if count != 3:
            raise RuntimeError('cargo changed during return; collection state needs recheck')
        env.context.anchor = old_anchor
        _report(env, spec, 'complete', count=count,
                source_search_mm=source_search)
        return count
    finally:
        # Errors propagate; neither a camera restore nor a reconnect resumes it.
        env.context.current_action = old_action
        env.control(spec.profile)._set_cube_profile('default')
