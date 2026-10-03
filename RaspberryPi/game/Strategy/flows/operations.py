"""Reusable collection, inspection and mechanism operations.

These handlers consume an operation specification and one execution environment.
They have no knowledge of a competition plan or of a numbered task. Search
origins/budgets belong to a collection session, so repeated pickups and inspection
refills use the same measured origin and recovery budget.
"""

from __future__ import annotations

from dataclasses import replace
import time

from ..errors import SearchRangeExhausted
from ..orange_search import OrangeSearchRecovery


def _collection(env, spec):
    collection = env.data.get('collection')
    if collection is None or collection['profile'] != spec.profile:
        raise RuntimeError('begin_collection is required for this pickup profile')
    return collection


def _selected_pickup(env, spec, controller):
    """Evaluate a conditional slot immediately before acquisition/gripping."""
    params = spec.parameters
    index = params.get('index', 1)
    if params.get('conditional_on_purple', False):
        cfg = controller.config
        limit = (cfg.orange_target_count if env.data.get('purple_grabbed', False)
                 else cfg.orange_target_count_without_purple)
    else:
        limit = params.get('max_count', index)
    return index <= limit


def anchor_wall(env, spec):
    controller = env.control(spec.profile)
    cfg, params = controller.config, spec.parameters
    env.phase(controller, 'WALL_APPROACH')
    controller._drive_until_wall(
        direction=params.get('direction', 'forward'),
        timeout_s=params.get('timeout_s', cfg.far_wall_timeout_s),
        speed_mm_s=params.get('speed_mm_s', cfg.far_wall_speed_mm_s),
        context=spec.name,
    )
    settle_s = params.get('settle_s', cfg.wall_settle_s)
    if settle_s:
        time.sleep(settle_s)
        controller._check_active()
    if params.get('recalibrate', False):
        controller._recalibrate_heading_zero(params.get('reference_cw_deg', 0.0))
    return True


def begin_collection(env, spec):
    controller = env.control(spec.profile)
    color = spec.parameters.get('color', 'orange')
    if color not in ('orange', 'purple'):
        raise ValueError(f'unsupported cube color: {color}')
    highland = hasattr(controller.config, 'orange_target_count_without_purple')
    detector_profile = spec.parameters.get(
        'detector_profile', f'task2_{color}' if highland else 'default')
    controller._set_cube_profile(detector_profile)
    env.robot.reset_vision_filter()
    controller._search_position_mm = 0.0
    origin = controller._capture_lateral_origin()
    if color == 'orange':
        controller._orange_recovery = OrangeSearchRecovery(origin=origin)
    env.data['collection'] = {
        'origin': origin,
        'color': color,
        'profile': spec.profile,
        'detector_profile': detector_profile,
        'exhausted': False,
        'acquired': False,
        # Completed mechanism sequences, not independently verified cube count.
        'pickups': 0,
    }
    region = 'purple' if color == 'purple' else ('orange' if highland else 'ground')
    env.data[f'{region}_origin'] = origin
    env.data[f'{region}_lateral_mm'] = None
    env.data[f'{region}_reverse_done'] = False
    if color == 'purple':
        env.data['purple_grabbed'] = False
        env.data['purple_route_done'] = False
    return color


def _fast_align(env, controller, block, color, profile):
    from ..optimizations.fast_alignment import align_cube
    return align_cube(controller, block, color_name=color,
                      phase_origin=env.data['collection']['origin'], profile=profile,
                      neighbor_observer=_neighbor_observer(env, color))


def _neighbor_observer(env, color):
    """Start one preview history for this acquisition, including moving handoff."""
    from ..optimizations.adaptive_blind import NeighborObserver
    collection = env.data['collection']
    collection.pop('neighbor_observer', None)
    if color != 'orange':
        return None
    profile = collection['profile']
    config = env.transition_config
    policy = config.pickup(profile, 'grap3' if profile.startswith('ground-') else 'grap1')
    if policy is None or policy.next_adaptive is None or config.alignment(profile, color) is None:
        return None
    observer = NeighborObserver(policy.next_adaptive)
    collection['neighbor_observer'] = observer
    return observer


def _acquire_purple(env, controller, alignment=None):
    cfg = controller.config
    attempts = 0
    while True:
        env.phase(controller, 'PURPLE_SEARCH')
        block = controller._find_cube(
            color_name='purple', min_confidence=cfg.purple_min_confidence,
            search_direction=-1.0,
            max_distance_mm=cfg.purple_search_max_distance_mm)
        env.phase(controller, 'PURPLE_ALIGN')
        if alignment is not None:
            if _fast_align(env,controller,block,'purple',alignment):
                return True
            attempts += 1
            if attempts >= alignment.max_attempts:
                raise RuntimeError('purple alignment unresolved; absence not confirmed')
            continue
        if controller._align_cube(
                block, color_name='purple', min_confidence=cfg.purple_min_confidence,
                timeout_s=cfg.purple_align_timeout_s, timeout_is_success=True):
            return True


def _acquire_orange(env, controller, alignment=None):
    cfg = controller.config
    separate_orange_target = hasattr(cfg, 'orange_align_target_x_mm')
    attempts = 0
    while True:
        env.phase(controller, 'ORANGE_SEARCH')
        search = {
            'color_name': 'orange', 'min_confidence': cfg.orange_min_confidence,
            'search_direction': 1.0,
            'lock_x_jump_mm': cfg.orange_search_lock_x_jump_mm,
        }
        if separate_orange_target:
            search['ambiguity_margin_mm'] = cfg.orange_track_ambiguity_margin_mm
        block = controller._find_cube(**search)
        env.phase(controller, 'ORANGE_ALIGN')
        if alignment is not None:
            if _fast_align(env,controller,block,'orange',alignment):
                return True
            attempts += 1
            if attempts >= alignment.max_attempts:
                return False
            continue
        if not separate_orange_target:
            if controller._align_orange(block):
                return True
            continue
        if controller._align_cube(
                block, color_name='orange', min_confidence=cfg.orange_min_confidence,
                align_min_x_mm=cfg.orange_align_min_x_mm,
                align_max_x_mm=cfg.orange_align_max_x_mm,
                align_target_x_mm=cfg.orange_align_target_x_mm,
                ambiguity_margin_mm=cfg.orange_track_ambiguity_margin_mm,
                timeout_s=cfg.orange_coarse_align_timeout_s,
                timeout_is_success=True):
            if controller._last_alignment_timed_out or controller._fine_align_orange(block):
                return True


def acquire_cube(env, spec):
    controller = env.control(spec.profile)
    collection = _collection(env, spec)
    collection['acquired'] = False
    collection.pop('neighbor_observer', None)
    if collection['exhausted'] or not _selected_pickup(env, spec, controller):
        return False
    controller._set_cube_profile(collection['detector_profile'])
    config = getattr(env,'transition_config',None)
    alignment = config.alignment(spec.profile,collection['color']) if config is not None else None
    try:
        acquired = (_acquire_purple(env, controller, alignment)
                    if collection['color'] == 'purple'
                    else _acquire_orange(env, controller, alignment))
        collection['acquired'] = acquired
        return acquired
    except SearchRangeExhausted:
        collection['exhausted'] = True
        print(f'[{spec.name}] {collection["color"]} search budget exhausted; '
              'continue without another grab')
        return False
    finally:
        # Purple detection has a separate calibrated profile. Restore the
        # default before its mechanism/transit, as in the measured route.
        if collection['color'] == 'purple':
            controller._set_cube_profile('default')


def grab_cube(env, spec):
    controller = env.control(spec.profile)
    collection = _collection(env, spec)
    if not collection['acquired'] or not _selected_pickup(env, spec, controller):
        return False
    method = spec.parameters.get('method')
    if method not in ('grap1', 'grap2', 'grap3'):
        raise ValueError(f'unsupported grab method: {method}')
    env.phase(controller, 'GRAB')
    followup_route = spec.parameters.get('followup_route')
    callback = (None if followup_route is None else
                lambda: env.run_route(followup_route, spec.profile))
    controller._grab_with_wall_press(
        getattr(env.robot.actions, method),
        recalibrate_heading_zero=collection['color'] == 'orange',
        chassis_followup=callback,
    )
    collection['acquired'] = False
    collection['pickups'] += 1
    if collection['color'] == 'purple':
        # This flag retains the existing conditional pickup semantics: Grap2
        # completed. It is not sensor confirmation that a cube was captured.
        env.data['purple_grabbed'] = True
    else:
        env.robot.reset_vision_filter()
        time.sleep(controller.config.post_grab_settle_s)
        controller._check_active()
    return True


def inspect_cargo(env, spec):
    """Inspect/refill using the current collection's cumulative search budget.

    The count camera can return unknown. Only a full or unknown result permits
    the optional chassis route during camera restoration. Refills share the
    initial search origin and consume the same remaining search budget. Plans
    opting into cross-region refill can make one separately bounded alternate
    visit when this region is exhausted, then reanchor before continuing.
    """
    controller = env.control(spec.profile)
    collection = _collection(env, spec)
    if collection['color'] != 'orange':
        raise ValueError('cargo refill requires an orange collection session')
    exit_route = spec.parameters.get('exit_route')
    callback = (None if exit_route is None else
                lambda: env.run_route(exit_route, spec.profile))
    try:
        while not collection['exhausted']:
            env.phase(controller, 'COUNT_CHECK')
            count = env.robot.check_carried_cube_count(
                chassis_followup=callback, allow_visual_failure=True,
                allow_idle=collection.get('pickups') == 0)
            env.data['carried_count'] = count
            if spec.parameters.get('cross_region_refill'):
                from ..refill_policy import checked_count
                checked_count(count)
            if count is None or count == 3:
                return count
            if count not in (0, 1, 2):
                raise RuntimeError(f'invalid carried cube count: {count}')
            # A refill is a local recovery within this operation, not another
            # mission or nested runtime. It retains the same hardware guard.
            for index in range(1, 4 - count):
                params = {'index': index, 'method': spec.parameters['method']}
                refill = replace(spec, kind='acquire_cube',
                                 name=f'{spec.name}.refill-{index}', parameters=params)
                if not acquire_cube(env, refill):
                    if collection['exhausted']:
                        from .refill import on_exhausted
                        return on_exhausted(env, spec)
                    return None
                grab_cube(env, replace(refill, kind='grab_cube'))
        from .refill import on_exhausted
        return on_exhausted(env, spec)
    finally:
        controller._set_cube_profile('default')


def unload(env, spec):
    controller = env.control(spec.profile)
    env.phase(controller, 'UNLOAD')
    controller._unload_cubes()
    return True


def load_staged(env, spec):
    controller = env.control(spec.profile)
    cfg = controller.config
    controller._check_active()
    env.phase(controller, 'HATCH_OPEN')
    env.robot.actions.hatch_open(settle_ms=cfg.hatch_open_settle_ms)
    env.phase(controller, 'WALL_APPROACH')
    controller._drive_until_wall(
        timeout_s=cfg.far_wall_timeout_s, speed_mm_s=cfg.far_wall_speed_mm_s,
        direction='forward', context=spec.name)
    controller._check_active()
    env.phase(controller, 'HATCH_CLOSE')
    env.robot.actions.hatch_close(settle_ms=cfg.hatch_close_settle_ms)
    controller._check_active()
    return True


OPERATIONS = {
    'anchor_wall': anchor_wall,
    'begin_collection': begin_collection,
    'acquire_cube': acquire_cube,
    'grab_cube': grab_cube,
    'inspect_cargo': inspect_cargo,
    'unload': unload,
    'load_staged': load_staged,
}
