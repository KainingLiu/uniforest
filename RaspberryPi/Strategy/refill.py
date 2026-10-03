"""Main runtime adapter for count-driven alternate orange-area collection."""

from .orange_search import OrangeSearchRecovery
from .refill_policy import BothOrangeAreasExhausted, checked_count, missing
from .refill_routes import transfer


def _fresh_count(c):
    c._check_active()
    c.state = type(c.state).COUNT_CHECK
    if not c.robot.chassis.set_speeds([0, 0, 0, 0]):
        raise RuntimeError('failed to stop before orange count')
    count = checked_count(c.robot.check_carried_cube_count(
        allow_visual_failure=False, allow_idle=True))
    c._check_active()
    return count


def _begin(c, profile):
    c._set_cube_profile(profile)
    c.robot.reset_vision_filter()
    c._search_position_mm = 0.
    c._orange_recovery = OrangeSearchRecovery(origin=c._capture_lateral_origin())


def collect_other_region(c, count):
    from .competition import CompetitionProgram, SearchRangeExhausted
    from .task1 import Task1Config, Task1_2Config, Task1_3Config
    from .task2 import Task2Config, Task2_2Config, Task2Program
    from .task4 import Task4Config, Task4_2Config
    count = checked_count(count)
    if count == 3:
        return
    source = 'highland' if hasattr(c.config, 'orange_target_count_without_purple') else 'ground'
    target = 'ground' if source == 'highland' else 'highland'
    if source == 'ground':
        ground = c.config
        highland = Task2_2Config() if isinstance(ground, (Task1_2Config, Task1_3Config)) else Task2Config()
    else:
        highland = c.config
        ground = Task1_2Config() if isinstance(highland, Task2_2Config) else Task1Config()
    configs = dict(ground_config=ground, highland_config=highland,
                   unload_config=Task4_2Config() if isinstance(highland, Task2_2Config) else Task4Config())
    print(f'[{c.TASK_LABEL}] Orange refill {source} -> {target}; count={count}, missing={missing(count)}')
    transfer(c, source, target, **configs)
    alternate = (Task2Program(c.robot, highland, context=c.context) if target == 'highland'
                 else CompetitionProgram(c.robot, ground, context=c.context))
    alternate._heading_zero_deg = c._heading_zero_deg
    _begin(alternate, 'task2_orange' if target == 'highland' else 'default')
    grab = alternate._grab_task2_orange if target == 'highland' else alternate._grab_task1_orange
    try:
        count = _fresh_count(alternate)
        while missing(count):
            alternate._check_active()
            try:
                grab()
            except SearchRangeExhausted:
                count = _fresh_count(alternate)
                if count != 3:
                    diagnostics = getattr(c.robot, 'diagnostics', None)
                    if diagnostics is not None:
                        diagnostics.write('orange_collection_pending', count=count,
                                          reason='both_areas_exhausted_pending')
                    raise BothOrangeAreasExhausted(count)
                break
            count = _fresh_count(alternate)
        # Only full cargo rejoins the original task's already-calibrated exit.
        transfer(alternate, target, source, **configs)
        c._heading_zero_deg = alternate._heading_zero_deg
        _begin(c, 'task2_orange' if source == 'highland' else 'default')
        if _fresh_count(c) != 3:
            raise RuntimeError('cargo changed during return; collection state needs recheck')
    finally:
        c._set_cube_profile('default')
