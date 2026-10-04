"""Unified executor for named strategies and reusable task sequences."""

import uuid
from collections import deque

from .context import BuildApproach, TaskContext
from .plans import PLAN_IDS, PLANS, StrategyPlan, validate_plan
from .tasks import TASK_IDS, TASK_LIBRARY, TaskStep
from .results import TaskResult
from .plan_d_state import PlanDState


# Old Task2 selectors follow collect-build, including its Task0-2 positioning.
PLAN_ALIASES = {'classic': 'PlanA', 'plana': 'PlanA', 'planb': 'PlanB', 'planc': 'PlanC',
                'pland': 'PlanD'}
LEGACY_SELECTIONS = {
    **PLAN_ALIASES,
    'all': 'PlanA', 'round1': 'set1', 'round2': 'set2', 'task0': 'task0-1',
    'task1': 'task1-1', 'task1-r1': 'task1-1', 'task1-r2': 'task1-2',
    'task2': 'collect-build-1', 'task2-r1': 'collect-build-1',
    'task2-r2': 'collect-build-2',
}
TASK_CHOICES = (*TASK_IDS, *LEGACY_SELECTIONS)
STRATEGY_CHOICES = (*PLAN_IDS, *PLAN_ALIASES)
SELECTION_CHOICES = (*TASK_IDS, *PLAN_IDS, *LEGACY_SELECTIONS)


def refill_steps(task_id, missing):
    """Only original collection tasks can insert a single, non-recursive detour."""
    if task_id not in ('task1-1', 'task1-2', 'task2-1', 'task2-2'):
        return ()
    if type(missing) is not int or not 0 <= missing <= 3:
        raise ValueError(f'invalid refill amount: {missing}')
    if missing == 0:
        return ()
    if task_id in ('task1-1', 'task1-2'):
        return (TaskStep('task2-0', {'orange_target_count': missing}),)
    return (TaskStep('task0-3'),
            TaskStep('task1-0', {'target_cube_count': missing}))


def resolve_selection(selection='PlanA'):
    canonical = LEGACY_SELECTIONS.get(selection, selection)
    if canonical in PLANS:
        return PLANS[canonical]
    if canonical in TASK_LIBRARY:
        return StrategyPlan(canonical, (TaskStep(canonical),))
    raise ValueError(f'unknown strategy/task: {selection}')


def run_plan(robot, plan, *, context=None, heading_zero_deg=None,
             task_library=TASK_LIBRARY):
    """Robot lifecycle stays with the caller; every task is a fresh instance."""
    if context is not None and context.robot is not robot:
        raise ValueError('task context belongs to a different robot')
    if context is not None and heading_zero_deg is not None:
        raise ValueError('supply either context or heading_zero_deg, not both')
    if context is not None and context.plan_d is not None:
        raise ValueError('PlanD state cannot be resumed; supply a fresh task context')
    handoff = (None if heading_zero_deg is None else
               BuildApproach(heading_zero_deg, 'explicit entry pose'))
    validate_plan(plan, task_library=task_library,
                  initial_handoff=handoff is not None or (
                      context is not None and context.build_approach is not None))
    if plan.name == 'PlanD':
        # Reject malformed calibration before any round can move the robot.
        # Unverified low-layer scales remain a supported Task5 skip.
        from .building_profiles import load_building_profiles, validate_building_profile_targets
        for step in plan.steps:
            if step.task_id == 'task5':
                config = task_library[step.task_id].configuration(step)
                profiles = load_building_profiles(config.building_profiles_path)
                validate_building_profile_targets(profiles, config.building_target_z_mm)
    if context is None:
        context = TaskContext(robot, build_approach=handoff)
    report = getattr(robot, 'set_collection_context', None)
    diagnostics = getattr(robot, 'diagnostics', None)
    if not robot.strategy_lock.acquire(blocking=False):
        raise RuntimeError('another strategy is already running on this robot')
    previous_cancel = robot.actions._cancel_event
    robot.actions.set_cancel_event(context.cancel_event)
    try:
        context.check_active(require_telemetry=False)
        if plan.name == 'PlanD':
            context.plan_d = PlanDState()
        if report is not None:
            report(flow_id=uuid.uuid4().hex, task=plan.name, phase='STARTUP')
        pending = deque(plan.steps)
        resume_task1 = None
        index = 0
        while pending:
            step = pending.popleft()
            context.check_active(require_telemetry=False)
            definition = task_library[step.task_id]
            if definition.requires is None:
                context.build_approach = None
            elif context.build_approach is None:
                raise RuntimeError(f'{step.task_id}: previous task did not publish its handoff')
            context.current_task = step.task_id
            print(f'[Strategy {plan.name}] Starting {step.task_id}')
            if diagnostics is not None:
                diagnostics.write('task_start', task=step.task_id,
                                  selection=plan.name, step=index)
            program = definition.create(context, step)
            raw = program.run()
            context.check_active()
            outcome = raw if isinstance(raw, TaskResult) else TaskResult.from_code(
                int(raw), task=step.task_id)
            if not outcome.ok or outcome.code != 0:
                robot.transport.emergency_stop()
                if report is not None:
                    report(task=step.task_id, phase='FAILED')
                if diagnostics is not None:
                    diagnostics.write('task_failed', task=step.task_id,
                                      code=outcome.code, status=outcome.status.value,
                                      message=outcome.message)
                return outcome.code or 1
            if definition.provides == 'build_approach' and context.build_approach is None:
                raise RuntimeError(f'{step.task_id}: successful task omitted its handoff')
            if diagnostics is not None:
                diagnostics.write('task_complete', task=step.task_id,
                                  status=outcome.status.value)
            print(f'[Strategy {plan.name}] {step.task_id} complete')
            if resume_task1 is not None:
                source_id, source_program = resume_task1
                if step.task_id != 'task2-0':
                    raise RuntimeError('Task1 delivery must resume immediately after Task2-0')
                context.check_active()
                if context.plan_d is not None:
                    context.plan_d.require_refill_result(source_id)
                context.current_task = source_id
                print(f'[Strategy {plan.name}] Resuming {source_id} at Tag6')
                if diagnostics is not None:
                    diagnostics.write('task_resume', task=source_id, phase='DELIVERY_TAG_ALIGN')
                source_program.resume_delivery_after_refill()
                context.check_active()
                if context.plan_d is not None:
                    context.plan_d.finish_refill(source_id)
                if diagnostics is not None:
                    diagnostics.write('task_resume_complete', task=source_id)
                resume_task1 = None
            inserted = refill_steps(step.task_id, getattr(program, 'refill_missing_count', 0))
            if inserted:
                # Validate the detour plus the remaining handoff chain before
                # any detour movement. Preserve every original following step.
                validate_plan(StrategyPlan(plan.name, (*inserted, *pending)),
                              task_library=task_library,
                              initial_handoff=context.build_approach is not None)
                pending.extendleft(reversed(inserted))
                if step.task_id in ('task1-1', 'task1-2'):
                    if context.plan_d is not None:
                        context.plan_d.begin_refill(step.task_id)
                    resume_task1 = (step.task_id, program)
                print(f'[Strategy {plan.name}] Insert refill: '
                      + ' -> '.join(s.task_id for s in inserted))
                if diagnostics is not None:
                    diagnostics.write('refill_inserted', source_task=step.task_id,
                                      missing=program.refill_missing_count,
                                      tasks=[s.task_id for s in inserted])
            index += 1
        if report is not None:
            report(task=plan.name, phase='FINISHED')
        return 0
    except BaseException:
        robot.transport.emergency_stop()
        if report is not None:
            report(task=context.current_task or plan.name, phase='INTERRUPTED_OR_FAILED')
        raise
    finally:
        robot.actions.set_cancel_event(previous_cancel)
        context.close()
        robot.strategy_lock.release()


def run_tasks(robot, selection='PlanA', *, context=None, heading_zero_deg=None):
    plan = resolve_selection(selection)
    if selection in LEGACY_SELECTIONS:
        print(f'[Strategy] Legacy selector {selection!r} -> '
              f'{plan.name}: ' + ' -> '.join(step.task_id for step in plan.steps))
    return run_plan(robot, plan, context=context, heading_zero_deg=heading_zero_deg)


__all__ = ['TASK_CHOICES', 'STRATEGY_CHOICES', 'SELECTION_CHOICES',
           'resolve_selection', 'run_plan', 'run_tasks']
