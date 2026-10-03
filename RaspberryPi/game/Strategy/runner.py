"""Run one complete functional action plan, without numbered task programs."""
from dataclasses import asdict
from contextlib import suppress
import math
import uuid

from .context import ExecutionContext
from .execution import ExecutionRuntime
from .flows.factory import ActionEnvironment
from .plans import PLANS, PLAN_IDS, validate_plan

# Old command-line names only translate to new data plans; no old program runs.
LEGACY_SELECTIONS = {'all':'PlanA','classic':'PlanA','plana':'PlanA','planb':'PlanB',
    'round1':'set1','round2':'set2','task0':'depart-a','task0-1':'depart-a',
    'task0-2':'depart-b','task0-3':'return-orange','task1':'collect-orange-1',
    'task1-r1':'collect-orange-1','task1-r2':'collect-orange-2',
    'task2':'collect-build-1','task2-r1':'collect-build-1','task2-r2':'collect-build-2',
    'task5':'build-staged'}
for i in (1,2,3):
    LEGACY_SELECTIONS[f'task1-{i}'] = f'collect-orange-{i}'
    LEGACY_SELECTIONS[f'task3-{i}'] = f'build-{i}'
for i in (1,2):
    LEGACY_SELECTIONS[f'task2-{i}'] = f'collect-mixed-{i}'
    LEGACY_SELECTIONS[f'task4-{i}'] = f'unload-{i}'
SELECTION_CHOICES = (*PLAN_IDS, *LEGACY_SELECTIONS)


def resolve_selection(selection='PlanA'):
    name = LEGACY_SELECTIONS.get(selection, selection)
    if name not in PLANS:
        raise ValueError(f'unknown strategy/flow: {selection}')
    return PLANS[name]


def run_plan(robot, plan, *, context=None, heading_zero_deg=None, transition_config=None):
    from protocol.transport import Transport
    if isinstance(robot.transport, Transport) and not robot.transport.execution_active:
        raise RuntimeError('extended execution requires a negotiated additive firmware session')
    if (transition_config is not None and transition_config.pickups and
            not (getattr(robot.actions,'pickup_full_lift_validated',False) or
                 transition_config.trial_run and getattr(robot.actions,'pickup_trial_enabled',False))):
        raise ValueError('pickup transitions require verified full-lift firmware on this Robot')
    if context is not None and context.robot is not robot:
        raise ValueError('execution context belongs to a different robot')
    if context is not None and heading_zero_deg is not None:
        raise ValueError('supply context or heading zero, not both')
    known_zero = context.heading_zero_deg if context is not None else heading_zero_deg
    validate_plan(plan, heading_zero_deg=known_zero,
                  initial_anchor=context.anchor if context is not None else None)
    if transition_config is not None:
        from .transition_switches import validate_transition_selection
        validate_transition_selection(transition_config, plan)
    if context is None:
        context = ExecutionContext(robot, heading_zero_deg=known_zero, anchor=plan.entry_anchor)
    if not robot.strategy_lock.acquire(blocking=False):
        raise RuntimeError('another strategy is already running on this robot')
    previous_cancel = robot.actions._cancel_event
    robot.actions.set_cancel_event(context.cancel_event)
    report = getattr(robot,'set_collection_context',None)
    diagnostics = getattr(robot,'diagnostics',None)
    env = None
    try:
        context.check_active(require_telemetry=False)
        env = (ActionEnvironment(robot,context) if transition_config is None else
               ActionEnvironment(robot,context,transition_config=transition_config))
        # Only wait for telemetry/cameras; this never starts a mission routine.
        env.control(plan.steps[0].profile)._wait_ready()
        if context.heading_zero_deg is None:
            context.heading_zero_deg = ((robot.telem.yaw_deg + plan.entry_heading_deg + 180) % 360) - 180
        if env.transition_config.motion_planning_enabled:
            start_planning=getattr(env.motion_planning,'start',None)
            if start_planning is not None:
                start_planning(env,plan)
        if report:
            report(flow_id=uuid.uuid4().hex,task=plan.name,phase='STARTUP')
        def trace(event):
            if diagnostics:
                diagnostics.write('execution_phase',selection=plan.name,detail=asdict(event))
        runtime = ExecutionRuntime(guard=context.check_active,stop=env.stop,
            emergency_stop=robot.transport.emergency_stop,close=context.close,
            cancel_event=context.cancel_event,context={'selection':plan.name},trace=trace)
        compiled = env.compile(plan)
        results = runtime.run(compiled)
        if report:
            report(task=plan.name,phase='FINISHED')
        return 0
    except BaseException:
        with suppress(BaseException):
            robot.transport.emergency_stop()
        if report:
            with suppress(BaseException):
                report(task=context.current_action or plan.name,phase='INTERRUPTED_OR_FAILED')
        raise
    finally:
        if env is not None:
            try:
                env.abort()
            finally:
                close_planning=getattr(env.motion_planning,'close',None)
                if close_planning is not None:
                    close_planning(env)
        robot.actions.set_cancel_event(previous_cancel)
        context.close()
        robot.strategy_lock.release()


def run_selection(robot, selection='PlanA', *, context=None, heading_zero_deg=None, transition_config=None):
    plan=resolve_selection(selection)
    if selection in LEGACY_SELECTIONS:
        print(f'[Strategy] Legacy selector {selection!r} -> {plan.name}')
    return run_plan(robot,plan,context=context,heading_zero_deg=heading_zero_deg,
                    transition_config=transition_config)
