"""Functional plan validation, hardware continuity and task-free execution."""
import contextlib
import io
import threading
import time
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from Strategy.context import ExecutionContext
from Strategy.controllers import RobotController
from Strategy.flows.model import ActionSpec
from Strategy.flows.factory import ActionEnvironment
from Strategy.plans import PLANS, StrategyPlan, validate_plan
from Strategy.runner import resolve_selection, run_plan

class FakeActionSession:
    """Explicit milestone fake; routes run outside the mechanism session."""

    def __init__(self, events=None, *, release='third_release', check=None):
        self.events = events
        self.release = release
        self.on_check = check
        self.done = self.closed = self.chassis_ready = False

    def wait_chassis_ready(self):
        if not self.chassis_ready and self.events is not None:
            self.events.append(self.release)
        self.chassis_ready = True

    def check(self):
        if self.closed:
            raise RuntimeError('fake action session closed')
        if self.on_check is not None:
            self.on_check()
        if self.events is not None:
            self.events.append('action_check')

    def wait_done(self):
        if not self.done and self.events is not None:
            self.events.append('build_done')
        self.done = True

    def close(self):
        if not self.closed:
            if not self.done:
                self.abort()
            else:
                self.closed = True

    def abort(self):
        self.closed = True


def robot_fixture():
    """Only records requested calls; does not simulate robot motion."""
    robot = SimpleNamespace(
        telem=SimpleNamespace(yaw_deg=-143.0, uptime_ms=1000),
        has_vision=True, has_field_localization=True,
        strategy_lock=threading.Lock(),
        set_collection_context=Mock(), diagnostics=SimpleNamespace(write=Mock()),
        reset_field_localization_filter=Mock(), reset_vision_filter=Mock(),
        set_cube_detection_profile=Mock(),
    )
    robot.transport = SimpleNamespace(connected=True, emergency_stop_generation=0)
    def emergency_stop():
        robot.transport.emergency_stop_generation += 1
    robot.transport.emergency_stop = Mock(side_effect=emergency_stop)
    robot.telemetry_age = 0.0
    robot.link_generation = 0
    robot.inspection_link_snapshot = lambda: (
        robot.telem, time.monotonic() - robot.telemetry_age, time.monotonic(), robot.link_generation)
    robot.actions = SimpleNamespace(_cancel_event=None)
    robot.actions.set_cancel_event = lambda event: setattr(robot.actions, '_cancel_event', event)
    robot.chassis = SimpleNamespace(turn=Mock(), set_speeds=Mock(return_value=True))
    @contextlib.contextmanager
    def monitor_action(check):
        check()
        try:
            yield
            check()
        finally:
            pass
    robot.chassis.monitor_action = monitor_action
    return robot



class CompositionTests(unittest.TestCase):
    def setUp(self):
        quiet=contextlib.redirect_stdout(io.StringIO())
        quiet.__enter__()
        self.addCleanup(quiet.__exit__,None,None,None)
        self.robot=robot_fixture()
        self.robot.move_chassis=Mock(return_value=SimpleNamespace(timed_out=False,cancelled=False))

    def test_plans_are_flat_actions_and_legacy_selectors_only_translate_names(self):
        for plan in PLANS.values():
            validate_plan(plan,heading_zero_deg=37 if plan.needs_heading_zero else None)
            self.assertTrue(all(isinstance(s,ActionSpec) for s in plan.steps))
            self.assertTrue(all(not hasattr(s,'task_id') for s in plan.steps))
        self.assertEqual(sum(s.kind=='build' for s in PLANS['PlanA'].steps),3)
        self.assertEqual(sum(s.kind=='build' for s in PLANS['PlanB'].steps),2)
        self.assertIs(resolve_selection('task1-1'),PLANS['collect-orange-1'])
        self.assertIs(resolve_selection(),PLANS['PlanA'])
        self.assertIs(resolve_selection('task2-r2'),PLANS['collect-build-2'])

    def test_invalid_plan_rejected_before_hardware_or_diagnostics(self):
        bad=[ActionSpec('missing','bad','ground-1'),
             ActionSpec('navigate','bad','ground-1',{'route':'orange_to_build'}),
             ActionSpec('rebase_heading','bad','ground-1'),
             ActionSpec('load_staged','bad','ground-1'),
             ActionSpec('grab_cube','bad','ground-1',{'method':'grap3'}),
             ActionSpec('navigate','bad','depart-a',{'route':'depart_a','after_build':True}),
             ActionSpec('anchor_wall','bad','ground-1',{'speed_mm_s':-1}),
             ActionSpec('begin_collection','bad','ground-1',{'color':'purple'})]
        for spec in bad:
            with self.subTest(spec=spec),self.assertRaises((TypeError,ValueError)):
                run_plan(self.robot,StrategyPlan('bad',(spec,)))
        self.robot.move_chassis.assert_not_called()
        self.robot.transport.emergency_stop.assert_not_called()
        self.robot.set_collection_context.assert_not_called()

    def test_missing_heading_wrong_anchor_and_nonfinite_heading_reject_before_motion(self):
        with self.assertRaises(ValueError):run_plan(self.robot,PLANS['build-1'])
        for value in (float('nan'),float('inf')):
            with self.assertRaises(ValueError):run_plan(self.robot,PLANS['build-1'],heading_zero_deg=value)
        context=ExecutionContext(self.robot,heading_zero_deg=37,anchor='start')
        with self.assertRaises(ValueError):run_plan(self.robot,PLANS['unload-1'],context=context)
        self.robot.transport.emergency_stop.assert_not_called()

    def test_fault_or_cancellation_between_actions_prevents_later_motion(self):
        for fault in ('stop','disconnect','stale','cancel','restart','reconnect'):
            robot=robot_fixture()
            context=ExecutionContext(robot)
            def move(*args,**kwargs):
                if fault=='stop':robot.transport.emergency_stop()
                if fault=='disconnect':robot.transport.connected=False
                if fault=='stale':robot.telemetry_age=1
                if fault=='cancel':context.cancel_event.set()
                if fault=='restart':robot.telem.uptime_ms=0
                if fault=='reconnect':robot.link_generation+=1
                return SimpleNamespace(timed_out=False,cancelled=False)
            robot.move_chassis=Mock(side_effect=move)
            steps=tuple(ActionSpec('navigate',str(i),'depart-a',{'route':'depart_a'}) for i in (1,2))
            with self.subTest(fault=fault),self.assertRaises(RuntimeError):
                run_plan(robot,StrategyPlan('fault',steps),context=context)
            self.assertEqual(robot.move_chassis.call_count,1)
            self.assertTrue(context.closed)
            self.assertFalse(robot.strategy_lock.locked())

    def test_only_one_robot_owner_and_no_motion_on_conflicting_context(self):
        self.robot.strategy_lock.acquire()
        with self.assertRaises(RuntimeError):run_plan(self.robot,PLANS['depart-a'])
        self.robot.move_chassis.assert_not_called()
        self.assertTrue(self.robot.strategy_lock.locked())
        self.robot.strategy_lock.release()
        with self.assertRaises(ValueError):
            run_plan(self.robot,PLANS['depart-a'],context=ExecutionContext(robot_fixture()))

    def test_compile_exposes_actions_and_build_pair_without_constructing_old_programs(self):
        context=ExecutionContext(self.robot,heading_zero_deg=37,anchor='build_approach')
        env=ActionEnvironment(self.robot,context)
        compiled=env.compile(PLANS['build-2'])
        self.assertEqual(len(compiled.flow.actions),len(PLANS['build-2'].steps))
        self.assertEqual(compiled.preview()[-1]['transition_candidates'],('build_release_to_route',))
        self.assertNotIn('Strategy.competition',sys.modules)
        for i in range(6):self.assertNotIn(f'Strategy.task{i}',sys.modules)
        self.assertNotIn('Strategy.tasks',sys.modules)

    def test_moving_between_acquisition_and_grab_is_rejected_before_motion(self):
        steps=list(PLANS['collect-orange-1'].steps)
        movement=next(s for s in steps if s.name.endswith('.delivery'))
        steps.remove(movement)
        grip=next(i for i,s in enumerate(steps) if s.name.endswith('.grab.3'))
        steps.insert(grip,movement)
        with self.assertRaisesRegex(ValueError,'immediately preceding'):
            run_plan(self.robot,StrategyPlan('bad-order',steps,'ground_area'))
        self.robot.move_chassis.assert_not_called()

    def test_secondary_stop_error_preserves_primary_action_failure_and_restores_detector(self):
        self.robot.move_chassis.side_effect=ValueError('primary motion fault')
        self.robot.transport.emergency_stop.side_effect=OSError('send failed')
        with self.assertRaisesRegex(ValueError,'primary motion fault'):
            run_plan(self.robot,PLANS['depart-a'])
        self.robot.set_cube_detection_profile.assert_called_with('default')
        self.assertFalse(self.robot.strategy_lock.locked())

    def test_build_output_anchor_uses_serial_fallback_when_required_for_route_entry(self):
        from Strategy.execution import ExecutionRuntime
        from unittest.mock import call
        steps=(ActionSpec('build','build','building-1',ends_at='tower'),
               ActionSpec('navigate','depart','building-1',
                          {'route':'build_return','after_build':True},requires_anchor='tower',ends_at='ground_area'))
        plan=StrategyPlan('anchored',steps,'build_approach',180,True)
        context=ExecutionContext(self.robot,heading_zero_deg=37,anchor='build_approach')
        env=ActionEnvironment(self.robot,context)
        events=[]
        self.robot.actions.begin=Mock(return_value=FakeActionSession(events))
        env.run_route=Mock(side_effect=lambda *args:events.append('route'))
        validate_plan(plan,heading_zero_deg=37)
        runtime=ExecutionRuntime(guard=context.check_active,stop=env.stop,
                                 emergency_stop=self.robot.transport.emergency_stop)
        runtime.run(env.compile(plan))
        self.assertLess(events.index('build_done'),events.index('route'))
        self.assertEqual(context.anchor,'ground_area')

    def test_production_sources_do_not_import_removed_task_modules(self):
        import ast
        from pathlib import Path
        root=Path(__file__).resolve().parents[1]
        paths=list((root/'Strategy').rglob('*.py'))+[root/'main.py',root/'task2_main.py']
        for path in paths:
            for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
                if isinstance(node,ast.ImportFrom):
                    module=node.module or ''
                    self.assertNotIn(module, {'competition','tasks',*[f'task{i}' for i in range(6)]},str(path))
                    self.assertFalse(module.startswith(('Strategy.task','Strategy.competition')),str(path))

    def test_cli_preview_and_missing_entry_heading_do_not_construct_robot(self):
        import main
        for args,code in ((['--strategy','PlanA','--show-plan'],0),
                          (['--flow','build-1','--show-plan'],0),
                          (['--flow','build-1'],2),(['--list-flows'],0)):
            with self.subTest(args=args),patch('sys.argv',['main.py',*args]),patch('main.Robot') as robot,contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(main.main(),code)
                robot.assert_not_called()

if __name__=='__main__':unittest.main()
