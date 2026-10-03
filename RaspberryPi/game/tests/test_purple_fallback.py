"""Missing purple is a collection outcome; hardware faults still abort."""

import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from protocol.commands import ACTION_BUILD, ACTION_GRAP1, ACTION_GRAP2
from Strategy.errors import SearchRangeExhausted
from Strategy.execution import ExecutionRuntime
from Strategy.flows.factory import ActionEnvironment
from Strategy.flows import operations
from Strategy.plans import PLANS
from tests.test_functional_operations import operation_fixture, spec


class PurpleFallbackTests(unittest.TestCase):
    def setUp(self):
        output = contextlib.redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)

    def environment(self, plan_name, purple_present=False, purple_fault=None, count=3):
        fixture, _ = operation_fixture()
        robot, context = fixture.robot, fixture.context
        context.anchor = PLANS[plan_name].entry_anchor
        robot.actions.pickup_full_lift_validated = False
        robot.actions.begin = Mock(side_effect=lambda action_id: SimpleNamespace(
            done=True, wait_chassis_ready=Mock(), wait_done=Mock(),
            check=Mock(), close=Mock(), abort=Mock()))
        robot.begin_carried_cube_inspection = Mock(side_effect=lambda **kw: SimpleNamespace(
            count=count, inspect=Mock(return_value=count), finish_restore=Mock(),
            check_restore=Mock(), close=Mock(), abort=Mock()))
        env = ActionEnvironment(robot, context)
        env.run_route = Mock()
        env._align_tag = Mock(return_value=True)

        def find(**kwargs):
            if kwargs['color_name'] == 'purple':
                if purple_fault:
                    raise purple_fault
                if not purple_present:
                    raise SearchRangeExhausted('purple area empty')
            return SimpleNamespace(x=0, z=150)

        for profile in {step.profile for step in PLANS[plan_name].steps}:
            control = env.control(profile)
            control._find_cube = Mock(side_effect=find)
            for name in ('_align_cube', '_align_orange', '_fine_align_orange',
                         '_align_building', '_align_building_or_continue', '_drive_until_wall',
                         '_checked_move'):
                setattr(control, name, Mock(return_value=True))
            control._grab_press_step = Mock(return_value=lambda: True)
        runtime = ExecutionRuntime(guard=context.check_active, stop=env.stop,
            emergency_stop=robot.transport.emergency_stop, close=env.abort)
        return env, runtime

    def test_both_plans_complete_with_three_orange_when_purple_is_absent(self):
        for plan_name, rounds in (('PlanA', 3), ('PlanB', 2)):
            for purple_present in (False, True):
                with self.subTest(plan=plan_name, purple_present=purple_present):
                    env, runtime = self.environment(plan_name, purple_present)
                    runtime.run(env.compile(PLANS[plan_name]))
                    actions = [call.args[0] for call in env.robot.actions.begin.call_args_list]
                    self.assertEqual(actions.count(ACTION_GRAP2), rounds if purple_present else 0)
                    self.assertEqual(actions.count(ACTION_GRAP1), rounds * (2 if purple_present else 3))
                    self.assertEqual(actions.count(ACTION_BUILD), rounds)
                    self.assertEqual(env.data['carried_count'], 3)
                    self.assertEqual(
                        [call.args for call in env.run_route.call_args_list],
                        [(step.parameters['route'], step.profile)
                         for step in PLANS[plan_name].steps if step.kind == 'navigate'])
                    env.robot.transport.emergency_stop.assert_not_called()

    def test_purple_hardware_fault_aborts_without_orange_substitution(self):
        env, runtime = self.environment('PlanB', purple_fault=RuntimeError('telemetry lost'))
        with self.assertRaisesRegex(RuntimeError, 'telemetry lost'):
            runtime.run(env.compile(PLANS['PlanB']))
        env.robot.actions.begin.assert_not_called()
        env.robot.transport.emergency_stop.assert_called_once()

    def test_alignment_failure_does_not_claim_purple_is_absent(self):
        env, control = operation_fixture('highland-1')
        control._find_cube = Mock(return_value=object())
        operations.begin_collection(env, spec('begin_collection', 'highland-1', color='purple'))
        alignment = SimpleNamespace(max_attempts=2)
        with patch.object(operations, '_fast_align', return_value=False) as align:
            with self.assertRaisesRegex(RuntimeError, 'absence not confirmed'):
                operations._acquire_purple(env, control, alignment)
            self.assertEqual(align.call_count, 2)
        self.assertFalse(env.data['purple_grabbed'])
        with patch.object(operations, '_fast_align', side_effect=RuntimeError('cancelled')):
            with self.assertRaisesRegex(RuntimeError, 'cancelled'):
                operations._acquire_purple(env, control, alignment)

    def test_pure_orange_build_keeps_full_cargo_requirement(self):
        for count in (None, 0, 1, 2, 3):
            with self.subTest(count=count):
                env, _ = self.environment('PlanA')
                env.data.update(purple_grabbed=False, carried_count=count)
                build = next(s for s in PLANS['PlanA'].steps if s.kind == 'build')
                if count == 3:
                    env._start_build(build)
                    env.robot.actions.begin.assert_called_once_with(ACTION_BUILD)
                else:
                    with self.assertRaisesRegex(RuntimeError, 'count must be confirmed'):
                        env._start_build(build)
                    env.robot.actions.begin.assert_not_called()


if __name__ == '__main__':
    unittest.main()
