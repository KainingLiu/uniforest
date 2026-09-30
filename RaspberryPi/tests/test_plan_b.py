"""PlanB route order, 180-degree handoff and interruption regression tests."""

import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from Strategy.competition import SearchRangeExhausted
from Strategy.common import VisualAlignmentUnavailable
from Strategy.context import BuildApproach, TaskContext
from Strategy.plans import PLANS, StrategyPlan
from Strategy.runner import resolve_selection, run_plan
from Strategy.tasks import TASK_LIBRARY, TaskStep
from Strategy.task0 import Task0_1Program, Task0_2Program, Task0_3Program
from Strategy.task2 import Task2Program, Task2_2Program
from Strategy.task4 import Task4Program, Task4_2Program
from Strategy.task5 import Task5Program
from tests.test_strategy_composition import robot_fixture


class PlanBTests(unittest.TestCase):
    def setUp(self):
        output = contextlib.redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)

    def test_plan_b_reuses_task2_and_has_exact_sequence(self):
        expected = ['task0-2', 'task2-1', 'task4-1', 'task2-2', 'task4-2',
                    'task0-3', 'task1-1', 'task0-3', 'task1-2', 'task5']
        self.assertEqual([step.task_id for step in PLANS['PlanB'].steps], expected)
        self.assertIs(resolve_selection('planb'), PLANS['PlanB'])
        self.assertIs(TASK_LIBRARY['task2-1'].program_type, Task2Program)
        self.assertIs(TASK_LIBRARY['task2-2'].program_type, Task2_2Program)
        self.assertIs(TASK_LIBRARY['task0-3'].program_type, Task0_3Program)
        self.assertIs(TASK_LIBRARY['task5'].program_type, Task5Program)
        from agent.tools import RobotToolExecutor
        self.assertEqual(RobotToolExecutor(dry_run=True).run_strategy('PlanB').value['tasks'], expected)

    def test_task0_routes_use_requested_distances_speed_and_acceleration(self):
        for task_type, legs in ((Task0_1Program, [('forward', 1200)]),
                               (Task0_2Program, [('forward', 900), ('right', 2700)])):
            with self.subTest(task=task_type.TASK_LABEL):
                robot = robot_fixture()
                robot.move_chassis = Mock(return_value=SimpleNamespace(timed_out=False, cancelled=False))
                self.assertEqual(task_type(robot, context=TaskContext(robot)).run(), 0)
                self.assertEqual(robot.move_chassis.call_args_list, [
                    call(direction, distance, 1000, hold_ms=0, accel_ms=800, route_mode=True)
                    for direction, distance in legs])
                if task_type is Task0_2Program:
                    robot.chassis.turn.assert_called_once_with(180, 120, hold_ms=0, settle_cycles=1)
                else:
                    robot.chassis.turn.assert_not_called()

    def test_task0_2_corrects_final_heading_in_startup_frame_and_stops_on_fault(self):
        for fault in (None, 'stop', 'stale'):
            with self.subTest(fault=fault):
                robot = robot_fixture()
                robot.telem.yaw_deg = 37
                def move(direction, *args, **kwargs):
                    if direction == 'right':
                        robot.telem.yaw_deg = 35  # Two degrees of travel drift.
                        if fault == 'stop': robot.transport.emergency_stop()
                        if fault == 'stale': robot.telemetry_age = 1
                    return SimpleNamespace(timed_out=False, cancelled=False)
                robot.move_chassis = Mock(side_effect=move)
                task = Task0_2Program(robot, context=TaskContext(robot))
                if fault:
                    with self.assertRaises(RuntimeError): task.run()
                    robot.chassis.turn.assert_not_called()
                else:
                    self.assertEqual(task.run(), 0)
                    robot.chassis.turn.assert_called_once_with(178, 120, hold_ms=0, settle_cycles=1)

    def test_task0_failure_or_stop_after_forward_prevents_right_move(self):
        for fault in ('timeout', 'cancel', 'stop', 'disconnect', 'stale'):
            with self.subTest(fault=fault):
                robot = robot_fixture()
                def forward(*args, **kwargs):
                    if fault == 'stop': robot.transport.emergency_stop()
                    if fault == 'disconnect': robot.transport.connected = False
                    if fault == 'stale': robot.telemetry_age = 1
                    return SimpleNamespace(timed_out=fault == 'timeout', cancelled=fault == 'cancel')
                robot.move_chassis = Mock(side_effect=forward)
                with self.assertRaises(RuntimeError):
                    Task0_2Program(robot, context=TaskContext(robot)).run()
                self.assertEqual(robot.move_chassis.call_count, 1)
                robot.chassis.turn.assert_not_called()

    def test_task0_3_turns_from_180_then_moves_left_and_approaches_wall(self):
        robot = robot_fixture()
        task = Task0_3Program(robot, context=TaskContext(robot))
        events = Mock()
        events.attach_mock(robot.chassis.turn, 'turn')
        robot.move_chassis = Mock(return_value=SimpleNamespace(timed_out=False, cancelled=False))
        events.attach_mock(robot.move_chassis, 'move')
        task._drive_until_wall = Mock()
        events.attach_mock(task._drive_until_wall, 'wall')
        self.assertEqual(task.run(), 0)
        self.assertEqual(task._heading_zero_deg, 37)
        self.assertEqual(events.mock_calls, [
            call.turn(180, 120, hold_ms=0, settle_cycles=1),
            call.move('left', 2600, 1000, hold_ms=0, accel_ms=800, route_mode=True),
            call.wall(timeout_s=4, speed_mm_s=300, direction='left',
                      context='Task0-3 left wall approach')])
        robot.transport.emergency_stop.assert_not_called()

    def test_task0_3_stops_before_next_motion_on_stop_disconnect_or_stale_telemetry(self):
        for stage in ('turn', 'move'):
            for fault in ('stop', 'disconnect', 'stale'):
                with self.subTest(stage=stage, fault=fault):
                    robot = robot_fixture()
                    task = Task0_3Program(robot, context=TaskContext(robot))
                    def fail(*args, **kwargs):
                        if fault == 'stop': robot.transport.emergency_stop()
                        if fault == 'disconnect': robot.transport.connected = False
                        if fault == 'stale': robot.telemetry_age = 1
                        return SimpleNamespace(timed_out=False, cancelled=False)
                    robot.move_chassis = Mock(return_value=SimpleNamespace(timed_out=False, cancelled=False))
                    if stage == 'turn': robot.chassis.turn.side_effect = fail
                    else: robot.move_chassis.side_effect = fail
                    task._drive_until_wall = Mock()
                    with self.assertRaises(RuntimeError): task.run()
                    self.assertEqual(robot.move_chassis.call_count, int(stage == 'move'))
                    task._drive_until_wall.assert_not_called()

    def task4_fixture(self, task_type=Task4Program):
        robot = robot_fixture()
        events = []
        context = TaskContext(robot, build_approach=BuildApproach(37, 'task2-test'))
        task = task_type(robot, context=context)
        def move(direction, distance, speed, **kwargs):
            events.append(('move', direction, distance, speed, kwargs))
            return SimpleNamespace(timed_out=False, cancelled=False)
        robot.move_chassis = Mock(side_effect=move)
        robot.actions.hatch_open = Mock(side_effect=lambda **kw: events.append(('open', kw)))
        robot.actions.hatch_close = Mock(side_effect=lambda **kw: events.append(('close', kw)))
        robot.chassis.turn = Mock(side_effect=lambda *a, **kw: events.append(('turn', a, kw)))
        def wall(**kwargs):
            task._check_active()
            events.append(('wall', kwargs['direction'], kwargs['speed_mm_s'], kwargs['timeout_s']))
        task._drive_until_wall = Mock(side_effect=wall)
        return robot, context, task, events

    def test_task4_routes_unload_timing_and_finish_without_turning_from_180(self):
        for task_type, right_mm, final_mm in ((Task4Program, 0, 800), (Task4_2Program, 300, 500)):
            with self.subTest(task=task_type.TASK_LABEL):
                robot, context, task, events = self.task4_fixture(task_type)
                self.assertEqual(task.run(), 0)
                move_kw = dict(hold_ms=0, accel_ms=300, route_mode=True)
                long_kw = dict(hold_ms=0, accel_ms=800, route_mode=True)
                expected = [('move', 'left', 600, 1000, long_kw), ('wall', 'left', 300, 4)]
                if right_mm:
                    expected.append(('move', 'right', right_mm, 400, move_kw))
                expected += [('wall', 'forward', 300, 4), ('open', {'settle_ms': 300}),
                             ('move', 'backward', 300, 400, move_kw), ('close', {'settle_ms': 0}),
                             ('move', 'right', final_mm, 1000, long_kw)]
                self.assertEqual(events, expected)
                # yaw=-143 and zero=37 mean an entry heading of 180 degrees.
                robot.chassis.turn.assert_not_called()
                self.assertEqual(task._heading_error(180), 0)
                self.assertEqual(task._heading_zero_deg, 37)
                self.assertIsNone(context.build_approach)
                robot.transport.emergency_stop.assert_not_called()

    def test_task4_does_not_continue_after_hatch_or_motion_fault(self):
        for fault in ('open-stop', 'open-disconnect', 'reverse-stale', 'reverse-failed', 'close-stop'):
            with self.subTest(fault=fault):
                robot, context, task, events = self.task4_fixture()
                if fault.startswith('open-'):
                    def open_hatch(**kwargs):
                        if fault == 'open-stop': robot.transport.emergency_stop()
                        else: robot.transport.connected = False
                    robot.actions.hatch_open.side_effect = open_hatch
                if fault.startswith('reverse-'):
                    original = robot.move_chassis.side_effect
                    def move(direction, *args, **kwargs):
                        result = original(direction, *args, **kwargs)
                        if direction == 'backward':
                            if fault == 'reverse-stale': robot.telemetry_age = 1
                            else: result.cancelled = True
                        return result
                    robot.move_chassis.side_effect = move
                if fault == 'close-stop':
                    robot.actions.hatch_close.side_effect = lambda **kw: robot.transport.emergency_stop()
                with self.assertRaises(RuntimeError): task.run()
                self.assertFalse(any(event[:3] == ('move', 'right', 800) for event in events))
                robot.chassis.turn.assert_not_called()
                if fault != 'close-stop': robot.actions.hatch_close.assert_not_called()

    def test_invalid_or_consumed_task4_handoff_rejected_before_motion(self):
        for steps in (('task4-1',), ('task2-1', 'task4-1', 'task4-2'),
                      ('task2-1', 'task0-1', 'task4-1')):
            robot = robot_fixture()
            with self.assertRaises(ValueError):
                run_plan(robot, StrategyPlan('invalid', tuple(map(TaskStep, steps))))
            robot.set_collection_context.assert_not_called()
        robot, context, task, events = self.task4_fixture()
        task._preflight()
        with self.assertRaisesRegex(RuntimeError, 'handoff'):
            Task4_2Program(robot, context=context)._preflight()
        self.assertEqual(events, [])
        for value in (float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                run_plan(robot, resolve_selection('task4-1'), heading_zero_deg=value)

    def test_full_plan_b_continues_without_vision_and_builds_twice_in_task5(self):
        robot = robot_fixture()
        robot.has_vision = robot.has_field_localization = False
        robot.move_chassis = Mock(return_value=SimpleNamespace(timed_out=False, cancelled=False))
        robot.check_carried_cube_count = Mock(return_value=None)
        robot.actions.hatch_open, robot.actions.hatch_close = Mock(), Mock()
        robot.actions.build = Mock(side_effect=lambda **kw: kw['chassis_followup'](lambda: None))
        zeros = []
        def recalibrate(task, reference_cw_deg=0):
            task._heading_zero_deg = 37.0
        original_preflight = Task4Program._preflight
        def record_entry(task):
            original_preflight(task)
            zeros.append((task.TASK_LABEL, task._heading_zero_deg))
        with patch('Strategy.competition.TaskControl._drive_until_wall'), \
             patch('Strategy.competition.TaskControl._turn_to_heading'), \
             patch.object(Task4Program, '_preflight', record_entry), \
             patch('Strategy.competition.TaskControl._capture_lateral_origin', return_value=0), \
             patch('Strategy.competition.TaskControl._measure_lateral_displacement_mm', return_value=0), \
             patch('Strategy.competition.TaskControl._recalibrate_heading_zero', recalibrate), \
             patch('Strategy.competition.TaskControl._align_delivery_tag', side_effect=VisualAlignmentUnavailable('no tag')), \
             patch('Strategy.competition.TaskControl._chassis_followup', side_effect=lambda route: lambda check: route()), \
             patch('Strategy.task3.Task3Program._align_building', side_effect=VisualAlignmentUnavailable('no building')), \
             patch('Strategy.competition.TaskControl._find_cube', side_effect=SearchRangeExhausted()):
            self.assertEqual(run_plan(robot, PLANS['PlanB']), 0)
        self.assertEqual(zeros, [('task4-1', 37), ('task4-2', 37)])
        self.assertEqual(robot.actions.hatch_open.call_args_list,
                         [call(settle_ms=300)] * 4 + [call(settle_ms=200)] * 2)
        self.assertEqual(robot.actions.hatch_close.call_args_list,
                         [call(settle_ms=0)] * 4 + [call(settle_ms=400)] * 2)
        completed = [c.kwargs['task'] for c in robot.diagnostics.write.call_args_list
                     if c.args == ('task_complete',)]
        self.assertEqual(completed, [s.task_id for s in PLANS['PlanB'].steps])
        self.assertEqual(robot.actions.build.call_count, 2)
        robot.transport.emergency_stop.assert_not_called()

    def test_new_cli_previews_and_bad_task4_entries_never_connect(self):
        import main
        for args, expected in ((['--strategy', 'PlanB', '--show-plan'], 0),
                               (['--task', 'task0-3', '--show-plan'], 0),
                               (['--task', 'task5', '--show-plan'], 0),
                               (['--task', 'task4-2', '--show-plan'], 0),
                               (['--task', 'task4-1'], 2),
                               (['--task', 'task4-2', '--heading-zero-deg', 'nan'], 2)):
            with self.subTest(args=args), patch('sys.argv', ['main.py', *args]), \
                 patch('main.Robot') as robot, contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(main.main(), expected)
                robot.assert_not_called()
        from tools.install_desktop_entries import ENTRIES
        self.assertEqual(len(ENTRIES), 2)
        self.assertEqual(ENTRIES['uniforest-all.desktop'][0], 'PlanA')
        self.assertNotIn('uniforest-task0.desktop', ENTRIES)
        self.assertIn('PlanB', [selection for selection, _ in ENTRIES.values()])


if __name__ == '__main__':
    unittest.main()
