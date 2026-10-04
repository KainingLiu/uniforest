"""Task2-0 route, bounded pickup interface and faults; no hardware is opened."""

import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from Strategy.competition import SearchRangeExhausted
from Strategy.context import TaskContext
from Strategy.plans import PLANS, StrategyPlan, validate_plan
from Strategy.runner import resolve_selection, run_plan
from Strategy.task2 import Task2_0Config, Task2_0Program, Task2State
from Strategy.tasks import TASK_LIBRARY, TaskDefinition, TaskStep
from main import parse_args
from tests.test_strategy_composition import robot_fixture


class Task2_0Tests(unittest.TestCase):
    def setUp(self):
        quiet = contextlib.redirect_stdout(io.StringIO())
        quiet.__enter__()
        self.addCleanup(quiet.__exit__, None, None, None)

    def fixture(self, count=3):
        robot = robot_fixture()
        events = []
        def move(direction, distance, speed, **kwargs):
            events.append(('move', direction, distance, speed, kwargs['accel_ms']))
            return SimpleNamespace(cancelled=False, timed_out=False)
        robot.move_chassis = Mock(side_effect=move)
        robot.actions.grap1 = Mock(side_effect=lambda: events.append(('grap1',)))
        robot.actions.grap2 = Mock()
        robot.check_carried_cube_count = Mock()
        robot.chassis.capture_motor_positions = Mock(return_value=(10, 20, 30, 40))
        task = Task2_0Program(robot, Task2_0Config(orange_target_count=count),
                              context=TaskContext(robot, current_task='task2-0'))
        task._turn_to_heading = Mock(side_effect=lambda angle: events.append(('turn', angle)))
        task._drive_until_wall = Mock(side_effect=lambda **kw: events.append(
            ('wall', kw['direction'], kw['speed_mm_s'], kw['timeout_s'])))
        task._find_cube = Mock(return_value=object())
        task._align_cube = Mock(return_value=True)
        task._fine_align_orange = Mock(return_value=True)
        task._grab_with_wall_press = Mock(side_effect=lambda grab, **kwargs: grab())
        task._measure_lateral_displacement_mm = Mock(return_value=125)
        return robot, task, events

    def test_default_route_uses_requested_moves_and_only_grap1(self):
        robot, task, events = self.fixture()
        self.assertEqual(task.run(), 0)
        self.assertEqual(events, [
            ('move', 'backward', 2900, 800, 800),
            ('turn', 0), ('move', 'left', 400, 400, 300),
            ('wall', 'left', 300, 2.5), ('wall', 'forward', 300, 2.5),
            ('grap1',), ('grap1',), ('grap1',),
            ('move', 'backward', 100, 400, 300),
            ('move', 'right', 425, 400, 300),
            ('turn', 180), ('move', 'forward', 2750, 800, 800),
        ])
        self.assertEqual(task.completed_grabs, 3)
        self.assertFalse(task.search_exhausted)
        self.assertEqual(task.state, Task2State.FINISHED)
        self.assertEqual(task._heading_zero_deg, robot.telem.yaw_deg)
        self.assertEqual(task.context.build_approach.heading_zero_deg, robot.telem.yaw_deg)
        self.assertEqual(task.context.build_approach.source_task, 'task2-0')
        task._measure_lateral_displacement_mm.assert_called_once_with((10, 20, 30, 40))
        robot.actions.grap2.assert_not_called()
        robot.check_carried_cube_count.assert_not_called()
        robot.transport.emergency_stop.assert_not_called()
        robot.set_cube_detection_profile.assert_has_calls([call('task2_orange'), call('default')])
        task._grab_with_wall_press.assert_has_calls(
            [call(robot.actions.grap1, recalibrate_heading_zero=True)] * 3)

    def test_requested_pickups_share_one_budget_without_filling_to_three(self):
        for count in (1, 2, 3):
            with self.subTest(count=count):
                robot, task, _ = self.fixture(count)
                recoveries = []
                def search(**kwargs):
                    recoveries.append(task._orange_recovery)
                    task._search_position_mm += 100
                    task._orange_recovery.search_elapsed_s += .5
                    return object()
                task._find_cube.side_effect = search
                self.assertEqual(task.run(), 0)
                self.assertEqual(robot.actions.grap1.call_count, count)
                self.assertEqual(task.completed_grabs, count)
                self.assertEqual(task._search_position_mm, count * 100)
                self.assertEqual(task._orange_recovery.search_elapsed_s, count * .5)
                self.assertTrue(all(r is recoveries[0] for r in recoveries))
                self.assertEqual(task._orange_recovery.origin, (10, 20, 30, 40))
                robot.check_carried_cube_count.assert_not_called()

    def test_exhausted_vision_finishes_with_partial_or_zero_actions(self):
        for done in (0, 1):
            robot, task, events = self.fixture()
            task._find_cube.side_effect = [object()] * done + [SearchRangeExhausted()]
            self.assertEqual(task.run(), 0)
            self.assertEqual(task.completed_grabs, done)
            self.assertTrue(task.search_exhausted)
            self.assertEqual(robot.actions.grap1.call_count, done)
            robot.transport.emergency_stop.assert_not_called()
            robot.set_cube_detection_profile.assert_called_with('default')
            self.assertEqual(events[-4:], [
                ('move', 'backward', 100, 400, 300),
                ('move', 'right', 425, 400, 300),
                ('turn', 180), ('move', 'forward', 2750, 800, 800),
            ])

    def test_faults_still_abort_without_further_pickups(self):
        for fault in ('action', 'cancel', 'disconnect', 'stale', 'stop'):
            with self.subTest(fault=fault):
                robot, task, events = self.fixture()
                def fail(**kwargs):
                    if fault == 'action':
                        raise RuntimeError('mechanism failed')
                    if fault == 'cancel':
                        task.context.cancel_event.set()
                    elif fault == 'disconnect':
                        robot.transport.connected = False
                    elif fault == 'stale':
                        robot.telemetry_age = 1
                    elif fault == 'stop':
                        robot.transport.emergency_stop_generation += 1
                    raise SearchRangeExhausted()
                task._find_cube.side_effect = fail
                with self.assertRaises(RuntimeError):
                    task.run()
                self.assertEqual(task.state, Task2State.FAULT)
                robot.transport.emergency_stop.assert_called_once()
                robot.actions.grap1.assert_not_called()
                robot.set_cube_detection_profile.assert_called_with('default')
                self.assertEqual(len([e for e in events if e[0] == 'move']), 2)
                task._measure_lateral_displacement_mm.assert_not_called()

    def test_entry_uses_existing_180_degree_heading_convention(self):
        _, task, _ = self.fixture()
        task._preflight()
        self.assertEqual(task.config.initial_heading_cw_deg, 180)
        self.assertEqual(task._heading_zero_deg, 37)
        self.assertEqual(task.config.delivery_turn_speed_deg_s, 120)

    def test_count_validation_happens_before_any_plan_step(self):
        for count in (0, 4, -1, True, False, 1.5, '2', None):
            with self.subTest(count=count):
                robot = robot_fixture()
                with self.assertRaises(ValueError):
                    run_plan(robot, StrategyPlan('invalid count', (
                        TaskStep('task0-1'), TaskStep('task2-0', {'orange_target_count': count}))))
                robot.set_collection_context.assert_not_called()
                robot.transport.emergency_stop.assert_not_called()

    def test_registration_parameters_and_handoff_are_isolated(self):
        step = TaskStep('task2-0', {'orange_target_count': 1})
        cfg = TASK_LIBRARY['task2-0'].configuration(step)
        self.assertEqual(cfg.orange_target_count, 1)
        self.assertEqual(Task2_0Config().orange_target_count, 3)
        self.assertEqual(parse_args(['--task', 'task2-0']).task, 'task2-0')
        self.assertEqual(resolve_selection('task2-0').steps, (TaskStep('task2-0'),))
        with self.assertRaises(ValueError):
            resolve_selection('task2-3')
        for next_task in ('task3-1', 'task4-1'):
            validate_plan(StrategyPlan('shared exit route', (step, TaskStep(next_task))))
        self.assertTrue(all(s.task_id != 'task2-0' for p in PLANS.values() for s in p.steps))

    def test_executor_continues_after_search_exhaustion(self):
        robot = robot_fixture()
        events = []
        def exhausted_route(task):
            # Exercise the real task with only physical operations replaced.
            task._checked_move = Mock()
            task._turn_to_heading = Mock()
            task._drive_until_wall = Mock()
            task._capture_lateral_origin = Mock(return_value=(0, 0, 0, 0))
            task._measure_lateral_displacement_mm = Mock(return_value=125)
            task._grab_task2_orange = Mock(side_effect=SearchRangeExhausted())
            original_route(task)
            events.append('vision fallback')
        original_route = Task2_0Program._run_collection_route
        from Strategy.task0 import Task0_1Program
        with patch.object(Task2_0Program, '_run_collection_route', exhausted_route), \
             patch.object(Task0_1Program, 'run', side_effect=lambda: events.append('next task') or 0):
            self.assertEqual(run_plan(robot, StrategyPlan('continue', (
                TaskStep('task2-0'), TaskStep('task0-1')))), 0)
        self.assertEqual(events, ['vision fallback', 'next task'])
        robot.transport.emergency_stop.assert_not_called()

    def test_exit_compensation_measures_before_reverse_and_uses_both_speed_bands(self):
        for net_right in (50, 51, 550, 1049, 1050):
            with self.subTest(net_right=net_right):
                robot, task, events = self.fixture(1)
                def measure(origin):
                    self.assertEqual(origin, (10, 20, 30, 40))
                    # Only the approach moves have run when the encoder offset is read.
                    self.assertEqual(robot.move_chassis.call_count, 2)
                    return net_right
                task._measure_lateral_displacement_mm.side_effect = measure
                self.assertEqual(task.run(), 0)
                expected = [('move', 'backward', 100, 400, 300)]
                distance = abs(550 - net_right)
                if distance:
                    direction = 'right' if net_right < 550 else 'left'
                    speed, accel = (800, 800) if distance >= 500 else (400, 300)
                    expected.append(('move', direction, distance, speed, accel))
                expected.extend([('turn', 180), ('move', 'forward', 2750, 800, 800)])
                self.assertEqual(events[6:], expected)

    def test_executor_delivers_heading_to_following_task3_or_task4(self):
        from Strategy.task3 import Task3Program
        from Strategy.task4 import Task4Program
        for next_id, next_type in (('task3-1', Task3Program), ('task4-1', Task4Program)):
            with self.subTest(next_id=next_id):
                robot, task, _ = self.fixture(1)
                observed = []
                def consume(program):
                    handoff = program.context.take_build_approach()
                    observed.append((handoff.heading_zero_deg, handoff.source_task))
                    return 0
                library = dict(TASK_LIBRARY)
                library['task2-0'] = TaskDefinition(
                    lambda robot, config, *, context: task,
                    Task2_0Config, provides='build_approach')
                with patch.object(next_type, 'run', consume):
                    self.assertEqual(run_plan(robot, StrategyPlan('handoff', (
                        TaskStep('task2-0', {'orange_target_count': 1}), TaskStep(next_id))),
                        context=task.context, task_library=library), 0)
                self.assertEqual(observed, [(robot.telem.yaw_deg, 'task2-0')])


if __name__ == '__main__':
    unittest.main()
