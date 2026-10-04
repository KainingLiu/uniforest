"""Task1-0 pickup selection and stop-before-Tag6 boundary; no hardware."""

import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from Strategy.competition import SearchRangeExhausted
from Strategy.context import TaskContext
from Strategy.plans import PLANS, StrategyPlan, validate_plan
from Strategy.runner import resolve_selection, run_plan
from Strategy.task1 import Task1Config, Task1Program, Task1_0Config, Task1_0Program
from Strategy.tasks import TASK_LIBRARY, TaskStep
from main import parse_args
from tests.test_strategy_composition import robot_fixture


class Task1_0Tests(unittest.TestCase):
    def setUp(self):
        quiet = contextlib.redirect_stdout(io.StringIO())
        quiet.__enter__()
        self.addCleanup(quiet.__exit__, None, None, None)

    def fixture(self, count=3, *, exit_count=2):
        robot = robot_fixture()
        events = []
        def move(direction, distance, speed, **kwargs):
            events.append(('move', direction, distance, speed, kwargs['accel_ms']))
            return SimpleNamespace(timed_out=False, cancelled=False)
        robot.move_chassis = Mock(side_effect=move)
        robot.actions.grap3 = Mock(side_effect=lambda: events.append(('grap3',)))
        def inspect(**kwargs):
            self.assertTrue(kwargs['inspect_while_moving'])
            self.assertTrue(kwargs['allow_visual_failure'])
            self.assertIsNone(task.context.build_approach)
            kwargs['chassis_followup']()
            # Do not publish until both the route and inspection restore return.
            self.assertIsNone(task.context.build_approach)
            return exit_count
        robot.check_carried_cube_count = Mock(side_effect=inspect)
        robot.actions.hatch_open = Mock()
        robot.actions.hatch_close = Mock()
        robot.chassis.capture_motor_positions = Mock(return_value=(10, 20, 30, 40))
        task = Task1_0Program(robot, Task1_0Config(target_cube_count=count),
                              context=TaskContext(robot, current_task='task1-0'))
        task._drive_until_wall = Mock(side_effect=lambda **kw: events.append(('wall',)))
        task._find_orange = Mock(return_value=object())
        task._align_orange = Mock(return_value=True)
        task._grab_with_wall_press = Mock(side_effect=lambda grab, **kw: grab())
        task._turn_to_heading = Mock(side_effect=lambda angle, **kw: events.append(('turn', angle)))
        task._align_delivery_tag_or_continue = Mock()
        def measure(origin):
            self.assertEqual(origin, (10, 20, 30, 40))
            self.assertEqual(robot.move_chassis.call_count, 0)
            return 125
        task._measure_lateral_displacement_mm = Mock(side_effect=measure)
        return robot, task, events

    def test_requested_count_stops_before_tag6_without_inspection_or_unloading(self):
        for count in (1, 2, 3):
            with self.subTest(count=count):
                robot, task, events = self.fixture(count)
                self.assertEqual(task.run(), 0)
                self.assertEqual(events, [('wall',)] + [('grap3',)] * count + [
                    ('move', 'backward', 400, 400, 300), ('turn', 90),
                    ('move', 'forward', 2675, 1000, 800), ('turn', 180)])
                self.assertEqual(task.completed_grabs, count)
                self.assertFalse(task.search_exhausted)
                task._align_delivery_tag_or_continue.assert_not_called()
                robot.reset_field_localization_filter.assert_not_called()
                robot.check_carried_cube_count.assert_not_called()
                robot.actions.hatch_open.assert_not_called()
                robot.actions.hatch_close.assert_not_called()
                robot.transport.emergency_stop.assert_not_called()
                self.assertEqual(task.context.build_approach.source_task, 'task1-0')
                self.assertEqual(task.context.build_approach.heading_zero_deg, task._heading_zero_deg)
                self.assertIsNone(task.context.build_approach.carried_cube_count)
                self.assertEqual(task.config.far_wall_speed_mm_s, 300)
                self.assertEqual(task.config.delivery_turn_speed_deg_s, 120)

    def test_search_origin_and_budget_are_shared_across_requested_pickups(self):
        _, task, _ = self.fixture()
        states = []
        def search():
            states.append(task._orange_recovery)
            task._search_position_mm += 50
            task._orange_recovery.search_elapsed_s += .25
            return object()
        task._find_orange.side_effect = search
        self.assertEqual(task.run(), 0)
        self.assertEqual(task._search_position_mm, 150)
        self.assertEqual(task._orange_recovery.search_elapsed_s, .75)
        self.assertTrue(all(state is states[0] for state in states))
        self.assertEqual(task._orange_recovery.origin, (10, 20, 30, 40))

    def test_search_exhaustion_continues_to_tag6_approach_without_alignment(self):
        for completed in (0, 1):
            robot, task, events = self.fixture()
            task._find_orange.side_effect = [object()] * completed + [SearchRangeExhausted()]
            self.assertEqual(task.run(), 0)
            self.assertTrue(task.search_exhausted)
            self.assertEqual(task.completed_grabs, completed)
            self.assertEqual(events[-4:], [
                ('move', 'backward', 400, 400, 300), ('turn', 90),
                ('move', 'forward', 2675, 1000, 800), ('turn', 180)])
            robot.transport.emergency_stop.assert_not_called()
            task._align_delivery_tag_or_continue.assert_not_called()
            robot.check_carried_cube_count.assert_called_once()
            self.assertEqual(task.context.build_approach.carried_cube_count, 2)
            self.assertEqual(task.refill_missing_count, 0)

    def test_exhausted_exit_publishes_measured_count_after_route_and_restore(self):
        for count in (0, 1, 2, 3, None):
            with self.subTest(count=count):
                robot, task, events = self.fixture(exit_count=count)
                task._find_orange.side_effect = SearchRangeExhausted()
                self.assertEqual(task.run(), 0)
                self.assertEqual(task.completed_grabs, 0)  # count comes from vision, not grabs
                self.assertEqual(task.context.build_approach.carried_cube_count,
                                 2 if count is None else count)
                self.assertEqual(task.refill_missing_count, 0)
                self.assertEqual(events.count(('move', 'backward', 400, 400, 300)), 1)
                robot.actions.hatch_open.assert_not_called()
                robot.actions.hatch_close.assert_not_called()
                task._align_delivery_tag_or_continue.assert_not_called()
                robot.transport.emergency_stop.assert_not_called()

    def test_failed_refill_inspection_never_publishes_a_build_handoff(self):
        for failure in ('link', 'inspection', 'invalid_count'):
            with self.subTest(failure=failure):
                robot, task, _ = self.fixture(exit_count=4)
                task._find_orange.side_effect = SearchRangeExhausted()
                if failure == 'link':
                    def fail(**kwargs):
                        robot.transport.connected = False
                        return None
                    robot.check_carried_cube_count.side_effect = fail
                elif failure == 'inspection':
                    robot.check_carried_cube_count.side_effect = RuntimeError('servo fault')
                with self.assertRaises(RuntimeError):
                    task.run()
                self.assertIsNone(task.context.build_approach)
                robot.transport.emergency_stop.assert_called_once()

    def test_faults_never_continue_into_delivery(self):
        for fault in ('action', 'cancel', 'disconnect', 'stale', 'stop'):
            with self.subTest(fault=fault):
                robot, task, _ = self.fixture()
                def fail():
                    if fault == 'action':
                        raise RuntimeError('Grap3 mechanism fault')
                    if fault == 'cancel':
                        task.context.cancel_event.set()
                    elif fault == 'disconnect':
                        robot.transport.connected = False
                    elif fault == 'stale':
                        robot.telemetry_age = 1
                    else:
                        robot.transport.emergency_stop_generation += 1
                    raise SearchRangeExhausted()
                task._find_orange.side_effect = fail
                with self.assertRaises(RuntimeError):
                    task.run()
                robot.move_chassis.assert_not_called()
                task._turn_to_heading.assert_not_called()
                robot.transport.emergency_stop.assert_called_once()

    def test_invalid_quantity_fails_before_any_plan_motion(self):
        for count in (0, 4, -1, True, False, 2.5, '2', None):
            robot = robot_fixture()
            with self.subTest(count=count), self.assertRaises(ValueError):
                run_plan(robot, StrategyPlan('bad count', (
                    TaskStep('task0-1'), TaskStep('task1-0', {'target_cube_count': count}))))
            robot.set_collection_context.assert_not_called()
            robot.transport.emergency_stop.assert_not_called()

    def test_selectors_quantity_and_task_boundaries(self):
        self.assertEqual(Task1_0Config().target_cube_count, 3)
        self.assertEqual(Task1Config().target_cube_count, 3)
        step = TaskStep('task1-0', {'target_cube_count': 2})
        self.assertEqual(TASK_LIBRARY['task1-0'].configuration(step).target_cube_count, 2)
        self.assertEqual(parse_args(['--task', 'task1-0']).task, 'task1-0')
        self.assertEqual(resolve_selection('task1-0').steps, (TaskStep('task1-0'),))
        validate_plan(StrategyPlan('refill build entry', (step, TaskStep('task3-1'))))
        validate_plan(StrategyPlan('two pickups', (step, TaskStep('task2-0'), TaskStep('task3-1'))))
        self.assertTrue(all(s.task_id != 'task1-0' for p in PLANS.values() for s in p.steps))

    def test_original_task1_count_callback_still_reverses_exactly_once(self):
        for overlap in (True, False):
            robot = robot_fixture()
            task = Task1Program(robot, context=TaskContext(robot))
            moves = []
            task._drive_until_wall = Mock()
            task._capture_lateral_origin = Mock(return_value='origin')
            task._measure_lateral_displacement_mm = Mock(return_value=125)
            task._checked_move = lambda direction, distance, speed: moves.append((direction, distance))
            task._turn_to_heading = Mock()
            def collect(count, grab, *, chassis_followup):
                self.assertEqual(count, 3)
                if overlap:
                    chassis_followup()
            task._collect_orange_with_count_check = Mock(side_effect=collect)
            task._preflight()
            reverse_done = task._run_first_task()
            self.assertEqual(reverse_done, overlap)
            task._run_delivery_approach(reverse_done=reverse_done)
            self.assertEqual(moves, [('backward', 400), ('forward', 2675)])


if __name__ == '__main__':
    unittest.main()
