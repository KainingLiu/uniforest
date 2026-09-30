"""Check route speed selection at encoder-derived compensation boundaries."""

import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call

from Strategy.context import TaskContext
from Strategy.task2 import Task2Program, Task2_2Program
from tests.test_strategy_composition import robot_fixture


def task_fixture(task_type=Task2Program):
    robot = robot_fixture()
    robot.move_chassis = Mock(return_value=SimpleNamespace(cancelled=False, timed_out=False))
    task = task_type(robot, context=TaskContext(robot))
    task._turn_to_heading = Mock()
    task._drive_until_wall = Mock()
    return robot, task


def move_call(direction, distance, speed, accel):
    return call(direction, distance, speed, hold_ms=0, accel_ms=accel, route_mode=True)


class RouteSpeedProfileTests(unittest.TestCase):
    def setUp(self):
        output = contextlib.redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)

    def test_cruise_acceleration_applies_in_all_four_directions(self):
        robot, task = task_fixture()
        for direction in ('forward', 'backward', 'left', 'right'):
            task._checked_move(direction, 700, 1000)
            robot.move_chassis.assert_called_with(
                direction, 700, 1000, hold_ms=0, accel_ms=800, route_mode=True)
        task._checked_move('right', 300, 400)
        self.assertEqual(robot.move_chassis.call_args, move_call('right', 300, 400, 300))
        task._checked_move('right', 700, 1000, accel_ms=600)
        self.assertEqual(robot.move_chassis.call_args, move_call('right', 700, 1000, 600))

    def test_purple_return_uses_actual_distance_and_preserves_short_reverse(self):
        for task_type, base_mm in ((Task2Program, 350), (Task2_2Program, 500)):
            for distance, speed, accel in ((499.5, 400, 300), (500, 1000, 800),
                                           (1000, 1000, 800)):
                with self.subTest(task=task_type.TASK_LABEL, distance=distance):
                    robot, task = task_fixture(task_type)
                    task._measure_lateral_displacement_mm = Mock(return_value=base_mm-distance)
                    task._run_post_return_wall_approach = Mock()
                    task._run_post_purple_route('purple-origin')
                    self.assertEqual(robot.move_chassis.call_args_list, [
                        move_call('backward', 100, 400, 300),
                        move_call('forward', distance, speed, accel)])

    def test_orange_correction_uses_actual_magnitude_for_both_directions(self):
        # Run the actual collection route with recognition/actuation mocked.
        # Crossing the target reverses direction without changing the policy.
        for task_type in (Task2Program, Task2_2Program):
            for direction in ('left', 'right', None):
                for distance in ((499.5, 500, 800) if direction else (0,)):
                    with self.subTest(task=task_type.TASK_LABEL, direction=direction,
                                      distance=distance):
                        robot, task = task_fixture(task_type)
                        offset = distance if direction == 'left' else -distance
                        task._capture_lateral_origin = Mock(return_value='orange-origin')
                        task._measure_lateral_displacement_mm = Mock(return_value=550+offset)
                        task._try_grab_purple = Mock(return_value=False)
                        task._run_post_purple_route = Mock()
                        task._collect_orange_with_count_check = Mock()
                        self.assertEqual(task.run(), 0)
                        initial = 2500 if task_type is Task2Program else 2350
                        expected = [move_call('backward', initial, 800, 800),
                                    move_call('forward', 250, 400, 300),
                                    move_call('backward', 100, 400, 300)]
                        if direction:
                            speed, accel = (1000, 800) if distance >= 500 else (400, 300)
                            expected.append(move_call(direction, distance, speed, accel))
                        expected.append(move_call('forward', 2750, 800, 800))
                        self.assertEqual(robot.move_chassis.call_args_list, expected)
                        robot.transport.emergency_stop.assert_not_called()


if __name__ == '__main__':
    unittest.main()
