"""Route speed selection at encoder-derived compensation boundaries."""

import contextlib
import io
import unittest
from unittest.mock import Mock, call

from Strategy.flows import routes
from tests.test_functional_operations import operation_fixture


def route_fixture(profile='highland-1'):
    env, control = operation_fixture(profile)
    control._turn_to_heading = Mock()
    control._drive_until_wall = Mock()
    return env, control


def move_call(direction, distance, speed, accel):
    return call(direction, distance, speed, hold_ms=0, accel_ms=accel, route_mode=True)


class RouteSpeedProfileTests(unittest.TestCase):
    def setUp(self):
        output = contextlib.redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)

    def test_cruise_acceleration_applies_in_all_four_directions(self):
        env, control = route_fixture()
        for direction in ('forward', 'backward', 'left', 'right'):
            control._checked_move(direction, 700, 1000)
            env.robot.move_chassis.assert_called_with(
                direction, 700, 1000, hold_ms=0, accel_ms=800, route_mode=True)
        control._checked_move('right', 300, 400)
        self.assertEqual(env.robot.move_chassis.call_args, move_call('right', 300, 400, 300))
        control._checked_move('right', 700, 1000, accel_ms=600)
        self.assertEqual(env.robot.move_chassis.call_args, move_call('right', 700, 1000, 600))

    def test_purple_return_uses_actual_distance_and_preserves_short_reverse(self):
        for profile, base_mm in (('highland-1', 350), ('highland-2', 500)):
            for distance, speed, accel in ((499.5, 400, 300), (500, 1000, 800),
                                           (1000, 1000, 800)):
                with self.subTest(profile=profile, distance=distance):
                    env, control = route_fixture(profile)
                    env.data['purple_origin'] = 'purple-origin'
                    control._measure_lateral_displacement_mm = Mock(return_value=base_mm-distance)
                    routes.purple_to_orange(env, profile)
                    self.assertEqual(env.robot.move_chassis.call_args_list, [
                        move_call('backward', 100, 400, 300),
                        move_call('forward', distance, speed, accel)])

    def test_orange_correction_uses_actual_magnitude_for_both_directions(self):
        for profile in ('highland-1', 'highland-2'):
            for direction in ('left', 'right', None):
                for distance in ((499.5, 500, 800) if direction else (0,)):
                    with self.subTest(profile=profile, direction=direction, distance=distance):
                        env, control = route_fixture(profile)
                        offset = distance if direction == 'left' else -distance
                        env.data['orange_origin'] = 'orange-origin'
                        control._measure_lateral_displacement_mm = Mock(return_value=550+offset)
                        routes.to_purple(env, profile)
                        routes.orange_to_build(env, profile)
                        initial = 2500 if profile == 'highland-1' else 2350
                        expected = [move_call('backward', initial, 800, 800),
                                    move_call('forward', 250, 400, 300),
                                    move_call('backward', 100, 400, 300)]
                        if direction:
                            speed, accel = (1000, 800) if distance >= 500 else (400, 300)
                            expected.append(move_call(direction, distance, speed, accel))
                        expected.append(move_call('forward', 2750, 800, 800))
                        self.assertEqual(env.robot.move_chassis.call_args_list, expected)
                        env.robot.transport.emergency_stop.assert_not_called()

    def test_reverse_overlap_does_not_repeat_or_remeasure_after_departure(self):
        env, control = route_fixture()
        env.data['orange_origin'] = 'orange-origin'
        control._measure_lateral_displacement_mm = Mock(side_effect=[250, 900])
        routes.orange_depart_reverse(env, 'highland-1')
        routes.orange_to_build(env, 'highland-1')
        self.assertEqual(env.data['orange_lateral_mm'], 250)
        control._measure_lateral_displacement_mm.assert_called_once_with('orange-origin')
        self.assertEqual(env.robot.move_chassis.call_args_list, [
            move_call('backward', 100, 400, 300),
            move_call('right', 300, 400, 300),
            move_call('forward', 2750, 800, 800),
        ])


if __name__ == '__main__':
    unittest.main()
