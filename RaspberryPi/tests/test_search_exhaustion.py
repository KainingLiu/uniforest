"""Route continuation on search exhaustion, with hardware faults kept fatal."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from Strategy.competition import CompetitionProgram, SearchRangeExhausted
from Strategy.task1 import Task1Round2Program
from Strategy.task2 import Task2Program, Task2Round2Program


class SearchExhaustionTests(unittest.TestCase):
    def make_program(self, program_type):
        robot = Mock()
        robot.move_chassis.return_value = SimpleNamespace(
            timed_out=False, cancelled=False)
        program = program_type(robot)
        for name in ('_preflight', '_drive_until_wall', '_recalibrate_heading_zero',
                     '_press_wall_before_grab', '_turn_to_heading',
                     '_align_delivery_tag', '_align_building'):
            setattr(program, name, Mock())
        program._capture_lateral_origin = Mock(return_value='origin')
        program._measure_lateral_displacement_mm = Mock(return_value=1900.0)
        program._align_orange = Mock(return_value=True)
        program._align_cube = Mock(return_value=True)
        program._fine_align_orange = Mock(return_value=True)
        return program, robot

    def test_task1_both_rounds_deliver_partial_or_empty_load(self):
        for cls in (CompetitionProgram, Task1Round2Program):
            for count in (0, 1, 2):
                with self.subTest(program=cls.__name__, count=count):
                    program, robot = self.make_program(cls)
                    self.assertEqual(program.config.search_max_distance_mm, 1800)
                    program._find_orange = Mock(side_effect=[
                        *([object()] * count), SearchRangeExhausted('limit')])
                    with patch('Strategy.competition.time.sleep'):
                        self.assertEqual(program.run(), 0)
                    self.assertEqual(robot.actions.grap3.call_count, count)
                    self.assertEqual(program._cube_lateral_displacement_mm, 1900)
                    self.assertTrue(any(
                        c.args[:2] == ('forward', 900.0)
                        for c in robot.move_chassis.call_args_list))
                    robot.transport.emergency_stop.assert_not_called()

    def test_task2_both_rounds_build_partial_or_empty_load(self):
        for cls in (Task2Program, Task2Round2Program):
            for purple in (False, True):
                for count in range(2 if purple else 3):
                    with self.subTest(program=cls.__name__, purple=purple, count=count):
                        program, robot = self.make_program(cls)
                        program._try_grab_purple = Mock(return_value=purple)
                        program._measure_lateral_displacement_mm.side_effect = [-100, 1900]
                        program._find_cube = Mock(side_effect=[
                            *([object()] * count), SearchRangeExhausted('limit')])
                        with patch('Strategy.competition.time.sleep'):
                            self.assertEqual(program.run(), 0)
                        self.assertEqual(robot.actions.grap1.call_count, count)
                        robot.actions.build.assert_called_once()
                        robot.set_cube_detection_profile.assert_called_with('default')
                        self.assertTrue(any(
                            c.args[:2] == ('left', 1200.0)
                            for c in robot.move_chassis.call_args_list))
                        robot.transport.emergency_stop.assert_not_called()

    def test_search_hardware_and_cancellation_errors_still_abort(self):
        for cls in (CompetitionProgram, Task1Round2Program,
                    Task2Program, Task2Round2Program):
            for message in ('telemetry lost', 'cancelled',
                            'orange cube not found within search range'):
                with self.subTest(program=cls.__name__, error=message):
                    program, robot = self.make_program(cls)
                    program._try_grab_purple = Mock(return_value=True)
                    program._measure_lateral_displacement_mm.return_value = -100
                    program._find_orange = Mock(side_effect=RuntimeError(message))
                    program._find_cube = Mock(side_effect=RuntimeError(message))
                    with patch('Strategy.competition.time.sleep'):
                        with self.assertRaisesRegex(RuntimeError, message):
                            program.run()
                    robot.transport.emergency_stop.assert_called_once()
                    robot.actions.grap1.assert_not_called()
                    robot.actions.grap3.assert_not_called()
                    robot.actions.build.assert_not_called()

    def test_search_exhaustion_stops_at_1800_and_keeps_cumulative_budget(self):
        from tests.test_orange_search import SearchRobot
        robot = SearchRobot()
        program = CompetitionProgram(robot)
        program._search_position_mm = 1500.0
        with patch('time.monotonic', lambda: robot.now), \
                patch('time.time', lambda: 100 + robot.now), \
                patch('time.sleep', robot.sleep):
            with self.assertRaises(SearchRangeExhausted):
                program._find_orange()
            self.assertAlmostEqual(robot.now, 1.0, delta=0.03)
            self.assertEqual(program._search_position_mm, 1800)
            self.assertEqual(robot.commands[-1][1], 0)
            robot.commands.clear()
            with self.assertRaises(SearchRangeExhausted):
                program._find_orange()
            self.assertEqual(robot.commands, [])
