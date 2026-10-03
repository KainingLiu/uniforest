"""Count-based orange recovery contracts for the main Task runtime."""

import contextlib
import io
from pathlib import Path
from unittest.mock import Mock, patch
import unittest

from Strategy.competition import CompetitionProgram, TaskControl, SearchRangeExhausted
from Strategy.context import TaskContext
from Strategy.orange_search import OrangeSearchRecovery
from Strategy.refill_policy import BothOrangeAreasExhausted, should_change_region
from Strategy.task1 import Task1_2Program, Task1_3Program
from Strategy.task2 import Task2Program, Task2_2Program
from tests.test_strategy_composition import robot_fixture


class MainRefillTests(unittest.TestCase):
    def setUp(self):
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.robot = robot_fixture()
        self.robot.actions.grap1 = Mock()
        self.robot.actions.grap2 = Mock()
        self.robot.actions.grap3 = Mock()
        self.robot.actions.hatch_open = Mock()
        self.robot.actions.hatch_close = Mock()
        self.robot.check_carried_cube_count = Mock(side_effect=(2, 2, 3, 3))
        self.context = TaskContext(self.robot)
        for name in ('_checked_move', '_drive_until_wall', '_turn_to_heading',
                     '_align_delivery_tag', '_align_orange', '_align_cube', '_fine_align_orange'):
            self.stack.enter_context(patch.object(TaskControl, name, return_value=True))
        self.stack.enter_context(patch.object(TaskControl, '_capture_lateral_origin', side_effect=object))
        self.stack.enter_context(patch.object(TaskControl, '_measure_lateral_displacement_mm', return_value=0))
        self.stack.enter_context(patch.object(TaskControl, '_find_cube', return_value=object()))
        self.stack.enter_context(patch.object(TaskControl, '_grab_with_wall_press',
                                              side_effect=lambda grab, **kw: grab()))

    def program(self, cls=CompetitionProgram):
        c = cls(self.robot, context=self.context)
        c._heading_zero_deg = 0
        c._orange_recovery = OrangeSearchRecovery(origin=object())
        c._search_position_mm = 1800
        return c

    def run_exhausted(self, c):
        original_grab = Mock(side_effect=SearchRangeExhausted())
        c._collect_orange_with_count_check(1, original_grab)

    def test_both_directions_all_rounds_take_only_missing_orange_and_keep_cargo(self):
        for cls in (CompetitionProgram, Task1_2Program, Task1_3Program, Task2Program, Task2_2Program):
            with self.subTest(cls=cls.__name__):
                self.robot.actions.grap1.reset_mock()
                self.robot.actions.grap3.reset_mock()
                self.robot.check_carried_cube_count.side_effect = (2, 2, 3, 3)
                c = self.program(cls)
                cfg, origin = c.config, c._orange_recovery.origin
                self.run_exhausted(c)
                method = 'grap3' if isinstance(c, Task2Program) else 'grap1'
                getattr(self.robot.actions, method).assert_called_once()
                self.robot.actions.grap2.assert_not_called()
                self.robot.actions.hatch_open.assert_not_called()
                self.robot.actions.hatch_close.assert_not_called()
                self.assertIs(c.config, cfg)
                self.assertIsNot(c._orange_recovery.origin, origin)

    def test_short_count_with_search_remaining_stays_in_same_area(self):
        c = self.program()
        self.robot.check_carried_cube_count.side_effect = (2, 3)
        grab = Mock()
        with patch('Strategy.refill.collect_other_region') as change:
            c._collect_orange_with_count_check(3, grab)
            change.assert_not_called()
        self.assertEqual(grab.call_count, 4)

    def test_exhausted_area_is_checked_and_full_count_does_not_transfer(self):
        self.robot.check_carried_cube_count.side_effect = (3,)
        with patch('Strategy.refill.collect_other_region') as change:
            self.run_exhausted(self.program())
            change.assert_not_called()
        self.robot.check_carried_cube_count.assert_called_once()

    def test_attempts_do_not_replace_count_or_limit_another_capture_after_a_miss(self):
        self.robot.check_carried_cube_count.side_effect = (2, 2, 2, 3, 3)
        self.run_exhausted(self.program())
        self.assertEqual(self.robot.actions.grap1.call_count, 2)

    def test_both_areas_empty_is_pending_and_never_returns_or_unloads(self):
        self.robot.check_carried_cube_count.side_effect = (2, 2, 2)
        with patch('Strategy.refill.transfer') as transfer, \
             patch.object(TaskControl, '_find_cube', side_effect=SearchRangeExhausted()):
            with self.assertRaises(BothOrangeAreasExhausted) as error:
                self.run_exhausted(self.program())
            self.assertEqual(error.exception.count, 2)
            self.assertEqual(transfer.call_count, 1)
        self.robot.actions.hatch_open.assert_not_called()

    def test_pending_stops_main_task_before_delivery(self):
        c = self.program()
        c._preflight = Mock()
        c._run_first_task = lambda: self.run_exhausted(c)
        c._run_delivery_route = Mock()
        self.robot.check_carried_cube_count.side_effect = (2, 2, 2)
        with patch('Strategy.refill.transfer'), \
             patch.object(TaskControl, '_find_cube', side_effect=SearchRangeExhausted()):
            with self.assertRaises(BothOrangeAreasExhausted): c.run()
        c._run_delivery_route.assert_not_called()
        self.robot.transport.emergency_stop.assert_called_once()

    def test_unknown_count_or_transfer_fault_never_grabs(self):
        self.robot.check_carried_cube_count.side_effect = (None,)
        with patch('Strategy.refill.transfer') as transfer:
            with self.assertRaises(RuntimeError): self.run_exhausted(self.program())
            transfer.assert_not_called()
        self.robot.check_carried_cube_count.side_effect = (2,)
        with patch('Strategy.refill.transfer', side_effect=RuntimeError('link lost')):
            with self.assertRaisesRegex(RuntimeError, 'link lost'): self.run_exhausted(self.program())
        self.robot.actions.grap1.assert_not_called()

    def test_missing_purple_sets_three_orange_and_never_grabs_purple(self):
        c = self.program(Task2Program)
        c._find_cube = Mock(side_effect=SearchRangeExhausted())
        grabbed = c._try_grab_purple()
        self.assertFalse(grabbed)
        self.assertEqual(c._orange_target_count_for_run(grabbed), 3)
        self.assertEqual(c._orange_target_count_for_run(True), 2)
        self.robot.actions.grap2.assert_not_called()

    def test_decision_contract_is_identical_to_independent_game_deployment(self):
        root = Path(__file__).resolve().parents[1]
        self.assertEqual((root/'Strategy/refill_policy.py').read_bytes(),
                         (root/'game/Strategy/refill_policy.py').read_bytes())
        for count in range(4):
            self.assertFalse(should_change_region(count, exhausted=False))
            self.assertEqual(should_change_region(count, exhausted=True), count < 3)

    def test_robot_count_adapter_only_enables_idle_when_explicit(self):
        from robot import Robot
        robot = Robot.__new__(Robot)
        with patch('control.carried_cube_inspection.inspect_carried_cubes', return_value=2) as inspect:
            self.assertEqual(robot.check_carried_cube_count(), 2)
            self.assertFalse(inspect.call_args.kwargs['allow_idle'])
            self.assertEqual(robot.check_carried_cube_count(allow_idle=True), 2)
            self.assertTrue(inspect.call_args.kwargs['allow_idle'])


if __name__ == '__main__':
    unittest.main()
