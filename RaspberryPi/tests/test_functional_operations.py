"""Reusable action contracts using fake hardware, without mission programs."""

import contextlib
import io
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call

from Strategy.context import ExecutionContext
from Strategy.controllers import Phase, RobotController
from Strategy.errors import SearchRangeExhausted
from Strategy.flows import ActionSpec
from Strategy.flows import operations as ops
from Strategy.settings import PROFILES


def operation_fixture(profile='ground-1'):
    """Real controller/config binding with non-moving robot endpoints."""
    robot = SimpleNamespace(
        telem=SimpleNamespace(yaw_deg=12.0, uptime_ms=1000),
        transport=SimpleNamespace(connected=True, emergency_stop_generation=0,
                                  emergency_stop=Mock()),
        actions=SimpleNamespace(grap1=Mock(), grap2=Mock(), grap3=Mock(),
                                hatch_open=Mock(), hatch_close=Mock()),
        chassis=SimpleNamespace(capture_motor_positions=Mock(return_value=object()),
                                lateral_displacement_mm=Mock(return_value=0.0),
                                set_speeds=Mock(return_value=True), turn=Mock()),
        reset_vision_filter=Mock(), reset_field_localization_filter=Mock(),
        set_cube_detection_profile=Mock(), set_collection_context=Mock(),
        has_vision=True, has_field_localization=True,
        check_carried_cube_count=Mock(return_value=3),
        move_chassis=Mock(return_value=SimpleNamespace(cancelled=False, timed_out=False)),
    )
    robot.inspection_link_snapshot = lambda: (
        robot.telem, time.monotonic(), time.monotonic(), 0)
    context = ExecutionContext(robot, heading_zero_deg=0.0)
    control = RobotController(robot, PROFILES[profile], context=context)
    env = SimpleNamespace(robot=robot, context=context, data={},
                          control=Mock(return_value=control), run_route=Mock())
    env.phase = lambda control, name: setattr(control, 'state', Phase[name])
    return env, control


def spec(kind, profile='ground-1', **parameters):
    return ActionSpec(kind, f'test.{kind}', profile, parameters)


class FunctionalOperationTests(unittest.TestCase):
    def setUp(self):
        output = contextlib.redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)

    def test_missed_purple_is_an_outcome_and_never_starts_grap2(self):
        env, c = operation_fixture('highland-1')
        ops.begin_collection(env, spec('begin_collection', 'highland-1', color='purple'))
        c._find_cube = Mock(side_effect=SearchRangeExhausted('empty'))
        c._grab_with_wall_press = Mock()
        self.assertFalse(ops.acquire_cube(env, spec('acquire_cube', 'highland-1')))
        self.assertFalse(ops.grab_cube(env, spec('grab_cube', 'highland-1', method='grap2')))
        self.assertFalse(env.data['purple_grabbed'])
        c._grab_with_wall_press.assert_not_called()
        env.robot.set_cube_detection_profile.assert_called_with('default')
        env.robot.transport.emergency_stop.assert_not_called()

    def test_new_purple_visit_resets_departure_flag_and_result_is_post_completion(self):
        env, c = operation_fixture('highland-1')
        env.data.update(purple_route_done=True, purple_grabbed=True)
        ops.begin_collection(env, spec('begin_collection', 'highland-1', color='purple'))
        self.assertFalse(env.data['purple_route_done'])
        self.assertFalse(env.data['purple_grabbed'])
        env.data['collection']['acquired'] = True
        def mechanism(grab, *, recalibrate_heading_zero, chassis_followup):
            self.assertFalse(env.data['purple_grabbed'])
            self.assertFalse(recalibrate_heading_zero)
            self.assertIs(grab, env.robot.actions.grap2)
            chassis_followup()
        c._grab_with_wall_press = Mock(side_effect=mechanism)
        self.assertTrue(ops.grab_cube(env, spec(
            'grab_cube', 'highland-1', method='grap2', followup_route='purple_to_orange')))
        self.assertTrue(env.data['purple_grabbed'])
        self.assertFalse(env.data['collection']['acquired'])
        env.run_route.assert_called_once_with('purple_to_orange', 'highland-1')

    def test_third_orange_slot_is_conditioned_on_completed_purple_sequence(self):
        for purple_grabbed in (False, True):
            with self.subTest(purple_grabbed=purple_grabbed):
                env, c = operation_fixture('highland-1')
                env.data['purple_grabbed'] = purple_grabbed
                ops.begin_collection(env, spec('begin_collection', 'highland-1', color='orange'))
                self.assertEqual(env.data['purple_grabbed'], purple_grabbed)
                c._find_cube = Mock(return_value=object())
                c._align_cube = Mock(return_value=True)
                c._fine_align_orange = Mock(return_value=True)
                c._grab_with_wall_press = Mock()
                params = dict(index=3, conditional_on_purple=True)
                acquired = ops.acquire_cube(env, spec('acquire_cube', 'highland-1', **params))
                grabbed = ops.grab_cube(env, spec('grab_cube', 'highland-1', method='grap1', **params))
                self.assertEqual(acquired, not purple_grabbed)
                self.assertEqual(grabbed, not purple_grabbed)
                self.assertEqual(c._find_cube.call_count, int(not purple_grabbed))
                self.assertEqual(c._grab_with_wall_press.call_count, int(not purple_grabbed))

    def test_inspection_refill_keeps_origin_and_remaining_search_budget(self):
        env, c = operation_fixture()
        ops.begin_collection(env, spec('begin_collection', color='orange'))
        origin, recovery = env.data['collection']['origin'], c._orange_recovery
        c._search_position_mm = 420.0
        c._find_cube = Mock(return_value=object())
        c._align_orange = Mock(return_value=True)
        c._grab_with_wall_press = Mock()
        env.robot.check_carried_cube_count.side_effect = [1, 3]
        self.assertEqual(ops.inspect_cargo(env, spec('inspect_cargo', method='grap3')), 3)
        self.assertEqual(c._grab_with_wall_press.call_count, 2)
        self.assertEqual(env.robot.check_carried_cube_count.call_count, 2)
        self.assertIs(env.data['collection']['origin'], origin)
        self.assertIs(c._orange_recovery, recovery)
        self.assertEqual(c._search_position_mm, 420.0)
        env.robot.set_cube_detection_profile.assert_called_with('default')

    def test_search_exhaustion_skips_later_grabs_and_camera_inspection(self):
        env, c = operation_fixture()
        ops.begin_collection(env, spec('begin_collection', color='orange'))
        c._find_cube = Mock(side_effect=SearchRangeExhausted('spent'))
        c._grab_with_wall_press = Mock()
        self.assertFalse(ops.acquire_cube(env, spec('acquire_cube')))
        self.assertFalse(ops.grab_cube(env, spec('grab_cube', method='grap3')))
        self.assertIsNone(ops.inspect_cargo(env, spec('inspect_cargo', method='grap3')))
        c._grab_with_wall_press.assert_not_called()
        env.robot.check_carried_cube_count.assert_not_called()

    def test_refill_exhaustion_does_not_reset_budget_or_reinspect(self):
        env, c = operation_fixture()
        ops.begin_collection(env, spec('begin_collection', color='orange'))
        recovery = c._orange_recovery
        c._search_position_mm = 1790.0
        c._find_cube = Mock(side_effect=SearchRangeExhausted('spent'))
        c._grab_with_wall_press = Mock()
        env.robot.check_carried_cube_count.return_value = 0
        self.assertIsNone(ops.inspect_cargo(env, spec('inspect_cargo', method='grap3')))
        self.assertTrue(env.data['collection']['exhausted'])
        self.assertIs(c._orange_recovery, recovery)
        self.assertEqual(c._search_position_mm, 1790.0)
        self.assertEqual(env.robot.check_carried_cube_count.call_count, 1)
        c._grab_with_wall_press.assert_not_called()

    def test_inspection_passes_exit_to_existing_restoration_overlap(self):
        for count in (None, 3):
            with self.subTest(count=count):
                env, c = operation_fixture()
                ops.begin_collection(env, spec('begin_collection', color='orange'))
                def inspection(*, chassis_followup, allow_visual_failure):
                    self.assertTrue(allow_visual_failure)
                    chassis_followup()
                    return count
                env.robot.check_carried_cube_count.side_effect = inspection
                self.assertEqual(ops.inspect_cargo(env, spec(
                    'inspect_cargo', method='grap3', exit_route='ground_delivery_reverse')), count)
                env.run_route.assert_called_once_with('ground_delivery_reverse', 'ground-1')

    def test_unload_retains_open_reverse_close_order_and_settle_times(self):
        env, c = operation_fixture()
        events = Mock()
        events.attach_mock(env.robot.actions.hatch_open, 'open')
        events.attach_mock(env.robot.move_chassis, 'move')
        events.attach_mock(env.robot.actions.hatch_close, 'close')
        ops.unload(env, spec('unload'))
        self.assertEqual(events.mock_calls, [
            call.open(settle_ms=300),
            call.move('backward', 300.0, 400.0, hold_ms=0, accel_ms=300, route_mode=True),
            call.close(settle_ms=0),
        ])

    def test_staged_load_retains_open_contact_close_order_and_settle_times(self):
        env, c = operation_fixture('staged-building')
        c._drive_until_wall = Mock()
        events = Mock()
        events.attach_mock(env.robot.actions.hatch_open, 'open')
        events.attach_mock(c._drive_until_wall, 'wall')
        events.attach_mock(env.robot.actions.hatch_close, 'close')
        ops.load_staged(env, spec('load_staged', 'staged-building'))
        self.assertEqual(events.mock_calls, [
            call.open(settle_ms=200),
            call.wall(timeout_s=4.0, speed_mm_s=300.0, direction='forward', context='test.load_staged'),
            call.close(settle_ms=400),
        ])


if __name__ == '__main__':
    unittest.main()
