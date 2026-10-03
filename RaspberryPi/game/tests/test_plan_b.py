"""Flat PlanB action order, anchored routes and interruption regressions."""

import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from Strategy.errors import SearchRangeExhausted
from Strategy.common import VisualAlignmentUnavailable
from Strategy.context import ExecutionContext
from Strategy.controllers import RobotController
from Strategy.flows.factory import ActionEnvironment
from Strategy.plans import PLANS
from Strategy.runner import resolve_selection, run_plan
from tests.test_strategy_composition import FakeActionSession, robot_fixture


class PlanBTests(unittest.TestCase):
    def setUp(self):
        output = contextlib.redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)

    def test_plan_b_has_flat_functional_collection_and_build_sequence(self):
        plan = PLANS['PlanB']
        self.assertIs(resolve_selection('planb'), plan)
        self.assertEqual([(s.profile, s.parameters['color']) for s in plan.steps
                          if s.kind == 'begin_collection'], [
            ('highland-1', 'purple'), ('highland-1', 'orange'),
            ('highland-2', 'purple'), ('highland-2', 'orange'),
            ('ground-1', 'orange'), ('ground-2', 'orange')])
        builds = [i for i, s in enumerate(plan.steps) if s.kind == 'build']
        unloads = [i for i, s in enumerate(plan.steps) if s.kind == 'unload']
        self.assertEqual(len(builds), 2)
        self.assertEqual(len(unloads), 4)
        self.assertLess(max(unloads), min(builds))
        self.assertTrue(all(s.kind != 'task' for s in plan.steps))
        self.assertEqual(plan.steps[0].parameters['route'], 'depart_b')

    def run_selection(self, robot, selection, *, context=None):
        plan = resolve_selection(selection)
        if context is None:
            context = ExecutionContext(robot, anchor=plan.entry_anchor)
        return run_plan(robot, plan, context=context)

    def test_departure_routes_use_requested_distances_speed_and_acceleration(self):
        for selection, legs in (('depart-a', [('forward', 1200)]),
                                ('depart-b', [('forward', 900), ('right', 2700)])):
            with self.subTest(selection=selection):
                robot = robot_fixture()
                robot.move_chassis = Mock(return_value=SimpleNamespace(timed_out=False, cancelled=False))
                self.assertEqual(self.run_selection(robot, selection), 0)
                self.assertEqual(robot.move_chassis.call_args_list, [
                    call(direction, distance, 2000, hold_ms=0, accel_ms=1600, route_mode=True)
                    for direction, distance in legs])
                if selection == 'depart-b':
                    robot.chassis.turn.assert_called_once_with(180, 120, hold_ms=0, settle_cycles=1)
                else:
                    robot.chassis.turn.assert_not_called()

    def test_depart_b_corrects_final_heading_in_startup_frame_and_stops_on_fault(self):
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
                if fault:
                    with self.assertRaises(RuntimeError): self.run_selection(robot, 'depart-b')
                    robot.chassis.turn.assert_not_called()
                else:
                    self.assertEqual(self.run_selection(robot, 'depart-b'), 0)
                    robot.chassis.turn.assert_called_once_with(178, 120, hold_ms=0, settle_cycles=1)

    def test_departure_failure_or_stop_after_forward_prevents_right_move(self):
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
                    self.run_selection(robot, 'depart-b')
                self.assertEqual(robot.move_chassis.call_count, 1)
                robot.chassis.turn.assert_not_called()

    def test_return_orange_turns_from_180_then_moves_left_and_approaches_wall(self):
        robot = robot_fixture()
        plan = resolve_selection('return-orange')
        context = ExecutionContext(robot, anchor=plan.entry_anchor)
        events = Mock()
        events.attach_mock(robot.chassis.turn, 'turn')
        robot.move_chassis = Mock(return_value=SimpleNamespace(timed_out=False, cancelled=False))
        events.attach_mock(robot.move_chassis, 'move')
        wall = Mock()
        events.attach_mock(wall, 'wall')
        with patch.object(RobotController, '_drive_until_wall', wall):
            self.assertEqual(run_plan(robot, plan, context=context), 0)
        self.assertEqual(context.heading_zero_deg, 37)
        self.assertEqual(events.mock_calls, [
            call.turn(180, 120, hold_ms=0, settle_cycles=1),
            call.move('left', 2600, 2000, hold_ms=0, accel_ms=1600, route_mode=True),
            call.wall(timeout_s=4, speed_mm_s=300, direction='left',
                      context='LEFT_WALL_APPROACH')])
        robot.transport.emergency_stop.assert_not_called()

    def test_return_orange_stops_before_next_motion_on_stop_disconnect_or_stale(self):
        for stage in ('turn', 'move'):
            for fault in ('stop', 'disconnect', 'stale'):
                with self.subTest(stage=stage, fault=fault):
                    robot = robot_fixture()
                    def fail(*args, **kwargs):
                        if fault == 'stop': robot.transport.emergency_stop()
                        if fault == 'disconnect': robot.transport.connected = False
                        if fault == 'stale': robot.telemetry_age = 1
                        return SimpleNamespace(timed_out=False, cancelled=False)
                    robot.move_chassis = Mock(return_value=SimpleNamespace(timed_out=False, cancelled=False))
                    if stage == 'turn': robot.chassis.turn.side_effect = fail
                    else: robot.move_chassis.side_effect = fail
                    with patch.object(RobotController, '_drive_until_wall') as wall:
                        with self.assertRaises(RuntimeError):
                            self.run_selection(robot, 'return-orange')
                        self.assertEqual(robot.move_chassis.call_count, int(stage == 'move'))
                        wall.assert_not_called()

    def unload_fixture(self, selection='unload-1'):
        robot = robot_fixture()
        events = []
        plan = resolve_selection(selection)
        context = ExecutionContext(robot, heading_zero_deg=37, anchor=plan.entry_anchor)
        env = ActionEnvironment(robot, context)
        control = env.control(selection)
        def move(direction, distance, speed, **kwargs):
            events.append(('move', direction, distance, speed, kwargs))
            return SimpleNamespace(timed_out=False, cancelled=False)
        robot.move_chassis = Mock(side_effect=move)
        robot.actions.hatch_open = Mock(side_effect=lambda **kw: events.append(('open', kw)))
        robot.actions.hatch_close = Mock(side_effect=lambda **kw: events.append(('close', kw)))
        robot.chassis.turn = Mock(side_effect=lambda *a, **kw: events.append(('turn', a, kw)))
        def wall(**kwargs):
            control._check_active()
            events.append(('wall', kwargs['direction'], kwargs['speed_mm_s'], kwargs['timeout_s']))
        control._drive_until_wall = Mock(side_effect=wall)
        def run():
            with patch('Strategy.runner.ActionEnvironment', return_value=env):
                return run_plan(robot, plan, context=context)
        return robot, context, SimpleNamespace(run=run, control=control), events

    def test_unload_routes_unload_timing_and_finish_without_turning_from_180(self):
        for selection, right_mm, final_mm in (('unload-1', 0, 800), ('unload-2', 300, 500)):
            with self.subTest(selection=selection):
                robot, context, flow, events = self.unload_fixture(selection)
                self.assertEqual(flow.run(), 0)
                move_kw = dict(hold_ms=0, accel_ms=300, route_mode=True)
                long_kw = dict(hold_ms=0, accel_ms=1600, route_mode=True)
                expected = [('move', 'left', 600, 2000, long_kw), ('wall', 'left', 300, 4)]
                if right_mm:
                    expected.append(('move', 'right', right_mm, 400, move_kw))
                expected += [('wall', 'forward', 300, 4), ('open', {'settle_ms': 300}),
                             ('move', 'backward', 300, 400, move_kw), ('close', {'settle_ms': 0}),
                             ('move', 'right', final_mm, 2000, long_kw)]
                self.assertEqual(events, expected)
                # yaw=-143 and zero=37 mean an entry heading of 180 degrees.
                robot.chassis.turn.assert_not_called()
                self.assertEqual(flow.control._heading_error(180), 0)
                self.assertEqual(context.heading_zero_deg, 37)
                robot.transport.emergency_stop.assert_not_called()

    def test_unload_does_not_continue_after_hatch_or_motion_fault(self):
        for fault in ('open-stop', 'open-disconnect', 'reverse-stale', 'reverse-failed', 'close-stop'):
            with self.subTest(fault=fault):
                robot, context, flow, events = self.unload_fixture()
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
                with self.assertRaises(RuntimeError): flow.run()
                self.assertFalse(any(event[:3] == ('move', 'right', 800) for event in events))
                robot.chassis.turn.assert_not_called()
                if fault != 'close-stop': robot.actions.hatch_close.assert_not_called()

    def test_invalid_unload_anchor_and_nonfinite_heading_rejected_before_motion(self):
        robot = robot_fixture()
        plan = resolve_selection('unload-1')
        context = ExecutionContext(robot, heading_zero_deg=37, anchor='start')
        with self.assertRaisesRegex(ValueError, 'anchor'):
            run_plan(robot, plan, context=context)
        robot.set_collection_context.assert_not_called()
        for value in (float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                run_plan(robot, plan, heading_zero_deg=value)
        with self.assertRaises(ValueError):
            run_plan(robot, plan)

    def test_full_plan_b_continues_without_vision_and_builds_twice_from_staged_materials(self):
        robot = robot_fixture()
        robot.has_vision = robot.has_field_localization = False
        robot.move_chassis = Mock(return_value=SimpleNamespace(timed_out=False, cancelled=False))
        robot.check_carried_cube_count = Mock(return_value=None)
        robot.actions.hatch_open, robot.actions.hatch_close = Mock(), Mock()
        robot.actions.begin = Mock(side_effect=lambda action_id: FakeActionSession())
        unload_headings = []
        def wall(control, **kwargs):
            if hasattr(control.config, 'initial_lateral_left_mm'):
                unload_headings.append(control._heading_zero_deg)
        def recalibrate(control, reference_cw_deg=0):
            control._heading_zero_deg = 37.0
        with patch.object(RobotController, '_drive_until_wall', wall), \
             patch.object(RobotController, '_turn_to_heading'), \
             patch.object(RobotController, '_capture_lateral_origin', return_value=0), \
             patch.object(RobotController, '_measure_lateral_displacement_mm', return_value=0), \
             patch.object(RobotController, '_recalibrate_heading_zero', recalibrate), \
             patch.object(RobotController, '_align_delivery_tag', side_effect=VisualAlignmentUnavailable('no tag')), \
             patch.object(RobotController, '_chassis_followup', side_effect=lambda route: lambda check: route()), \
             patch.object(RobotController, '_align_building', side_effect=VisualAlignmentUnavailable('no building')), \
             patch.object(RobotController, '_find_cube', side_effect=SearchRangeExhausted()):
            self.assertEqual(run_plan(robot, PLANS['PlanB']), 0)
        self.assertEqual(unload_headings, [37.0] * 4)
        self.assertEqual(robot.actions.hatch_open.call_args_list,
                         [call(settle_ms=300)] * 4 + [call(settle_ms=200)] * 2)
        self.assertEqual(robot.actions.hatch_close.call_args_list,
                         [call(settle_ms=0)] * 4 + [call(settle_ms=400)] * 2)
        self.assertEqual(robot.actions.begin.call_count, 2)
        robot.transport.emergency_stop.assert_not_called()

    def test_new_cli_previews_and_bad_unload_entries_never_connect(self):
        import main
        for args, expected in ((['--strategy', 'PlanB', '--show-plan'], 0),
                               (['--flow', 'return-orange', '--show-plan'], 0),
                               (['--flow', 'build-staged', '--show-plan'], 0),
                               (['--flow', 'unload-2', '--show-plan'], 0),
                               (['--flow', 'unload-1'], 2),
                               (['--flow', 'unload-2', '--heading-zero-deg', 'nan'], 2)):
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
