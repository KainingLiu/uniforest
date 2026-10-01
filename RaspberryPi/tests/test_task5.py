"""Staged-building functional flow and mechanical overlap checks."""

import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from control.actions import Actions
from protocol.commands import (ActionStatus, ACTION_IDLE, ACTION_RUNNING,
                               ACTION_CHASSIS_READY, ACTION_DONE)
from Strategy.common import VisualAlignmentUnavailable
from Strategy.context import ExecutionContext
from Strategy.controllers import RobotController
from Strategy.runner import resolve_selection, run_plan
from tests.test_strategy_composition import FakeActionSession, robot_fixture


def flow_fixture():
    robot = robot_fixture()
    plan = resolve_selection('build-staged')
    context = ExecutionContext(robot, anchor=plan.entry_anchor)
    flow = SimpleNamespace()
    events = []
    monitor_check = None

    @contextlib.contextmanager
    def monitor(check):
        nonlocal monitor_check
        monitor_check = check
        check()
        try:
            yield
            check()
        finally:
            monitor_check = None

    robot.chassis.monitor_action = monitor

    def check_motion():
        context.check_active()
        if monitor_check is not None:
            monitor_check()

    def move(direction, distance, speed, **kwargs):
        check_motion()
        events.append(('move', direction, distance, speed, kwargs))
        return SimpleNamespace(timed_out=False, cancelled=False)

    robot.move_chassis = Mock(side_effect=move)
    def wall(**kwargs):
        check_motion()
        events.append(('wall', kwargs['direction'], kwargs['speed_mm_s'], kwargs['timeout_s']))
    flow.wall = Mock(side_effect=wall)
    robot.actions.hatch_open = Mock(side_effect=lambda **kw: events.append(('open', kw)))
    robot.actions.hatch_close = Mock(side_effect=lambda **kw: events.append(('close', kw)))
    flow.align = Mock(side_effect=lambda: events.append('building_align'))
    robot.actions.begin = Mock(side_effect=lambda action_id: FakeActionSession(events))
    def run():
        with patch.object(RobotController, '_drive_until_wall', flow.wall), \
             patch.object(RobotController, '_align_building', flow.align):
            return run_plan(robot, plan, context=context)
    flow.run = run
    return robot, context, flow, events


class StagedBuildingTests(unittest.TestCase):
    def setUp(self):
        output = contextlib.redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)

    def test_exact_route_hatch_timing_and_build_overlap(self):
        robot, context, flow, events = flow_fixture()
        with patch('Strategy.controllers.time.sleep', side_effect=AssertionError('No extra route waits')):
            self.assertEqual(flow.run(), 0)
        kw = dict(hold_ms=0, accel_ms=300, route_mode=True)
        long_kw = dict(hold_ms=0, accel_ms=800, route_mode=True)
        load = [('open', {'settle_ms': 200}), ('wall', 'forward', 300, 4),
                ('close', {'settle_ms': 400}), ('move', 'backward', 250, 400, kw),
                ('move', 'right', 940, 1000, long_kw), 'building_align']
        self.assertEqual([e for e in events if e != 'action_check'], [
            ('move', 'left', 700, 1000, long_kw), ('wall', 'left', 300, 4), *load,
            'third_release', ('move', 'backward', 100, 400, kw),
            ('move', 'left', 840, 800, long_kw), ('wall', 'left', 300, 4),
            ('move', 'right', 300, 400, kw), 'build_done', *load,
            'third_release', ('move', 'backward', 100, 400, kw),
            ('move', 'left', 440, 800, long_kw), 'build_done'])
        # Supervision must continue throughout both overlapped return routes.
        self.assertGreaterEqual(events.count('action_check'), 10)
        self.assertEqual(context.heading_zero_deg, 37)
        self.assertTrue(context.closed)
        robot.chassis.turn.assert_not_called()
        self.assertEqual(robot.reset_vision_filter.call_count, 2)
        robot.transport.emergency_stop.assert_not_called()

    def test_building_vision_failure_still_runs_both_builds(self):
        robot, context, flow, events = flow_fixture()
        robot.has_vision = robot.has_field_localization = False
        flow.align.side_effect = VisualAlignmentUnavailable('building lost')
        self.assertEqual(flow.run(), 0)
        self.assertEqual(robot.actions.begin.call_count, 2)
        self.assertIn(('move', 'left', 440, 800,
                       dict(hold_ms=0, accel_ms=800, route_mode=True)), events)
        robot.transport.emergency_stop.assert_not_called()

    def test_hatch_or_chassis_fault_stops_before_build(self):
        for fault in ('open-stop', 'close-disconnect', 'reverse-stale', 'reverse-cancel'):
            with self.subTest(fault=fault):
                robot, context, flow, events = flow_fixture()
                if fault == 'open-stop':
                    robot.actions.hatch_open.side_effect = lambda **kw: robot.transport.emergency_stop()
                if fault == 'close-disconnect':
                    robot.actions.hatch_close.side_effect = lambda **kw: setattr(robot.transport, 'connected', False)
                if fault.startswith('reverse-'):
                    original = robot.move_chassis.side_effect
                    def move(direction, *args, **kwargs):
                        result = original(direction, *args, **kwargs)
                        if direction == 'backward':
                            if fault == 'reverse-stale': robot.telemetry_age = 1
                            else: result.cancelled = True
                        return result
                    robot.move_chassis.side_effect = move
                with self.assertRaises(RuntimeError): flow.run()
                robot.actions.begin.assert_not_called()
                self.assertTrue(context.closed)
                self.assertEqual(robot.actions.hatch_open.call_count, 1)

    def test_build_fault_does_not_start_second_hatch_cycle_or_finish(self):
        for during_route in (False, True):
            with self.subTest(during_route=during_route):
                robot, context, flow, events = flow_fixture()
                def begin(action_id):
                    checks = 0
                    def failed_check():
                        nonlocal checks
                        checks += 1
                        if checks >= 3:
                            raise RuntimeError('mechanism fault')
                    if during_route:
                        return FakeActionSession(check=failed_check)
                    raise RuntimeError('Build rejected')
                robot.actions.begin.side_effect = begin
                with self.assertRaisesRegex(RuntimeError, 'fault|rejected'): flow.run()
                self.assertEqual(robot.actions.hatch_open.call_count, 1)
                self.assertEqual(robot.actions.begin.call_count, 1)
                self.assertTrue(context.closed)
                self.assertFalse(any(e[:3] == ('move', 'left', 440) for e in events if isinstance(e, tuple)))

    def test_actual_action_client_starts_routes_at_chassis_ready_and_waits_for_done(self):
        robot, context, flow, events = flow_fixture()
        clock = SimpleNamespace(now=100.0)
        samples = [ACTION_RUNNING, ACTION_RUNNING, ACTION_CHASSIS_READY,
                   ACTION_CHASSIS_READY, ACTION_DONE]
        token = action_id = polls = 0
        active = False
        status = ActionStatus(0, 0, ACTION_IDLE, 0, 100000)
        received_at = clock.now
        started_routes = []
        transport = robot.transport
        def query():
            nonlocal polls, status, received_at
            state = ACTION_IDLE
            if active:
                state = samples[min(polls, len(samples)-1)]
                polls += 1
            status = ActionStatus(token, action_id, state, polls, int(clock.now * 1000))
            received_at = clock.now
            return True
        def start(new_token, new_action_id, test_mode):
            nonlocal token, action_id, polls, active
            token, action_id, polls, active = new_token, new_action_id, 0, True
            events.append('build_start')
            return True
        transport.query_action_status = query
        transport.get_action_status = lambda: (status, received_at)
        transport.start_action = start
        actions = Actions(Mock(), SimpleNamespace(_t=transport), transport=transport)
        original_begin = actions.begin
        def begin(*args, **kwargs):
            session = original_begin(*args, **kwargs)
            original_wait_done = session.wait_done
            recorded = False
            def wait_done(*wait_args, **wait_kwargs):
                nonlocal recorded
                original_wait_done(*wait_args, **wait_kwargs)
                self.assertEqual(status.state, ACTION_DONE)
                if not recorded:
                    events.append('build_done')
                    recorded = True
            session.wait_done = wait_done
            return session
        actions.begin = begin
        actions.hatch_open = robot.actions.hatch_open
        actions.hatch_close = robot.actions.hatch_close
        robot.actions = actions
        original_move = robot.move_chassis.side_effect
        def move(direction, distance, *args, **kwargs):
            if direction == 'backward' and distance == 100:
                started_routes.append(status.state)
            return original_move(direction, distance, *args, **kwargs)
        robot.move_chassis.side_effect = move
        def wait(ms): clock.now += ms / 1000
        actions._wait = wait
        with patch('control.actions.time.monotonic', side_effect=lambda: clock.now):
            self.assertEqual(flow.run(), 0)
        self.assertEqual(started_routes, [ACTION_CHASSIS_READY] * 2)
        builds = [i for i, e in enumerate(events) if e == 'build_done']
        opens = [i for i, e in enumerate(events) if isinstance(e, tuple) and e[0] == 'open']
        self.assertLess(builds[0], opens[1])
        self.assertEqual(events[-1], 'build_done')
        robot.transport.emergency_stop.assert_not_called()


if __name__ == '__main__':
    unittest.main()
