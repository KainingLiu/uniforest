"""Phased A-board action sessions against a deterministic transport replay."""

from types import SimpleNamespace
from contextlib import nullcontext
from enum import Enum
import threading
import unittest
from unittest.mock import Mock, patch

from control.actions import Actions, ActionCancelled
from protocol.commands import (
    ActionStatus, ACTION_BUILD, ACTION_GRAP1, ACTION_GRAP2, ACTION_GRAP3,
    ACTION_IDLE, ACTION_RUNNING, ACTION_CHASSIS_READY, ACTION_DONE,
)


class ActionReplay:
    def __init__(self, states=(ACTION_RUNNING, ACTION_CHASSIS_READY, ACTION_DONE), *,
                 pickup_full_lift_validated=False):
        self.now = 100.0
        self.states = list(states)
        self.active = False
        self.token = self.action_id = self.polls = 0
        self.connected = True
        self.emergency_stop_generation = 0
        self.stop_calls = 0
        self.freeze = False
        self.on_start = lambda: None
        self.change_status = lambda status: status
        self.status = ActionStatus(0, 0, ACTION_IDLE, 0, 100000)
        self.received_at = self.now
        self.actions = Actions(Mock(), SimpleNamespace(_t=self), transport=self,
                               pickup_full_lift_validated=pickup_full_lift_validated)
        self.actions._wait = lambda ms: self.advance(ms / 1000)

    def advance(self, seconds):
        self.now += seconds

    def query_action_status(self):
        if self.freeze:
            return True
        state = ACTION_IDLE
        if self.active:
            state = self.states[min(self.polls, len(self.states) - 1)]
            self.polls += 1
        self.status = self.change_status(ActionStatus(
            self.token, self.action_id, state, self.polls, round(self.now * 1000)))
        self.received_at = self.now
        return True

    def get_action_status(self):
        return self.status, self.received_at

    def start_action(self, token, action_id, test_mode):
        self.token, self.action_id = token, action_id
        self.active = True
        self.on_start()
        return True

    def emergency_stop(self):
        self.stop_calls += 1
        self.emergency_stop_generation += 1


class ActionSessionTests(unittest.TestCase):
    def replay(self, *args, **kwargs):
        replay = ActionReplay(*args, **kwargs)
        clock = patch('control.actions.time.monotonic', side_effect=lambda: replay.now)
        clock.start()
        self.addCleanup(clock.stop)
        return replay

    def test_phases_keep_lock_until_mechanism_and_route_complete(self):
        replay = self.replay()
        session = replay.actions.begin(ACTION_BUILD)
        self.assertTrue(replay.actions._action_lock.locked())
        session.wait_chassis_ready()
        self.assertTrue(session.chassis_ready)
        self.assertFalse(session.done)

        def route(check):
            with self.assertRaisesRegex(RuntimeError, 'another mechanical action'):
                replay.actions.begin(ACTION_BUILD)
            replay.advance(.06)
            check()
            self.assertTrue(session.done)
            self.assertTrue(replay.actions._action_lock.locked())

        session.run_followup(route)
        session.wait_done()
        session.close()
        session.close()
        self.assertTrue(session.closed)
        self.assertFalse(replay.actions._action_lock.locked())
        self.assertEqual(replay.stop_calls, 0)

    def test_unfinished_close_stops_once_and_cannot_resume(self):
        replay = self.replay()
        session = replay.actions.begin(ACTION_BUILD)
        session.close()
        session.close()
        self.assertEqual(replay.stop_calls, 1)
        self.assertFalse(replay.actions._action_lock.locked())
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            session.check()
        self.assertEqual(replay.stop_calls, 1)

    def test_emergency_stop_during_start_cannot_become_new_baseline(self):
        replay = self.replay()
        replay.on_start = replay.emergency_stop
        with self.assertRaises(ActionCancelled):
            replay.actions.begin(ACTION_BUILD)
        self.assertFalse(replay.actions._action_lock.locked())
        self.assertEqual(replay.stop_calls, 2)

    def test_stop_during_probe_prevents_start_command(self):
        replay = self.replay()
        original_query = replay.query_action_status

        def query():
            result = original_query()
            replay.emergency_stop()
            return result

        replay.query_action_status = query
        with self.assertRaises(ActionCancelled):
            replay.actions.begin(ACTION_BUILD)
        self.assertFalse(replay.active)
        self.assertFalse(replay.actions._action_lock.locked())

    def test_route_cannot_start_before_clearance(self):
        replay = self.replay(states=(ACTION_RUNNING,))
        session = replay.actions.begin(ACTION_BUILD)
        route = Mock()
        with self.assertRaisesRegex(RuntimeError, 'not released'):
            session.run_followup(route)
        route.assert_not_called()
        self.assertTrue(session.closed)
        self.assertEqual(replay.stop_calls, 1)

    def test_route_fault_after_done_still_stops(self):
        replay = self.replay(states=(ACTION_DONE,))
        session = replay.actions.begin(ACTION_BUILD)
        session.wait_chassis_ready()

        def route(check):
            replay.connected = False
            check()

        with self.assertRaisesRegex(RuntimeError, 'disconnected'):
            session.run_followup(route)
        self.assertEqual(replay.stop_calls, 1)
        self.assertTrue(session.closed)

    def test_stale_telemetry_stops_and_releases_lock(self):
        replay = self.replay(states=(ACTION_RUNNING,))
        session = replay.actions.begin(ACTION_BUILD)
        session.check()
        replay.freeze = True
        replay.advance(.6)
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            session.check()
        self.assertEqual(replay.stop_calls, 1)
        self.assertFalse(replay.actions._action_lock.locked())

    def test_fresh_repeated_clock_is_rejected(self):
        replay = self.replay(states=(ACTION_RUNNING,))
        session = replay.actions.begin(ACTION_BUILD)
        session.check()
        replay.change_status = lambda status: ActionStatus(
            status.token, status.action_id, status.state, status.stage, 100000)
        replay.advance(.6)
        with self.assertRaisesRegex(RuntimeError, 'clock stopped'):
            session.check()
        self.assertEqual(replay.stop_calls, 1)

    def test_restart_and_foreign_action_are_rejected(self):
        for fault in ('restart', 'token', 'action_id'):
            with self.subTest(fault=fault):
                replay = self.replay(states=(ACTION_RUNNING,))
                session = replay.actions.begin(ACTION_BUILD)
                session.check()

                def changed(status):
                    return ActionStatus(
                        status.token + (fault == 'token'),
                        status.action_id + (fault == 'action_id'),
                        status.state, status.stage,
                        0 if fault == 'restart' else status.uptime_ms)

                replay.change_status = changed
                replay.advance(.06)
                with self.assertRaisesRegex(RuntimeError, 'restarted|identity changed'):
                    session.check()
                self.assertEqual(replay.stop_calls, 1)
                self.assertTrue(session.closed)

    def test_nonblocking_companion_must_finish_before_followup(self):
        replay = self.replay(states=(ACTION_CHASSIS_READY, ACTION_DONE))
        companion_polls = []
        routes = []

        def companion():
            companion_polls.append(replay.now)
            return len(companion_polls) == 3

        replay.actions.grap1(
            parallel_step=companion,
            chassis_followup=lambda check: routes.append(len(companion_polls)))
        self.assertEqual(routes, [3])
        self.assertEqual(replay.action_id, ACTION_GRAP1)
        self.assertEqual(replay.stop_calls, 0)

    def test_grap3_wrapper_supports_supervised_chassis_followup(self):
        replay = self.replay(pickup_full_lift_validated=True)
        seen = []

        def route(check):
            seen.append(replay.status.state)
            check()

        replay.actions.grap3(chassis_followup=route)
        self.assertEqual(seen, [ACTION_CHASSIS_READY])
        self.assertEqual(replay.action_id, ACTION_GRAP3)
        self.assertEqual(replay.status.state, ACTION_DONE)
        self.assertEqual(replay.stop_calls, 0)

    def test_pickup_ready_from_unvalidated_firmware_waits_for_done(self):
        for action_id in (ACTION_GRAP1, ACTION_GRAP2, ACTION_GRAP3):
            with self.subTest(action_id=action_id):
                replay = self.replay(states=(ACTION_CHASSIS_READY, ACTION_DONE))
                session = replay.actions.begin(action_id)
                session.check()
                self.assertFalse(session.chassis_ready)
                self.assertFalse(session.done)
                session.wait_chassis_ready()
                self.assertTrue(session.done)
                route_states = []
                session.run_followup(lambda check: route_states.append(replay.status.state))
                self.assertEqual(route_states, [ACTION_DONE])
                session.close()
                self.assertEqual(replay.stop_calls, 0)

    def test_explicitly_validated_pickup_can_depart_at_full_lift(self):
        replay = self.replay(states=(ACTION_CHASSIS_READY, ACTION_DONE),
                             pickup_full_lift_validated=True)
        seen = []
        replay.actions.grap2(chassis_followup=lambda check: seen.append(replay.status.state))
        self.assertEqual(seen, [ACTION_CHASSIS_READY])
        self.assertEqual(replay.stop_calls, 0)

    def test_pickup_firmware_validation_requires_explicit_bool(self):
        with self.assertRaises(TypeError):
            ActionReplay(pickup_full_lift_validated='false')

    def test_abort_send_failure_preserves_original_fault_and_releases_lock(self):
        replay = self.replay(states=(ACTION_DONE,))
        failure = OSError('stop send failed')
        replay.emergency_stop = Mock(side_effect=failure)
        with self.assertRaisesRegex(ValueError, 'original route fault'):
            with replay.actions.begin(ACTION_BUILD) as session:
                session.wait_done()
                raise ValueError('original route fault')
        self.assertIs(session.cleanup_error, failure)
        self.assertFalse(session.abort())
        session.close()
        replay.emergency_stop.assert_called_once()
        self.assertFalse(replay.actions._action_lock.locked())

    def test_close_status_failure_survives_failed_stop_request(self):
        replay = self.replay(states=(ACTION_DONE,))
        session = replay.actions.begin(ACTION_BUILD)
        session.wait_done()
        replay.connected = False
        replay.emergency_stop = Mock(side_effect=OSError('stop send failed'))
        with self.assertRaisesRegex(RuntimeError, 'disconnected'):
            session.close()
        self.assertTrue(session.closed)
        self.assertFalse(replay.actions._action_lock.locked())

    def test_context_exception_stops_even_after_done(self):
        replay = self.replay(states=(ACTION_DONE,))
        with self.assertRaisesRegex(ValueError, 'route failed'):
            with replay.actions.begin(ACTION_BUILD) as session:
                session.wait_done()
                raise ValueError('route failed')
        self.assertEqual(replay.stop_calls, 1)
        self.assertFalse(replay.actions._action_lock.locked())

    def test_parallel_callback_failure_stops_mechanism(self):
        replay = self.replay()
        with self.assertRaisesRegex(RuntimeError, 'companion failed'):
            replay.actions.grap1(parallel_step=Mock(side_effect=RuntimeError('companion failed')))
        self.assertEqual(replay.stop_calls, 1)
        self.assertFalse(replay.actions._action_lock.locked())

    def test_cancelled_wait_cannot_start_followup(self):
        replay = self.replay(states=(ACTION_RUNNING,))
        event = threading.Event()
        replay.actions.set_cancel_event(event)
        session = replay.actions.begin(ACTION_BUILD)
        event.set()
        with self.assertRaises(ActionCancelled):
            session.wait_chassis_ready()
        self.assertTrue(session.closed)
        self.assertEqual(replay.stop_calls, 1)

    def test_action_timeout_stops_running_firmware_action(self):
        replay = self.replay(states=(ACTION_RUNNING,))
        with patch('control.actions.ACTION_TIMEOUT_S', .2):
            with self.assertRaisesRegex(RuntimeError, 'timed out'):
                replay.actions.build()
        self.assertEqual(replay.stop_calls, 1)
        self.assertFalse(replay.actions._action_lock.locked())

    def test_missing_capability_response_stops_without_starting_motion(self):
        replay = self.replay()
        replay.freeze = True
        replay.received_at = 99.0
        with self.assertRaisesRegex(RuntimeError, 'interface unavailable'):
            replay.actions.begin(ACTION_BUILD)
        self.assertFalse(replay.active)
        self.assertEqual(replay.stop_calls, 1)
        self.assertFalse(replay.actions._action_lock.locked())

    def test_failed_probe_query_releases_lock(self):
        replay = self.replay()
        replay.query_action_status = lambda: False
        with self.assertRaisesRegex(RuntimeError, 'failed to query'):
            replay.actions.begin(ACTION_BUILD)
        self.assertFalse(replay.active)
        self.assertEqual(replay.stop_calls, 1)
        self.assertFalse(replay.actions._action_lock.locked())


class BuildSessionCleanupTests(unittest.TestCase):
    def environment(self):
        from Strategy.flows import ActionSpec
        from Strategy.flows.factory import ActionEnvironment
        from Strategy.plans import StrategyPlan
        from tests.test_functional_operations import operation_fixture

        fixture, _ = operation_fixture('building-1')
        robot, context = fixture.robot, fixture.context
        context.heading_zero_deg = 37.0
        context.anchor = 'build_approach'
        session = SimpleNamespace(wait_chassis_ready=Mock(), wait_done=Mock(),
                                  check=Mock(), close=Mock(), abort=Mock())
        robot.actions.begin = Mock(return_value=session)
        robot.chassis.monitor_action = lambda check: nullcontext()
        robot.begin_cube_camera_pose_change = Mock(return_value=10)
        robot.end_cube_camera_pose_change = Mock(return_value=True)
        from Strategy.transition_config import TransitionConfig
        from Strategy.transition_switches import TransitionSwitches
        env = ActionEnvironment(robot, context, transition_config=TransitionConfig(
            switches=TransitionSwitches(overrides={'build-return': True})))
        env.run_route = Mock()
        plan = StrategyPlan('build-cleanup', (
            ActionSpec('build', 'build', 'building-1', requires_anchor='build_approach'),
            ActionSpec('navigate', 'return', 'building-1',
                       {'route': 'build_return', 'after_build': True},
                       requires_anchor='build_approach', ends_at='ground_collection'),
        ), entry_anchor='build_approach', needs_heading_zero=True)
        return env, session, env.compile(plan)

    @staticmethod
    def run_flow(env, compiled):
        from Strategy.execution import ExecutionRuntime
        runtime = ExecutionRuntime(
            guard=env.context.check_active, stop=env.stop,
            emergency_stop=env.robot.transport.emergency_stop,
            close=env.context.close)
        try:
            return runtime.run(compiled)
        finally:
            # ActionEnvironment owns mechanism lifetimes; the runner performs
            # this same cleanup after the flat execution runtime has stopped.
            env.abort()

    def test_route_failure_survives_session_cleanup_failure(self):
        env, session, compiled = self.environment()
        session.abort.side_effect = OSError('cleanup failed')
        env.run_route.side_effect = ValueError('original route failure')
        with self.assertRaisesRegex(ValueError, 'original route failure'):
            self.run_flow(env, compiled)
        session.abort.assert_called_once()
        session.close.assert_not_called()
        env.robot.end_cube_camera_pose_change.assert_not_called()
        env.robot.transport.emergency_stop.assert_called_once()

    def test_rejected_pose_restore_faults_instead_of_finishing_flow(self):
        env, session, compiled = self.environment()
        env.robot.end_cube_camera_pose_change.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'camera pose restoration'):
            self.run_flow(env, compiled)
        session.wait_done.assert_called_once()
        session.abort.assert_called_once()
        env.robot.transport.emergency_stop.assert_called_once()
        self.assertEqual(env.context.anchor, 'build_approach')


if __name__ == '__main__':
    unittest.main()
