import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from control.actions import Actions, ActionCancelled
from protocol.commands import (
    ActionStatus, ACTION_GRAP1, ACTION_GRAP2, ACTION_GRAP3, ACTION_IDLE,
    ACTION_RUNNING, ACTION_DONE, ACTION_CANCELLED, ACTION_REJECTED, ACTION_TIMEOUT,
)


class FakeClock:
    now = 10.0
    def monotonic(self):
        return self.now
    def wait(self, ms):
        self.now += ms / 1000


class FakeTransport:
    def __init__(self, clock):
        self.clock = clock
        self.sample = None
        self.token = 0
        self.action_id = 0
        self.state = ACTION_IDLE
        self.started = []
        self.stops = 0
        self.polls = 0
        self.respond = True
        self.freeze = False
        self.finish_after = 3
        self.final = ACTION_DONE
        self.send_ok = True

    def query_action_status(self):
        self.polls += 1
        if self.respond:
            if self.token and self.polls >= self.finish_after:
                self.state = self.final
            uptime = 100 if self.freeze else int(self.clock.now * 1000)
            self.sample = (ActionStatus(self.token, self.action_id,
                                       self.state, 1, uptime), self.clock.now)
        return self.send_ok

    def get_action_status(self):
        return self.sample

    def start_action(self, token, action_id, test_mode):
        self.token, self.action_id, self.state = token, action_id, ACTION_RUNNING
        self.started.append((action_id, test_mode))
        return self.send_ok

    def emergency_stop(self):
        self.stops += 1


class GrapClientTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.transport = FakeTransport(self.clock)
        self.actions = Actions(Mock(), SimpleNamespace(_t=self.transport))
        self.actions._wait = self.clock.wait
        self.patch = patch('control.actions.time.monotonic', self.clock.monotonic)
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def test_all_grabs_send_one_board_request(self):
        for method, action_id in [('grap1', ACTION_GRAP1), ('grap2', ACTION_GRAP2),
                                  ('grap3', ACTION_GRAP3)]:
            with self.subTest(method=method):
                self.transport.polls = 0
                getattr(self.actions, method)(test_mode=True)
                self.assertEqual(self.transport.started[-1], (action_id, True))
        self.assertEqual(len(self.transport.started), 3)
        self.actions.servo.set_angle.assert_not_called()
        self.assertEqual(self.transport.stops, 0)

    def test_old_firmware_never_receives_action(self):
        self.transport.respond = False
        with self.assertRaisesRegex(RuntimeError, 'flash new firmware'):
            self.actions.grap3()
        self.assertEqual(self.transport.started, [])
        self.assertEqual(self.transport.stops, 1)

    def test_cached_capability_does_not_allow_motion(self):
        self.transport.sample = (ActionStatus(0,0,0,0,1), 9.0)
        self.test_old_firmware_never_receives_action()

    def test_board_cancel_timeout_and_rejection_stop_host(self):
        for state in (ACTION_CANCELLED, ACTION_TIMEOUT, ACTION_REJECTED):
            with self.subTest(state=state):
                self.transport.final = state
                self.transport.state = ACTION_IDLE
                self.transport.token = 0
                self.transport.polls = 0
                with self.assertRaises(RuntimeError):
                    self.actions.grap3()
        self.assertEqual(self.transport.stops, 3)

    def test_cancelled_before_start_has_no_motion(self):
        event = threading.Event()
        event.set()
        self.actions.set_cancel_event(event)
        with self.assertRaises(ActionCancelled):
            self.actions.grap1()
        self.assertEqual(self.transport.started, [])
        self.assertEqual(self.transport.stops, 1)

    def test_cancelled_during_wait_stops_board(self):
        event = threading.Event()
        self.actions.set_cancel_event(event)
        def wait(ms):
            self.clock.wait(ms)
            event.set()
        self.actions._wait = wait
        with self.assertRaises(ActionCancelled):
            self.actions.grap2()
        self.assertEqual(len(self.transport.started), 1)
        self.assertEqual(self.transport.stops, 1)

    def test_frozen_uptime_is_not_live_telemetry(self):
        self.transport.freeze = True
        self.transport.finish_after = 10000
        with self.assertRaisesRegex(RuntimeError, 'clock stopped'):
            self.actions.grap3()
        self.assertEqual(self.transport.stops, 1)

    def test_lost_status_stops_action(self):
        original = self.transport.query_action_status
        def query():
            if self.transport.polls >= 2:
                self.transport.respond = False
            return original()
        self.transport.query_action_status = query
        with self.assertRaisesRegex(RuntimeError, 'stale|lost'):
            self.actions.grap3()
        self.assertEqual(self.transport.stops, 1)

    def test_wrong_token_cannot_complete_new_action(self):
        original = self.transport.start_action
        def start(*args):
            original(*args)
            self.transport.token = 0
            return True
        self.transport.start_action = start
        with self.assertRaisesRegex(RuntimeError, 'did not accept'):
            self.actions.grap3()

    def test_busy_board_is_not_overwritten(self):
        self.transport.state = ACTION_RUNNING
        with self.assertRaisesRegex(RuntimeError, 'already executing'):
            self.actions.grap1()
        self.assertFalse(self.transport.started)

    def test_send_failure_stops_action(self):
        self.transport.send_ok = False
        with self.assertRaises(RuntimeError):
            self.actions.build()
        self.assertEqual(self.transport.stops, 1)
