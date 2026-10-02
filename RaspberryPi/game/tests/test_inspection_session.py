"""Cargo observation/restore sessions use one guarded mechanism owner."""

import contextlib
import io
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from control.carried_cube_inspection import (
    begin_carried_inspection, inspect_carried_cubes,
)
from protocol.commands import ACTION_DONE


class InspectionReplay:
    def __init__(self, *, count=3, camera='live'):
        self.now = 100.0
        self.count, self.camera = count, camera
        self.events = []
        self._running = True
        self.cancelled = self.stale = False
        self.epoch = 0
        self.diagnostics = SimpleNamespace(write=Mock())
        self.transport = SimpleNamespace(
            connected=True, emergency_stop_generation=0,
            query_action_status=lambda: True,
            get_action_status=lambda: (
                SimpleNamespace(state=ACTION_DONE, uptime_ms=0), self.now),
            set_servo_angle_checked=self.servo,
            emergency_stop=Mock(side_effect=self.stop),
        )
        self.actions = SimpleNamespace(
            _action_lock=threading.Lock(), _check_cancelled=self.check_cancelled)
        self.chassis = SimpleNamespace(
            set_speeds=Mock(return_value=True), monitor_action=self.monitor)
        self.reset_vision_filter = Mock(side_effect=lambda **kw: self.events.append(('reset', kw)))
        self.clock = SimpleNamespace(monotonic=lambda: self.now, sleep=self.sleep)

    def stop(self):
        self.transport.emergency_stop_generation += 1
        self.events.append('emergency_stop')
        return True

    def check_cancelled(self):
        if self.cancelled:
            raise RuntimeError('inspection cancelled')

    def servo(self, index, angle, check):
        check()
        self.events.append(('servo', index, angle, self.now))

    def sleep(self, seconds):
        self.now += seconds

    def inspection_link_snapshot(self):
        telem = SimpleNamespace(
            uptime_ms=round(self.now * 1000), stepper_busy=0,
            motors=[SimpleNamespace(speed_rpm=0)] * 4)
        return telem, self.now - (.2 if self.stale else 0), self.now, self.epoch

    @property
    def cube_raw_frame(self):
        if self.camera == 'none':
            return None
        if self.camera == 'after_pose' and any(
                isinstance(e, tuple) and e[0] == 'servo' for e in self.events):
            return None
        return object(), self.now

    @contextlib.contextmanager
    def monitor(self, check):
        check()
        yield
        check()

    @contextlib.contextmanager
    def patched(self):
        with patch('control.carried_cube_inspection.time', self.clock), \
             patch('control.carried_cube_inspection.observe', return_value=({}, None)), \
             patch('control.carried_cube_inspection.classify', return_value={'count': self.count}), \
             contextlib.redirect_stdout(io.StringIO()):
            yield

    @property
    def servo_commands(self):
        return [(e[1], e[2]) for e in self.events
                if isinstance(e, tuple) and e[0] == 'servo']


class InspectionSessionTests(unittest.TestCase):
    def test_count_returns_before_restore_and_route_holds_mechanism_lock(self):
        replay = InspectionReplay()
        with replay.patched():
            session = begin_carried_inspection(replay)
            self.assertEqual(replay.servo_commands, [])
            self.assertEqual(session.inspect(), 3)
            self.assertEqual(session.count, 3)
            self.assertFalse(session.restored)
            self.assertEqual(replay.servo_commands, [(0, 37.2), (1, 120), (1, 90)])
            restore_started = replay.now
            self.assertFalse(session.check_restore())
            self.assertEqual(replay.now, restore_started)
            with self.assertRaisesRegex(RuntimeError, 'another mechanical action'):
                begin_carried_inspection(replay)
            replay.events.append('route_start')
            replay.sleep(.199)
            self.assertFalse(session.check_restore())
            replay.sleep(.002)
            self.assertTrue(session.check_restore())
            # The next grab remains excluded until the route's owner closes.
            self.assertTrue(replay.actions._action_lock.locked())
            replay.events.append('route_end')
            session.finish_restore()
            session.close()
        self.assertEqual(replay.servo_commands[-1], (0, 97.2))
        self.assertFalse(replay.actions._action_lock.locked())
        replay.reset_vision_filter.assert_called_once_with(after_inspection=True)
        replay.transport.emergency_stop.assert_not_called()

    def test_legacy_partial_count_finishes_restore_before_refill_and_skips_route(self):
        for count in (0, 1, 2):
            with self.subTest(count=count):
                replay = InspectionReplay(count=count)
                followup = Mock()
                with replay.patched():
                    self.assertEqual(inspect_carried_cubes(replay, chassis_followup=followup), count)
                followup.assert_not_called()
                self.assertEqual(replay.servo_commands[-2:], [(1, 90), (0, 97.2)])
                self.assertFalse(replay.actions._action_lock.locked())
                replay.transport.emergency_stop.assert_not_called()

    def test_full_and_unknown_legacy_counts_overlap_original_restore_timing(self):
        for count in (3, None):
            with self.subTest(count=count):
                replay = InspectionReplay(count=count)
                def followup():
                    self.assertEqual(replay.servo_commands[-1], (1, 90))
                    self.assertTrue(replay.actions._action_lock.locked())
                    replay.sleep(.250)
                with replay.patched():
                    self.assertEqual(inspect_carried_cubes(replay, chassis_followup=followup), count)
                self.assertEqual(replay.servo_commands[-1], (0, 97.2))
                self.assertFalse(replay.actions._action_lock.locked())

    def test_visual_unknown_before_and_after_pose_preserves_restore_semantics(self):
        for camera in ('none', 'after_pose'):
            with self.subTest(camera=camera):
                replay = InspectionReplay(camera=camera)
                with replay.patched():
                    session = begin_carried_inspection(replay, allow_visual_failure=True)
                    self.assertIsNone(session.inspect())
                    self.assertEqual(session.restored, camera == 'none')
                    session.finish_restore()
                    session.close()
                expected = [] if camera == 'none' else [(0, 37.2), (1, 120), (1, 90), (0, 97.2)]
                self.assertEqual(replay.servo_commands, expected)
                replay.transport.emergency_stop.assert_not_called()

    def test_cancel_stale_reconnect_or_stop_prevents_remaining_servo_commands(self):
        for fault in ('cancel', 'stale', 'reconnect', 'stop'):
            with self.subTest(fault=fault):
                replay = InspectionReplay()
                with replay.patched():
                    session = begin_carried_inspection(replay)
                    session.inspect()
                    before = replay.servo_commands
                    if fault == 'cancel': replay.cancelled = True
                    if fault == 'stale': replay.stale = True
                    if fault == 'reconnect': replay.epoch += 1
                    if fault == 'stop': replay.transport.emergency_stop_generation += 1
                    replay.sleep(.250)
                    with self.assertRaises(RuntimeError):
                        session.check_restore()
                    session.abort()
                    session.close()
                    self.assertTrue(session.closed)
                self.assertEqual(replay.servo_commands, before)
                self.assertFalse(replay.actions._action_lock.locked())
                replay.transport.emergency_stop.assert_called_once()

    def test_fault_during_restoration_releases_lock_without_masking_original(self):
        replay = InspectionReplay()
        with replay.patched():
            session = begin_carried_inspection(replay)
            session.inspect()
            replay.transport.set_servo_angle_checked = Mock(side_effect=RuntimeError('servo ack lost'))
            replay.transport.emergency_stop = Mock(side_effect=RuntimeError('stop link lost'))
            replay.sleep(.250)
            with self.assertRaisesRegex(RuntimeError, 'servo ack lost'):
                session.finish_restore()
            self.assertRegex(str(session.cleanup_error), 'stop link lost')
            self.assertFalse(session.abort())
        self.assertFalse(replay.actions._action_lock.locked())
        replay.transport.emergency_stop.assert_called_once()

    def test_abandoned_session_stops_once_and_cannot_resume(self):
        replay = InspectionReplay()
        with replay.patched():
            session = begin_carried_inspection(replay)
            session.inspect()
            before = replay.servo_commands
            session.close()
            session.close()
            replay.sleep(1)
            with self.assertRaisesRegex(RuntimeError, 'closed'):
                session.finish_restore()
        self.assertEqual(replay.servo_commands, before)
        self.assertFalse(replay.actions._action_lock.locked())
        replay.transport.emergency_stop.assert_called_once()

    def test_begin_rejects_stale_link_and_leaves_lock_available(self):
        replay = InspectionReplay()
        replay.stale = True
        with replay.patched(), self.assertRaises(RuntimeError):
            begin_carried_inspection(replay)
        self.assertFalse(replay.actions._action_lock.locked())
        self.assertEqual(replay.servo_commands, [])

    def test_route_failure_never_sends_delayed_restore_servo(self):
        replay = InspectionReplay()
        def failed_route():
            raise RuntimeError('route cancelled')
        with replay.patched(), self.assertRaisesRegex(RuntimeError, 'route cancelled'):
            inspect_carried_cubes(replay, chassis_followup=failed_route)
        self.assertEqual(replay.servo_commands, [(0, 37.2), (1, 120), (1, 90)])
        self.assertFalse(replay.actions._action_lock.locked())
        replay.transport.emergency_stop.assert_called_once()

    def test_cancel_between_begin_and_context_entry_releases_owner(self):
        replay = InspectionReplay()
        with replay.patched():
            session = begin_carried_inspection(replay)
            replay.cancelled = True
            with self.assertRaisesRegex(RuntimeError, 'cancelled'):
                with session:
                    self.fail('cancelled inspection entered its body')
        self.assertFalse(replay.actions._action_lock.locked())
        self.assertEqual(replay.servo_commands, [])
        replay.transport.emergency_stop.assert_called_once()

    def test_invalid_count_type_or_range_never_starts_restore_or_departure(self):
        for count in (True, False, 1.0, '3', -1, 4):
            with self.subTest(count=count):
                replay = InspectionReplay(count=count)
                with replay.patched():
                    session = begin_carried_inspection(replay)
                    with self.assertRaisesRegex(RuntimeError, 'invalid carried cube count'):
                        session.inspect()
                self.assertEqual(replay.servo_commands, [(0, 37.2), (1, 120)])
                self.assertTrue(session.closed)
                self.assertFalse(replay.actions._action_lock.locked())
                replay.transport.emergency_stop.assert_called_once()


if __name__ == '__main__':
    unittest.main()
