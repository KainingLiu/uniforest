"""Wall-contact feedback replay; never connects to hardware."""

import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from Strategy.controllers import RobotController
from Strategy.settings import GroundCollectionConfig
from Strategy.context import ExecutionContext
from Strategy.wall_controller import StallConfirmation
from tests.test_strategy_composition import robot_fixture


def feedback(speeds, currents):
    return SimpleNamespace(motors=[
        SimpleNamespace(speed_rpm=speed, torque_current=current)
        for speed, current in zip(speeds, currents)])


class WallReplay:
    def __init__(self, sample=lambda now: (60, 3000)):
        self.now = 0.0
        self.sample = sample
        self.freeze = False
        self.after_sleep = lambda: None
        self.robot = robot_fixture()
        self.robot.chassis.mecanum_rpm = Mock(return_value=[700, -700, -700, 700])
        self.task = RobotController(self.robot, context=ExecutionContext(self.robot))
        self.refresh()

    def refresh(self):
        speed, current = self.sample(self.now)
        self.robot.telem.motors = feedback([speed] * 4, [current] * 4).motors
        self.robot.telem.uptime_ms += 10

    def sleep(self, duration):
        self.now += duration
        self.after_sleep()
        if not self.freeze:
            self.refresh()

    @contextlib.contextmanager
    def clock(self):
        self.output = io.StringIO()
        with patch('Strategy.controllers.time.monotonic', side_effect=lambda: self.now), \
             patch('Strategy.controllers.time.sleep', side_effect=self.sleep), \
             contextlib.redirect_stdout(self.output):
            yield


class WallContactTests(unittest.TestCase):
    def test_relaxed_speed_still_requires_load_and_original_wheel_count(self):
        cfg = GroundCollectionConfig()
        for direction, speeds, currents, expected in (
            ('forward', [700, 700, -80, 80], [0, 0, -2500, 2500], True),
            ('forward', [0, 0, 81, 80], [3000] * 4, False),
            ('forward', [0] * 4, [3000, 3000, 2499, 3000], False),
            ('left', [-80, 80, 80, 700], [-2500, 2500, 2500, 0], True),
            ('left', [80, 80, 81, 700], [3000] * 4, False),
            ('left', [0] * 4, [2499] * 4, False),
            ('left', [700] * 4, [4000] * 4, False),
        ):
            with self.subTest(direction=direction, speeds=speeds, currents=currents):
                self.assertEqual(RobotController._stall_sample(
                    feedback(speeds, currents), cfg, direction), expected)

    def test_short_dropout_pauses_time_and_cannot_confirm_on_failed_sample(self):
        tracker = StallConfirmation()
        samples = [(0.0, True), (.10, True), (.12, False), (.16, False),
                   (.18, True), (.30, True), (.39, True)]
        results = [tracker.update(stalled, now, .3, dropout_s=.08)
                   for now, stalled in samples]
        self.assertEqual(results, [False] * 6 + [True])
        self.assertFalse(tracker.update(False, .40, .3, dropout_s=.08))

    def test_long_dropout_resets_even_if_only_recovery_frame_sees_the_gap(self):
        for extra_failed_frame in (False, True):
            with self.subTest(extra_failed_frame=extra_failed_frame):
                tracker = StallConfirmation()
                for now, stalled in [(0, True), (.15, True), (.17, False)]:
                    self.assertFalse(tracker.update(stalled, now, .3, dropout_s=.08))
                if extra_failed_frame:
                    self.assertFalse(tracker.update(False, .26, .3, dropout_s=.08))
                self.assertFalse(tracker.update(True, .27, .3, dropout_s=.08))
                self.assertFalse(tracker.update(True, .43, .3, dropout_s=.08))
                self.assertTrue(tracker.update(True, .58, .3, dropout_s=.08))

    def test_sparse_contact_spikes_do_not_accumulate_as_continuous_contact(self):
        tracker = StallConfirmation()
        for i in range(100):
            self.assertFalse(tracker.update(i % 2 == 0, i * .02, .3, dropout_s=.08))

    def test_default_tracker_retains_strict_reset(self):
        tracker = StallConfirmation()
        for now, stalled in [(0, True), (.20, True), (.21, False), (.22, True), (.4, True)]:
            self.assertFalse(tracker.update(stalled, now, .3))
        self.assertTrue(tracker.update(True, .53, .3))

    def test_wall_loop_confirms_light_slip_and_brief_spike_before_timeout(self):
        replay = WallReplay(lambda now: (130 if .60 <= now < .65 else 60, 3000))
        with replay.clock():
            replay.task._drive_until_wall(direction='left', timeout_is_success=False)
        self.assertGreaterEqual(replay.now, .8)  # Startup guard plus actual good time.
        self.assertLess(replay.now, 1.1)
        self.assertIn('confirmed', replay.output.getvalue())
        self.assertNotIn('timed out', replay.output.getvalue())
        replay.robot.chassis.set_speeds.assert_called_with([0, 0, 0, 0])

    def test_normal_motion_or_no_load_cannot_be_mistaken_for_wall_contact(self):
        for sample in (lambda now: (700, 3000), lambda now: (0, 1000)):
            replay = WallReplay(sample)
            with replay.clock(), self.assertRaisesRegex(RuntimeError, 'not detected'):
                replay.task._drive_until_wall(direction='left', timeout_is_success=False)
            self.assertGreaterEqual(replay.now, 4)
            replay.robot.chassis.set_speeds.assert_called_with([0, 0, 0, 0])

    def test_wall_loop_stops_on_repeated_telemetry_disconnect_or_emergency_stop(self):
        for fault in ('stale', 'disconnect', 'stop'):
            with self.subTest(fault=fault):
                replay = WallReplay()
                def fail():
                    if replay.now < .60:
                        return
                    if fault == 'stale': replay.freeze = True
                    if fault == 'disconnect': replay.robot.transport.connected = False
                    if fault == 'stop': replay.robot.transport.emergency_stop()
                replay.after_sleep = fail
                with replay.clock(), self.assertRaises(RuntimeError):
                    replay.task._drive_until_wall(direction='left')
                self.assertLess(replay.now, 1.1)
                self.assertNotIn('confirmed', replay.output.getvalue())
                replay.robot.chassis.set_speeds.assert_called_with([0, 0, 0, 0])

    def test_grab_press_uses_same_dropout_tolerance_and_stops_on_stale_feedback(self):
        for stale in (False, True):
            with self.subTest(stale=stale):
                replay = WallReplay(lambda now: (130 if .19 <= now < .24 else 60, 3000))
                if stale:
                    replay.after_sleep = lambda: setattr(replay, 'freeze', replay.now >= .16)
                def grab(*, parallel_step):
                    while not parallel_step():
                        replay.sleep(.05)
                with replay.clock():
                    if stale:
                        with self.assertRaisesRegex(RuntimeError, 'telemetry lost'):
                            replay.task._grab_with_wall_press(grab)
                    else:
                        replay.task._grab_with_wall_press(grab)
                self.assertLess(replay.now, .6)
                if not stale:
                    self.assertGreaterEqual(replay.now, .35)
                    self.assertIn('press confirmed', replay.output.getvalue())
                replay.robot.chassis.set_speeds.assert_called_with([0, 0, 0, 0])


if __name__ == '__main__':
    unittest.main()
