"""Replay specified encoder samples through the real loop; no physics simulation."""
import unittest
from threading import Event
from unittest.mock import patch

from control.chassis import Chassis, COUNTS_PER_CM, LATERAL_DISTANCE_SCALE
from protocol.commands import MotorFeedback, TelemBatch


class RouteReplay:
    connected = True
    emergency_stop_generation = 0

    def __init__(self, samples, *, signs=(-1, 1, 1, -1), scale=1.0):
        self.samples, self.signs, self.scale = samples, signs, scale
        self.now, self.step = 100.0, 0
        self.commands = []
        self.cancel = Event()
        self.after_sample = lambda: None
        self.chassis = Chassis(self)

    def set_chassis_speed(self, rpm):
        self.commands.append((self.step, tuple(rpm)))
        return True

    def emergency_stop(self):
        self.emergency_stop_generation += 1

    def sample(self, distance, speeds, yaw=0.0):
        counts = round(distance * self.scale * COUNTS_PER_CM / 10.0)
        self.chassis.update_telem(TelemBatch(
            motors=[MotorFeedback(cumulative_pos=counts * sign, speed_rpm=speed)
                    for sign, speed in zip(self.signs, speeds)], yaw_deg=yaw))

    def sleep(self, duration):
        self.now += duration
        item = self.samples[min(self.step, len(self.samples) - 1)]
        self.step += 1
        if item is not None:
            self.sample(*item)
        self.after_sample()

    def run(self, *, route_mode=True, distance=100.0):
        with patch('control.chassis.time.monotonic', side_effect=lambda: self.now):
            self.sample(0, [0] * 4)
            return self.chassis._move_linear(
                int(distance * self.scale * COUNTS_PER_CM / 10.0),
                list(self.signs), self.chassis._mm_s_to_rpm(400), self,
                lambda: self.chassis.telem, distance, sleep_fn=self.sleep,
                cancel_event=self.cancel, distance_scale=self.scale,
                hold_ms=0, accel_ms=300, route_mode=route_mode)


class ChassisRouteTests(unittest.TestCase):
    def test_arrival_brakes_all_wheels_but_waits_for_slow_wheel_feedback(self):
        replay = RouteReplay([(94, [40, 40, 40, 70], 1.0)] * 5
                             + [(94, [0] * 4, 1.0)])
        result = replay.run()
        self.assertFalse(result.timed_out)
        self.assertGreaterEqual(replay.step, 9)
        self.assertTrue(all(rpm == (0, 0, 0, 0)
                            for step, rpm in replay.commands if step >= 1))
        self.assertAlmostEqual(result.estimated_chassis_distance_mm, 94, delta=.02)

    def test_drift_beyond_window_resumes_signed_correction(self):
        replay = RouteReplay([(94, [0] * 4), (112, [60] * 4), (106, [0] * 4)])
        result = replay.run()
        command = next(rpm for step, rpm in replay.commands if step == 2)
        self.assertTrue(all(rpm * sign < 0 for rpm, sign in zip(command, replay.signs)))
        self.assertFalse(result.timed_out)
        self.assertAlmostEqual(result.estimated_chassis_distance_mm, 106, delta=.02)

    def test_forward_backward_and_calibrated_lateral_share_chassis_window(self):
        for signs, scale in [((-1, 1, 1, -1), 1), ((1, -1, -1, 1), 1),
                             ((1, 1, -1, -1), LATERAL_DISTANCE_SCALE),
                             ((-1, -1, 1, 1), LATERAL_DISTANCE_SCALE)]:
            with self.subTest(signs=signs):
                replay = RouteReplay([(91, [0] * 4), (92.2, [0] * 4)],
                                     signs=signs, scale=scale)
                result = replay.run()
                self.assertFalse(result.timed_out)
                self.assertTrue(any(rpm * sign > 0 for rpm, sign in zip(
                    next(rpm for step, rpm in replay.commands if step == 1), signs)))
                self.assertTrue(all(rpm == (0, 0, 0, 0)
                                    for step, rpm in replay.commands if step >= 2))

    def test_distance_braking_already_applies_during_acceleration(self):
        samples = [(0, [100] * 4)] * 6 + [(80, [100] * 4), (100, [0] * 4)]
        route, direct = RouteReplay(samples), RouteReplay(samples)
        route.run()
        direct.run(route_mode=False)
        route_rpm = next(rpm for step, rpm in route.commands if step == 7)
        direct_rpm = next(rpm for step, rpm in direct.commands if step == 7)
        self.assertLess(max(map(abs, route_rpm)), max(map(abs, direct_rpm)) / 2)

    def test_direct_positioning_keeps_its_original_tighter_window(self):
        result = RouteReplay([(94, [0] * 4)]).run(route_mode=False)
        self.assertTrue(result.timed_out)

    def test_cached_arrival_frame_cannot_complete_then_stale_telemetry_aborts(self):
        replay = RouteReplay([(94, [0] * 4), None])
        with self.assertRaisesRegex(RuntimeError, 'telemetry fault'):
            replay.run()
        self.assertGreater(replay.now, 100.5)
        self.assertEqual(replay.emergency_stop_generation, 1)

    def test_stop_disconnect_cancel_and_send_failure_cannot_report_success(self):
        for fault in ('stop', 'disconnect', 'cancel', 'send'):
            with self.subTest(fault=fault):
                replay = RouteReplay([(94, [0] * 4)])
                def inject():
                    if fault == 'stop': replay.emergency_stop()
                    elif fault == 'disconnect': replay.connected = False
                    elif fault == 'cancel': replay.cancel.set()
                    else: replay.set_chassis_speed = lambda rpm: False
                replay.after_sample = inject
                if fault == 'cancel':
                    self.assertTrue(replay.run().cancelled)
                    self.assertEqual(replay.commands[-1][1], (0, 0, 0, 0))
                else:
                    with self.assertRaises(RuntimeError): replay.run()

    def test_tiny_route_cannot_finish_at_its_origin(self):
        replay = RouteReplay([(0, [0] * 4)] * 8 + [(4, [0] * 4)])
        result = replay.run(distance=4)
        self.assertFalse(result.timed_out)
        self.assertGreaterEqual(replay.step, 9)
        self.assertGreater(result.estimated_chassis_distance_mm, 3.9)


if __name__ == '__main__':
    unittest.main()
