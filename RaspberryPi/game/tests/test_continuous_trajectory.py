"""Simulation-only tuning values; these are not validated field route parameters."""
from dataclasses import replace
import math
import unittest
from unittest.mock import patch

from control.chassis import (Chassis, COUNTS_PER_CM, LATERAL_DISTANCE_SCALE,
                             MECANUM_RPM_PER_CM_S, TURN_DEG_S_TO_RPM)
from control.trajectory import (BodyVelocity, CubicRoute, PoseSample,
                                TrajectoryProfile, Waypoint, follow_trajectory)
from protocol.commands import MotorFeedback, TelemBatch


def profile(**changes):
    return replace(TrajectoryProfile(
        validated=True, max_speed_mm_s=500, max_accel_mm_s2=800,
        max_yaw_speed_deg_s=80, max_yaw_accel_deg_s2=120, max_wheel_rpm=2000,
        position_gain_s=3, yaw_gain_s=3, position_tolerance_mm=5,
        yaw_tolerance_deg=1, settle_speed_mm_s=5, settle_yaw_speed_deg_s=1,
        settle_time_s=.1, max_tracking_error_mm=150, max_tracking_yaw_error_deg=30,
        timeout_s=30, control_period_s=.02, max_telemetry_age_s=.3), **changes)


def rpm(velocity):
    return Chassis.mecanum_rpm(velocity.vx_mm_s/10,
                              velocity.vy_mm_s*LATERAL_DISTANCE_SCALE/10,
                              -velocity.yaw_deg_s)


class Plant:
    """First-order body-velocity lag, encoder integration and wrapped IMU."""
    connected = True
    emergency_stop_generation = 0

    def __init__(self, initial=BodyVelocity()):
        self.now = 100.0
        self.x = self.y = self.yaw = 0.0
        self.measured = self.command = initial
        self.commands = []
        self.poses = []
        self.steps = 0
        self.stops = 0
        self.fault = lambda: None
        self.stale = False
        self.received_at = self.now
        self.wheels = [0.0] * 4
        self.chassis = Chassis(self)

    def pose(self):
        return PoseSample(self.x, self.y, self.yaw, self.measured, self.received_at)

    def send(self, command):
        self.command = command
        self.commands.append((self.now, command))
        return True

    def set_chassis_speed(self, rpms):
        tr, tl, bl, br = rpms
        return self.send(BodyVelocity(
            (-tr+tl+bl-br)*10/(4*MECANUM_RPM_PER_CM_S),
            (tr+tl-bl-br)*10/(4*MECANUM_RPM_PER_CM_S*LATERAL_DISTANCE_SCALE),
            sum(rpms)/(4*TURN_DEG_S_TO_RPM)))

    def emergency_stop(self):
        self.stops += 1
        self.emergency_stop_generation += 1

    def publish(self):
        self.chassis.update_telem(TelemBatch(
            motors=[MotorFeedback(cumulative_pos=round(p), speed_rpm=round(v))
                    for p, v in zip(self.wheels, rpm(self.measured))],
            yaw_deg=(-self.yaw+180) % 360-180, uptime_ms=round((self.now-90)*1000)))

    def sleep(self, dt):
        self.now += dt
        self.steps += 1
        alpha = 1-math.exp(-dt/.08)
        self.measured = BodyVelocity(*(a+alpha*(b-a) for a, b in zip(
            (self.measured.vx_mm_s, self.measured.vy_mm_s, self.measured.yaw_deg_s),
            (self.command.vx_mm_s, self.command.vy_mm_s, self.command.yaw_deg_s))))
        angle = math.radians(self.yaw+self.measured.yaw_deg_s*dt/2)
        self.x += (math.cos(angle)*self.measured.vx_mm_s
                   - math.sin(angle)*self.measured.vy_mm_s)*dt
        self.y += (math.sin(angle)*self.measured.vx_mm_s
                   + math.cos(angle)*self.measured.vy_mm_s)*dt
        self.yaw += self.measured.yaw_deg_s*dt
        self.wheels = [p+v*10/MECANUM_RPM_PER_CM_S*COUNTS_PER_CM/10*dt
                       for p, v in zip(self.wheels, rpm(self.measured))]
        self.poses.append((self.x, self.y, self.yaw, self.measured))
        if not self.stale:
            self.received_at = self.now
            self.publish()
        self.fault()

    def run(self, points, settings=None, **kwargs):
        return follow_trajectory(points, settings or profile(), read_pose=self.pose,
                                 send_velocity=self.send, wheel_rpm=rpm,
                                 check=lambda: self.fault(), emergency_stop=self.emergency_stop,
                                 clock=lambda: self.now, sleep=self.sleep, **kwargs)


CURVE = (Waypoint(0, 0, 0), Waypoint(400, 150, 45), Waypoint(600, 600, 180))


class ContinuousTrajectoryTests(unittest.TestCase):
    def test_curve_tracks_turn_and_translation_with_nonzero_intermediate_speed(self):
        plant = Plant()
        result = plant.run(CURVE)
        self.assertLess(math.hypot(plant.x-600, plant.y-600), 5)
        self.assertAlmostEqual(plant.yaw, 180, delta=1)
        self.assertEqual(result.points_passed, 2)
        near = [v for x, y, _, v in plant.poses if math.hypot(x-400, y-150) < 10]
        self.assertTrue(near)
        self.assertTrue(all(math.hypot(v.vx_mm_s, v.vy_mm_s) > 40 for v in near))
        self.assertTrue(any(abs(v.vx_mm_s)>10 and abs(v.vy_mm_s)>10
                            and abs(v.yaw_deg_s)>10 for _, v in plant.commands))
        self.assertEqual(plant.commands[-1][1], BodyVelocity())
        self.assertEqual(plant.stops, 0)

    def test_cubic_tangents_are_continuous_and_yaw_turn_does_not_wrap(self):
        route = CubicRoute(CURVE, profile())
        t = route.knots[1]
        p, v = route.sample(t)
        _, before = route.sample(t-1e-6)
        _, after = route.sample(t+1e-6)
        self.assertEqual(p, CURVE[1])
        self.assertGreater(math.hypot(v.vx_mm_s, v.vy_mm_s), 0)
        self.assertAlmostEqual(before.vx_mm_s, after.vx_mm_s, delta=.001)
        self.assertAlmostEqual(before.vy_mm_s, after.vy_mm_s, delta=.001)
        self.assertAlmostEqual(before.yaw_deg_s, after.yaw_deg_s, delta=.001)
        plant = Plant()
        plant.run((Waypoint(0, 0, 0), Waypoint(0, 0, 270)))
        self.assertAlmostEqual(plant.yaw, 270, delta=1)
        # Lag may require a small final reverse correction after an overshoot.
        self.assertTrue(all(v.yaw_deg_s >= -1e-6
                            for _, _, yaw, v in plant.poses if yaw < 250))

    def test_measured_nonzero_entry_is_used_without_forced_stop(self):
        plant = Plant(BodyVelocity(150, 0, 0))
        plant.run((Waypoint(0, 0, 0), Waypoint(800, 0, 0)))
        self.assertEqual(plant.commands[0][1], BodyVelocity(150, 0, 0))
        self.assertTrue(all(v.vx_mm_s > 100 for _, v in plant.commands[:10]))

    def test_acceleration_and_all_four_wheels_obey_profile(self):
        settings = profile(max_wheel_rpm=1400, max_tracking_error_mm=300,
                           max_tracking_yaw_error_deg=60)
        plant = Plant()
        plant.run(CURVE, settings)
        previous = BodyVelocity()
        for _, v in plant.commands:
            self.assertLessEqual(max(map(abs, rpm(v))), settings.max_wheel_rpm+1e-6)
            self.assertLessEqual(math.hypot(v.vx_mm_s-previous.vx_mm_s,
                                           v.vy_mm_s-previous.vy_mm_s),
                                 settings.max_accel_mm_s2*settings.control_period_s+1e-6)
            self.assertLessEqual(abs(v.yaw_deg_s-previous.yaw_deg_s),
                                 settings.max_yaw_accel_deg_s2*settings.control_period_s+1e-6)
            previous = v

    def test_stale_telemetry_aborts_instead_of_auto_resuming(self):
        plant = Plant()
        plant.stale = True
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            plant.run(CURVE)
        self.assertEqual(plant.stops, 1)
        self.assertLess(plant.now-100, .5)

    def test_guard_and_stop_failures_preserve_original_failure(self):
        plant = Plant()
        def guard():
            if plant.steps >= 5:
                raise RuntimeError('cancelled route')
        def broken_stop():
            raise OSError('stop send failed')
        plant.fault, plant.emergency_stop = guard, broken_stop
        with self.assertRaisesRegex(RuntimeError, 'cancelled route'):
            plant.run(CURVE)
        self.assertEqual(len(plant.commands), 5)

    def test_send_failure_emergency_stops(self):
        plant = Plant()
        plant.send = lambda velocity: False
        with self.assertRaisesRegex(RuntimeError, 'send failed'):
            plant.run(CURVE)
        self.assertEqual(plant.stops, 1)

    def test_invalid_config_or_points_never_send(self):
        bad_points = [(Waypoint(0, 0, 0),), (Waypoint(0, 0, 0),)*2,
                      (Waypoint(0, 0, 0), Waypoint(float('nan'), 0, 0)),
                      (Waypoint(1, 0, 0), Waypoint(10, 0, 0))]
        for points in bad_points:
            with self.subTest(points=points):
                plant = Plant()
                with self.assertRaises(ValueError):
                    plant.run(points)
                self.assertEqual(plant.commands, [])
        for changes in ({'validated': False}, {'max_accel_mm_s2': 0},
                        {'max_speed_mm_s': float('inf')}, {'control_period_s': .1},
                        {'timeout_s': .1}, {'max_wheel_rpm': 40000}):
            with self.subTest(changes=changes):
                plant = Plant()
                with self.assertRaises(ValueError):
                    plant.run(CURVE, profile(**changes))
                self.assertEqual(plant.commands, [])

    def test_overlimit_entry_and_stalled_route_abort(self):
        plant = Plant(BodyVelocity(10000, 0, 0))
        with self.assertRaisesRegex(RuntimeError, 'entry velocity'):
            plant.run(CURVE)
        self.assertEqual(plant.commands, [])
        plant = Plant()
        plant.pose = lambda: PoseSample(0, 0, 0, BodyVelocity(), plant.now)
        with self.assertRaisesRegex(RuntimeError, 'tracking error'):
            plant.run(CURVE)

    def test_timeout_is_a_failure_and_cannot_be_accepted_as_near_arrival(self):
        plant = Plant()
        plant.pose = lambda: PoseSample(0, 0, 0, BodyVelocity(), plant.now)
        with self.assertRaisesRegex(TimeoutError, 'did not settle'):
            plant.run((Waypoint(0, 0, 0), Waypoint(100, 0, 0)),
                      profile(timeout_s=2, max_tracking_error_mm=500))
        self.assertEqual(plant.stops, 1)

    def test_cached_final_frame_cannot_confirm_measured_stop(self):
        plant = Plant()
        cached = None
        def read():
            nonlocal cached
            current = plant.pose()
            if (cached is None and plant.now > 100.5 and abs(plant.x-100) < 5
                    and plant.command == BodyVelocity()):
                cached = current
            return cached or current
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            follow_trajectory((Waypoint(0, 0, 0), Waypoint(100, 0, 0)), profile(),
                              read_pose=read, send_velocity=plant.send, wheel_rpm=rpm,
                              check=lambda: None, emergency_stop=plant.emergency_stop,
                              clock=lambda: plant.now, sleep=plant.sleep)
        self.assertEqual(plant.stops, 1)

    def test_shared_body_velocity_and_encoder_helpers_use_current_motor_order(self):
        plant = Plant(BodyVelocity(80, -140, 35))
        with patch('control.chassis.time.monotonic', side_effect=lambda: plant.now):
            plant.publish()
            measured = plant.chassis.measured_body_velocity()
            self.assertAlmostEqual(measured.vx_mm_s, 80, delta=.3)
            self.assertAlmostEqual(measured.vy_mm_s, -140, delta=.3)
            self.assertAlmostEqual(measured.yaw_deg_s, 35, delta=.1)
            initial = plant.chassis.capture_motor_positions()
            counts = round(100*COUNTS_PER_CM/10)
            for motor, sign in zip(plant.chassis.telem.motors, (-1, 1, 1, -1)):
                motor.cumulative_pos += counts*sign
            self.assertAlmostEqual(plant.chassis.forward_displacement_mm(initial),
                                   100, delta=.01)
            self.assertAlmostEqual(plant.chassis.lateral_displacement_mm(initial), 0)

    def test_adapter_encoder_layout_lateral_scale_and_imu_wrap(self):
        plant = Plant()
        points = (Waypoint(0, 0, 0), Waypoint(300, 400, 180), Waypoint(500, 600, 270))
        with patch('control.chassis.time.monotonic', side_effect=lambda: plant.now), \
                patch('control.chassis.time.sleep', side_effect=plant.sleep):
            plant.publish()
            result = plant.chassis.follow_trajectory(points, profile(), check=lambda: None)
        self.assertAlmostEqual(plant.x, 500, delta=6)
        self.assertAlmostEqual(plant.y, 600, delta=6)
        self.assertAlmostEqual(plant.yaw, 270, delta=1.1)
        self.assertAlmostEqual(result.final_pose.yaw_deg, 270, delta=1.1)
        self.assertEqual(plant.stops, 0)

    def test_adapter_disconnect_reboot_and_emergency_stop_are_terminal(self):
        for fault in ('disconnect', 'reboot', 'emergency', 'cancel', 'stale'):
            with self.subTest(fault=fault):
                plant = Plant()
                def inject():
                    if plant.steps != 5:
                        return
                    if fault == 'disconnect':
                        plant.connected = False
                    elif fault == 'reboot':
                        plant.chassis.telem.uptime_ms = 1
                    elif fault == 'emergency':
                        plant.emergency_stop_generation += 1
                    elif fault == 'cancel':
                        raise RuntimeError('cancelled')
                    else:
                        plant.stale = True
                plant.fault = inject
                with patch('control.chassis.time.monotonic', side_effect=lambda: plant.now), \
                        patch('control.chassis.time.sleep', side_effect=plant.sleep):
                    plant.publish()
                    with self.assertRaises(RuntimeError):
                        plant.chassis.follow_trajectory(CURVE, profile(), check=inject)
                self.assertEqual(plant.stops, 1)
                self.assertLess(plant.steps, 30)


if __name__ == '__main__':
    unittest.main()
