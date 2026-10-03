"""Moving pickup acquisition against a deterministic encoder/camera plant."""

from dataclasses import replace
from types import SimpleNamespace
import unittest

from control.chassis import MECANUM_RPM_PER_CM_S
from Strategy.controllers import RobotController
from Strategy.execution.blind import BlindMotionFault
from Strategy.execution.pickup_motion import (
    PickupMotionProfile, acquire_after_blind, measured_lateral_speed,
)
from Strategy.orange_search import OrangeSearchRecovery
from Strategy.settings import GroundCollectionConfig, HighlandCollectionConfig


def profile(**changes):
    # Synthetic plant values only. Production requires explicit field validation.
    values = dict(acceleration_mm_s2=200.0, braking_mm_s2=200.0,
                  max_distance_mm=500.0, max_duration_s=6.0,
                  braking_margin_mm=5.0, telemetry_timeout_s=0.15,
                  frame_timeout_s=0.1, tick_s=0.02,
                  max_command_delay_s=0.01, settled_speed_mm_s=1.0,
                  validated=True)
    values.update(changes)
    return PickupMotionProfile(**values)


class Plant:
    def __init__(self, *, position=100.0, speed=40.0, target=150.0,
                 config=None):
        self.now = 100.0
        self.position, self.speed, self.target = position, speed, target
        self.commands = []
        self.epoch, self.capture_delay, self.freeze_frame = 7, 0.0, False
        self.freeze_uptime = self.reject_command = False
        self.frame_override = None
        self.yaw = 30.0
        self.guard_calls = 0
        self.telem = SimpleNamespace(uptime_ms=1000, yaw_deg=self.yaw,
                                    motors=[SimpleNamespace(speed_rpm=0) for _ in range(4)])
        self.transport = SimpleNamespace(connected=True, emergency_stop_generation=0)
        self.chassis = SimpleNamespace(lateral_distance_scale=1.0,
                                      set_speeds=self.command,
                                      mecanum_rpm=lambda vx, vy, yaw: (vx, vy, yaw))
        self.robot = self
        self.config = config or replace(
            GroundCollectionConfig(), search_speed_mm_s=60.0, align_kp=3.0,
            align_min_speed_mm_s=10.0, align_creep_min_speed_mm_s=5.0,
            align_max_speed_mm_s=60.0, align_confirm_frames=2,
            orange_fine_min_x_mm=-3.0, orange_fine_max_x_mm=3.0,
            align_target_x_mm=0.0)
        self._orange_recovery = OrangeSearchRecovery(origin=(0, 0, 0, 0))
        self._search_position_mm = position
        self._alignment_speed = RobotController._alignment_speed
        self.update_speed()

    def update_speed(self):
        value = self.speed / 10.0 * MECANUM_RPM_PER_CM_S
        for motor, sign in zip(self.telem.motors, (1, 1, -1, -1)):
            motor.speed_rpm = sign * value

    @property
    def vision_result(self):
        target_x = getattr(self.config, 'orange_align_target_x_mm', self.config.align_target_x_mm)
        blocks = [] if self.target is None else [SimpleNamespace(
            x=self.target - self.position + target_x, y=0.0, z=150.0,
            color_name='orange', confidence=90.0)]
        frame = SimpleNamespace(timestamp=self.now + 1000000.0,
                                captured_monotonic=(100.0 if self.freeze_frame else self.now)
                                - self.capture_delay, pose_epoch=self.epoch,
                                all_blocks=blocks)
        return self.frame_override(frame) if self.frame_override else frame

    def command(self, values):
        stop = list(values) == [0, 0, 0, 0]
        self.speed = 0.0 if stop else values[1] * 10.0
        self.commands.append((self.now, self.speed, 0.0 if stop else values[2], stop))
        self.update_speed()
        return not self.reject_command

    def _check_active(self):
        pass

    def _measure_lateral_displacement_mm(self, origin):
        if origin != (0, 0, 0, 0):
            raise AssertionError('lost collection origin')
        return self.position

    def guard(self):
        self.guard_calls += 1

    def sleep(self, seconds):
        self.position += self.speed * seconds
        self.now += seconds
        if not self.freeze_uptime:
            self.telem.uptime_ms += round(seconds * 1000)

    def run(self, **changes):
        options = dict(initial_speed_mm_s=self.speed, guard=self.guard,
                       pose_epoch=7, arm_reset_at_s=99.0,
                       phase_origin=(0, 0, 0, 0), profile=profile(),
                       clock=lambda: self.now, sleep=self.sleep)
        options.update(changes)
        return acquire_after_blind(self, **options)


class PickupMotionTests(unittest.TestCase):
    def test_nonzero_handoff_search_and_alignment_share_velocity_controller(self):
        plant = Plant()
        self.assertTrue(plant.run())
        self.assertEqual(plant.commands[0][1], 40.0)
        self.assertFalse(plant.commands[0][3])
        self.assertTrue(plant.commands[-1][3])
        self.assertLess(abs(plant.position - plant.target), 3.1)
        self.assertGreaterEqual(plant._alignment_valid_frames, 2)
        self.assertTrue(all(not command[3] for command in plant.commands[:-1]))
        for previous, current in zip(plant.commands, plant.commands[1:-1]):
            self.assertLessEqual(abs(current[1] - previous[1]),
                                 200.0 * (current[0] - previous[0]) + 1e-8)

    def test_uses_highland_orange_calibration_not_purple_target(self):
        cfg = replace(HighlandCollectionConfig(), align_target_x_mm=999.0,
                      orange_align_target_x_mm=15.0,
                      orange_fine_min_x_mm=12.0, orange_fine_max_x_mm=18.0,
                      align_min_speed_mm_s=10.0, align_creep_min_speed_mm_s=5.0)
        plant = Plant(config=cfg)
        self.assertTrue(plant.run())
        self.assertTrue(12 <= plant._last_alignment_block.x <= 18)

    def test_preserves_heading_anchor_while_laterally_moving(self):
        plant = Plant(target=None)
        plant.run(profile=profile(max_duration_s=0.2), heading_yaw_deg=35.0)
        self.assertGreater(plant.commands[0][2], 0.0)
        self.assertLessEqual(plant.commands[0][2], plant.config.delivery_heading_max_yaw_deg_s)

    def test_old_pose_and_capture_frames_never_authorize_grab(self):
        for invalid in ('epoch', 'captured_before_reset', 'old_capture_new_processing'):
            with self.subTest(invalid=invalid):
                plant = Plant(target=100.0, speed=0.0)
                if invalid == 'epoch':
                    plant.epoch = 6
                else:
                    plant.capture_delay = 2.0 if invalid == 'captured_before_reset' else 0.3
                self.assertFalse(plant.run())
                self.assertEqual(plant._alignment_valid_frames, 0)
                self.assertTrue(plant.commands[-1][3])

    def test_repeated_frame_cannot_confirm_alignment(self):
        plant = Plant(target=100.0, speed=0.0)
        plant.freeze_frame = True
        self.assertFalse(plant.run())
        self.assertEqual(plant._alignment_valid_frames, 0)

    def test_locked_target_loss_stops_without_switching_to_unrelated_block(self):
        plant = Plant(target=200.0)
        def drop(frame):
            if plant.now > 100.08:
                frame.all_blocks[0].x += 500.0
            return frame
        plant.frame_override = drop
        self.assertFalse(plant.run())
        self.assertLess(plant.now, 101.0)
        self.assertLess(plant._last_alignment_block.x, 200.0)
        self.assertTrue(plant.commands[-1][3])

    def test_cumulative_collection_origin_includes_preceding_blind_move(self):
        plant = Plant(position=487.0, speed=40.0, target=None)
        recovery = plant._orange_recovery
        recovery.armed, recovery.retry_after_mm, recovery.search_elapsed_s = False, 499.0, 0.4
        self.assertFalse(plant.run())
        self.assertLess(plant.position, 500.0)
        self.assertGreaterEqual(plant._search_position_mm, 487.0)
        self.assertIs(plant._orange_recovery, recovery)
        self.assertFalse(recovery.armed)
        self.assertEqual(recovery.retry_after_mm, 499.0)

    def test_fresh_empty_frames_search_within_encoder_budget(self):
        plant = Plant(position=440.0, target=None)
        self.assertFalse(plant.run())
        self.assertLess(plant.position, 500.0)
        self.assertGreater(plant.position, 440.0)
        self.assertTrue(plant.commands[-1][3])

    def test_phase_search_time_budget_is_shared(self):
        plant = Plant(target=None)
        cfg = plant.config
        plant._orange_recovery.search_elapsed_s = (
            cfg.search_max_distance_mm / cfg.search_speed_mm_s
            + cfg.orange_edge_timeout_s + cfg.target_cube_count * cfg.vision_observe_s)
        self.assertFalse(plant.run())
        self.assertEqual(len(plant.commands), 1)
        self.assertTrue(plant.commands[0][3])

    def test_stale_telemetry_and_link_fault_are_hard_errors(self):
        for mode in ('uptime', 'link', 'emergency'):
            with self.subTest(mode=mode):
                plant = Plant(target=None)
                if mode == 'uptime':
                    plant.freeze_uptime = True
                elif mode == 'link':
                    plant.transport.connected = False
                else:
                    def guard():
                        if plant.now > 100.03:
                            plant.transport.emergency_stop_generation = 1
                    plant.guard = guard
                with self.assertRaises(BlindMotionFault):
                    plant.run()
                self.assertTrue(plant.commands[-1][3])

    def test_cancel_and_rejected_command_stop_and_raise(self):
        for mode in ('cancel', 'command'):
            with self.subTest(mode=mode):
                plant = Plant(target=None)
                if mode == 'cancel':
                    plant.guard = lambda: False
                else:
                    plant.reject_command = True
                with self.assertRaises(BlindMotionFault):
                    plant.run()
                self.assertTrue(plant.commands[-1][3])

    def test_callback_can_reuse_blind_controller_without_builtin_heading_command(self):
        plant = Plant()
        commands = []
        def command(speed):
            commands.append(speed)
            return plant.command((0.0, speed / 10.0, 0.0))
        self.assertTrue(plant.run(command_speed=command))
        self.assertEqual(commands[0], 40.0)

    def test_unvalidated_and_malformed_profiles_are_rejected(self):
        plant = Plant()
        with self.assertRaises(ValueError):
            plant.run(profile=profile(validated=False))
        for key, value in [('braking_mm_s2', 0), ('tick_s', float('nan')),
                           ('validated', 1), ('max_distance_mm', 4)]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                profile(**{key: value})
        self.assertEqual(plant.commands, [])

    def test_measured_speed_uses_wheel_feedback_and_calibrated_scale(self):
        plant = Plant(speed=50.0)
        plant.chassis.lateral_distance_scale = 2.0
        self.assertAlmostEqual(measured_lateral_speed(plant), 25.0)

    def test_reverse_initial_velocity_can_reacquire_without_crossing_phase_origin(self):
        plant = Plant(speed=-20.0, target=85.0)
        self.assertTrue(plant.run())
        self.assertEqual(plant.commands[0][1], -20.0)
        self.assertGreaterEqual(plant.position, 0.0)

    def test_slow_command_and_reboot_are_hard_failures(self):
        for mode in ('delay', 'reboot'):
            with self.subTest(mode=mode):
                plant = Plant(target=None)
                def command(speed):
                    if mode == 'delay':
                        plant.now += 0.02
                    else:
                        plant.telem.uptime_ms = 10
                    return plant.command((0, speed / 10, 0))
                with self.assertRaises(BlindMotionFault):
                    plant.run(command_speed=command)
                self.assertTrue(plant.commands[-1][3])

    def test_nonfinite_blocks_cannot_trigger_alignment(self):
        plant = Plant(target=100.0, speed=0.0)
        def invalid(frame):
            frame.all_blocks[0].x = float('nan')
            return frame
        plant.frame_override = invalid
        self.assertFalse(plant.run(profile=profile(max_duration_s=0.5)))
        self.assertEqual(plant._alignment_valid_frames, 0)

    def test_capture_at_reset_time_is_not_a_restored_camera_frame(self):
        plant = Plant(target=100.0, speed=0.0)
        plant.freeze_frame = True
        self.assertFalse(plant.run(arm_reset_at_s=100.0))
        self.assertEqual(plant._alignment_valid_frames, 0)


if __name__ == '__main__':
    unittest.main()
