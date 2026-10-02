"""Measured blind-motion envelopes and milestone gates, without robot hardware."""

from dataclasses import replace
import math
import unittest
from unittest.mock import Mock

from Strategy.execution.blind import (
    BlindMotionFault, BlindMotionProfile, BlindSample, BlindStatus, run_blind_transition,
)


def profile(**overrides):
    # Synthetic test values only; no corresponding production profile is enabled.
    values = dict(direction=1, cruise_speed_mm_s=20.0, acceleration_mm_s2=100.0,
                  braking_mm_s2=100.0, max_distance_mm=1000.0, max_duration_s=1.0,
                  braking_margin_mm=5.0, telemetry_timeout_s=0.1, frame_timeout_s=0.1,
                  tick_s=0.02, max_command_delay_s=0.005, validated=True)
    values.update(overrides)
    return BlindMotionProfile(**values)


class Plant:
    """An exact-speed synthetic encoder, not a robot dynamics validation."""
    def __init__(self, *, lift_at=2, reset_at=6, frame_at=7):
        self.now = 100.0
        self.position = self.velocity = 0.0
        self.commands = []
        self.index = 0
        self.reset_time = None
        self.lift_at, self.reset_at, self.frame_at = lift_at, reset_at, frame_at
        self.stop = Mock()
        self.handoff = Mock()

    def sample(self):
        index = self.index
        self.index += 1
        reset = index >= self.reset_at
        if reset and self.reset_time is None:
            self.reset_time = self.now
        return BlindSample(17, 3, self.now, self.position, self.velocity,
                           index >= self.lift_at, reset, self.reset_time,
                           self.now if index >= self.frame_at else None,
                           3 if index >= self.frame_at else None)

    def command(self, value):
        self.commands.append((self.now, value, self.index - 1))
        self.velocity = value

    def sleep(self, seconds):
        self.position += self.velocity * seconds
        self.now += seconds

    def run(self, **overrides):
        kwargs = dict(token=17, epoch=3, read_sample=self.sample,
                      command_speed=self.command, stop=self.stop, guard=lambda: None,
                      handoff=self.handoff, clock=lambda: self.now, sleep=self.sleep)
        config = overrides.pop('profile', profile())
        kwargs.update(overrides)
        return run_blind_transition(config, **kwargs)


class BlindTransitionTests(unittest.TestCase):
    def test_near_zero_feedback_does_not_abort_a_valid_grab_handoff(self):
        plant=Plant(lift_at=0,reset_at=3,frame_at=4)
        def sample():
            value=plant.sample()
            return replace(value,displacement_mm=-.001,velocity_mm_s=-.1) if plant.index<=2 else value
        result=plant.run(read_sample=sample)
        self.assertEqual(result.status,BlindStatus.HANDED_OFF)
        plant.stop.assert_not_called()

    def test_bounded_reverse_feedback_returns_to_visual_adjustment(self):
        plant=Plant(lift_at=0,reset_at=3,frame_at=4)
        result=plant.run(read_sample=lambda:replace(plant.sample(),displacement_mm=-1.5,velocity_mm_s=-20))
        self.assertEqual(result.status.value,'direction_recovery')
        plant.stop.assert_called_once()
        plant.handoff.assert_not_called()

    def test_tiny_idle_feedback_before_lift_keeps_commands_zero(self):
        plant=Plant(lift_at=2,reset_at=4,frame_at=5)
        def sample():
            value=plant.sample()
            return replace(value,displacement_mm=.001,velocity_mm_s=-.1) if not value.lifted else value
        result=plant.run(read_sample=sample)
        self.assertEqual(result.status,BlindStatus.HANDED_OFF)
        self.assertTrue(all(v==0 for _,v,i in plant.commands if i<2))

    def test_reverse_motion_outside_recovery_envelope_remains_a_fault(self):
        for feedback in ({'displacement_mm': -6.}, {'velocity_mm_s': -200.}):
            with self.subTest(feedback=feedback):
                plant = Plant(lift_at=0)
                with self.assertRaisesRegex(BlindMotionFault, 'recovery margin'):
                    plant.run(read_sample=lambda: replace(plant.sample(), **feedback))
                plant.stop.assert_called_once()
                plant.handoff.assert_not_called()

    def test_lift_and_camera_gates_preserve_nonzero_velocity_on_handoff(self):
        plant = Plant()
        result = plant.run()
        self.assertEqual(result.status, BlindStatus.HANDED_OFF)
        self.assertTrue(all(speed == 0 for _, speed, index in plant.commands if index < 2))
        self.assertTrue(any(speed > 0 for _, speed, index in plant.commands if 2 <= index < 6))
        self.assertGreater(result.velocity_mm_s, 0)
        plant.handoff.assert_called_once_with(result.velocity_mm_s)
        plant.stop.assert_not_called()
        self.assertGreater(plant.commands[-1][1], 0)

    def test_frame_before_reset_and_wrong_epoch_cannot_handoff(self):
        for invalid_frame in ('old', 'wrong_epoch'):
            with self.subTest(invalid_frame=invalid_frame):
                plant = Plant(lift_at=0, reset_at=1, frame_at=0)
                def sample():
                    value = plant.sample()
                    return replace(value, frame_captured_at_s=99.0) if invalid_frame == 'old' else replace(
                        value, frame_epoch=2)
                result = plant.run(read_sample=sample)
                self.assertEqual(result.status, BlindStatus.CAMERA_UNAVAILABLE)
                plant.handoff.assert_not_called()
                plant.stop.assert_called_once()

    def test_frame_at_exact_reset_time_waits_for_a_new_frame(self):
        plant = Plant(lift_at=0, reset_at=2, frame_at=2)
        result = plant.run()
        self.assertEqual(result.status, BlindStatus.HANDED_OFF)
        self.assertEqual(plant.index, 4)

    def test_camera_ready_without_handoff_stops(self):
        plant = Plant()
        result = plant.run(handoff=None)
        self.assertEqual(result.status, BlindStatus.CAMERA_READY_STOPPED)
        plant.stop.assert_called_once()

    def test_duration_includes_waiting_for_lift_and_never_moves_early(self):
        plant = Plant(lift_at=1000, reset_at=1000, frame_at=1000)
        result = plant.run()
        self.assertEqual(result.status, BlindStatus.CAMERA_UNAVAILABLE)
        self.assertGreaterEqual(result.duration_s, 1.0)
        self.assertTrue(all(speed == 0 for _, speed, _ in plant.commands))
        plant.stop.assert_called_once()

    def test_signed_reverse_motion_uses_same_envelope_and_handoff(self):
        plant = Plant()
        result = plant.run(profile=profile(direction=-1))
        self.assertEqual(result.status, BlindStatus.HANDED_OFF)
        self.assertLess(result.displacement_mm, 0)
        self.assertLess(result.velocity_mm_s, 0)
        self.assertTrue(all(speed <= 0 for _, speed, _ in plant.commands))

    def test_acceleration_and_time_braking_are_bounded_between_commands(self):
        plant = Plant(lift_at=0, reset_at=1000, frame_at=1000)
        config = profile()
        result = plant.run(profile=config)
        self.assertEqual(result.status, BlindStatus.CAMERA_UNAVAILABLE)
        self.assertTrue(any(later[1] < earlier[1] for earlier, later in
                            zip(plant.commands, plant.commands[1:])))
        for earlier, later in zip(plant.commands, plant.commands[1:]):
            delta = later[1] - earlier[1]
            dt = later[0] - earlier[0]
            self.assertLessEqual(delta, config.acceleration_mm_s2 * dt + 1e-8)
            self.assertGreaterEqual(delta, -config.braking_mm_s2 * dt - 1e-8)
        self.assertLessEqual(max(speed for _, speed, _ in plant.commands), config.cruise_speed_mm_s)
        plant.stop.assert_called_once()

    def test_measured_progress_and_speed_enforce_braking_envelope(self):
        plant = Plant(lift_at=0, reset_at=1000, frame_at=1000)
        def sample():
            return replace(plant.sample(), displacement_mm=92.0, velocity_mm_s=30.0)
        result = plant.run(profile=profile(max_distance_mm=100.0), read_sample=sample)
        self.assertEqual(result.status, BlindStatus.BOUND_REACHED)
        self.assertEqual(result.reason, 'measured_braking_envelope')
        self.assertEqual(result.velocity_mm_s, 30.0)
        self.assertEqual(plant.commands, [])
        plant.stop.assert_called_once()

    def test_command_cap_reserves_next_tick_sample_age_and_command_delay(self):
        plant = Plant(lift_at=0, reset_at=1, frame_at=2)
        config = profile(max_distance_mm=10.0, braking_margin_mm=1.0,
                         cruise_speed_mm_s=100.0, acceleration_mm_s2=10000.0)
        def sample():
            return replace(plant.sample(), timestamp_s=plant.now - 0.03)
        plant.run(profile=config, read_sample=sample)
        first_speed = plant.commands[0][1]
        needed = (first_speed ** 2 / (2 * config.braking_mm_s2)
                  + first_speed * (0.03 + config.tick_s + config.max_command_delay_s))
        self.assertLessEqual(needed, config.max_distance_mm - config.braking_margin_mm + 1e-8)

    def test_stale_or_wrong_token_telemetry_is_hard_fault(self):
        variants = (
            {'token': 18}, {'epoch': 4}, {'timestamp_s': 99.0},
            {'timestamp_s': 101.0}, {'velocity_mm_s': math.nan},
            {'displacement_mm': -10.0},
        )
        for overrides in variants:
            with self.subTest(overrides=overrides):
                plant = Plant()
                with self.assertRaises(BlindMotionFault):
                    plant.run(read_sample=lambda: replace(plant.sample(), **overrides))
                plant.stop.assert_called_once()
                plant.handoff.assert_not_called()

    def test_lift_regression_is_hard_fault(self):
        plant = Plant(lift_at=0, reset_at=1000, frame_at=1000)
        def sample():
            value = plant.sample()
            return replace(value, lifted=False) if plant.index > 2 else value
        with self.assertRaisesRegex(BlindMotionFault, 'lift.*regressed'):
            plant.run(read_sample=sample)
        plant.stop.assert_called_once()

    def test_reset_regression_or_reset_time_change_is_hard_fault(self):
        for change in ('state', 'time'):
            with self.subTest(change=change):
                plant = Plant(lift_at=0, reset_at=1, frame_at=1000)
                def sample():
                    value = plant.sample()
                    if plant.index < 3:
                        return value
                    return replace(value, arm_reset=False) if change == 'state' else replace(
                        value, arm_reset_at_s=plant.now)
                with self.assertRaises(BlindMotionFault):
                    plant.run(read_sample=sample)
                plant.stop.assert_called_once()

    def test_cancel_during_handoff_stops_before_reporting_success(self):
        plant = Plant()
        active = [True]
        with self.assertRaises(BlindMotionFault):
            plant.run(guard=lambda: active[0],
                      handoff=lambda _speed: active.__setitem__(0, False))
        plant.stop.assert_called_once()

    def test_guard_cancel_and_failed_commands_stop_without_handoff(self):
        for failure in ('guard', 'command', 'late_command'):
            with self.subTest(failure=failure):
                plant = Plant(lift_at=0)
                kwargs = {}
                if failure == 'guard':
                    kwargs['guard'] = lambda: plant.index < 3
                elif failure == 'command':
                    kwargs['command_speed'] = lambda _speed: False
                else:
                    kwargs['command_speed'] = lambda _speed: plant.sleep(0.01)
                with self.assertRaises(BlindMotionFault):
                    plant.run(**kwargs)
                plant.stop.assert_called_once()
                plant.handoff.assert_not_called()

    def test_handoff_failure_stops_and_preserves_original_error(self):
        plant = Plant()
        def failure(_velocity):
            raise ValueError('controller rejected state')
        plant.stop.side_effect = IOError('serial unavailable')
        with self.assertRaisesRegex(ValueError, 'controller rejected state'):
            plant.run(handoff=failure)
        plant.stop.assert_called_once()

    def test_profile_validation_and_unvalidated_profiles_never_command_motion(self):
        for overrides in ({'direction': 0}, {'direction': True}, {'tick_s': 0},
                          {'cruise_speed_mm_s': math.inf}, {'acceleration_mm_s2': math.nan},
                          {'braking_margin_mm': 1000}, {'max_command_delay_s': -1},
                          {'max_duration_s': 10 ** 1000}, {'max_command_delay_s': 1.0},
                          {'tick_s': 0.2}, {'validated': 1},
                          {'feedback_position_tolerance_mm': -1},
                          {'feedback_speed_tolerance_mm_s': math.nan}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                profile(**overrides)
        plant = Plant()
        with self.assertRaisesRegex(ValueError, 'field-validated'):
            plant.run(profile=profile(validated=False))
        self.assertEqual(plant.commands, [])
        plant.stop.assert_not_called()


if __name__ == '__main__':
    unittest.main()
