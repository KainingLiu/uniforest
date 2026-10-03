"""Check actual Tag control outputs against specified poses, without hardware."""
import contextlib
from dataclasses import replace
import io
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from control.chassis import Chassis
from Strategy.controllers import RobotController
from Strategy.settings import (GroundCollectionConfig, GroundCollection2Config,
                               BuildingConfig, Building2Config)


class TagReplay:
    def __init__(self, samples, config=GroundCollectionConfig()):
        self.samples = samples  # distance mm, lateral mm, gyro error deg
        self.now, self.index = 100.0, -1
        self.commands, self.velocities = [], []
        self.command_events = []
        self.robot = SimpleNamespace(field_pose=None, telem=SimpleNamespace(yaw_deg=0))
        self.robot.chassis = SimpleNamespace(
            set_speeds=self.record_speeds,
            mecanum_rpm=self.mecanum)
        self.program = RobotController(self.robot)
        self.program._heading_zero_deg = 0
        self.program.config = replace(config, delivery_tag_align_timeout_s=1.0)

    def record_speeds(self, rpm):
        self.commands.append(tuple(rpm))
        self.command_events.append((self.index, tuple(rpm)))

    def mecanum(self, vx, vy, wz):
        self.velocities.append((self.index, vx, vy, wz))
        return Chassis.mecanum_rpm(vx, vy, wz)

    def sleep(self, seconds):
        self.now += seconds
        self.index += 1
        item = self.samples[min(self.index, len(self.samples) - 1)]
        if item is None:  # Same cached frame; must not advance confirmation.
            return
        if isinstance(item, (int, float)):  # New IMU reading, unchanged camera.
            self.robot.telem.yaw_deg = -180 - item
            return
        if item == 'lost':
            self.robot.field_pose = None
            return
        distance, lateral, error = item
        self.robot.field_pose = SimpleNamespace(timestamp=self.now, tag_solutions=[
            SimpleNamespace(tag_id=6, distance_m=distance/1000,
                            lateral_m=lateral/1000, score=0)])
        self.robot.telem.yaw_deg = -180 - error

    def run(self, *, independent_heading=False):
        clock = SimpleNamespace(monotonic=lambda: self.now, time=lambda: self.now,
                                sleep=self.sleep)
        self.log = io.StringIO()
        with patch('Strategy.controllers.time', clock), contextlib.redirect_stdout(self.log):
            self.program._align_delivery_tag(
                tag_id=6, target_distance_mm=425, heading_target_cw_deg=180,
                distance_tolerance_mm=8, lateral_tolerance_mm=8,
                fine_align_enabled=False, stop_axes_in_tolerance=True,
                independent_heading=independent_heading)


class TagAlignmentTests(unittest.TestCase):
    def test_recalibration_is_shared_with_the_next_action(self):
        robot = SimpleNamespace(telem=SimpleNamespace(yaw_deg=42.0))
        context = SimpleNamespace(heading_zero_deg=37.5)
        pickup = RobotController(robot, context=context, operation_name='pickup')
        self.assertEqual(pickup._heading_zero_deg, 37.5)
        with contextlib.redirect_stdout(io.StringIO()):
            pickup._recalibrate_heading_zero(reference_cw_deg=180.0)
        alignment = RobotController(robot, BuildingConfig(), context=context,
                                    operation_name='align-building')
        self.assertEqual(alignment._heading_zero_deg, -138.0)
        self.assertEqual(alignment._heading_error(180.0), 0.0)

    def test_four_action_profiles_wait_for_heading_then_confirm_all_axes(self):
        for config in (GroundCollectionConfig(), GroundCollection2Config(), BuildingConfig(), Building2Config()):
            with self.subTest(config=type(config).__name__):
                replay = TagReplay([(425, 0, 15)] * 3 + [(425, 0, 2)], config)
                replay.run()
                self.assertEqual(replay.index, 6)
                self.assertEqual(len(replay.velocities), 3)
                self.assertTrue(all(vx == vy == 0 and wz > 0
                                    for _, vx, vy, wz in replay.velocities))
                self.assertEqual(replay.commands[-1], (0, 0, 0, 0))

    def test_heading_in_tolerance_stays_stopped_while_translation_continues(self):
        for error in (2.0, .1, -.1):
            with self.subTest(error=error):
                replay = TagReplay([(445, 0, error)] * 3 + [(425, 0, error)])
                replay.run()
                self.assertTrue(all(wz == 0 for _, _, _, wz in replay.velocities))
                self.assertTrue(any(vx != 0 for _, vx, _, _ in replay.velocities))
                self.assertEqual(replay.commands[-1], (0, 0, 0, 0))

    def test_heading_restarts_after_two_outside_frames(self):
        replay = TagReplay([(445, 0, 2), (445, 0, 5), (445, 0, 5),
                            (445, 0, 2), (425, 0, 2)])
        replay.run()
        rotation = {index: wz for index, _, _, wz in replay.velocities}
        self.assertEqual(rotation[0], 0)
        self.assertEqual(rotation[1], 0)
        self.assertGreater(rotation[2], 0)
        self.assertEqual(rotation[3], 0)

    def test_position_must_remain_valid_through_confirmation(self):
        replay = TagReplay([(425, 0, 1)] * 2 + [(450, 25, 1)] * 3
                           + [(425, 0, 1)])
        replay.run()
        self.assertEqual(replay.index, 9)
        self.assertTrue(any(vx != 0 and vy != 0 for _, vx, vy, _ in replay.velocities))

    def test_repeated_frames_do_not_complete_and_finally_stops(self):
        replay = TagReplay([(425, 0, 1), None])
        with self.assertRaises(RuntimeError): replay.run()
        self.assertEqual(len(replay.velocities), 0)
        self.assertEqual(replay.commands[-1], (0, 0, 0, 0))

    def test_default_coarse_mode_still_requires_heading(self):
        replay = TagReplay([(425, 0, 15)])
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            replay.run()
        self.assertEqual(replay.commands[-1], (0, 0, 0, 0))


class IndependentHeadingTests(unittest.TestCase):
    def test_all_variants_keep_correcting_during_four_frame_confirmation(self):
        for config in (GroundCollectionConfig(), GroundCollection2Config(), BuildingConfig(), Building2Config()):
            with self.subTest(config=type(config).__name__):
                replay = TagReplay([(425, 0, 2)], config)
                replay.run(independent_heading=True)
                self.assertEqual(replay.log.getvalue().count('aligned '), 4)
                self.assertTrue(all(vx == vy == 0 and wz > 0
                                    for _, vx, vy, wz in replay.velocities))
                self.assertEqual(replay.commands[-1], (0, 0, 0, 0))

    def test_p_gain_deadband_and_output_cap_without_minimum_speed(self):
        for error, expected in ((1, 6), (-1, -6), (5, 30), (15, 45),
                                (-15, -45), (.5, 0), (-.5, 0)):
            with self.subTest(error=error):
                replay = TagReplay([(445, 0, error)])
                with self.assertRaisesRegex(RuntimeError, 'timed out'):
                    replay.run(independent_heading=True)
                self.assertAlmostEqual(replay.velocities[-1][3], expected)
                self.assertTrue(all(abs(wz) <= 45 for _, _, _, wz in replay.velocities))
                rotation = [0] + [wz for _, _, _, wz in replay.velocities]
                self.assertTrue(all(abs(b - a) <= 1.800001
                                    for a, b in zip(rotation, rotation[1:])))

    def test_fresh_gyro_corrects_cached_frame_and_deadband_stops_immediately(self):
        replay = TagReplay([(445, 0, 5), -5.0, -5.0, -5.0, .2, None])
        with self.assertRaises(RuntimeError):
            replay.run(independent_heading=True)
        velocities = {i: (vx, vy, wz) for i, vx, vy, wz in replay.velocities}
        self.assertGreater(velocities[0][2], 0)
        self.assertLess(velocities[2][2], 0)
        self.assertEqual(velocities[4][2], 0)
        self.assertEqual(velocities[0][:2], velocities[4][:2])
        self.assertEqual(replay.log.getvalue().count('aligned '), 0)

    def test_between_frame_heading_excursion_restarts_confirmation(self):
        replay = TagReplay([(425, 0, 1), 5.0, (425, 0, 1)])
        replay.run(independent_heading=True)
        # First confirmation was invalidated by gyro-only data, then four
        # distinct accepted visual frames must qualify again.
        self.assertEqual(replay.log.getvalue().count('aligned '), 5)
        self.assertEqual(replay.commands[-1], (0, 0, 0, 0))

    def test_heading_outside_tolerance_never_completes(self):
        replay = TagReplay([(425, 0, 5)])
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            replay.run(independent_heading=True)
        self.assertNotIn('aligned ', replay.log.getvalue())
        self.assertEqual(replay.commands[-1], (0, 0, 0, 0))

    def test_translation_excursion_still_interrupts_confirmation(self):
        replay = TagReplay([(425, 0, 1)] * 3 + [(450, 25, 1)] * 9
                           + [(425, 0, 1)])
        replay.run(independent_heading=True)
        self.assertGreater(replay.index, 12)
        self.assertEqual(replay.log.getvalue().count('aligned '), 7)
        self.assertTrue(any(vx != 0 and vy != 0
                            for _, vx, vy, _ in replay.velocities))
        self.assertEqual(replay.commands[-1], (0, 0, 0, 0))

    def test_repeated_frame_does_not_complete_and_staleness_stops_yaw(self):
        replay = TagReplay([(425, 0, 1), None])
        with self.assertRaises(RuntimeError):
            replay.run(independent_heading=True)
        self.assertEqual(replay.log.getvalue().count('aligned '), 1)
        self.assertTrue(any(wz > 0 for _, _, _, wz in replay.velocities))
        self.assertTrue(all(rpm == (0, 0, 0, 0) for i, rpm in replay.command_events
                            if i >= 16))

    def test_lost_or_rejected_visual_frame_cannot_resume_from_cached_data(self):
        for rejected in ('lost', (700, 0, 5)):
            with self.subTest(rejected=rejected):
                replay = TagReplay([(445, 0, 5), None, rejected, None])
                with self.assertRaises(RuntimeError):
                    replay.run(independent_heading=True)
                self.assertTrue(all(rpm == (0, 0, 0, 0)
                                    for i, rpm in replay.command_events if i >= 2))

    def test_hardware_guard_aborts_cached_frame_corrections(self):
        for fault in ('telemetry stale', 'communication lost', 'emergency stop'):
            with self.subTest(fault=fault):
                replay = TagReplay([(445, 0, 5), None])
                def check_active(**kwargs):
                    if replay.index >= 2:
                        raise RuntimeError(fault)
                replay.program.context = SimpleNamespace(check_active=check_active, heading_zero_deg=0.0)
                with self.assertRaisesRegex(RuntimeError, fault):
                    replay.run(independent_heading=True)
                self.assertEqual(replay.index, 2)
                self.assertEqual(replay.commands[-1], (0, 0, 0, 0))


if __name__ == '__main__':
    unittest.main()
