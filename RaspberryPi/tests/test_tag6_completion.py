"""Check actual Tag control outputs against specified poses, without hardware."""
import contextlib
from dataclasses import replace
import io
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from control.chassis import Chassis
from Strategy.competition import CompetitionProgram, FirstTaskConfig
from Strategy.task1 import Task1Round2Config
from Strategy.task2 import Task2Config, Task2Round2Config


class TagReplay:
    def __init__(self, samples, config=FirstTaskConfig()):
        self.samples = samples  # distance mm, lateral mm, gyro error deg
        self.now, self.index = 100.0, -1
        self.commands, self.velocities = [], []
        self.robot = SimpleNamespace(field_pose=None, telem=SimpleNamespace(yaw_deg=0))
        self.robot.chassis = SimpleNamespace(
            set_speeds=lambda rpm: self.commands.append(tuple(rpm)),
            mecanum_rpm=self.mecanum)
        self.program = CompetitionProgram.__new__(CompetitionProgram)
        self.program.robot = self.robot
        self.program._heading_zero_deg = 0
        self.program.config = replace(config, delivery_tag_align_timeout_s=1.0)

    def mecanum(self, vx, vy, wz):
        self.velocities.append((self.index, vx, vy, wz))
        return Chassis.mecanum_rpm(vx, vy, wz)

    def sleep(self, seconds):
        self.now += seconds
        self.index += 1
        item = self.samples[min(self.index, len(self.samples) - 1)]
        if item is None:  # Same cached frame; must not advance confirmation.
            return
        distance, lateral, error = item
        self.robot.field_pose = SimpleNamespace(timestamp=self.now, tag_solutions=[
            SimpleNamespace(tag_id=6, distance_m=distance/1000,
                            lateral_m=lateral/1000, score=0)])
        self.robot.telem.yaw_deg = -180 - error

    def run(self, translation_only=True):
        clock = SimpleNamespace(monotonic=lambda: self.now, time=lambda: self.now,
                                sleep=self.sleep)
        with patch('Strategy.competition.time', clock), contextlib.redirect_stdout(io.StringIO()):
            self.program._align_delivery_tag(
                tag_id=6, target_distance_mm=425, heading_target_cw_deg=180,
                distance_tolerance_mm=8, lateral_tolerance_mm=8,
                fine_align_enabled=False, stop_axes_in_tolerance=True,
                translation_only_completion=translation_only)


class Tag6CompletionTests(unittest.TestCase):
    def test_four_task_configs_complete_on_translation_while_yaw_still_outside(self):
        for config in (FirstTaskConfig(), Task1Round2Config(), Task2Config(), Task2Round2Config()):
            with self.subTest(config=type(config).__name__):
                replay = TagReplay([(425, 0, 15)], config)
                replay.run()
                self.assertEqual(replay.index, 3)
                self.assertEqual(len(replay.velocities), 3)
                self.assertTrue(all(vx == vy == 0 and wz > 0
                                    for _, vx, vy, wz in replay.velocities))
                self.assertEqual(replay.commands[-1], (0, 0, 0, 0))

    def test_small_yaw_errors_keep_correcting_in_both_directions(self):
        for error in (2.0, .1, -.1):
            with self.subTest(error=error):
                replay = TagReplay([(445, 0, error)] * 3 + [(425, 0, error)])
                replay.run()
                self.assertTrue(all(wz * error > 0 for _, _, _, wz in replay.velocities))
                self.assertTrue(any(vx != 0 for _, vx, _, _ in replay.velocities))
                self.assertEqual(replay.commands[-1], (0, 0, 0, 0))

    def test_exact_heading_needs_no_rotation(self):
        replay = TagReplay([(425, 0, 0)])
        replay.run()
        self.assertTrue(all(wz == 0 for _, _, _, wz in replay.velocities))

    def test_position_must_remain_valid_through_confirmation(self):
        replay = TagReplay([(425, 0, 10)] * 2 + [(450, 25, 10)] * 3
                           + [(425, 0, 10)])
        replay.run()
        self.assertEqual(replay.index, 9)
        self.assertTrue(any(vx != 0 and vy != 0 for _, vx, vy, _ in replay.velocities))

    def test_repeated_frames_do_not_complete_and_finally_stops(self):
        replay = TagReplay([(425, 0, 15), None])
        with self.assertRaises(RuntimeError): replay.run()
        self.assertEqual(len(replay.velocities), 1)
        self.assertEqual(replay.commands[-1], (0, 0, 0, 0))

    def test_default_coarse_mode_still_requires_heading(self):
        replay = TagReplay([(425, 0, 15)])
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            replay.run(translation_only=False)
        self.assertEqual(replay.commands[-1], (0, 0, 0, 0))


if __name__ == '__main__':
    unittest.main()
