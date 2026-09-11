"""Synthetic image and simulated-motion coverage for left-edge recovery."""
from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from Strategy.competition import CompetitionProgram, FirstTaskConfig, SearchRangeExhausted
from Strategy.orange_search import OrangeSearchRecovery
from vision.cube_detector import (
    BlockInfo, VisionResult, color_profiles_for, detect_all_blocks, roi_top_ratio_for,
)


class SearchRobot:
    def __init__(self, frames=None, x=0.0):
        self.now, self.x, self.speed = 0.0, x, 0.0
        self.commands, self.positions = [], []
        self.transport = SimpleNamespace(connected=True, emergency_stop_generation=0)
        self.chassis = self
        self.frames = frames or (lambda robot: robot.frame())
        self.frozen_telem = False
        self.send_ok = True
        self.on_sleep = lambda robot: None

    @property
    def telem(self):
        return SimpleNamespace(uptime_ms=0 if self.frozen_telem else round(self.now * 1000))

    @property
    def vision_result(self):
        return self.frames(self)

    def frame(self, clipped=False, blocks=()):
        return VisionResult(timestamp=100 + self.now, all_blocks=list(blocks),
                            orange_left_clipped_y_range=(300, 390) if clipped else None)

    def capture_motor_positions(self):
        return self.x

    def lateral_displacement_mm(self, origin):
        return self.x - origin

    @staticmethod
    def mecanum_rpm(vx, vy, wz):
        return [vy, vy, -vy, -vy]

    def set_speeds(self, rpm):
        self.speed = rpm[0] * 10
        self.commands.append((self.now, self.speed))
        return self.send_ok

    def sleep(self, seconds):
        self.x += self.speed * seconds
        self.now += seconds
        self.positions.append(self.x)
        self.on_sleep(self)


def cube(y=320):
    return BlockInfo(color_name='Orange', confidence=80, x=-30, z=200,
                     quad=np.array([[50, y], [150, y], [150, y + 60], [50, y + 60]]))


class OrangeSearchTests(unittest.TestCase):
    def program(self, robot, **config):
        program = CompetitionProgram(robot, replace(FirstTaskConfig(), **config))
        program._orange_recovery = OrangeSearchRecovery(origin=0.0)
        return program

    def run_search(self, program):
        robot = program.robot
        with patch('time.monotonic', lambda: robot.now), \
                patch('time.time', lambda: 100 + robot.now), \
                patch('time.sleep', robot.sleep):
            return program._find_orange()

    def test_recovers_left_edge_then_confirms_stopped_without_right_budget(self):
        robot = SearchRobot(x=500, frames=lambda r: r.frame(
            clipped=r.x > 430, blocks=() if r.x > 430 else [cube()]))
        program = self.program(robot)
        self.assertEqual(self.run_search(program).x, -30)
        self.assertTrue(any(speed == -200 for _, speed in robot.commands))
        self.assertFalse(any(speed > 0 for _, speed in robot.commands))
        self.assertEqual(program._search_position_mm, 0)
        self.assertEqual(robot.commands[-1][1], 0)

    def test_continuous_clipped_row_only_attempted_once_and_right_budget_is_separate(self):
        robot = SearchRobot(x=1500, frames=lambda r: r.frame(clipped=True))
        program = self.program(robot, search_max_distance_mm=180)
        with self.assertRaises(SearchRangeExhausted):
            self.run_search(program)
        speeds = [s for _, s in robot.commands]
        left_runs = sum(s < 0 and (i == 0 or speeds[i - 1] >= 0)
                        for i, s in enumerate(speeds))
        self.assertEqual(left_runs, 1)
        self.assertEqual(program._search_position_mm, 180)
        self.assertAlmostEqual(robot.x, 1500 - 1000 + 180, delta=5)
        self.assertGreater(robot.now, 5)

    def test_recovery_cannot_cross_phase_origin(self):
        for start in (0, 40):
            with self.subTest(start=start):
                robot = SearchRobot(x=start, frames=lambda r: r.frame(clipped=True))
                program = self.program(robot, search_max_distance_mm=30)
                with self.assertRaises(SearchRangeExhausted):
                    self.run_search(program)
                self.assertGreaterEqual(min(robot.positions), 0)
                if start == 0:
                    self.assertFalse(any(s < 0 for _, s in robot.commands))

    def test_pending_candidate_and_duplicate_frames_do_not_reverse(self):
        target = cube()
        robot = SearchRobot(frames=lambda r: r.frame(clipped=True, blocks=[target]))
        program = self.program(robot)
        self.assertEqual(self.run_search(program).x, -30)
        self.assertTrue(all(s == 0 for _, s in robot.commands))
        robot = SearchRobot(x=500)
        fixed = robot.frame(clipped=True)
        robot.frames = lambda r: fixed
        with self.assertRaisesRegex(RuntimeError, 'vision lost'):
            self.run_search(self.program(robot))
        self.assertFalse(any(s < 0 for _, s in robot.commands))

    def test_recovery_rejects_candidate_from_different_row(self):
        robot = SearchRobot(x=500, frames=lambda r: r.frame(
            clipped=r.now < .2, blocks=([] if r.now < .2 else
                                       [cube(y=40)] if r.now < .24 else [cube()])))
        program = self.program(robot, search_max_distance_mm=60)
        result = self.run_search(program)
        self.assertEqual(result.quad[0][1], 320)
        self.assertFalse(any(s > 0 for _, s in robot.commands))
        self.assertEqual(robot.commands[-1][1], 0)

    def test_time_limit_stops_recovery_even_when_encoders_do_not_move(self):
        robot = SearchRobot(x=500, frames=lambda r: r.frame(clipped=True))
        robot.lateral_displacement_mm = lambda origin: 500
        program = self.program(robot, search_max_distance_mm=30, orange_edge_timeout_s=.25)
        with self.assertRaises(SearchRangeExhausted):
            self.run_search(program)
        left_time = sum(robot.commands[i + 1][0] - t
                        for i, (t, s) in enumerate(robot.commands[:-1]) if s < 0)
        self.assertLess(left_time, .25)

    def test_candidate_at_right_limit_can_finish_confirmation(self):
        robot = SearchRobot(frames=lambda r: r.frame(blocks=[cube()] if r.x >= 30 else []))
        program = self.program(robot, search_max_distance_mm=30)
        self.assertEqual(self.run_search(program).x, -30)
        self.assertEqual(program._search_position_mm, 30)
        self.assertAlmostEqual(robot.x, 30)

    def test_empty_fresh_frames_exhaust_right_budget_without_recovery(self):
        robot = SearchRobot()
        program = self.program(robot, search_max_distance_mm=60)
        with self.assertRaises(SearchRangeExhausted):
            self.run_search(program)
        self.assertAlmostEqual(robot.now, .2)
        self.assertFalse(any(s < 0 for _, s in robot.commands))

    def test_faults_and_emergency_stop_never_resume_search(self):
        for fault, message in [('telem', 'telemetry lost'), ('camera', 'vision lost'),
                               ('disconnect', 'communication lost'), ('send', 'command failed'),
                               ('stop', 'cancelled')]:
            with self.subTest(fault=fault):
                robot = SearchRobot(x=500, frames=lambda r: r.frame(clipped=True))
                if fault == 'telem':
                    robot.frozen_telem = True
                elif fault == 'camera':
                    robot.frames = lambda r: None
                elif fault == 'disconnect':
                    robot.transport.connected = False
                elif fault == 'send':
                    robot.send_ok = False
                else:
                    robot.on_sleep = lambda r: setattr(r.transport, 'emergency_stop_generation', 1)
                with self.assertRaisesRegex(RuntimeError, message):
                    self.run_search(self.program(robot))
                self.assertEqual(robot.commands[-1][1], 0)


class OrangePresenceTests(unittest.TestCase):
    def detect(self, frame, profile='default', state=None):
        state = state if state is not None else dict(fx=800, fy=800, cx=320, cy=240)
        blocks = detect_all_blocks(frame, state, color_profiles_for(profile),
                                   roi_top_ratio=roi_top_ratio_for(profile))
        return blocks, state

    def image(self, width=640, height=480):
        return np.zeros((height, width, 3), dtype=np.uint8)

    def test_spanning_row_publishes_presence_without_fake_candidate(self):
        for width, height in ((640, 480), (1920, 1080)):
            frame = self.image(width, height)
            frame[int(height * .65):int(height * .8), :] = (0, 140, 255)
            blocks, state = self.detect(frame, 'task2_orange')
            self.assertEqual(blocks, [])
            low, high = state['orange_diagnostics']['left_clipped_y_range']
            self.assertAlmostEqual(low, height * .65, delta=3)
            self.assertAlmostEqual(high, height * .8, delta=3)

    def test_left_half_or_right_clipping_alone_does_not_request_recovery(self):
        for x1, x2 in ((20, 240), (400, 640)):
            frame = self.image()
            frame[310:380, x1:x2] = (0, 140, 255)
            _, state = self.detect(frame)
            self.assertIsNone(state['orange_diagnostics']['left_clipped_y_range'])

    def test_roi_and_small_noise_do_not_trigger_and_profile_change_clears_signal(self):
        for rows, cols in ((slice(50, 140), slice(0, 640)),
                           (slice(300, 305), slice(0, 5)),
                           (slice(400, 480), slice(0, 640))):
            frame = self.image()
            frame[rows, cols] = (0, 140, 255)
            _, state = self.detect(frame, 'task2_orange')
            self.assertIsNone(state['orange_diagnostics']['left_clipped_y_range'])
        frame = self.image()
        frame[300:390, :] = (0, 140, 255)
        _, state = self.detect(frame, 'task2_orange')
        self.assertIsNotNone(state['orange_diagnostics']['left_clipped_y_range'])
        self.detect(frame, 'task2_purple', state)
        self.assertEqual(state['orange_diagnostics'], {})


if __name__ == '__main__':
    unittest.main()
