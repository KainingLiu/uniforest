"""Arm-camera geometry gating, with a scripted camera and no hardware."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from vision.opencv.cube_detector import BlockInfo, CubeDetector, VisionResult


MODULE = 'vision.opencv.cube_detector'


class CameraPoseGateTests(unittest.TestCase):
    def setUp(self):
        self.detector = CubeDetector(camera_id='cube')
        self.detector._state['orange_diagnostics'] = {}
        self.now = 100.0
        self.clock = patch(MODULE + '.time.monotonic', side_effect=lambda: self.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.block = BlockInfo(color_name='Orange', x=20, z=300, confidence=95)

    def frames(self, *callbacks, detect=None):
        """Each callback runs inside one blocking read, before it returns."""
        source = iter(callbacks)
        detector = self.detector

        def read():
            callback = next(source, None)
            if callback is None:
                detector._running = False
                return False, None
            callback()
            return True, np.zeros((8, 8, 3), np.uint8)

        detector._running = True
        detector._cap = SimpleNamespace(read=read, release=lambda: None)
        with patch(MODULE + '.detect_all_blocks',
                   side_effect=detect or (lambda *a, **k: [self.block])) as inference, \
             patch(MODULE + '.time.sleep'):
            detector._capture_loop()
        return inference

    def test_motion_clears_geometry_but_raw_inspection_and_recording_continue(self):
        self.frames(lambda: None)
        self.assertTrue(self.detector.result.all_blocks)
        self.assertTrue(self.detector.camera_pose_ready)

        token = self.detector.begin_pose_change('grab')
        self.assertIsNone(self.detector.result)
        self.assertFalse(self.detector.camera_pose_ready)
        sink = Mock()
        self.detector.set_frame_sink(sink)
        inference = self.frames(lambda: None)

        inference.assert_not_called()
        self.assertIsNone(self.detector.result)
        self.assertIsNotNone(self.detector.raw_frame)
        metadata = sink.call_args.args[-1]
        self.assertEqual(metadata['camera_pose'],
                         {'epoch': token, 'geometry_allowed': False})

    def test_restoration_requires_a_new_read_after_settling(self):
        token = self.detector.begin_pose_change()
        self.assertTrue(self.detector.end_pose_change(token, settle_s=0.5))
        self.assertIsNone(self.detector.result)
        self.frames(lambda: None)
        self.assertIsNone(self.detector.result)

        # A read that began during settling also cannot publish, even if it
        # returns after the deadline.
        self.frames(lambda: setattr(self, 'now', 100.75))
        self.assertIsNone(self.detector.result)
        self.frames(lambda: None)
        self.assertEqual(self.detector.result.captured_monotonic, 100.75)
        self.assertEqual(self.detector.result.pose_epoch, token)
        self.assertTrue(self.detector.camera_pose_ready)

    def test_read_spanning_restoration_cannot_publish_geometry(self):
        token = self.detector.begin_pose_change()
        inference = self.frames(lambda: self.detector.end_pose_change(token))
        inference.assert_not_called()
        self.assertIsNone(self.detector.result)
        self.frames(lambda: None)
        self.assertEqual(self.detector.result.pose_epoch, token)

    def test_inflight_detection_cannot_reappear_after_pose_change(self):
        def arm_cycle(*args, **kwargs):
            token = self.detector.begin_pose_change()
            self.detector.end_pose_change(token)
            return [self.block]

        self.frames(lambda: None, detect=arm_cycle)
        self.assertIsNone(self.detector.result)
        self.frames(lambda: None)
        self.assertEqual(self.detector.result.all_blocks, [self.block])

    def test_older_restore_or_filter_reset_cannot_reopen_current_pose(self):
        first = self.detector.begin_pose_change()
        current = self.detector.begin_pose_change()
        self.assertFalse(self.detector.end_pose_change(first))
        self.detector.reset_filter()
        self.detector.reset_after_inspection()
        self.detector.set_detection_profile('task2_orange')
        self.frames(lambda: None).assert_not_called()
        self.assertIsNone(self.detector.result)
        self.assertTrue(self.detector.end_pose_change(current))
        self.frames(lambda: None)
        self.assertTrue(self.detector.camera_pose_ready)

    def test_detector_stop_invalidates_old_restore_token(self):
        token = self.detector.begin_pose_change()
        with patch(MODULE + '.cv2.destroyAllWindows'):
            self.detector.stop()
        self.assertFalse(self.detector.end_pose_change(token))
        self.frames(lambda: None).assert_not_called()
        new_token = self.detector.begin_pose_change('confirmed_restart_pose')
        self.assertTrue(self.detector.end_pose_change(new_token))
        self.frames(lambda: None)
        self.assertTrue(self.detector.camera_pose_ready)

    def test_acquisition_timestamp_precedes_slow_processing(self):
        def slow_detection(*args, **kwargs):
            self.now += 2.0
            return [self.block]

        self.frames(lambda: None, detect=slow_detection)
        self.assertEqual(self.detector.result.captured_monotonic, 100.0)
        self.assertEqual(self.now, 102.0)

    def test_invalid_settle_time_keeps_gate_closed(self):
        token = self.detector.begin_pose_change()
        for delay in (-1, float('nan'), float('inf')):
            with self.subTest(delay=delay), self.assertRaises(ValueError):
                self.detector.end_pose_change(token, settle_s=delay)
        self.frames(lambda: None).assert_not_called()

    def test_existing_result_constructors_remain_compatible(self):
        result = VisionResult(timestamp=123, all_blocks=[self.block])
        self.assertEqual(result.captured_monotonic, 0.0)
        self.assertEqual(result.pose_epoch, 0)

    def test_robot_facade_delegates_and_handles_disabled_camera(self):
        from robot import Robot
        robot = Robot.__new__(Robot)
        robot._vision = self.detector
        token = robot.begin_cube_camera_pose_change('grab')
        self.assertFalse(robot.cube_camera_pose_ready)
        self.assertTrue(robot.end_cube_camera_pose_change(token))
        self.frames(lambda: None)
        self.assertTrue(robot.cube_camera_pose_ready)
        self.assertIs(robot.vision_result, self.detector.result)
        robot._vision = None
        self.assertIsNone(robot.begin_cube_camera_pose_change())
        self.assertIsNone(robot.end_cube_camera_pose_change(None))
        self.assertFalse(robot.cube_camera_pose_ready)


if __name__ == '__main__':
    unittest.main()
