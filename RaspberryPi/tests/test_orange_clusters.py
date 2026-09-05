"""Projected top/front scenes, with no seam cue between touching cubes."""
import os
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from vision.cube_detector import color_profiles_for, detect_all_blocks


def scene(starts=(-15, -5), depth=45, y=3.5355, width=640):
    frame = np.zeros((480, width, 3), np.uint8)
    a = 2 ** -0.5
    rotation = np.array([[1, 0, 0], [0, -a, -a], [0, -a, a]])
    for start in starts:
        faces = [
            ([[start, 10, 0], [start+10, 10, 0],
              [start+10, 10, -5], [start, 10, -5]], (35, 120, 210)),
            ([[start, 0, 0], [start+10, 0, 0],
              [start+10, 10, 0], [start, 10, 0]], (50, 165, 255)),
        ]
        for face, color in faces:
            camera = np.asarray(face) @ rotation.T + [0, y, depth]
            pixels = camera[:, :2] / camera[:, 2:] * (0.8 * width) + [width/2, 240]
            cv2.fillPoly(frame, [np.round(pixels).astype(np.int32)], color)
    return frame


def detect(frame, profile="default", state=None, roi=0):
    if state is None:
        state = dict(fx=512, fy=512, cx=320, cy=240)
    return detect_all_blocks(frame, state, color_profiles_for(profile), roi_top_ratio=roi)


class OrangeClusterTests(unittest.TestCase):
    def test_touching_and_arbitrary_gaps(self):
        for profile in ("default", "task2_orange"):
            for gap in (0, 0.2, 3, 15):
                with self.subTest(profile=profile, gap=gap):
                    blocks = detect(scene((-15, -5+gap)), profile)
                    left = min(blocks, key=lambda b: b.x)
                    self.assertAlmostEqual(left.x, -100, delta=5)
                    self.assertAlmostEqual(left.z, 414.64, delta=8)
                    self.assertAlmostEqual(left.y, 0, delta=5)
                    self.assertEqual(len(blocks), 1 if gap < 1 else 2)

    def test_three_touching_blocks_return_first_not_cluster_center(self):
        blocks = detect(scene((-15, -5, 5)))
        self.assertEqual(len(blocks), 1)
        self.assertAlmostEqual(blocks[0].x, -100, delta=5)

    def test_single_cube_and_camera_motion_do_not_reuse_pose(self):
        state = dict(fx=512, fy=512, cx=320, cy=240)
        first = detect(scene((-5,)), state=state)[0]
        moved = detect(scene((-5,), depth=55), state=state)[0]
        self.assertAlmostEqual(first.x, 0, delta=5)
        self.assertAlmostEqual(moved.z-first.z, 100, delta=10)

    def test_left_clipped_and_empty_frames_do_not_invent_target(self):
        self.assertEqual(detect(scene((-35, -25, -15))), [])
        self.assertEqual(detect(np.zeros((480, 640, 3), np.uint8)), [])

    def test_rotated_strip_and_processing_resize(self):
        image = scene((-15, -5, 5))
        rotation = cv2.getRotationMatrix2D((320, 240), 7, 1)
        blocks = detect(cv2.warpAffine(image, rotation, (640, 480)))
        self.assertEqual(len(blocks), 1)
        self.assertAlmostEqual(blocks[0].x, -100, delta=8)
        self.assertAlmostEqual(blocks[0].z, 414.64, delta=10)
        enlarged = cv2.resize(image, (1920, 1440))
        blocks = detect(enlarged)
        self.assertEqual(len(blocks), 1)
        self.assertAlmostEqual(blocks[0].x, -100, delta=8)
        self.assertAlmostEqual(blocks[0].z, 414.64, delta=10)
        self.assertGreater(blocks[0].quad[:, 0].mean(), 500)

    def test_task2_roi(self):
        self.assertEqual(detect(scene((-5,)), "task2_orange", roi=0.5), [])
        blocks = detect(scene((-5,), y=14), "task2_orange", roi=0.5)
        self.assertEqual(len(blocks), 1)
        self.assertGreater(blocks[0].quad[:, 1].min(), 240)

    def test_purple_and_building_keep_legacy_geometry(self):
        state = dict(fx=512, fy=512, cx=320, cy=240)
        frame = np.zeros((480, 640, 3), np.uint8)
        frame[260:380, 200:400] = (150, 50, 100)
        profiles = color_profiles_for("default")
        actual = detect_all_blocks(frame, state, profiles)
        expected = detect_all_blocks(frame, state, [p for p in profiles if p["name"] == "Purple"])
        self.assertTrue(expected)
        self.assertEqual([(b.x,b.y,b.z) for b in actual], [(b.x,b.y,b.z) for b in expected])
        frame[260:380, 200:400] = (0, 140, 255)
        profiles = color_profiles_for("building")
        self.assertNotIn("orange_cluster_profile", profiles[0])
        blocks = detect_all_blocks(frame, state, profiles)
        self.assertEqual(len(blocks), 1)
        self.assertAlmostEqual(blocks[0].z, 512*100/200, delta=4)


if __name__ == "__main__":
    unittest.main()
