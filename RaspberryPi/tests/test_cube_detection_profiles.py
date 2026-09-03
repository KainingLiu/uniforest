import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision.cube_detector import (
    CONFIG,
    CubeDetector,
    color_profiles_for,
    detect_all_blocks,
    morphology_for,
    max_front_aspect_for,
    roi_top_ratio_for,
)


def profile_named(profiles, name):
    return next(profile for profile in profiles if profile['name'] == name)


class CubeDetectionProfileTests(unittest.TestCase):
    def test_task2_orange_has_independent_hsv_range(self):
        default = profile_named(color_profiles_for('default'), 'Orange')
        task2 = profile_named(color_profiles_for('task2_orange'), 'Orange')

        np.testing.assert_array_equal(default['hsv_low'], [2, 80, 45])
        np.testing.assert_array_equal(default['hsv_high'], [25, 255, 255])
        np.testing.assert_array_equal(task2['hsv_low'], [4, 65, 55])
        np.testing.assert_array_equal(task2['hsv_high'], [35, 255, 255])

        task2['hsv_low'][0] = 99
        self.assertEqual(default['hsv_low'][0], 2)

        self.assertEqual(roi_top_ratio_for('default'), 0.0)
        self.assertEqual(roi_top_ratio_for('task2_orange'), 0.5)
        self.assertEqual(morphology_for('default'), (5, 2))
        self.assertEqual(morphology_for('task2_orange'), (3, 1))
        self.assertEqual(max_front_aspect_for('task2_orange'), 5.2)

    def test_building_profile_includes_bright_top_surface(self):
        building = profile_named(color_profiles_for('building'), 'Orange')
        np.testing.assert_array_equal(building['hsv_low'], [0, 35, 40])
        np.testing.assert_array_equal(building['hsv_high'], [50, 255, 255])
        self.assertEqual(roi_top_ratio_for('building'), 0.0)
        self.assertEqual(morphology_for('building'), (5, 2))
        # The lit building top measures H~30-45, S~40-60, V~90-205 on the
        # field and must be inside the building band while the saturated
        # orange front (H~25) stays inside too.
        low = building['hsv_low']
        high = building['hsv_high']
        for pixel in ([33, 50, 180], [32, 55, 100], [25, 136, 202]):
            self.assertTrue(all(low[i] <= pixel[i] <= high[i]
                                for i in range(3)))

    def test_task2_roi_excludes_upper_half_only(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        orange_bgr = (0, 140, 255)
        frame[100:200, 250:390] = orange_bgr
        frame[300:400, 250:390] = orange_bgr
        state = {
            'fx': 332.0, 'fy': 207.0,
            'cx': 320.0, 'cy': 240.0,
        }

        blocks = detect_all_blocks(
            frame, state,
            color_profiles=color_profiles_for('task2_orange'),
            roi_top_ratio=roi_top_ratio_for('task2_orange'))

        orange_blocks = [block for block in blocks
                         if block.color_name == 'Orange']
        self.assertEqual(len(orange_blocks), 1)
        self.assertGreater(orange_blocks[0].quad[:, 1].mean(), 240.0)

    def test_touching_orange_blocks_split_at_dark_vertical_seam(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        orange_bgr = (0, 140, 255)
        frame[220:380, 170:470] = orange_bgr
        frame[220:380, 315:325] = (20, 20, 20)
        state = {'fx': 800.0, 'fy': 800.0, 'cx': 320.0, 'cy': 240.0}

        blocks = detect_all_blocks(
            frame, state,
            color_profiles=color_profiles_for('task2_orange'),
            roi_top_ratio=0.0,
            morph_kernel_size=3,
            morph_iterations=1)

        orange_blocks = [block for block in blocks
                         if block.color_name == 'Orange']
        self.assertEqual(len(orange_blocks), 2)
        centers = sorted(float(block.quad[:, 0].mean())
                         for block in orange_blocks)
        self.assertLess(centers[0], 315.0)
        self.assertGreater(centers[1], 325.0)

    def test_switching_profile_clears_stale_result(self):
        detector = CubeDetector(camera_id=0)
        detector._result = object()

        detector.set_detection_profile('task2_orange')

        self.assertEqual(detector.detection_profile, 'task2_orange')
        self.assertEqual(detector._roi_top_ratio, 0.5)
        self.assertIsNone(detector.result)

    def test_unknown_profile_is_rejected(self):
        detector = CubeDetector(camera_id=0)
        with self.assertRaises(ValueError):
            detector.set_detection_profile('missing')


if __name__ == '__main__':
    unittest.main()
