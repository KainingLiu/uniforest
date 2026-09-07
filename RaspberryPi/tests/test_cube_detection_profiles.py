import os
import sys
import unittest

import cv2
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
    def test_purple_roi_excludes_upper_objects_without_shifting_lower_coordinates(self):
        for height, width in ((480, 640), (720, 960)):
            with self.subTest(resolution=(width, height)):
                frame = np.zeros((height, width, 3), dtype=np.uint8)
                purple = cv2.cvtColor(np.uint8([[[125, 200, 180]]]), cv2.COLOR_HSV2BGR)[0, 0]
                frame[round(height * .06):round(height * .28),
                      round(width * .15):round(width * .35)] = purple
                frame[round(height * .55):round(height * .8),
                      round(width * .45):round(width * .65)] = purple
                state = dict(fx=800, fy=800, cx=width / 2, cy=height / 2)
                colors = color_profiles_for('task2_purple')
                self.assertEqual([p['name'] for p in colors], ['Purple'])
                full = detect_all_blocks(frame, state, colors)
                masked = detect_all_blocks(frame, state, colors,
                                           roi_top_ratio=roi_top_ratio_for('task2_purple'))
                self.assertEqual(len(full), 2)
                self.assertEqual(len(masked), 1)
                lower = min(full, key=lambda block: block.y)
                np.testing.assert_allclose(masked[0].quad, lower.quad)
                self.assertEqual((masked[0].x, masked[0].y, masked[0].z),
                                 (lower.x, lower.y, lower.z))
                self.assertTrue(np.all(masked[0].quad[:, 1] >= height * .4))

    def test_purple_profile_switch_clears_cached_result_and_restores_default(self):
        detector = CubeDetector(camera_id=0)
        detector._result = object()
        detector.set_detection_profile('task2_purple')
        self.assertEqual(detector._roi_top_ratio, .4)
        self.assertIsNone(detector.result)
        detector._result = object()
        detector.set_detection_profile('default')
        self.assertEqual(detector._roi_top_ratio, 0)
        self.assertIsNone(detector.result)

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
