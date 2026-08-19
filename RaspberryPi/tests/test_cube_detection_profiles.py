import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision.cube_detector import CubeDetector, color_profiles_for


def profile_named(profiles, name):
    return next(profile for profile in profiles if profile['name'] == name)


class CubeDetectionProfileTests(unittest.TestCase):
    def test_task2_orange_has_independent_hsv_range(self):
        default = profile_named(color_profiles_for('default'), 'Orange')
        task2 = profile_named(color_profiles_for('task2_orange'), 'Orange')

        np.testing.assert_array_equal(default['hsv_low'], [2, 80, 45])
        np.testing.assert_array_equal(default['hsv_high'], [25, 255, 255])
        np.testing.assert_array_equal(task2['hsv_low'], [5, 75, 80])
        np.testing.assert_array_equal(task2['hsv_high'], [35, 255, 255])

        task2['hsv_low'][0] = 99
        self.assertEqual(default['hsv_low'][0], 2)

    def test_switching_profile_clears_stale_result(self):
        detector = CubeDetector(camera_id=0)
        detector._result = object()

        detector.set_detection_profile('task2_orange')

        self.assertEqual(detector.detection_profile, 'task2_orange')
        self.assertIsNone(detector.result)

    def test_unknown_profile_is_rejected(self):
        detector = CubeDetector(camera_id=0)
        with self.assertRaises(ValueError):
            detector.set_detection_profile('missing')


if __name__ == '__main__':
    unittest.main()
