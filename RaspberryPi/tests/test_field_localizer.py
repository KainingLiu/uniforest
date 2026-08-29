import math
import os
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision.field_localizer import (
    DEFAULT_CALIB_FILE,
    _object_points,
    fuse_tag_solutions,
    load_camera_model,
    load_field_config,
    solve_tag_pose,
    TagSolution,
)


class FieldLocalizerTests(unittest.TestCase):
    def test_tag_camera_model_uses_specified_horizontal_fov(self):
        matrix, distortion, calibrated = load_camera_model(
            DEFAULT_CALIB_FILE, 640, 480)

        self.assertFalse(calibrated)
        self.assertAlmostEqual(matrix[0, 0], 166.5814562, places=5)
        self.assertAlmostEqual(matrix[1, 1], 166.5814562, places=5)
        self.assertTrue(np.allclose(distortion, 0.0))

    def test_tag_camera_uses_fixed_short_exposure(self):
        config = load_field_config()
        camera = config["camera"]

        self.assertEqual(camera["exposure"], 100)
        self.assertEqual(camera["gain"], 32)
        self.assertEqual(camera["height_m"], 0.25)
        self.assertEqual(
            config["tags"]["6"]["max_camera_height_error_m"], 0.35)
        self.assertEqual(
            config["tags"]["1"]["max_camera_height_error_m"], 0.35)
        self.assertEqual(
            config["tags"]["1"]["max_reprojection_error_px"], 4.5)
        for tag_id in ("2", "3", "4", "5", "6"):
            self.assertIn("max_camera_height_error_m", config["tags"][tag_id])
            self.assertIn("max_reprojection_error_px", config["tags"][tag_id])

    def test_single_tag_recovers_position_without_external_heading(self):
        config = load_field_config()
        camera_matrix = np.array([
            [554.0, 0.0, 320.0],
            [0.0, 554.0, 240.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        rotation = np.diag([1.0, -1.0, -1.0])
        rvec, _ = cv2.Rodrigues(rotation)
        camera_local = np.array([[0.0], [-0.075], [1.0]])
        tvec = -rotation @ camera_local
        corners, _ = cv2.projectPoints(
            _object_points(config["tag_size_m"]), rvec, tvec,
            camera_matrix, dist_coeffs)

        solution = solve_tag_pose(
            corners.reshape(4, 2), 1, config, camera_matrix, dist_coeffs)

        self.assertIsNotNone(solution)
        self.assertAlmostEqual(solution.x_m, -1.0, places=3)
        self.assertAlmostEqual(solution.y_m, 2.0, places=3)
        self.assertAlmostEqual(solution.camera_height_m, 0.25, places=3)
        self.assertLess(solution.reprojection_error_px, 0.01)

    def test_tag_specific_height_handles_lower_tag_3(self):
        config = load_field_config()
        self.assertEqual(config["tags"]["3"]["center_height_m"], 0.125)
        self.assertEqual(config["tags"]["4"]["center_height_m"], 0.125)
        self.assertEqual(
            config["tags"]["3"]["max_camera_height_error_m"], 0.35)
        self.assertEqual(
            config["tags"]["4"]["max_camera_height_error_m"], 0.35)
        for tag_id in ("1", "2", "5"):
            self.assertEqual(
                config["tags"][tag_id]["center_height_m"], 0.325)
        camera_matrix = np.array([
            [554.0, 0.0, 320.0],
            [0.0, 554.0, 240.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        rotation = np.diag([1.0, -1.0, -1.0])
        rvec, _ = cv2.Rodrigues(rotation)
        camera_local = np.array([[0.0], [0.125], [1.0]])
        tvec = -rotation @ camera_local
        corners, _ = cv2.projectPoints(
            _object_points(config["tag_size_m"]), rvec, tvec,
            camera_matrix, dist_coeffs)

        solution = solve_tag_pose(
            corners.reshape(4, 2), 3, config, camera_matrix, dist_coeffs)

        self.assertIsNotNone(solution)
        self.assertAlmostEqual(solution.camera_height_m, 0.25, places=3)
        self.assertLess(solution.reprojection_error_px, 0.01)

    def test_fusion_rejects_distant_solution(self):
        config = load_field_config()
        good_a = TagSolution(1, 1.0, 2.0, 10.0, 0.1, 0.2, 1000, 0.2)
        good_b = TagSolution(2, 1.1, 2.0, 12.0, 0.1, 0.3, 900, 0.3)
        outlier = TagSolution(3, -1.0, -2.0, 170.0, 0.1, 0.1, 1000, 0.1)

        fused = fuse_tag_solutions([good_a, good_b, outlier], config)

        self.assertIsNotNone(fused)
        self.assertEqual([item.tag_id for item in fused[-1]], [1, 2])


if __name__ == "__main__":
    unittest.main()
