"""Synthetic pixels exercise preview geometry; they do not measure field accuracy."""

import unittest
from unittest.mock import patch

import cv2
import numpy as np

from vision.opencv.cube_detector import VisionResult, detect_all_blocks, color_profiles_for
from vision.opencv.orange_cluster import Detector
from vision.opencv.orange_config import config_for
from vision.opencv.orange_fixed_geometry import world_to_image
from vision.opencv.orange_lookahead import row_previews, continuous_row
from Strategy.optimizations.adaptive_blind import (
    AdaptiveBlindProfile, NeighborObserver, expected_blind_distance,
)


def quad(left, right, profile='default'):
    return np.array([world_to_image(p, 1280, 720, profile) for p in
                     ((left, 0), (right, 0), (right, 10), (left, 10))], np.float32)


def scene(profile='default', *, seam=True, left=-5., right=15.):
    image = np.zeros((720, 1280, 3), np.uint8)
    corners = quad(left, right, profile)
    q = np.rint(corners).astype(np.int32)
    front = np.array([q[3], q[2], q[2]+[0, 35], q[3]+[0, 35]])
    cv2.fillConvexPoly(image, front, (0, 75, 110))
    cv2.fillConvexPoly(image, q, (0, 170, 240))
    if seam:
        cv2.fillConvexPoly(image, np.rint(quad(4.97, 5.03, profile)).astype(np.int32), (10, 10, 10))
    return image, corners


class OrangeLookaheadTests(unittest.TestCase):
    def test_pixels_to_motion_policy_distinguishes_continuous_row_and_empty_next_slot(self):
        for profile in ('default', 'task2_orange'):
            for right, expected, source in ((15., 100., 'continuous_row_pitch'),
                                            (5., 200., 'unseen_next_search')):
                with self.subTest(profile=profile, right=right):
                    image, _ = scene(profile, seam=False, right=right)
                    state = dict(fx=660., fy=410., cx=640., cy=360.)
                    blocks = detect_all_blocks(image, state, color_profiles=color_profiles_for(profile))
                    observer = NeighborObserver(AdaptiveBlindProfile(validated=True))
                    for t in (10., 10.02):
                        result = VisionResult(all_blocks=blocks, captured_monotonic=t, pose_epoch=3,
                            orange_lookahead=state['orange_lookahead'],
                            orange_lookahead_complete=state['orange_lookahead_complete'])
                        observer.observe(result, blocks[0], position_mm=0., speed_mm_s=0.,
                            target_x_mm=blocks[0].x, yaw_deg=0., now=t, link_epoch=0,
                            stop_generation=0, telemetry_id=round(t*1000), stopped=True,
                            acceleration_mm_s2=200.)
                    observation = observer.consume(now=10.02, pose_epoch=3, link_epoch=0, stop_generation=0)
                    self.assertIsNotNone(observation)
                    self.assertEqual(expected_blind_distance(observer.profile, observation,
                        position_mm=0., yaw_deg=0., now=10.1, link_epoch=0,
                        stop_generation=0, expected_pose_epoch=4, legacy_distance_mm=80.), (expected, source))

    def test_two_touching_cubes_with_observed_seam_have_two_previews(self):
        for profile in ('default', 'task2_orange'):
            with self.subTest(profile=profile):
                image, corners = scene(profile)
                found = row_previews(image, corners, profile=profile)
                self.assertEqual(len(found), 2)
                self.assertAlmostEqual(found[1][0][0]-found[0][0][0], 10., places=3)

    def test_wide_patch_without_seam_does_not_invent_cube_positions(self):
        image, corners = scene(seam=False)
        self.assertEqual(row_previews(image, corners, profile='default'), [])
        self.assertTrue(continuous_row(image, corners, profile='default'))

    def test_continuous_row_needs_enough_visible_width_and_no_large_gap(self):
        for profile in ('default', 'task2_orange'):
            image, corners = scene(profile, seam=False, right=5.)
            self.assertFalse(continuous_row(image, corners, profile=profile))
            image, corners = scene(profile, seam=False)
            cv2.fillConvexPoly(image, np.rint(quad(4., 6., profile)).astype(np.int32), (0, 0, 0))
            self.assertFalse(continuous_row(image, corners, profile=profile))

    def test_cropped_right_edge_cannot_close_the_last_preview(self):
        image, corners = scene()
        found = row_previews(image, corners, profile='default', clipped_right=True)
        self.assertEqual(len(found), 1)
        self.assertAlmostEqual(found[0][0][0], 0., places=3)

    def test_small_dark_mark_does_not_split_a_row(self):
        image, corners = scene(seam=False)
        center = np.rint(world_to_image((5., 5.), 1280, 720)).astype(int)
        cv2.circle(image, tuple(center), 4, (0, 0, 0), -1)
        self.assertEqual(row_previews(image, corners, profile='default'), [])

    def test_missing_cube_or_non_orange_interior_is_rejected(self):
        image, corners = scene()
        cv2.fillConvexPoly(image, np.rint(quad(5.2, 14.8)).astype(np.int32), (180, 180, 180))
        self.assertLessEqual(len(row_previews(image, corners, profile='default')), 1)

    def test_bad_or_offscreen_geometry_produces_no_preview(self):
        image, _ = scene()
        for corners in (np.full((4, 2), float('nan')), quad(100, 110), np.zeros((3, 2))):
            self.assertEqual(row_previews(image, corners, profile='default'), [])

    def test_real_detector_preserves_single_grasp_target_and_publishes_previews(self):
        for profile in ('default', 'task2_orange'):
            image, _ = scene(profile)
            state = dict(fx=660., fy=410., cx=640., cy=360.)
            blocks = detect_all_blocks(image, state, color_profiles=color_profiles_for(profile))
            self.assertEqual(len(blocks), 1)
            self.assertTrue(blocks[0].continuous_row)
            self.assertEqual(len(state['orange_lookahead']), 2)
            self.assertTrue(state['orange_lookahead_complete'])
            self.assertAlmostEqual(blocks[0].x, state['orange_lookahead'][0].x, delta=2.)
            detect_all_blocks(np.zeros_like(image), state, color_profiles=color_profiles_for(profile))
            self.assertEqual(state['orange_lookahead'], [])

    def test_separate_clusters_give_multiple_existing_targets_and_previews(self):
        # Keep both synthetic cubes fully inside the newly calibrated camera
        # view; the old +18 cm edge is now cropped and must reject a preview.
        image, _ = scene(seam=False, left=-6., right=4.)
        other, _ = scene(seam=False, left=6., right=16.)
        image = np.maximum(image, other)
        detector = Detector(config_for('default', 1280))
        blocks, info = detector.detect(image)
        self.assertEqual(len(blocks), 2)
        self.assertTrue(all(not b.continuous_row for b in blocks))
        self.assertEqual(len(info['lookahead_cubes']), 2)
        self.assertTrue(info['lookahead_complete'])

    def test_unresolved_row_is_marked_incomplete_even_with_a_grasp_target(self):
        image, _ = scene(seam=False)
        blocks, info = Detector(config_for('default', 1280)).detect(image)
        self.assertEqual(len(blocks), 1)
        self.assertTrue(blocks[0].continuous_row)
        self.assertFalse(info['lookahead_complete'])

    def test_preview_failure_does_not_remove_existing_grasp_target(self):
        image, _ = scene()
        with patch('vision.opencv.orange_cluster.row_previews', side_effect=ValueError('synthetic')):
            blocks, info = Detector(config_for('default', 1280)).detect(image)
        self.assertEqual(len(blocks), 1)
        self.assertEqual(info['lookahead_cubes'], [])
        self.assertFalse(info['lookahead_complete'])


if __name__ == '__main__':
    unittest.main()
