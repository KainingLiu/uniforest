"""Field failure replays; synthetic friction values are not calibrations."""
import contextlib
from dataclasses import replace
import io
import unittest

from Strategy.errors import SearchRangeExhausted
from Strategy.optimizations.fast_alignment import CaptureWindow
from Strategy.optimizations.trial import trial_configuration
from tests.test_fast_alignment import InertialPlant
from tests.test_visual_fallback import SearchReplay, block


def trial_profile():
    return trial_configuration().alignment('ground-1', 'orange')


class DeadbandPlant(InertialPlant):
    def __init__(self, deadband, **kwargs):
        super().__init__(**kwargs)
        self.deadband = deadband

    def sleep(self, seconds):
        requested = self.desired
        if abs(requested) < self.deadband:
            self.desired = 0
        super().sleep(seconds)
        self.desired = requested


class PickupWindowRegressions(unittest.TestCase):
    def setUp(self):
        quiet = contextlib.redirect_stdout(io.StringIO())
        quiet.__enter__()
        self.addCleanup(quiet.__exit__, None, None, None)

    def test_clipped_left_target_recovered_before_right_target_at_phase_origin(self):
        for visible_right in (False, True):
            with self.subTest(visible_right=visible_right):
                def frames(replay):
                    if replay.x <= -20:
                        return replay.frame([block()])
                    if replay.x >= 220:
                        return replay.frame([block()])
                    return replay.frame([block()] if visible_right else (), band=(100,200))
                replay = SearchReplay(frames, x=0)
                replay.run()
                self.assertLessEqual(replay.x, -20)
                self.assertEqual(replay.task._search_position_mm, 0)

    def test_continuous_clipping_has_one_bounded_attempt_even_left_of_origin(self):
        replay = SearchReplay(lambda r: r.frame(band=(100,200)), x=-30)
        replay.task.config = replace(replay.task.config, orange_edge_max_distance_mm=80)
        with self.assertRaises(SearchRangeExhausted):
            replay.run(limit=90)
        self.assertLess(min(replay.positions), -30)
        self.assertGreaterEqual(min(replay.positions), -110.001)
        starts = sum(v < 0 and (i == 0 or replay.commands[i-1][1] >= 0)
                     for i, (_, v) in enumerate(replay.commands))
        self.assertEqual(starts, 1)
        self.assertEqual(replay.commands[-1][1], 0)

    def test_inside_ten_mm_accepts_fresh_stationary_frames_without_recentering(self):
        for error in (-10, -9.9, 9.9, 10):
            with self.subTest(error=error):
                plant = DeadbandPlant(60, position=0, target=error)
                plant.capture_delay = .06
                self.assertTrue(plant.fast(profile=trial_profile()))
                self.assertLess(plant.now-100, .3)
                self.assertTrue(all(c[1] == 0 for c in plant.commands))

    def test_just_outside_window_converges_with_synthetic_low_speed_deadband(self):
        for error in (-10.5, 10.5, -12, 12):
            for deadband in (20, 60):
                with self.subTest(error=error, deadband=deadband):
                    plant = DeadbandPlant(deadband, position=0, target=error)
                    self.assertTrue(plant.fast(profile=trial_profile()))
                    self.assertLessEqual(abs(plant.target-plant.position), 10)
                    self.assertLess(plant.now-100, 1)
                    self.assertEqual(plant.commands[-1][1], 0)

    def test_command_alone_does_not_latch_before_chassis_has_moved(self):
        policy = CaptureWindow(trial_profile())
        speeds = [policy.update(10.5,0,0,.02,evidence=(i,i)).speed_mm_s
                  for i in range(5)]
        self.assertGreater(max(speeds), 60)
        self.assertFalse(policy.latched)

    def test_duplicate_frames_cannot_finish_even_within_ten_mm(self):
        plant = DeadbandPlant(60, position=0, target=9.9)
        plant.freeze_frame = True
        self.assertFalse(plant.fast(profile=replace(trial_profile(),max_duration_s=.4)))

    def test_correction_respects_remaining_window_braking_envelope(self):
        p = trial_profile()
        policy = CaptureWindow(p)
        speed = 0
        for i in range(30):
            decision = policy.update(10.01,speed,0,.02,evidence=(i,i))
            command = abs(decision.speed_mm_s)
            self.assertLessEqual(command*(p.tick_s+p.max_command_delay_s)
                                 +command**2/(2*p.braking_mm_s2),20.01+1e-9)


if __name__ == '__main__':
    unittest.main()
