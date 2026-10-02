"""One-run trial selection must never claim field validation or enable paths."""
import unittest
from dataclasses import asdict
from Strategy.optimizations.trial import trial_configuration
from Strategy.transition_config import TransitionConfig
from Strategy.optimizations.fast_alignment import CaptureWindow
from control.actions import Actions


class TrialTests(unittest.TestCase):
    def test_one_cm_window_accepts_inside_and_corrects_outside(self):
        for name, profile in trial_configuration().alignments.items():
            for error in (-10., -9., 9., 10.):
                with self.subTest(profile=name, error=error):
                    controller = CaptureWindow(profile)
                    for frame in (1, 2):
                        decision = controller.update(error, 0., 0., .02,
                                                     evidence=(frame, frame))
                    self.assertTrue(decision.complete)
                    self.assertEqual(decision.speed_mm_s, 0.)
            for error in (-11., 11.):
                with self.subTest(profile=name, error=error):
                    controller = CaptureWindow(profile)
                    decision = controller.update(error, 0., 0., .02, evidence=(1, 1))
                    self.assertFalse(decision.complete)
                    self.assertFalse(decision.exhausted)
                    self.assertGreater(decision.speed_mm_s * error, 0.)

    def test_trial_enables_requested_items_without_fabricating_validation(self):
        c=trial_configuration()
        self.assertTrue(c.trial_run)
        self.assertFalse(c.firmware_full_lift_validated)
        self.assertFalse(c.motion_planning_enabled)
        self.assertFalse(c.curves)
        self.assertEqual(len(c.pickups),7)
        self.assertEqual(len(c.alignments),7)
        for p in c.alignments.values():
            self.assertFalse(p.validated)
            self.assertTrue(p.trial_enabled)
            CaptureWindow(p)
        for key,p in c.pickups.items():
            if key.endswith('grap2'):
                self.assertTrue(p.departure)
            else:
                self.assertTrue(p.last_departure)
                self.assertEqual(p.next_cube_distance_mm,80)
                self.assertFalse(p.next_blind.validated)
                self.assertTrue(p.next_blind.trial_enabled)
                self.assertFalse(p.next_acquire.validated)

    def test_default_still_disables_unconfigured_trial(self):
        self.assertFalse(TransitionConfig().trial_run)
        a=Actions(None,None,transport=object(),pickup_trial_enabled=True)
        self.assertTrue(a.pickup_trial_enabled)
        self.assertFalse(a.pickup_full_lift_validated)
        import main
        self.assertFalse(main.parse_args([]).trial_optimizations)
