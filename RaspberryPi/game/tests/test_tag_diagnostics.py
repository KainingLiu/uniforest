"""Distinguish decoder failures from pose rejection and stale camera data."""
from types import SimpleNamespace
import unittest
from Strategy.tag_alignment import missing_tag_details


class TagDiagnosticTests(unittest.TestCase):
    def details(self, pose):
        return missing_tag_details(pose, 6, now=10, max_age_s=.3, not_before=9)

    def test_no_frame_and_frame_predating_alignment(self):
        self.assertIn('reason=no_frame', self.details(None))
        self.assertIn('no_new_frame_since_alignment', self.details(
            SimpleNamespace(timestamp=8)))

    def test_stale_frame_is_not_reported_as_decoder_failure(self):
        self.assertIn('reason=stale_frame', self.details(
            SimpleNamespace(timestamp=9.5, raw_tag_ids=())))

    def test_raw_tag_rejected_even_if_another_tag_is_valid(self):
        pose = SimpleNamespace(timestamp=9.9, raw_tag_ids=(6, 7), tag_ids=(7,),
            tag_solutions=[SimpleNamespace(tag_id=7)],
            rejection_reasons=('tag6:height_error(0.4>0.35m)',))
        text = self.details(pose)
        self.assertIn('reason=pose_rejected', text)
        self.assertIn('height_error', text)
        self.assertIn('accepted_ids=(7,)', text)

    def test_no_decoded_target(self):
        self.assertIn('reason=tag_not_decoded', self.details(
            SimpleNamespace(timestamp=9.9, raw_tag_ids=(7,))))

    def test_capture_fault_takes_precedence_over_old_empty_frame(self):
        text = self.details(SimpleNamespace(timestamp=1, raw_tag_ids=(),
                                            capture_error='capture_stalled_reopening'))
        self.assertIn('reason=camera_unavailable', text)
        self.assertIn('age=9.000s', text)
        self.assertIn('capture_stalled_reopening', text)


if __name__ == '__main__':
    unittest.main()
