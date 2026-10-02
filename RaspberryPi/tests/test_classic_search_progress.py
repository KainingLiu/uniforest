"""Report command budget, measured motion and vision evidence without changing search policy."""

import contextlib
import io
import unittest
from unittest.mock import Mock

from Strategy.errors import SearchRangeExhausted
from tests.test_visual_fallback import SearchReplay, block


class ClassicSearchProgressTests(unittest.TestCase):
    def setUp(self):
        self.output = io.StringIO()
        self.quiet = contextlib.redirect_stdout(self.output)
        self.quiet.__enter__()
        self.addCleanup(self.quiet.__exit__, None, None, None)

    def stationary_search(self):
        replay = SearchReplay(lambda r: r.frame(), x=0.)
        replay.on_sleep = lambda: setattr(replay, 'x', 0.)
        replay.diagnostics = Mock()
        return replay

    def test_frozen_encoders_are_visible_in_diagnostics(self):
        replay = self.stationary_search()
        with self.assertRaises(SearchRangeExhausted):
            replay.run(limit=1800.)
        self.assertEqual(replay.x, 0.)
        self.assertAlmostEqual(replay.now-replay.start, 6.)
        self.assertEqual(replay.commands[-1][1], 0.)
        replay.diagnostics.write.assert_called_once()
        fields = replay.diagnostics.write.call_args.kwargs
        self.assertEqual(fields['reason'], 'distance_budget')
        self.assertEqual(fields['encoder_span_mm'], 0.)
        self.assertEqual(fields['command_budget_mm'], 1800.)
        self.assertGreater(fields['fresh_frames'], 0)
        self.assertEqual(fields['candidate_frames'], 0)

    def test_missing_camera_and_empty_frames_have_different_evidence(self):
        replay = SearchReplay(lambda r: None, x=0.)
        replay.diagnostics = Mock()
        with self.assertRaises(SearchRangeExhausted):
            replay.run(limit=90.)
        self.assertEqual(replay.diagnostics.write.call_args.kwargs['fresh_frames'], 0)
        self.assertEqual(replay.diagnostics.write.call_args.kwargs['candidate_frames'], 0)

    def test_stationary_confirmed_target_still_returns_for_grabbing(self):
        replay = SearchReplay(lambda r: r.frame([block()]), x=0.)
        replay.diagnostics = Mock()
        replay.on_sleep = lambda: setattr(replay, 'x', 0.)
        self.assertEqual(replay.run().color_name, 'orange')
        self.assertTrue(all(speed == 0 for _, speed in replay.commands))
        fields = replay.diagnostics.write.call_args.kwargs
        self.assertEqual(fields['reason'], 'acquired')
        self.assertGreaterEqual(fields['detected_frames'], 2)
        self.assertEqual(fields['clipped_frames'], 0)

    def test_real_search_travel_preserves_empty_field_fallback(self):
        replay = SearchReplay(lambda r: r.frame(), x=0.)
        replay.diagnostics = Mock()
        with self.assertRaises(SearchRangeExhausted):
            replay.run(limit=90.)
        self.assertAlmostEqual(replay.x, 90.)
        self.assertEqual(replay.diagnostics.write.call_args.kwargs['reason'], 'distance_budget')


if __name__ == '__main__':
    unittest.main()
