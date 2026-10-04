"""Offline calibration import preserves existing evidence and validates before saving."""

import json
from pathlib import Path
import tempfile
import unittest

from Strategy.building_profiles import DEFAULT_BUILDING_PROFILES_PATH, load_building_profiles
from tools.calibrate_plan_d_building import update_profile


class PlanDCalibrationToolTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.path = Path(temporary.name) / 'profiles.json'
        self.original = DEFAULT_BUILDING_PROFILES_PATH.read_bytes()
        self.path.write_bytes(self.original)
        self.sample = dict(layer=1, distance_mm=120.0, top_row_px=160.0,
                           calibration_date='2026-10-04', result='Synthetic test fixture only')

    def test_preview_does_not_change_file_and_write_preserves_other_layers(self):
        profile = update_profile(self.path, **self.sample)
        self.assertEqual(profile['z_scale_mm_px'], 19200.0)
        self.assertEqual(self.path.read_bytes(), self.original)
        update_profile(self.path, **self.sample, write=True)
        result = load_building_profiles(self.path)
        self.assertEqual(result[1].z_scale_mm_px, 19200.0)
        before = json.loads(self.original)
        after = json.loads(self.path.read_text(encoding='utf-8'))
        for layer in ('2', '3'):
            self.assertEqual(after['profiles'][layer], before['profiles'][layer])

    def test_invalid_measurements_cannot_replace_original_file(self):
        for update in ({'layer': True}, {'layer': 3}, {'distance_mm': 0},
                       {'distance_mm': float('nan')}, {'distance_mm': True},
                       {'top_row_px': 480}, {'top_row_px': -1},
                       {'distance_mm': 120, 'top_row_px': 400},
                       {'calibration_date': '2026-13-04'}, {'result': ''}):
            with self.subTest(update=update), self.assertRaises((ValueError, TypeError)):
                update_profile(self.path, **(self.sample | update), write=True)
            self.assertEqual(self.path.read_bytes(), self.original)
            self.assertEqual(list(self.path.parent.iterdir()), [self.path])


if __name__ == '__main__':
    unittest.main()
