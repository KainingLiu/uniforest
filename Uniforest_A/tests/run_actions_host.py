"""Compile the real C executor against fake hardware; requires a native cc."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FirmwareActionsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.binary = str(Path(cls.temporary.name) / 'actions_test')
        subprocess.run([
            os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
            '-I' + str(ROOT / 'tests/host'), '-I' + str(ROOT / 'Core/Inc'),
            str(ROOT / 'Core/Src/actions.c'), str(ROOT / 'tests/host/actions_test.c'),
            '-o', cls.binary,
        ], check=True)

    def test_cancellation_busy_timeout_replay_and_tick_wrap(self):
        subprocess.run([self.binary], check=True, capture_output=True, text=True)

    def test_every_output_and_dwell_matches_pre_migration_python(self):
        traces = json.loads((ROOT / 'tests/host/actions_legacy_trace.json').read_text())
        for name, expected in traces.items():
            with self.subTest(action=name):
                action_id = 4 if name == 'build' else int(name[4])
                args = [self.binary, str(action_id)]
                if name.endswith('_test'):
                    args.append('test')
                result = subprocess.run(args, check=True, capture_output=True, text=True)
                actual = [json.loads(line) for line in result.stdout.splitlines()]
                self.assertEqual(actual, expected)


if __name__ == '__main__':
    unittest.main(verbosity=2)
