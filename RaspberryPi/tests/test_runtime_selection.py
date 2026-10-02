"""Compatibility against the pre-integration GitHub main and isolated game."""
import ast
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from utils.runtime_launcher import launch, split_runtime
from utils.run_logs import run_entry

ROOT = Path(__file__).resolve().parents[1]
BASELINE = json.loads((Path(__file__).parent / 'runtime_baseline.json').read_text(encoding='utf-8'))


def without_log_lines(text):
    return ''.join(line for line in text.splitlines(keepends=True)
                   if not line.startswith('[RunLog] '))


class RuntimeSelectionTests(unittest.TestCase):
    def test_main_existing_source_configuration_and_firmware_unchanged(self):
        for name, digest in BASELINE['protected'].items():
            with self.subTest(path=name):
                path = ROOT.parent / name
                if name.startswith('Uniforest_A/') and not (ROOT.parent / 'Uniforest_A').is_dir():
                    continue  # Pi deployments carry only RaspberryPi.
                self.assertEqual(hashlib.sha256(path.read_bytes().replace(b'\r\n', b'\n')).hexdigest(), digest)

    def test_entry_modules_equal_original_ast_after_removing_added_bootstrap(self):
        for entry, digest in BASELINE['entry_ast'].items():
            tree = ast.parse((ROOT / (entry + '.py')).read_text(encoding='utf-8'))
            # Only the one new early script guard is allowed; all original
            # imports, definitions, arguments and the old footer stay intact.
            self.assertIsInstance(tree.body[1], ast.If)
            self.assertEqual(tree.body[1].body[0].module, 'utils.runtime_launcher')
            del tree.body[1]
            # AST fields differ across Python 3.11/3.13. Parse the pinned source
            # with this interpreter so the comparison checks code, not version.
            self.assertEqual(ast.dump(tree), ast.dump(ast.parse(BASELINE['entry_sources'][entry])), entry)

    def test_every_legacy_selector_help_and_error_preserves_output_and_exit_code(self):
        with tempfile.TemporaryDirectory() as directory:
            baseline = Path(directory) / 'original'
            baseline.mkdir()
            for entry, source in BASELINE['entry_sources'].items():
                (baseline / (entry + '.py')).write_text(source, encoding='utf-8')
            for case in BASELINE['commands']:
                with self.subTest(arguments=case['arguments']):
                    expected = case
                    if case['code'] != 0 or '--help' in case['arguments']:
                        # argparse wrapping and camera/serial defaults vary by
                        # Python and OS. Execute only the frozen, read-only CLI
                        # cases with the same interpreter and protected imports.
                        old = subprocess.run([sys.executable, '-B',
                            str(baseline / case['arguments'][0]), *case['arguments'][1:]],
                            cwd=ROOT, env={**os.environ, 'PYTHONUTF8': '1', 'PYTHONPATH': str(ROOT)},
                            capture_output=True, text=True, encoding='utf-8', timeout=30)
                        expected = dict(code=old.returncode, stdout=old.stdout, stderr=old.stderr)
                    result = subprocess.run([sys.executable, '-B', *case['arguments']], cwd=ROOT,
                        env={**os.environ, 'PYTHONUTF8': '1', 'UNIFOREST_RUN_LOG_DIR': directory},
                        capture_output=True, text=True, encoding='utf-8', timeout=30)
                    self.assertEqual(result.returncode, expected['code'], result.stdout + result.stderr)
                    self.assertEqual(without_log_lines(result.stdout), expected['stdout'])
                    self.assertEqual(without_log_lines(result.stderr), expected['stderr'])

    def test_selector_is_exact_and_leaves_legacy_arguments_and_values_intact(self):
        legacy = ['--strategy', 'PlanB', '--diagnostics-log', 'relative.jsonl']
        self.assertEqual(split_runtime(legacy), ('main', legacy))
        self.assertEqual(split_runtime(['--runtime', 'main', *legacy]), ('main', legacy))
        self.assertEqual(split_runtime([*legacy, '--runtime=game']), ('game', legacy))
        self.assertEqual(split_runtime(['--', '--runtime', 'game']), ('main', ['--', '--runtime', 'game']))
        self.assertEqual(split_runtime(['--runt', 'game']), ('main', ['--runt', 'game']))

    def test_invalid_or_duplicate_runtime_never_imports_or_launches_controller(self):
        for args in (['--runtime'], ['--runtime='], ['--runtime', 'other'],
                     ['--runtime=main', '--runtime=game'], ['--runtime', '--show-plan']):
            with self.subTest(args=args), patch.object(sys, 'argv', ['main.py', *args]), \
                    patch('utils.runtime_launcher.os.execv') as execute, \
                    patch('utils.run_logs.importlib.import_module') as load, \
                    patch.object(sys, 'stderr', io.StringIO()):
                self.assertEqual(launch('main'), 2)
                execute.assert_not_called()
                load.assert_not_called()

    def test_game_exec_preserves_relative_arguments_interpreter_and_cwd(self):
        class Replaced(BaseException):
            pass
        before = Path.cwd()
        args = ['--runtime', 'game', '--transition-config', 'field.json', '--show-plan']
        with patch.object(sys, 'argv', ['main.py', *args]), \
                patch('utils.runtime_launcher.os.execv', side_effect=Replaced) as execute:
            with self.assertRaises(Replaced):
                launch('main')
        self.assertEqual(Path.cwd(), before)
        execute.assert_called_once_with(sys.executable,
            [sys.executable, '-u', '-B', str(ROOT / 'game/main.py'), *args[2:]])

    def test_game_real_process_uses_game_modules_and_marks_its_archive(self):
        with tempfile.TemporaryDirectory() as directory:
            # Use another cwd; the main task IDs do not include this game-only flow.
            for entry, args in [('main', ['--flow', 'build-staged', '--show-plan']),
                                ('task2_main', ['--show-plan']), ('robot', ['--help'])]:
                result = subprocess.run([sys.executable, '-B', str(ROOT / (entry + '.py')),
                    '--runtime=game', *args], cwd=directory,
                    env={**os.environ, 'PYTHONUTF8': '1', 'UNIFOREST_RUN_LOG_DIR': directory},
                    capture_output=True, text=True, encoding='utf-8', timeout=30)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            records = [json.loads(p.read_text()) for p in Path(directory).glob('*/run.json')]
            self.assertEqual(len(records), 3)
            self.assertEqual({r['runtime'] for r in records}, {'game'})
            self.assertEqual({r['entry'] for r in records}, {'main', 'task2_main', 'robot'})

    def test_default_main_does_not_depend_on_game_imports(self):
        with tempfile.TemporaryDirectory() as directory:
            script = '''import importlib.abc, sys
class RejectGame(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'game' or fullname.startswith('game.'):
            raise AssertionError('default main attempted game import')
sys.meta_path.insert(0, RejectGame())
from utils.runtime_launcher import launch
sys.argv = ['main.py', '--show-plan']
raise SystemExit(launch('main'))
'''
            result = subprocess.run([sys.executable, '-B', '-c', script], cwd=ROOT,
                env={**os.environ, 'PYTHONUTF8': '1', 'UNIFOREST_RUN_LOG_DIR': directory},
                capture_output=True, text=True, encoding='utf-8', timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            record = json.loads(next(Path(directory).glob('*/run.json')).read_text())
            self.assertEqual(record['runtime'], 'main')

    def test_optional_logging_storage_failure_does_not_disable_old_entry(self):
        with patch('utils.run_logs.RunArchive.__enter__', side_effect=OSError('disk full')), \
                patch('utils.run_logs.importlib.import_module') as load, \
                patch.object(sys, 'stderr', io.StringIO()):
            load.return_value.main.return_value = 7
            self.assertEqual(run_entry('main', require_archive=False), 7)
            load.return_value.main.assert_called_once_with()

    def test_calibration_and_legacy_wire_schema_are_identical_in_both_runtimes(self):
        paths = ['protocol/schema.json', 'protocol/commands.py',
                 'vision/opencv/camera_settings.json', 'vision/opencv/camera_calib.json',
                 'vision/opencv/camera_devices.json', 'vision/opencv/tag_camera_calib.json',
                 'vision/opencv/orange_config.py', 'vision/opencv/orange_fixed_geometry.py',
                 'vision/opencv/task1_orange_fixed_calibration.json',
                 'vision/opencv/task2_orange_fixed_calibration.json']
        for name in paths:
            with self.subTest(path=name):
                self.assertEqual((ROOT / name).read_text(encoding='utf-8'),
                                 (ROOT / 'game' / name).read_text(encoding='utf-8'))


if __name__ == '__main__':
    unittest.main()
