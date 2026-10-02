"""Run archives tested with temporary storage and hardware-free subprocesses."""
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

from utils.diagnostics import JsonlDiagnostics
from utils.run_logs import RunArchive, current_archive, run_entry

ROOT = Path(__file__).resolve().parents[1]


class RunLogTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / 'runs'

    def test_console_diagnostics_threads_and_optional_path_are_saved(self):
        terminal, errors = io.StringIO(), io.StringIO()
        extra = Path(self.temporary.name) / 'extra.jsonl'
        with contextlib.redirect_stdout(terminal), contextlib.redirect_stderr(errors):
            with RunArchive('main', root=self.root) as archive:
                print('normal output 中文')
                print('error output', file=sys.stderr)
                diagnostics = JsonlDiagnostics(str(extra))
                def worker(index):
                    print(f'worker-{index}')
                    diagnostics.write('sample', index=index)
                threads = [threading.Thread(target=worker, args=(i,)) for i in range(8)]
                for thread in threads:
                    thread.start()
                for thread in threads:
                    thread.join()
                # Flush is immediate: data is readable while the process is active.
                self.assertIn('normal output 中文', (archive.directory / 'console.log').read_text(encoding='utf-8'))
                self.assertEqual(len(archive.diagnostics_path.read_text().splitlines()), 8)
        console = (archive.directory / 'console.log').read_text(encoding='utf-8')
        self.assertIn('normal output 中文', terminal.getvalue())
        self.assertIn('error output', errors.getvalue())
        self.assertIn('error output', console)
        self.assertTrue(all(f'worker-{i}' in console for i in range(8)))
        self.assertEqual(extra.read_bytes(), archive.diagnostics_path.read_bytes())
        self.assertIsNone(current_archive())

    def test_automatic_diagnostics_do_not_require_explicit_path_or_duplicate_same_path(self):
        with contextlib.redirect_stdout(io.StringIO()), RunArchive('task2_main', root=self.root) as archive:
            JsonlDiagnostics().write('automatic')
            JsonlDiagnostics(str(archive.diagnostics_path)).write('same_path')
        events = [json.loads(line)['event'] for line in archive.diagnostics_path.read_text().splitlines()]
        self.assertEqual(events, ['automatic', 'same_path'])

    def test_keeps_latest_100_runs_and_preserves_unrelated_directories(self):
        self.root.mkdir()
        unrelated = self.root / 'field-calibration'
        unrelated.mkdir()
        (unrelated / 'keep.txt').write_text('keep')
        fake = self.root / '20000101-000000-000000-main-12345678'
        fake.mkdir()
        (fake / 'run.json').write_text('{}')
        paths = []
        with contextlib.redirect_stdout(io.StringIO()):
            for _ in range(103):
                with RunArchive('main', root=self.root) as archive:
                    paths.append(archive.directory)
        self.assertFalse(any(p.exists() for p in paths[:3]))
        self.assertTrue(all(p.exists() for p in paths[3:]))
        self.assertEqual(len([p for p in self.root.iterdir() if p.is_dir() and p not in (unrelated, fake)]), 100)
        self.assertTrue((unrelated / 'keep.txt').exists())
        self.assertTrue(fake.exists())

    def test_live_run_is_protected_and_retention_catches_up_after_exit(self):
        with contextlib.redirect_stdout(io.StringIO()):
            with RunArchive('main', root=self.root, keep=1) as old:
                with RunArchive('task2_main', root=self.root, keep=1) as new:
                    pass
                self.assertTrue(old.directory.exists())
                print('active log still writable')
                self.assertIn('still writable', (old.directory / 'console.log').read_text())
            self.assertFalse(old.directory.exists())
            self.assertTrue(new.directory.exists())

    def test_crashed_writer_lock_is_released_and_partial_log_survives(self):
        script = """import os
from utils.run_logs import RunArchive
with RunArchive('main'):
    print('before abrupt exit', flush=True)
    os._exit(17)
"""
        result = subprocess.run([sys.executable, '-B', '-c', script], cwd=ROOT,
                                env={**os.environ, 'UNIFOREST_RUN_LOG_DIR': str(self.root)},
                                capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 17)
        old = next(p for p in self.root.iterdir() if p.is_dir())
        self.assertIn('before abrupt exit', (old / 'console.log').read_text())
        self.assertEqual(json.loads((old / 'run.json').read_text())['status'], 'running')
        with contextlib.redirect_stdout(io.StringIO()), RunArchive('main', root=self.root, keep=1):
            self.assertFalse(old.exists())

    def test_launch_records_import_failures_interruptions_and_exit_codes(self):
        for failure, code in ((ImportError('missing dependency'), 1), (KeyboardInterrupt(), 130),
                              (SystemExit(2), 2)):
            with self.subTest(code=code), patch.dict(os.environ, {'UNIFOREST_RUN_LOG_DIR': str(self.root)}), \
                    patch('utils.run_logs.importlib.import_module', side_effect=failure), \
                    contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(run_entry('main'), code)
            latest = sorted(p for p in self.root.iterdir() if p.is_dir())[-1]
            metadata = json.loads((latest / 'run.json').read_text())
            self.assertEqual(metadata['exit_code'], code)
            self.assertEqual(metadata['status'], 'failed')
            self.assertIn('ended_at', metadata)
            if code == 1:
                self.assertIn('ImportError: missing dependency', (latest / 'console.log').read_text())

    def test_entry_subprocesses_log_help_preview_and_invalid_arguments(self):
        cases = [('main.py', ['--show-plan'], 0), ('task2_main.py', ['--help'], 0),
                 ('robot.py', ['--help'], 0), ('main.py', ['--invalid-option'], 2)]
        for name, args, code in cases:
            with self.subTest(entry=name, args=args):
                result = subprocess.run([sys.executable, '-B', str(ROOT / name), *args],
                                        cwd=self.temporary.name,
                                        env={**os.environ, 'UNIFOREST_RUN_LOG_DIR': str(self.root), 'PYTHONUTF8': '1'},
                                        capture_output=True, text=True, encoding='utf-8', timeout=20)
                self.assertEqual(result.returncode, code, result.stdout + result.stderr)
                latest = sorted(p for p in self.root.iterdir() if p.is_dir())[-1]
                record = json.loads((latest / 'run.json').read_text())
                self.assertEqual(record['entry'], Path(name).stem)
                self.assertEqual(record['exit_code'], code)
                self.assertTrue((latest / 'diagnostics.jsonl').is_file())
                console = (latest / 'console.log').read_text(encoding='utf-8')
                self.assertIn('[RunLog] Exit code:', console)
                if code == 2:
                    self.assertIn('unrecognized arguments', console)

    def test_storage_failure_before_launch_never_imports_robot_entry(self):
        self.root.parent.mkdir(exist_ok=True)
        self.root.write_text('a file blocks the directory')
        with patch.dict(os.environ, {'UNIFOREST_RUN_LOG_DIR': str(self.root)}), \
                patch('utils.run_logs.importlib.import_module') as load, \
                contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(run_entry('main'), 1)
        load.assert_not_called()

    def test_runtime_storage_error_does_not_interrupt_control(self):
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()) as errors:
            with RunArchive('main', root=self.root) as archive:
                with patch.object(archive.console, 'write', side_effect=OSError('disk full')):
                    print('control continues')
                    print('only one warning')
        self.assertEqual(errors.getvalue().count('Log storage failed'), 1)


if __name__ == '__main__':
    unittest.main()
