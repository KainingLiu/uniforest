"""Per-process console/diagnostic archives with bounded, active-safe retention."""
from contextvars import ContextVar
from datetime import datetime, timedelta, timezone
import importlib
import json
import os
from pathlib import Path
import re
import shutil
import sys
import threading
import traceback
import uuid

ROOT = Path(__file__).resolve().parents[1] / 'logs' / 'runs'
DEFAULT_RUNTIME = 'game'
KEEP_RUNS = 100
FORMAT = 'uniforest-run-log-v1'
RUN_NAME = re.compile(r'^\d{8}-\d{6}-\d{6}-[a-zA-Z0-9_]+-[0-9a-f]{8}$')
LOCAL_TIME = timezone(timedelta(hours=8), 'Asia/Shanghai')
_current = ContextVar('uniforest_run_archive', default=None)


def current_archive():
    return _current.get()


def _lock(path):
    """Return an exclusively locked file, or None when another process owns it."""
    stream = path.open('a+b')
    try:
        if os.name == 'nt':
            import msvcrt
            if stream.seek(0, 2) == 0:
                stream.write(b'\0')
                stream.flush()
            stream.seek(0)
            msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        stream.close()
        return None
    return stream


def _owned(path, root):
    if not RUN_NAME.fullmatch(path.name) or path.is_symlink() or not path.is_dir():
        return False
    if path.resolve().parent != root.resolve():
        return False
    try:
        return json.loads((path / 'run.json').read_text(encoding='utf-8')).get('format') == FORMAT
    except (OSError, ValueError, AttributeError):
        return False


def prune_runs(root, keep=KEEP_RUNS):
    """Only delete our old archives; live writers hold an OS lock until exit."""
    if keep < 1:
        raise ValueError('at least one run must be retained')
    root = Path(root).resolve()
    guard = _lock(root / '.retention.lock')
    if guard is None:
        return
    try:
        archives = sorted(p for p in root.iterdir() if _owned(p, root))
        for path in archives[:-keep]:
            active = _lock(path / '.active.lock')
            if active is None:
                continue
            # Windows cannot remove an open file. A unique run directory is
            # never reopened by a writer after its lifetime lock is released.
            active.close()
            if _owned(path, root):
                shutil.rmtree(path)
    finally:
        guard.close()


class _Tee:
    def __init__(self, original, archive):
        self.original, self.archive = original, archive

    def write(self, text):
        with self.archive.mutex:
            self.archive.append(self.archive.console, text)
            return self.original.write(text)

    def flush(self):
        with self.archive.mutex:
            self.original.flush()
            self.archive.flush(self.archive.console)

    def __getattr__(self, name):
        return getattr(self.original, name)


class RunArchive:
    def __init__(self, entry, *, root=None, keep=KEEP_RUNS):
        if not re.fullmatch(r'[a-zA-Z0-9_]+', entry) or keep < 1:
            raise ValueError('invalid archive entry or retention count')
        self.root = Path(root if root is not None else os.environ.get('UNIFOREST_RUN_LOG_DIR', ROOT)).resolve()
        self.keep = keep
        self.mutex = threading.RLock()
        self.stderr = sys.stderr
        self.failed = False
        self.closed = False
        now = datetime.now(LOCAL_TIME)
        self.directory = self.root / f'{now:%Y%m%d-%H%M%S-%f}-{entry}-{uuid.uuid4().hex[:8]}'
        self.diagnostics_path = self.directory / 'diagnostics.jsonl'
        self.metadata = dict(format=FORMAT, runtime=DEFAULT_RUNTIME, entry=entry, pid=os.getpid(),
                             started_at=now.isoformat(), status='running', exit_code=None)

    def _metadata(self):
        temporary = self.directory / 'run.json.tmp'
        temporary.write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        temporary.replace(self.directory / 'run.json')

    def warn(self, exc):
        if not self.failed:
            self.failed = True
            try:
                self.stderr.write(f'[RunLog] Log storage failed: {exc}\n')
                self.stderr.flush()
            except OSError:
                pass

    def flush(self, stream):
        if self.closed:
            return
        try:
            stream.flush()
        except OSError as exc:
            self.warn(exc)

    def append(self, stream, text):
        if self.closed:
            return
        try:
            stream.write(text)
            stream.flush()
        except OSError as exc:
            self.warn(exc)

    def diagnostic(self, line):
        with self.mutex:
            self.append(self.events, line)

    def cleanup(self):
        try:
            prune_runs(self.root, self.keep)
        except OSError as exc:
            self.warn(exc)

    def __enter__(self):
        self.directory.mkdir(parents=True, exist_ok=False)
        self.active = _lock(self.directory / '.active.lock')
        if self.active is None:
            raise OSError('new run archive is already locked')
        try:
            self._metadata()
            self.console = (self.directory / 'console.log').open('a', encoding='utf-8', errors='backslashreplace')
            try:
                self.events = self.diagnostics_path.open('a', encoding='utf-8')
            except BaseException:
                self.console.close()
                raise
        except BaseException:
            self.active.close()
            raise
        self.stdout, self.stderr = sys.stdout, sys.stderr
        self.token = _current.set(self)
        sys.stdout, sys.stderr = _Tee(self.stdout, self), _Tee(self.stderr, self)
        self.cleanup()
        print(f'[RunLog] {self.directory}', flush=True)
        return self

    def __exit__(self, exc_type, exc, tb):
        sys.stdout, sys.stderr = self.stdout, self.stderr
        _current.reset(self.token)
        with self.mutex:
            failed = exc_type is not None or self.metadata['exit_code'] not in (None, 0)
            self.metadata.update(ended_at=datetime.now(LOCAL_TIME).isoformat(),
                                 status='failed' if failed else 'finished')
            try:
                self._metadata()
            except OSError as error:
                self.warn(error)
            for stream in (self.console, self.events):
                try:
                    stream.close()
                except OSError as error:
                    self.warn(error)
            self.closed = True
        self.active.close()
        self.cleanup()


def _invoke_entry(module, function):
    try:
        result = getattr(importlib.import_module(module), function)()
        return 0 if result is None else int(result)
    except SystemExit as exc:
        if exc.code is not None and not isinstance(exc.code, int):
            print(exc.code, file=sys.stderr)
        return exc.code if isinstance(exc.code, int) else 0 if exc.code is None else 1
    except KeyboardInterrupt:
        print('[RunLog] Interrupted', file=sys.stderr)
        return 130
    except BaseException:
        traceback.print_exc()
        return 1


def run_entry(module, function='main', *, require_archive=True):
    """Archive before imports; optional storage never prevents legacy startup."""
    archive = RunArchive(module)
    try:
        archive.__enter__()
    except OSError as exc:
        print(f'[RunLog] Cannot create run archive: {exc}', file=sys.stderr)
        return 1 if require_archive else _invoke_entry(module, function)
    try:
        code = _invoke_entry(module, function)
        archive.metadata['exit_code'] = code
        print(f'[RunLog] Exit code: {code}', flush=True)
        return code
    finally:
        archive.__exit__(None, None, None)
