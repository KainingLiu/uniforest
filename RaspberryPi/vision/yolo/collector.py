"""Bounded, asynchronous collection of unlabelled cube-camera images.

The camera thread only rate-limits and copies selected frames into a bounded
queue. JPEG encoding, similarity checks, filesystem and SQLite access belong
to the writer thread. Collection failures must never stop robot control.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import queue
import shutil
import sqlite3
import threading
import time
import uuid

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = Path(__file__).with_name('collection_config.json')
GIB = 1024 ** 3


@dataclass(frozen=True)
class CollectionConfig:
    enabled: bool = True
    data_dir: str = 'vision/yolo/data/collection'
    sample_hz: float = 2.0
    static_keep_seconds: float = 5.0
    dedup_mean_abs_diff: float = 2.0
    jpeg_quality: int = 95
    queue_size: int = 4
    max_images_gb: float = 8.0
    min_free_gb: float = 2.0
    max_session_images: int = 5000
    batch_tag: str = ''

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError('enabled must be true or false')
        for name in ('sample_hz', 'static_keep_seconds', 'max_images_gb'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('dedup_mean_abs_diff', 'min_free_gb'):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and non-negative')
        for name, low, high in (('jpeg_quality', 1, 100), ('queue_size', 1, 64),
                                ('max_session_images', 1, 1000000)):
            value = getattr(self, name)
            if type(value) is not int or not low <= value <= high:
                raise ValueError(f'{name} must be an integer in [{low}, {high}]')
        if not isinstance(self.data_dir, str) or not self.data_dir.strip():
            raise ValueError('data_dir must be a non-empty path')
        if not isinstance(self.batch_tag, str):
            raise ValueError('batch_tag must be a string')

    @property
    def directory(self):
        path = Path(self.data_dir).expanduser()
        return (path if path.is_absolute() else ROOT / path).resolve()

    @classmethod
    def load(cls, *, enabled=None, data_dir=None):
        values = json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
        env_enabled = os.environ.get('UNIFOREST_COLLECT_DATA')
        if env_enabled is not None:
            if env_enabled not in ('0', '1'):
                raise ValueError('UNIFOREST_COLLECT_DATA must be 0 or 1')
            values['enabled'] = env_enabled == '1'
        if 'UNIFOREST_DATASET_DIR' in os.environ:
            values['data_dir'] = os.environ['UNIFOREST_DATASET_DIR']
        if 'UNIFOREST_DATASET_BATCH' in os.environ:
            values['batch_tag'] = os.environ['UNIFOREST_DATASET_BATCH']
        if enabled is not None:
            values['enabled'] = enabled
        if data_dir is not None:
            values['data_dir'] = str(data_dir)
        return cls(**values)


SCHEMA = """
CREATE TABLE IF NOT EXISTS sessions (
    session_id TEXT PRIMARY KEY,
    started_at REAL NOT NULL,
    ended_at REAL,
    status TEXT NOT NULL,
    reason TEXT,
    image_count INTEGER NOT NULL DEFAULT 0,
    image_bytes INTEGER NOT NULL DEFAULT 0,
    config_json TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS frames (
    frame_id TEXT PRIMARY KEY,
    session_id TEXT NOT NULL REFERENCES sessions(session_id),
    image_path TEXT NOT NULL UNIQUE,
    captured_at REAL NOT NULL,
    captured_monotonic REAL NOT NULL,
    task TEXT NOT NULL,
    phase TEXT NOT NULL,
    profile TEXT NOT NULL,
    annotation_status TEXT NOT NULL DEFAULT 'unlabelled',
    image_bytes INTEGER NOT NULL,
    metadata_json TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS frames_session ON frames(session_id);
CREATE INDEX IF NOT EXISTS frames_scene ON frames(task, phase, profile);
"""


class CollectionLimit(RuntimeError):
    pass


def _write_json(path, record):
    temporary = path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(record, ensure_ascii=False, indent=2,
                                   allow_nan=False), encoding='utf-8')
    temporary.replace(path)


class CubeDataCollector:
    """One collection session per camera/Robot start; no hardware commands."""

    def __init__(self, config: CollectionConfig, *, manual_capture=False):
        self.config = config
        self._manual_capture = manual_capture
        self.directory = config.directory
        self.session_id = (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
                           + '_' + uuid.uuid4().hex[:8])
        self.session_dir = self.directory / 'runs' / self.session_id
        self._queue = queue.Queue(maxsize=config.queue_size)
        self._stop = threading.Event()
        self._thread = None
        self._context_lock = threading.Lock()
        self._context = {'task': 'manual', 'phase': 'idle', 'flow_id': '',
                         'scene_note': ''}
        self._context_generation = 0
        self._next_sample = float('-inf')
        self._last_seen = float('-inf')
        self._last_offered_context = None
        self._accepting = False
        self.status = 'disabled' if not config.enabled else 'pending'
        self.reason = ''
        self.saved = 0
        self.dropped = 0
        self.duplicates = 0
        self.started_at = None

    def set_context(self, **fields):
        allowed = {'task', 'phase', 'flow_id', 'scene_note'}
        with self._context_lock:
            updates = {k: str(v) for k, v in fields.items() if k in allowed}
            if any(self._context[k] != v for k, v in updates.items()):
                self._context_generation += 1
                self._context.update(updates)

    def _capture_allowed(self, context, profile):
        if self._manual_capture:
            return True  # Explicit tools/collect_cube_data.py capture only.
        phase = context['phase']
        if phase in ('ORANGE_SEARCH', 'ORANGE_ALIGN'):
            return profile in ('default', 'task2_orange')
        return (phase in ('PURPLE_SEARCH', 'PURPLE_ALIGN')
                and profile == 'task2_purple')

    def start(self):
        if not self.config.enabled or self._thread is not None:
            return
        self.started_at = time.time()
        self.status = 'starting'
        self._accepting = True
        self._thread = threading.Thread(target=self._run,
                                        name='cube-dataset-writer', daemon=True)
        try:
            self._thread.start()
        except Exception as exc:
            self._thread = None
            self._accepting = False
            self.status, self.reason = 'error', str(exc)
            print(f'[Dataset] Could not start collection: {exc}')

    def offer_frame(self, frame, captured_monotonic, captured_at,
                    profile, camera):
        """Called once per successful camera read, before any overlay or ROI.

        The producer is the single camera thread. No encoding, image analysis,
        disk access or waiting for queue space happens here.
        """
        if not self._accepting or captured_monotonic <= self._last_seen:
            return
        self._last_seen = captured_monotonic
        with self._context_lock:
            context = dict(self._context)
            generation = self._context_generation
        if not self._capture_allowed(context, profile):
            return
        # A refill may return to exactly the same phase/profile after a grab.
        # Keep its first frame even if no excluded-phase frame reached us.
        context_key = (generation, profile, tuple(context.items()))
        changed = context_key != self._last_offered_context
        if not changed and captured_monotonic < self._next_sample:
            return
        if self._queue.full():
            self.dropped += 1
            return
        metadata = {
            'schema_version': 1,
            'session_id': self.session_id,
            'split_group': self.config.batch_tag or self.session_id,
            'batch_tag': self.config.batch_tag,
            'captured_at': captured_at,
            'captured_monotonic': captured_monotonic,
            'timestamp_source': 'host_after_camera_read',
            'profile': profile,
            'camera': camera,
            'annotation_status': 'unlabelled',
            **context,
        }
        try:
            self._queue.put_nowait((frame.copy(), metadata, context_key))
        except queue.Full:
            self.dropped += 1
            return
        self._last_offered_context = context_key
        self._next_sample = captured_monotonic + 1.0 / self.config.sample_hz

    def stop(self, timeout=2.0):
        """Bound shutdown waiting; a slow disk cannot hold robot stop hostage."""
        self._accepting = False
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            if self._thread.is_alive():
                print('[Dataset] Writer still closing; queued images may be lost on exit')

    def _session_record(self):
        return {'schema_version': 1, 'session_id': self.session_id,
                'started_at': self.started_at, 'status': self.status,
                'reason': self.reason, 'saved': self.saved,
                'queue_dropped': self.dropped, 'duplicates_skipped': self.duplicates,
                'config': asdict(self.config),
                'annotation_status': 'unlabelled'}

    def _run(self):
        connection = None
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            if shutil.disk_usage(self.directory).free < self.config.min_free_gb * GIB:
                raise CollectionLimit('minimum free disk space reached')
            connection = sqlite3.connect(self.directory / 'catalog.sqlite3', timeout=0.5)
            connection.execute('PRAGMA journal_mode=WAL')
            connection.execute('PRAGMA foreign_keys=ON')
            connection.executescript(SCHEMA)
            self.session_dir.mkdir(parents=True, exist_ok=False)
            (self.session_dir / 'images').mkdir()
            with connection:
                connection.execute(
                    'INSERT INTO sessions(session_id, started_at, status, config_json) '
                    'VALUES (?, ?, ?, ?)',
                    (self.session_id, self.started_at, 'running',
                     json.dumps(asdict(self.config))))
            self.status = 'running'
            _write_json(self.session_dir / 'session.json', self._session_record())
            print(f'[Dataset] Collecting unlabelled images: {self.session_dir}')
            last_thumb = None
            last_time = float('-inf')
            last_key = None
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    frame, metadata, context_key = self._queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                try:
                    # Keep colour information; compare with the last SAVED image.
                    thumb = cv2.resize(frame, (64, 48), interpolation=cv2.INTER_AREA)
                    captured = metadata['captured_monotonic']
                    difference = (np.abs(thumb.astype(np.int16)
                                         - last_thumb.astype(np.int16))
                                  if last_thumb is not None else None)
                    if (context_key == last_key and last_thumb is not None
                            and captured - last_time < self.config.static_keep_seconds
                            and difference.mean() < self.config.dedup_mean_abs_diff
                            # A small moving object can barely affect the mean.
                            and difference.max() < max(
                                8, 4 * self.config.dedup_mean_abs_diff)):
                        self.duplicates += 1
                        continue
                    self._save_frame(connection, frame, metadata)
                    last_thumb, last_time, last_key = thumb, captured, context_key
                finally:
                    self._queue.task_done()
            self.status = 'closed'
        except CollectionLimit as exc:
            self.status, self.reason = 'limit_reached', str(exc)
            print(f'[Dataset] Collection stopped: {exc}')
        except Exception as exc:
            self.status, self.reason = 'error', f'{type(exc).__name__}: {exc}'
            print(f'[Dataset] Collection disabled: {self.reason}')
        finally:
            self._accepting = False
            # Discard unsaved queued frames after a limit/error, releasing memory.
            while True:
                try:
                    self._queue.get_nowait()
                    self._queue.task_done()
                    self.dropped += 1
                except queue.Empty:
                    break
            ended_at = time.time()
            try:
                if connection is not None:
                    with connection:
                        connection.execute(
                            'UPDATE sessions SET ended_at=?, status=?, reason=? '
                            'WHERE session_id=?',
                            (ended_at, self.status, self.reason, self.session_id))
                if self.session_dir.exists():
                    _write_json(self.session_dir / 'session.json',
                                {**self._session_record(), 'ended_at': ended_at})
            except Exception as exc:
                print(f'[Dataset] Could not finalize collection metadata: {exc}')
            finally:
                if connection is not None:
                    connection.close()
            print(f'[Dataset] {self.status}: saved={self.saved}, '
                  f'duplicates={self.duplicates}, dropped={self.dropped}')

    def _save_frame(self, connection, frame, metadata):
        if self.saved >= self.config.max_session_images:
            raise CollectionLimit('session image limit reached')
        ok, encoded = cv2.imencode('.jpg', frame,
                                  [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality])
        if not ok:
            raise OSError('JPEG encoding failed')
        image_bytes = encoded.tobytes()
        name = f'{self.saved + 1:06d}'
        image_path = self.session_dir / 'images' / f'{name}.jpg'
        sidecar_path = image_path.with_suffix('.json')
        temporary = image_path.with_suffix('.jpg.tmp')
        metadata = {**metadata, 'frame_id': f'{self.session_id}/{name}',
                    'image_path': image_path.relative_to(self.directory).as_posix(),
                    'width': frame.shape[1], 'height': frame.shape[0]}
        record = json.dumps(metadata, ensure_ascii=False, allow_nan=False)
        try:
            # Serialize the budget check and insert across concurrent sessions.
            connection.execute('BEGIN IMMEDIATE')
            used = connection.execute(
                'SELECT COALESCE(SUM(image_bytes), 0) FROM sessions').fetchone()[0]
            if used + len(image_bytes) > self.config.max_images_gb * GIB:
                raise CollectionLimit('library image budget reached')
            required = len(image_bytes) + len(record.encode('utf-8')) + 65536
            if (shutil.disk_usage(self.directory).free - required
                    < self.config.min_free_gb * GIB):
                raise CollectionLimit('minimum free disk space reached')
            temporary.write_bytes(image_bytes)
            temporary.replace(image_path)
            _write_json(sidecar_path, metadata)
            connection.execute(
                'INSERT INTO frames(frame_id, session_id, image_path, captured_at, '
                'captured_monotonic, task, phase, profile, image_bytes, metadata_json) '
                'VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)',
                (metadata['frame_id'], self.session_id, metadata['image_path'],
                 metadata['captured_at'], metadata['captured_monotonic'],
                 metadata['task'], metadata['phase'], metadata['profile'],
                 len(image_bytes), record))
            connection.execute(
                'UPDATE sessions SET image_count=image_count+1, '
                'image_bytes=image_bytes+? WHERE session_id=?',
                (len(image_bytes), self.session_id))
            connection.commit()
        except Exception:
            connection.rollback()
            for path in (temporary, image_path, sidecar_path,
                         sidecar_path.with_suffix('.json.tmp')):
                path.unlink(missing_ok=True)
            raise
        self.saved += 1


def create_collector(*, enabled=None, data_dir=None):
    """Configuration errors disable collection, preserving robot availability."""
    try:
        config = CollectionConfig.load(enabled=enabled, data_dir=data_dir)
        return CubeDataCollector(config) if config.enabled else None
    except Exception as exc:
        print(f'[Dataset] Collection disabled (configuration): {exc}')
        return None


def add_collection_arguments(parser):
    parser.add_argument('--no-collect-data', action='store_true',
                        help='Disable automatic cube-image collection for this run')
    parser.add_argument('--dataset-dir', default=None,
                        help='Local image library directory (relative to RaspberryPi)')
