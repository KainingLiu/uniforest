#!/usr/bin/env python3
"""Inspect the automatic dataset, or collect stationary scenes without UART."""

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import sqlite3
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vision import CubeDetector, default_camera_selector
from vision.yolo.collector import CollectionConfig, CubeDataCollector


def library_stats(directory):
    """Read-only inspection; never creates a missing library or changes labels."""
    database = directory / 'catalog.sqlite3'
    if not database.is_file():
        return {'directory': str(directory), 'sessions': [], 'scenes': [],
                'total_images': 0, 'total_image_bytes': 0}
    connection = sqlite3.connect(database.as_uri() + '?mode=ro', uri=True)
    connection.row_factory = sqlite3.Row
    try:
        sessions = [dict(row) for row in connection.execute(
            'SELECT session_id, started_at, ended_at, status, reason, image_count, '
            'image_bytes FROM sessions ORDER BY started_at DESC')]
        scenes = [dict(row) for row in connection.execute(
            'SELECT task, phase, profile, annotation_status, COUNT(*) AS images '
            'FROM frames GROUP BY task, phase, profile, annotation_status '
            'ORDER BY task, phase, profile')]
        return {'directory': str(directory), 'sessions': sessions, 'scenes': scenes,
                'total_images': sum(row['image_count'] for row in sessions),
                'total_image_bytes': sum(row['image_bytes'] for row in sessions)}
    finally:
        connection.close()


def capture(args, config):
    config = replace(config, enabled=True,
                     sample_hz=args.hz if args.hz is not None else config.sample_hz)
    collector = CubeDataCollector(config, manual_capture=True)
    collector.set_context(task='manual_capture', phase='capture', scene_note=args.scene)
    detector = CubeDetector(camera_id=args.camera, show_gui=args.gui)
    detector.set_detection_profile(args.profile)
    detector.set_frame_sink(collector.offer_frame)
    try:
        collector.start()
        if not detector.start():
            return 1
        deadline = time.monotonic() + args.duration if args.duration else math.inf
        while detector.is_running and time.monotonic() < deadline:
            if collector.status not in ('starting', 'running'):
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        detector.stop()
        collector.stop()
    return 0 if collector.status == 'closed' and collector.saved else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)
    stats = subparsers.add_parser('stats', help='Read the local dataset index')
    stats.add_argument('--dataset-dir', default=None)
    stats.add_argument('--json', action='store_true')
    record = subparsers.add_parser('capture', help='Camera only; sends no robot commands')
    record.add_argument('--dataset-dir', default=None)
    record.add_argument('--camera', default=default_camera_selector())
    record.add_argument('--duration', type=float, default=60.0,
                        help='Seconds to record; 0 runs until Ctrl+C (default: 60)')
    record.add_argument('--hz', type=float, default=None)
    record.add_argument('--scene', default='', help='Human scene note; not a training label')
    record.add_argument('--profile', default='default',
                        choices=('default', 'task2_orange', 'task2_purple', 'building'))
    record.add_argument('--gui', action='store_true')
    args = parser.parse_args(argv)
    if args.command == 'capture' and (not math.isfinite(args.duration) or args.duration < 0):
        parser.error('--duration must be finite and non-negative')
    try:
        config = CollectionConfig.load(data_dir=args.dataset_dir)
        if args.command == 'capture':
            return capture(args, config)
        report = library_stats(config.directory)
        if args.json:
            print(json.dumps(report, ensure_ascii=False, indent=2))
        else:
            print(f"Dataset: {report['directory']}")
            print(f"Sessions: {len(report['sessions'])}; images: {report['total_images']}; "
                  f"image size: {report['total_image_bytes'] / 1024**2:.1f} MiB")
            for row in report['sessions'][:10]:
                print(f"  {row['session_id']}: {row['status']}, "
                      f"{row['image_count']} images {row['reason'] or ''}")
            for row in report['scenes']:
                print(f"  {row['task']} / {row['phase']} / {row['profile']}: "
                      f"{row['images']} ({row['annotation_status']})")
        return 0
    except (OSError, ValueError, TypeError, sqlite3.Error) as exc:
        print(f'[Dataset] {type(exc).__name__}: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
