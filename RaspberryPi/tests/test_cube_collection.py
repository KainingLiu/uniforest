"""Dataset persistence and robot integration tests; no camera/UART is opened."""

from dataclasses import replace
from contextlib import closing
import json
from pathlib import Path
import sqlite3
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from vision.yolo.collector import CollectionConfig, CubeDataCollector, create_collector
from vision import CubeDetector
from tools.collect_cube_data import library_stats


MODULE = 'vision.yolo.collector'


class CollectionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name) / 'library'
        self.config = CollectionConfig(data_dir=str(self.directory), min_free_gb=0,
                                       queue_size=16)
        self.frame = np.full((48, 64, 3), (20, 80, 160), dtype=np.uint8)
        self.camera = {'selector': 'cube', 'reported': {'exposure': 312.0}}

    def collector(self, **changes):
        collector = CubeDataCollector(replace(self.config, **changes))
        collector.set_context(task='Task1', phase='ORANGE_SEARCH')
        self.addCleanup(collector.stop)
        collector.start()
        return collector

    def offer(self, collector, at, frame=None, profile='default'):
        collector.offer_frame(self.frame if frame is None else frame,
                              at, 1700000000.0 + at, profile, self.camera)

    def frames(self):
        with closing(sqlite3.connect(self.directory / 'catalog.sqlite3')) as connection:
            return [json.loads(row[0]) for row in connection.execute(
                'SELECT metadata_json FROM frames ORDER BY captured_monotonic')]

    def test_full_colour_frame_and_capture_metadata_are_saved_unlabelled(self):
        collector = self.collector(batch_tag='same-layout-day-1')
        collector.set_context(task='Task2', phase='ORANGE_SEARCH')
        original = self.frame.copy()
        self.offer(collector, 10.0, profile='task2_orange')
        self.frame[:] = 255  # producer may reuse/mutate its buffer after offering
        collector.stop()
        self.assertEqual(collector.status, 'closed')
        records = self.frames()
        self.assertEqual(len(records), 1)
        record = records[0]
        image_path = self.directory / record['image_path']
        decoded = cv2.imread(str(image_path))
        self.assertEqual(decoded.shape, original.shape)
        self.assertLess(np.abs(decoded.astype(float) - original).mean(), 3)
        self.assertEqual(record['captured_monotonic'], 10.0)
        self.assertEqual(record['captured_at'], 1700000010.0)
        self.assertEqual(record['task'], 'Task2')
        self.assertEqual(record['phase'], 'ORANGE_SEARCH')
        self.assertEqual(record['profile'], 'task2_orange')
        self.assertEqual(record['split_group'], 'same-layout-day-1')
        self.assertEqual(record['camera'], self.camera)
        self.assertEqual(record['annotation_status'], 'unlabelled')
        self.assertEqual(json.loads(image_path.with_suffix('.json').read_text()), record)
        self.assertFalse(list(self.directory.rglob('*.txt')))
        self.assertEqual(library_stats(self.directory)['total_images'], 1)

    def test_identical_frames_are_rate_limited_but_periodically_retained(self):
        collector = self.collector()
        for at in (0.0, 0.25, 0.5, 1.0, 5.0, 5.0):
            self.offer(collector, at)
        collector.stop()
        self.assertEqual([r['captured_monotonic'] for r in self.frames()], [0.0, 5.0])
        self.assertEqual(collector.duplicates, 2)

    def test_phase_and_profile_changes_keep_first_frame_even_if_identical(self):
        collector = self.collector()
        self.offer(collector, 0.0)
        collector.set_context(task='Task2', phase='ORANGE_ALIGN')
        self.offer(collector, 0.1)
        self.offer(collector, 0.2, profile='task2_orange')
        collector.stop()
        records = self.frames()
        self.assertEqual(len(records), 3)
        self.assertEqual([r['phase'] for r in records],
                         ['ORANGE_SEARCH', 'ORANGE_ALIGN', 'ORANGE_ALIGN'])
        self.assertEqual(records[-1]['profile'], 'task2_orange')

    def test_automatic_collection_only_saves_pre_grab_search_and_alignment(self):
        collector = self.collector()
        states = [
            ('STARTUP', 'default'), ('INITIAL_MOVE', 'default'),
            ('ORANGE_SEARCH', 'default'), ('ORANGE_ALIGN', 'task2_orange'),
            ('GRAB', 'default'), ('ORANGE_GRAB', 'task2_orange'),
            ('COUNT_CHECK', 'default'), ('DELIVERY_TAG_ALIGN', 'default'),
            ('PURPLE_SEARCH', 'default'),  # wrong profile at a transition
            ('PURPLE_SEARCH', 'task2_purple'), ('PURPLE_ALIGN', 'task2_purple'),
            ('TAG6_ALIGN', 'default'), ('BUILDING_ALIGN', 'building'),
            ('BUILD', 'building'), ('UNLOAD', 'default'),
            ('FINISHED', 'default'), ('FAULT', 'default'),
        ]
        for at, (phase, profile) in enumerate(states):
            collector.set_context(phase=phase)
            self.offer(collector, float(at), profile=profile)
        collector.stop()
        self.assertEqual([r['phase'] for r in self.frames()],
                         ['ORANGE_SEARCH', 'ORANGE_ALIGN',
                          'PURPLE_SEARCH', 'PURPLE_ALIGN'])

    def test_same_search_after_grab_keeps_first_frame(self):
        collector = self.collector()
        self.offer(collector, 0.0)
        collector.set_context(phase='GRAB')
        collector.set_context(phase='ORANGE_SEARCH')
        self.offer(collector, 0.1)
        collector.stop()
        self.assertEqual(len(self.frames()), 2)

    def test_idle_robot_does_not_save_but_explicit_capture_still_does(self):
        collector = self.collector()
        collector.set_context(task='manual', phase='idle')
        self.offer(collector, 0.0)
        collector.stop()
        self.assertEqual(self.frames(), [])
        manual = CubeDataCollector(self.config, manual_capture=True)
        manual.set_context(task='manual_capture', phase='capture')
        manual.start()
        try:
            self.offer(manual, 1.0, profile='building')
        finally:
            manual.stop()
        self.assertEqual([r['phase'] for r in self.frames()], ['capture'])

    def test_small_local_colour_change_is_not_discarded_as_static_background(self):
        collector = self.collector()
        changed = self.frame.copy()
        changed[20:24, 30:34] = (200, 10, 10)
        self.offer(collector, 0.0)
        self.offer(collector, 0.5, frame=changed)
        collector.stop()
        self.assertEqual(len(self.frames()), 2)

    def test_disabled_collection_does_not_create_library(self):
        collector = self.collector(enabled=False)
        self.offer(collector, 0.0)
        collector.stop()
        self.assertFalse(self.directory.exists())
        self.assertEqual(collector.status, 'disabled')

    def test_full_queue_drops_frames_without_waiting_for_writer(self):
        collector = CubeDataCollector(replace(self.config, queue_size=1))
        collector.set_context(task='Task1', phase='ORANGE_SEARCH')
        entered, release = threading.Event(), threading.Event()
        original_save = collector._save_frame

        def slow_save(*args):
            entered.set()
            if not release.wait(3):
                raise RuntimeError('test writer was not released')
            return original_save(*args)

        with patch.object(collector, '_save_frame', side_effect=slow_save):
            collector.start()
            try:
                self.offer(collector, 0.0)
                self.assertTrue(entered.wait(3))
                self.offer(collector, 0.5)
                self.offer(collector, 1.0)
                self.assertEqual(collector.dropped, 1)
                self.assertEqual(collector._queue.qsize(), 1)
            finally:
                release.set()
                collector.stop()
        self.assertEqual(collector.status, 'closed')

    def test_session_limit_preserves_saved_images(self):
        collector = self.collector(max_session_images=1, dedup_mean_abs_diff=0)
        self.offer(collector, 0.0)
        self.offer(collector, 0.5)
        collector.stop()
        self.assertEqual(collector.status, 'limit_reached')
        self.assertEqual(len(self.frames()), 1)
        self.assertEqual(len(list(self.directory.rglob('*.jpg'))), 1)

    def test_global_budget_includes_previous_sessions(self):
        first = self.collector()
        self.offer(first, 0.0)
        first.stop()
        used = library_stats(self.directory)['total_image_bytes']
        second = self.collector(max_images_gb=(used + 1) / 1024 ** 3)
        self.offer(second, 1.0)
        second.stop()
        self.assertEqual(second.status, 'limit_reached')
        self.assertEqual(library_stats(self.directory)['total_images'], 1)
        self.assertEqual(len(library_stats(self.directory)['sessions']), 2)

    def test_low_disk_disables_collection_without_raising_on_camera_thread(self):
        with patch(MODULE + '.shutil.disk_usage', return_value=SimpleNamespace(free=0)):
            collector = self.collector(min_free_gb=1)
            self.offer(collector, 0.0)
            collector.stop()
        self.assertEqual(collector.status, 'limit_reached')
        self.assertEqual(collector.saved, 0)

    def test_sidecar_failure_rolls_back_index_and_removes_partial_image(self):
        from vision.yolo.collector import _write_json

        def fail_image_metadata(path, record):
            if path.name == '000001.json':
                raise OSError('simulated full disk')
            return _write_json(path, record)

        with patch(MODULE + '._write_json', side_effect=fail_image_metadata):
            collector = self.collector()
            self.offer(collector, 0.0)
            collector.stop()
        self.assertEqual(collector.status, 'error')
        self.assertEqual(self.frames(), [])
        self.assertEqual(list(self.directory.rglob('*.jpg')), [])
        self.assertEqual(list(self.directory.rglob('*.tmp')), [])

    def test_stats_missing_library_is_read_only(self):
        self.assertEqual(library_stats(self.directory)['total_images'], 0)
        self.assertFalse(self.directory.exists())

    def test_invalid_configuration_disables_optional_collector(self):
        with patch.dict('os.environ', {'UNIFOREST_COLLECT_DATA': 'invalid'}):
            self.assertIsNone(create_collector())
        for changes in ({'sample_hz': 0}, {'sample_hz': float('nan')},
                        {'queue_size': 0}, {'jpeg_quality': 101}):
            with self.assertRaises(ValueError):
                replace(self.config, **changes)

    def run_fake_camera(self, sink):
        detector = CubeDetector(camera_id='cube')
        detector.set_detection_profile('task2_orange')
        detector.set_frame_sink(sink)
        detector._running = True
        detector._state['orange_diagnostics'] = {}
        source = iter([self.frame.copy(), self.frame.copy()])

        def read():
            frame = next(source, None)
            if frame is None:
                detector._running = False
                return False, None
            return True, frame

        detector._cap = SimpleNamespace(read=read)
        with patch(CubeDetector.__module__ + '.detect_all_blocks', return_value=[]) as detect:
            detector._capture_loop()
        return detector, detect.call_count

    def test_actual_camera_hook_keeps_frames_even_when_detector_finds_nothing(self):
        collector = self.collector()
        detector, calls = self.run_fake_camera(collector.offer_frame)
        collector.stop()
        self.assertEqual(calls, 2)
        self.assertFalse(detector.result.is_valid)
        self.assertEqual(len(self.frames()), 1)
        record = self.frames()[0]
        self.assertEqual(record['profile'], 'task2_orange')
        saved = cv2.imread(str(self.directory / record['image_path']))
        self.assertGreater(saved[:24].mean(), 30)  # no top-half ROI blackout

    def test_broken_frame_sink_does_not_stop_detection(self):
        sink = Mock(side_effect=OSError('simulated recorder failure'))
        detector, calls = self.run_fake_camera(sink)
        self.assertEqual(calls, 2)
        sink.assert_called_once()
        self.assertIsNone(detector._frame_sink)


class WorkflowIntegrationTests(unittest.TestCase):
    def test_robot_enables_collector_on_existing_vision_without_opening_hardware(self):
        from robot import Robot
        collector = Mock()
        with patch(MODULE + '.create_collector', return_value=collector) as create:
            robot = Robot(enable_vision=True, camera_id='cube')
        create.assert_called_once_with(enabled=None, data_dir=None)
        self.assertEqual(robot._vision._frame_sink, collector.offer_frame)
        self.assertFalse(robot.transport.connected)
        self.assertFalse(robot._vision.is_running)
        collector.start.assert_not_called()

    def test_existing_task_states_are_reported_without_motion(self):
        from Strategy.competition import CompetitionProgram, CompetitionState
        from Strategy.task2 import Task2Round2Program, Task2State
        from Strategy.task0 import Task0Program, Task0State
        reporter = Mock()
        robot = SimpleNamespace(set_collection_context=reporter)
        program = CompetitionProgram(robot)
        program.state = CompetitionState.ORANGE_SEARCH
        reporter.assert_called_with(task='Task1', phase='ORANGE_SEARCH')
        program = Task2Round2Program(robot)
        program.state = Task2State.BUILDING_ALIGN
        reporter.assert_called_with(task='Task2-R2', phase='BUILDING_ALIGN')
        program = Task0Program(robot)
        program.state = Task0State.INITIAL_MOVE
        reporter.assert_called_with(task='Task0', phase='INITIAL_MOVE')
        # Existing fake/legacy robot facades without a collector remain usable.
        self.assertEqual(CompetitionProgram(SimpleNamespace()).state,
                         CompetitionState.STARTUP)

    def test_agent_repeated_workflows_get_distinct_flow_ids(self):
        from Strategy.runner import run_tasks
        reporter = Mock()
        robot = SimpleNamespace(set_collection_context=reporter)
        factory = lambda robot: SimpleNamespace(run=lambda: 0)
        self.assertEqual(run_tasks(robot, 'task1', task1_factory=factory), 0)
        self.assertEqual(run_tasks(robot, 'task1', task1_factory=factory), 0)
        flows = [call.kwargs['flow_id'] for call in reporter.call_args_list
                 if 'flow_id' in call.kwargs]
        self.assertEqual(len(set(flows)), 2)

    def test_robot_stops_hardware_before_waiting_for_dataset_flush(self):
        from robot import Robot
        robot = Robot.__new__(Robot)
        events = []
        robot._heartbeat_stop = threading.Event()
        robot._heartbeat_thread = None
        robot._vision = SimpleNamespace(stop=lambda: events.append('camera_stop'))
        robot._localizer = None
        robot.transport = SimpleNamespace(
            emergency_stop=lambda: events.append('emergency_stop'),
            disconnect=lambda: events.append('disconnect'))
        robot._data_collector = SimpleNamespace(stop=lambda: events.append('flush'))
        with patch('robot.time.sleep'):
            robot.stop()
        self.assertLess(events.index('emergency_stop'), events.index('flush'))
        self.assertLess(events.index('disconnect'), events.index('flush'))


if __name__ == '__main__':
    unittest.main()
