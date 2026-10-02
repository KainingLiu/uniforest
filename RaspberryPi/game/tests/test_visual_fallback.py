"""Visual/search control replay. No UART, camera or real motion is used."""
import contextlib
from dataclasses import replace
import io
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from Strategy.controllers import RobotController
from Strategy.errors import SearchRangeExhausted
from Strategy.settings import (GroundCollectionConfig, GroundCollection2Config,
                               GroundCollection3Config, HighlandCollectionConfig,
                               HighlandCollection2Config, BuildingConfig)
from Strategy.common import VisualAlignmentUnavailable
from Strategy.orange_search import OrangeSearchRecovery
from Strategy.context import ExecutionContext
from Strategy.runner import run_selection
from protocol.commands import ACTION_DONE
from vision.orange_cluster import Detector
from vision.orange_config import config_for
from vision.cube_detector import VisionResult
from control.carried_cube_inspection import inspect_carried_cubes
from tests.test_strategy_composition import FakeActionSession, robot_fixture


def block(y=120):
    return SimpleNamespace(color_name='orange', confidence=95, x=0, y=0, z=100,
                           quad=((0, y), (80, y), (80, y+40), (0, y+40)))


class SearchReplay:
    def __init__(self, frames, *, config=None, x=200):
        self.now, self.start, self.x, self.speed = 100.0, 100.0, float(x), 0.0
        self.commands, self.positions = [], []
        self.frames, self.frozen_telem = frames, False
        self.on_sleep = lambda: None
        self.transport = SimpleNamespace(connected=True, emergency_stop_generation=0)
        self.chassis = SimpleNamespace(mecanum_rpm=lambda vx, vy, wz: (vy*10, 0, 0, 0),
                                      set_speeds=self.command)
        self.task = RobotController(self, config or GroundCollectionConfig())
        self.task._capture_lateral_origin = lambda: 0
        self.task._measure_lateral_displacement_mm = lambda origin: self.x - origin
        self.task._orange_recovery = OrangeSearchRecovery(origin=0)
        self.clock = SimpleNamespace(monotonic=lambda: self.now, time=lambda: self.now,
                                     sleep=self.sleep)

    @property
    def telem(self):
        return SimpleNamespace(uptime_ms=1 if self.frozen_telem else round(self.now*1000))

    @property
    def vision_result(self):
        return self.frames(self)

    def frame(self, blocks=(), band=None, stamp=None):
        return VisionResult(all_blocks=list(blocks), timestamp=self.now if stamp is None else stamp,
                            orange_left_clipped_y_range=band)

    def command(self, rpm):
        self.speed = rpm[0]
        self.commands.append((self.now, self.speed))
        return True

    def sleep(self, seconds):
        if self.now - self.start > 30:
            raise AssertionError('search failed to terminate within replay limit')
        self.now += seconds
        self.x += self.speed * seconds
        self.positions.append(self.x)
        self.on_sleep()

    def run(self, limit=300):
        with patch('Strategy.orange_search.time', self.clock), \
             patch('Strategy.controllers.time', self.clock), \
             patch('Strategy.cube_tracker.time', self.clock):
            return self.task._find_cube(color_name='orange', min_confidence=25,
                                       search_direction=1, max_distance_mm=limit)


class VisualTests(unittest.TestCase):
    def setUp(self):
        quiet = contextlib.redirect_stdout(io.StringIO())
        quiet.__enter__()
        self.addCleanup(quiet.__exit__, None, None, None)

    def test_detector_reports_clipped_row_before_corner_fitting_at_original_scale(self):
        detector = Detector(config_for('default', 640))
        detector.cfg.PROCESS_WIDTH = 320
        mask = np.zeros((240, 320), np.uint8)
        mask[90:150, :100] = 255
        with patch.object(detector, '_orange_mask', return_value=mask), \
             patch.object(detector, '_process_cluster', return_value=[]):
            cubes, info = detector.detect(np.zeros((480, 640, 3), np.uint8))
        self.assertEqual(cubes, [])
        self.assertEqual(info['left_clipped_y_range'], (180, 300))
        # Use the actual detector signal to drive the real search controller.
        def frames(replay):
            if any(speed < 0 for _, speed in replay.commands):
                return replay.frame([block(200)])
            return replay.frame(band=info['left_clipped_y_range'])
        replay = SearchReplay(frames)
        self.assertEqual(replay.run().color_name, 'orange')
        self.assertTrue(any(speed < 0 for _, speed in replay.commands))
        self.assertEqual(replay.commands[-1][1], 0)

    def test_empty_small_and_top_edge_contours_do_not_trigger_recovery(self):
        detector = Detector(config_for('default', 640))
        for kind in ('empty', 'small', 'top', 'right'):
            mask = np.zeros((480, 640), np.uint8)
            if kind == 'small': mask[200:203, :3] = 255
            if kind == 'top': mask[:150, :200] = 255
            if kind == 'right': mask[200:300, 400:] = 255
            with self.subTest(kind=kind), patch.object(detector, '_orange_mask', return_value=mask), \
                 patch.object(detector, '_process_cluster', return_value=[]):
                _, info = detector.detect(np.zeros((480, 640, 3), np.uint8))
                self.assertIsNone(info['left_clipped_y_range'])

    def test_narrow_left_fragment_reports_row_without_grasp_coordinates(self):
        detector = Detector(config_for('default', 640))
        mask = np.zeros((480, 640), np.uint8)
        mask[100:260, :29] = 255  # Aspect 0.18; substantial clipped cube sliver.
        with patch.object(detector, '_orange_mask', return_value=mask), \
             patch.object(detector, '_process_cluster') as fit:
            cubes, info = detector.detect(np.zeros((480, 640, 3), np.uint8))
        self.assertEqual(cubes, [])
        fit.assert_not_called()
        self.assertEqual(info['left_clipped_y_range'], (100, 260))
        replay = SearchReplay(lambda r: r.frame(band=info['left_clipped_y_range']))
        with self.assertRaises(SearchRangeExhausted):
            replay.run()
        self.assertTrue(any(speed < 0 for _, speed in replay.commands))

    def test_full_width_and_roi_clipped_rows_publish_hints_before_fitting(self):
        for profile, low, high in (('default', 0, 480),
                                   ('task2_orange', 240, 440),
                                   ('task2_orange', 240, 480)):
            with self.subTest(profile=profile, rows=(low, high)):
                cfg = config_for(profile, 640)
                cfg.ROI_TOP_RATIO = .5 if profile == 'task2_orange' else 0
                detector = Detector(cfg)
                mask = np.zeros((480, 640), np.uint8)
                mask[low:high, :] = 255
                with patch.object(detector, '_orange_mask', return_value=mask), \
                     patch.object(detector, '_process_cluster', return_value=[]):
                    cubes, info = detector.detect(np.zeros((480, 640, 3), np.uint8))
                self.assertEqual(cubes, [])
                self.assertEqual(info['left_clipped_y_range'], (low, high))
                replay = SearchReplay(lambda r: r.frame(band=info['left_clipped_y_range']),
                                      config=(HighlandCollectionConfig() if profile == 'task2_orange'
                                              else GroundCollectionConfig()))
                with self.assertRaises(SearchRangeExhausted):
                    replay.run()
                self.assertTrue(any(speed < 0 for _, speed in replay.commands))

    def test_roi_excludes_left_fragment_entirely_above_pickup_area(self):
        cfg = config_for('task2_orange', 640)
        cfg.ROI_TOP_RATIO = .5
        detector = Detector(cfg)
        mask = np.zeros((480, 640), np.uint8)
        mask[100:200, :300] = 255
        with patch.object(detector, '_orange_mask', return_value=mask):
            _, info = detector.detect(np.zeros((480, 640, 3), np.uint8))
        self.assertIsNone(info['left_clipped_y_range'])

    def test_all_orange_profiles_use_same_search_and_stop_for_confirmation(self):
        for config in (GroundCollectionConfig(), GroundCollection2Config(),
                       GroundCollection3Config(), HighlandCollectionConfig(),
                       HighlandCollection2Config()):
            with self.subTest(profile=type(config).__name__):
                replay = SearchReplay(lambda r: r.frame([block()]), config=config)
                replay.run()
                self.assertTrue(all(speed == 0 for _, speed in replay.commands))
                self.assertGreater(replay.now, replay.start)

    def test_no_camera_completes_original_rightward_budget_without_visual_exception(self):
        for frames in (lambda r: None, lambda r: r.frame([block()], stamp=r.start-1)):
            replay = SearchReplay(frames)
            with self.assertRaises(SearchRangeExhausted): replay.run(limit=90)
            self.assertAlmostEqual(replay.task._search_position_mm, 90)
            self.assertTrue(all(speed > 0 for _, speed in replay.commands[:-1]))
            self.assertAlmostEqual(replay.now - replay.start, .3)

    def test_repeated_frame_cannot_confirm_target_or_trigger_recovery(self):
        replay = SearchReplay(lambda r: r.frame(band=(100, 200), stamp=r.start))
        with self.assertRaises(SearchRangeExhausted): replay.run(limit=90)
        self.assertFalse(any(speed < 0 for _, speed in replay.commands))
        replay = SearchReplay(lambda r: r.frame([block()], stamp=r.start))
        with self.assertRaises(SearchRangeExhausted): replay.run(limit=90)

    def test_recovery_respects_local_distance_and_does_not_repeat_on_continuous_clipping(self):
        replay = SearchReplay(lambda r: r.frame(band=(100, 200)), x=30)
        with self.assertRaises(SearchRangeExhausted): replay.run(limit=90)
        self.assertGreaterEqual(min(replay.positions), 30-replay.task.config.orange_edge_max_distance_mm-.001)
        self.assertLess(min(replay.positions), 0)
        starts = sum(speed < 0 and (i == 0 or replay.commands[i-1][1] >= 0)
                     for i, (_, speed) in enumerate(replay.commands))
        self.assertEqual(starts, 1)

    def test_recovery_starts_at_origin_without_first_moving_right(self):
        replay = SearchReplay(lambda r: r.frame(band=(100, 200)), x=0)
        with self.assertRaises(SearchRangeExhausted):
            replay.run(limit=90)
        first_left = next(i for i, (_, speed) in enumerate(replay.commands) if speed < 0)
        self.assertFalse(any(speed > 0 for _, speed in replay.commands[:first_left]))
        self.assertLess(min(replay.positions), 0)
        starts = sum(speed < 0 and (i == 0 or replay.commands[i-1][1] >= 0)
                     for i, (_, speed) in enumerate(replay.commands))
        self.assertEqual(starts, 1)

    def test_stalled_recovery_consumes_one_attempt_then_resumes_bounded_search(self):
        replay = SearchReplay(lambda r: r.frame(band=(100, 200)), x=0)
        replay.on_sleep = lambda: setattr(replay, 'x', 0)
        with self.assertRaises(SearchRangeExhausted):
            replay.run(limit=90)
        self.assertFalse(replay.task._orange_recovery.armed)
        self.assertTrue(any(speed < 0 for _, speed in replay.commands))
        self.assertTrue(any(speed > 0 for _, speed in replay.commands))
        self.assertAlmostEqual(replay.task._search_position_mm, 90)

    def test_wrong_row_is_not_acquired_during_recovery_and_camera_loss_resumes_right(self):
        def frames(r):
            if any(speed < 0 for _, speed in r.commands):
                return r.frame([block(350)], band=(100, 200)) if r.speed < 0 else None
            return r.frame(band=(100, 200))
        replay = SearchReplay(frames)
        with self.assertRaises(SearchRangeExhausted): replay.run(limit=90)
        negative = [i for i, (_, speed) in enumerate(replay.commands) if speed < 0]
        self.assertTrue(negative)
        self.assertTrue(any(speed > 0 for _, speed in replay.commands[negative[-1]+1:]))

    def test_camera_loss_during_recovery_does_not_terminate_mission(self):
        replay = SearchReplay(lambda r: None if any(v < 0 for _, v in r.commands)
                              else r.frame(band=(100, 200)))
        with self.assertRaises(SearchRangeExhausted): replay.run(limit=90)
        self.assertTrue(any(speed < 0 for _, speed in replay.commands))
        self.assertAlmostEqual(replay.task._search_position_mm, 90)

    def test_time_budget_is_shared_across_pickups_and_excludes_mechanical_wait(self):
        replay = SearchReplay(lambda r: r.frame([block()]))
        replay.run()
        elapsed = replay.task._orange_recovery.search_elapsed_s
        replay.now += 20  # Mechanical action time is not active search time.
        replay.start = replay.now
        replay.run()
        self.assertAlmostEqual(replay.task._orange_recovery.search_elapsed_s, 2*elapsed)
        replay.task._orange_recovery.search_elapsed_s = 100
        with self.assertRaises(SearchRangeExhausted): replay.run()

    def test_hardware_faults_still_escape_search(self):
        for fault in ('disconnect', 'stop', 'stale', 'send'):
            with self.subTest(fault=fault):
                replay = SearchReplay(lambda r: None)
                if fault == 'disconnect': replay.on_sleep = lambda: setattr(replay.transport, 'connected', False)
                if fault == 'stop': replay.on_sleep = lambda: setattr(replay.transport, 'emergency_stop_generation', 1)
                if fault == 'stale': replay.frozen_telem = True
                if fault == 'send': replay.chassis.set_speeds = Mock(return_value=False)
                with self.assertRaises(RuntimeError) as error: replay.run(limit=900)
                self.assertNotIsInstance(error.exception, SearchRangeExhausted)

    def test_orange_budget_exhaustion_skips_count_and_returns_to_route(self):
        robot = robot_fixture()
        robot.check_carried_cube_count = Mock()
        from Strategy.flows.model import ActionSpec
        from Strategy.flows.operations import (begin_collection, acquire_cube,
                                                grab_cube, inspect_cargo)
        controller = RobotController(robot)
        controller._capture_lateral_origin = Mock(return_value=0)
        controller._find_cube = Mock(side_effect=SearchRangeExhausted())
        environment = SimpleNamespace(robot=robot, data={},
            control=lambda profile: controller, phase=Mock())
        spec = ActionSpec('begin_collection', 'ground-pickup', 'ground-1',
                          {'color': 'orange', 'method': 'grap3'})
        begin_collection(environment, spec)
        self.assertFalse(acquire_cube(environment, replace(spec, kind='acquire_cube')))
        self.assertFalse(grab_cube(environment, replace(spec, kind='grab_cube')))
        self.assertIsNone(inspect_cargo(environment, replace(spec, kind='inspect_cargo')))
        self.assertTrue(environment.data['collection']['exhausted'])
        robot.check_carried_cube_count.assert_not_called()

    def test_tag_and_building_only_catch_typed_visual_failures(self):
        robot = robot_fixture()
        task = RobotController(robot, context=ExecutionContext(robot))
        for method, wrapper in (('_align_delivery_tag', task._align_delivery_tag_or_continue),
                                ('_align_building', task._align_building_or_continue)):
            with patch.object(task, method, side_effect=VisualAlignmentUnavailable('target lost')):
                self.assertFalse(wrapper())
            with patch.object(task, method, side_effect=RuntimeError('motor failure')):
                with self.assertRaisesRegex(RuntimeError, 'motor failure'): wrapper()
        robot.transport.emergency_stop_generation += 1
        with patch.object(task, '_align_building', side_effect=VisualAlignmentUnavailable('lost')):
            with self.assertRaisesRegex(RuntimeError, 'emergency stop'):
                task._align_building_or_continue()

    def test_actual_tag_and_building_loops_continue_after_lost_target_timeout(self):
        now = [100.0]
        clock = SimpleNamespace(monotonic=lambda: now[0], time=lambda: now[0],
                                sleep=lambda dt: now.__setitem__(0, now[0]+dt))
        robot = robot_fixture()
        robot.has_vision = robot.has_field_localization = False
        robot.field_pose = robot.vision_result = None
        task = RobotController(robot, replace(BuildingConfig(), delivery_tag_lost_timeout_s=.1,
                                          building_lost_timeout_s=.1))
        task._heading_zero_deg = 0
        with patch('Strategy.controllers.time', clock), patch('Strategy.building_alignment.time', clock):
            self.assertFalse(task._align_delivery_tag_or_continue())
            self.assertFalse(task._align_building_or_continue())
        self.assertGreaterEqual(now[0], 100.2)
        robot.transport.emergency_stop.assert_not_called()

    def test_total_search_time_caps_repeated_candidate_holds(self):
        # Ambiguous pairs never lock, but repeatedly appearing candidates must
        # not renew the search deadline and keep the robot in this task forever.
        def frames(r):
            phase = int((r.now-r.start) / .1) % 2
            return r.frame([block(), block()] if phase == 0 else [])
        replay = SearchReplay(frames)
        with self.assertRaises(SearchRangeExhausted): replay.run(limit=1800)
        self.assertLessEqual(replay.now-replay.start, 13.1)

    def test_whole_classic_sequence_continues_without_cameras(self):
        # Run actual PlanA functional actions; only physical control is replaced.
        robot = robot_fixture()
        robot.has_vision = robot.has_field_localization = False
        robot.move_chassis = Mock(return_value=SimpleNamespace(timed_out=False, cancelled=False))
        robot.check_carried_cube_count = Mock(return_value=None)
        events = []
        def begin(action_id):
            events.append('build')
            return FakeActionSession()
        robot.actions.begin = begin
        robot.actions.hatch_open = Mock()
        robot.actions.hatch_close = Mock()
        def move(task, direction, distance, speed, **kwargs): events.append((task.operation_name, direction, distance))
        with patch('Strategy.controllers.RobotController._checked_move', move), \
             patch('Strategy.controllers.RobotController._drive_until_wall'), \
             patch('Strategy.controllers.RobotController._turn_to_heading'), \
             patch('Strategy.controllers.RobotController._capture_lateral_origin', return_value=0), \
             patch('Strategy.controllers.RobotController._measure_lateral_displacement_mm', return_value=0), \
             patch('Strategy.controllers.RobotController._recalibrate_heading_zero'), \
             patch('Strategy.controllers.RobotController._find_cube', side_effect=SearchRangeExhausted()), \
             patch('Strategy.controllers.RobotController._align_delivery_tag', side_effect=VisualAlignmentUnavailable('no tag')), \
             patch('Strategy.controllers.RobotController._chassis_followup', side_effect=lambda callback: lambda check: callback()), \
             patch.object(RobotController, '_align_building', side_effect=VisualAlignmentUnavailable('no building')):
            self.assertEqual(run_selection(robot, 'PlanA'), 0)
        self.assertEqual(events.count('build'), 3)
        self.assertTrue(any(isinstance(event, tuple) and event[1:] == ('left', 2200)
                            for event in events))
        self.assertTrue(any(isinstance(event, tuple) and event[1:] == ('left', 3000)
                            for event in events))
        robot.transport.emergency_stop.assert_not_called()
        robot.set_cube_detection_profile.assert_not_called()


class CountReplay:
    def __init__(self, mode):
        self.now = 100.0
        self.mode, self.servos = mode, []
        self._running = True
        self.diagnostics = SimpleNamespace(write=Mock())
        self.transport = SimpleNamespace(connected=True, emergency_stop_generation=0,
            query_action_status=lambda: True, emergency_stop=Mock(),
            get_action_status=lambda: (SimpleNamespace(state=ACTION_DONE, uptime_ms=0), self.now),
            set_servo_angle_checked=self.servo)
        self.actions = SimpleNamespace(_action_lock=threading.Lock(), _check_cancelled=lambda: None)
        self.chassis = SimpleNamespace(set_speeds=lambda rpm: True,
                                      monitor_action=lambda callback: contextlib.nullcontext())
        self.reset_vision_filter = Mock()
        self.clock = SimpleNamespace(monotonic=lambda: self.now, sleep=self.sleep)

    def servo(self, index, angle, check):
        check()
        self.servos.append((index, angle))

    def sleep(self, seconds): self.now += seconds

    def inspection_link_snapshot(self):
        telem = SimpleNamespace(uptime_ms=round(self.now*1000), stepper_busy=0,
                                motors=[SimpleNamespace(speed_rpm=0)]*4)
        return telem, self.now, self.now, 0

    @property
    def cube_raw_frame(self):
        if self.mode == 'cancel':
            self.transport.emergency_stop_generation += 1
            return None
        if self.mode == 'none' or self.servos:
            return None
        return np.zeros((4, 4, 3), np.uint8), self.now

    def run(self, allow=True):
        self.followup = Mock()
        with patch('control.carried_cube_inspection.time', self.clock), \
             patch('control.carried_cube_inspection.observe', return_value=({}, None)), \
             patch('control.carried_cube_inspection.classify', return_value={'count': 3}), \
             contextlib.redirect_stdout(io.StringIO()):
            return inspect_carried_cubes(self, chassis_followup=self.followup, allow_visual_failure=allow)


class CountFallbackTests(unittest.TestCase):
    def test_no_frame_returns_unknown_without_moving_arm(self):
        replay = CountReplay('none')
        self.assertIsNone(replay.run())
        self.assertEqual(replay.servos, [])
        replay.followup.assert_called_once()
        replay.transport.emergency_stop.assert_not_called()

    def test_camera_loss_after_pose_restores_arm_before_return(self):
        replay = CountReplay('after_pose')
        self.assertIsNone(replay.run())
        self.assertEqual(replay.servos, [(0, 37.2), (1, 120), (1, 90), (0, 97.2)])
        replay.reset_vision_filter.assert_called_once_with(after_inspection=True)
        replay.followup.assert_called_once()
        replay.transport.emergency_stop.assert_not_called()

    def test_strict_diagnostics_and_cancellation_still_fail(self):
        for mode, allow in [('none', False), ('cancel', True)]:
            replay = CountReplay(mode)
            with self.assertRaises(RuntimeError): replay.run(allow)
            replay.followup.assert_not_called()
            replay.transport.emergency_stop.assert_called_once()


if __name__ == '__main__':
    unittest.main()
