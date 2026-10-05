"""One-shot refill integration and cooperative inspection; no hardware opened."""

import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from control.carried_cube_inspection import inspect_carried_cubes
from protocol.commands import ACTION_IDLE, ACTION_RUNNING
from Strategy.competition import CompetitionProgram, SearchRangeExhausted, TaskControl
from Strategy.common import VisualAlignmentUnavailable
from Strategy.context import BuildApproach, TaskContext
from Strategy.plans import PLANS, StrategyPlan
from Strategy.runner import refill_steps, run_plan
from Strategy.task2 import Task2Program
from Strategy.task3 import Task3Program
from Strategy.tasks import TaskStep
from tests.test_strategy_composition import robot_fixture
from tests.test_visual_fallback import CountReplay


class MovingCount(CountReplay):
    def __init__(self, count=2, duration=2.0, fault=None, missing_frame=False):
        super().__init__('moving')
        self.count, self.duration, self.fault = count, duration, fault
        self.missing_frame = missing_frame
        self.events = []
        self.monitor = None
        self.chassis.monitor_action = self.monitor_action
        self.chassis.set_speeds = Mock(return_value=True)
        self.transport.get_action_status = lambda: (
            SimpleNamespace(state=ACTION_IDLE, uptime_ms=0), self.now)

    @property
    def cube_raw_frame(self):
        if self.missing_frame:
            return None
        return np.zeros((4, 4, 3), np.uint8), self.now

    @contextlib.contextmanager
    def monitor_action(self, callback):
        self.monitor = callback
        try:
            callback()
            yield
            callback()
        finally:
            self.monitor = None

    def servo(self, index, angle, check):
        super().servo(index, angle, check)
        self.events.append(('servo', index, angle, self.now))

    def route(self):
        self.events.append(('route_start', self.now))
        end = self.now + self.duration
        while self.now < end:
            if self.fault and self.now > end - self.duration + .25:
                if self.fault == 'disconnect':
                    self.transport.connected = False
                elif self.fault == 'estop':
                    self.transport.emergency_stop_generation += 1
                elif self.fault == 'route':
                    raise RuntimeError('route failed')
                else:
                    self.actions._check_cancelled = Mock(side_effect=RuntimeError('cancelled'))
            self.monitor()
            self.sleep(.02)
        self.events.append(('route_end', self.now))

    def run_moving(self):
        with patch('control.carried_cube_inspection.time', self.clock), \
             patch('control.carried_cube_inspection.observe', return_value=({}, None)), \
             patch('control.carried_cube_inspection.classify', return_value={'count': self.count}):
            return inspect_carried_cubes(self, chassis_followup=self.route,
                                        allow_visual_failure=True, inspect_while_moving=True)


class SingleRefillTests(unittest.TestCase):
    def setUp(self):
        quiet = contextlib.redirect_stdout(io.StringIO())
        quiet.__enter__()
        self.addCleanup(quiet.__exit__, None, None, None)

    def test_count_pose_capture_and_restore_overlap_route_for_all_counts(self):
        for count in (0, 1, 2, 3, None):
            with self.subTest(count=count):
                replay = MovingCount(count)
                self.assertEqual(replay.run_moving(), count)
                start = next(e[1] for e in replay.events if e[0] == 'route_start')
                end = next(e[1] for e in replay.events if e[0] == 'route_end')
                poses = [e for e in replay.events if e[0] == 'servo']
                self.assertEqual(replay.servos, [(0, 37.2), (1, 120), (1, 90), (0, 97.2)])
                self.assertTrue(all(start < e[-1] < end for e in poses[1:]))
                replay.chassis.set_speeds.assert_not_called()
                self.assertFalse(replay.actions._action_lock.locked())
                self.assertIsNone(replay.monitor)
                replay.transport.emergency_stop.assert_not_called()

    def test_short_route_waits_for_arm_restore_and_missing_camera_still_runs_route(self):
        replay = MovingCount(duration=.1)
        self.assertEqual(replay.run_moving(), 2)
        end = next(e[1] for e in replay.events if e[0] == 'route_end')
        self.assertGreater(replay.events[-1][-1], end)
        self.assertEqual(replay.servos[-1], (0, 97.2))
        replay = MovingCount(missing_frame=True)
        self.assertIsNone(replay.run_moving())
        self.assertEqual([e[0] for e in replay.events], ['route_start', 'route_end'])

    def test_faults_stop_route_and_never_send_late_restore(self):
        for fault in ('disconnect', 'estop', 'cancel', 'route'):
            with self.subTest(fault=fault):
                replay = MovingCount(fault=fault)
                with self.assertRaises(RuntimeError):
                    replay.run_moving()
                self.assertNotIn('route_end', [e[0] for e in replay.events])
                self.assertNotIn((0, 97.2), replay.servos)
                self.assertFalse(replay.actions._action_lock.locked())
                self.assertIsNone(replay.monitor)
                replay.transport.emergency_stop.assert_called_once()

    def test_active_mechanism_rejected_before_any_route_or_servo(self):
        replay = MovingCount()
        replay.transport.get_action_status = lambda: (
            SimpleNamespace(state=ACTION_RUNNING, uptime_ms=0), replay.now)
        with self.assertRaises(RuntimeError):
            replay.run_moving()
        self.assertEqual(replay.events, [])

    def test_moving_wheels_are_allowed_but_busy_stepper_is_not(self):
        for busy in (False, True):
            replay = MovingCount()
            original = replay.inspection_link_snapshot
            def snapshot():
                telem, received, pong, epoch = original()
                telem.stepper_busy = int(busy)
                for motor in telem.motors:
                    motor.speed_rpm = 300
                return telem, received, pong, epoch
            replay.inspection_link_snapshot = snapshot
            if busy:
                with self.assertRaises(RuntimeError):
                    replay.run_moving()
                self.assertEqual(replay.events, [])
            else:
                self.assertEqual(replay.run_moving(), 2)

    def test_purple_exhaustion_defers_count_until_orange_exit(self):
        robot = robot_fixture()
        robot.check_carried_cube_count = Mock()
        task = Task2Program(robot)
        task._find_cube = Mock(side_effect=SearchRangeExhausted())
        self.assertFalse(task._search_and_align_purple())
        self.assertTrue(task._purple_search_exhausted)
        grab = Mock()
        task._collect_orange_with_count_check(task._orange_target_count_for_run(False), grab)
        self.assertEqual(grab.call_count, 3)
        self.assertFalse(task._orange_search_exhausted)
        robot.check_carried_cube_count.assert_not_called()

    def run_routes(self, plan, count, *, refill_fault=None, resume_fault=None,
                   refill_grabs=0, refill_count=...):
        robot = robot_fixture()
        robot.has_vision = robot.has_field_localization = False
        robot.move_chassis = Mock(return_value=SimpleNamespace(timed_out=False, cancelled=False))
        events, starts = [], []
        self.route_robot, self.route_events = robot, events
        robot.actions.hatch_open = Mock(side_effect=lambda **kw: events.append(('hatch_open',)))
        robot.actions.hatch_close = Mock(side_effect=lambda **kw: events.append(('hatch_close',)))
        def build(count, **kw):
            events.append(('build', count))
            kw['chassis_followup'](lambda: events.append(('action_check', count)))
        for count_value in (1, 2, 3):
            setattr(robot.actions, f'build{count_value}',
                    Mock(side_effect=lambda n=count_value, **kw: build(n, **kw)))
        robot.actions.build = Mock(side_effect=lambda **kw: build(3, **kw))

        def diagnostics(kind, **kw):
            if kind == 'task_start':
                starts.append(kw['task'])
                events.append(('start', kw['task']))
        robot.diagnostics.write = Mock(side_effect=diagnostics)

        def inspect(**kw):
            self.assertTrue(kw['inspect_while_moving'])
            self.assertTrue(kw['allow_visual_failure'])
            events.append(('count_start', starts[-1]))
            kw['chassis_followup']()
            events.append(('count_end', starts[-1]))
            return (refill_count if starts[-1] == 'task1-0' and refill_count is not ...
                    else count)
        robot.check_carried_cube_count = Mock(side_effect=inspect)

        def move(task, direction, distance, speed, **kw):
            events.append((task.TASK_LABEL, direction, distance))
            if task.TASK_LABEL == 'task2-0' and refill_fault:
                raise RuntimeError('refill mechanism fault')
        def calibrate(task, **kw):
            # Model a new wall-calibrated zero at each orange collection area.
            task._heading_zero_deg = {'task1-0': 23.5, 'task2-0': 31.5}.get(task.TASK_LABEL, 15.0)
        def align(task, **kw):
            if task.TASK_LABEL.startswith('task3'):
                events.append(('build_heading', task._heading_zero_deg))
            else:
                events.append(('tag6', task.TASK_LABEL, task._heading_zero_deg))
                if resume_fault:
                    if resume_fault == 'estop':
                        robot.transport.emergency_stop_generation += 1
                    elif resume_fault == 'stale':
                        robot.telemetry_age = 1
                    elif resume_fault == 'disconnect':
                        robot.transport.connected = False
                    else:
                        raise RuntimeError('tag motion fault')
            raise VisualAlignmentUnavailable('no tag')
        original_grab = Task2Program._grab_task2_orange
        def grab(task):
            if task.TASK_LABEL == 'task2-0' and task.completed_grabs < refill_grabs:
                events.append(('refill_grab',))
                return
            return original_grab(task)
        original_task1_grab = CompetitionProgram._grab_task1_orange
        def task1_grab(task):
            if task.TASK_LABEL == 'task1-0' and task.completed_grabs < refill_grabs:
                events.append(('refill_grab',))
                return
            return original_task1_grab(task)
        with patch.object(TaskControl, '_checked_move', move), \
             patch.object(TaskControl, '_drive_until_wall'), \
             patch.object(TaskControl, '_turn_to_heading'), \
             patch.object(TaskControl, '_capture_lateral_origin', return_value=0), \
             patch.object(TaskControl, '_measure_lateral_displacement_mm', return_value=125), \
             patch.object(TaskControl, '_recalibrate_heading_zero', calibrate), \
             patch.object(TaskControl, '_find_cube', side_effect=SearchRangeExhausted()), \
             patch.object(TaskControl, '_align_delivery_tag', align), \
             patch.object(Task2Program, '_grab_task2_orange', grab), \
             patch.object(CompetitionProgram, '_grab_task1_orange', task1_grab), \
             patch.object(TaskControl, '_chassis_followup', side_effect=lambda f: lambda check: f()), \
             patch.object(Task3Program, '_align_building_or_continue', return_value=False):
            self.assertEqual(run_plan(robot, plan), 0)
        robot.transport.emergency_stop.assert_not_called()
        return robot, starts, events

    def test_task1_resumes_tag6_after_refill_with_original_variant_and_new_heading(self):
        for source in ('task1-1', 'task1-2'):
            for count in (0, 1, 2, 3, None):
                with self.subTest(source=source, count=count):
                    robot, starts, events = self.run_routes(
                        StrategyPlan('one shot', (TaskStep(source), TaskStep('task0-3'))), count)
                    inserted = count != 3
                    self.assertEqual(starts, [source] + (['task2-0'] if inserted else []) + ['task0-3'])
                    robot.actions.hatch_open.assert_called_once_with(settle_ms=300)
                    robot.actions.hatch_close.assert_called_once_with(settle_ms=0)
                    robot.check_carried_cube_count.assert_called_once()
                    self.assertEqual(events.count((source, 'backward', 400)), 1)
                    self.assertEqual(events.count((source, 'forward', 2675)), 1)
                    tag = ('tag6', source, 31.5 if inserted else 15.0)
                    self.assertEqual(events.count(tag), 1)
                    if inserted:
                        self.assertLess(events.index(('task2-0', 'forward', 2750)), events.index(tag))
                    lateral = 100 if source == 'task1-1' else 400
                    tail = [tag, (source, 'right', lateral), ('hatch_open',),
                            (source, 'backward', 300), ('hatch_close',),
                            (source, 'left', lateral), ('start', 'task0-3')]
                    indices = [events.index(e) for e in tail]
                    self.assertEqual(indices, sorted(indices))
                    reports = [c.kwargs for c in robot.diagnostics.write.call_args_list
                               if c.args[0] == 'refill_inserted']
                    self.assertEqual([r['missing'] for r in reports],
                                     [1 if count is None else 3-count] if inserted else [])

    def test_task2_refill_rejoins_task3_with_new_heading_and_no_recursive_refill(self):
        for source in ('task2-1', 'task2-2'):
            for count in (0, 1, 2, 3, None):
                with self.subTest(source=source, count=count):
                    robot, starts, events = self.run_routes(
                        StrategyPlan('one shot', (TaskStep(source), TaskStep('task3-1'))), count)
                    inserted = count != 3
                    self.assertEqual(starts, [source] + (['task0-3', 'task1-0'] if inserted else []) + ['task3-1'])
                    self.assertIn((source, 'forward', 2750), events)
                    self.assertIn(('build_heading', 23.5 if inserted else 15.0), events)
                    self.assertEqual(robot.check_carried_cube_count.call_count, 2 if inserted else 1)
                    expected_count = 2 if count is None else count
                    self.assertEqual([e for e in events if e[0] == 'build'],
                                     [('build', expected_count)] if expected_count else [])
                    reports = [c.kwargs for c in robot.diagnostics.write.call_args_list
                               if c.args[0] == 'refill_inserted']
                    self.assertEqual([r['missing'] for r in reports],
                                     [1 if count is None else 3-count] if inserted else [])

    def test_task1_resume_runs_after_zero_partial_or_full_refill(self):
        for grabs in (0, 1, 2):
            robot, starts, events = self.run_routes(
                StrategyPlan('single task', (TaskStep('task1-2'),)), 1, refill_grabs=grabs)
            self.assertEqual(starts, ['task1-2', 'task2-0'])
            self.assertEqual(events.count(('refill_grab',)), grabs)
            self.assertIn(('tag6', 'task1-2', 31.5), events)
            robot.actions.hatch_open.assert_called_once()

    def test_refill_and_resumed_hardware_faults_do_not_unload_or_run_later_task(self):
        for fault in ('refill', 'motion', 'estop', 'disconnect', 'stale'):
            with self.subTest(fault=fault), self.assertRaises(RuntimeError):
                self.run_routes(StrategyPlan('fault', (
                    TaskStep('task1-1'), TaskStep('task0-3'))), None,
                    refill_fault=fault == 'refill',
                    resume_fault=None if fault == 'refill' else fault)
            self.route_robot.actions.hatch_open.assert_not_called()
            self.assertNotIn(('start', 'task0-3'), self.route_events)
            self.assertTrue(self.route_robot.transport.emergency_stop.called)

    def test_normal_count_failure_without_exhaustion_does_not_request_refill(self):
        robot = robot_fixture()
        robot.check_carried_cube_count = Mock(return_value=None)
        task = TaskControl(robot)
        grab = Mock()
        task._collect_orange_with_count_check(3, grab)
        self.assertEqual(grab.call_count, 3)
        self.assertFalse(task._orange_search_exhausted)
        self.assertEqual(task.refill_missing_count, 0)

    def test_full_plans_preserve_original_steps_and_finish_with_refill_exhausted(self):
        for name in ('PlanA', 'PlanB', 'PlanC'):
            robot, starts, _ = self.run_routes(PLANS[name], 2)
            expected = []
            for step in PLANS[name].steps:
                expected.append(step.task_id)
                expected.extend(s.task_id for s in refill_steps(step.task_id, 1))
            self.assertEqual(starts, expected)
            self.assertEqual(robot.check_carried_cube_count.call_count,
                             sum((s.task_id in ('task1-1', 'task1-2', 'task2-1', 'task2-2'))
                                 + (s.task_id in ('task2-1', 'task2-2')) for s in PLANS[name].steps))

    def test_task3_uses_latest_refill_inspection_instead_of_initial_count_or_grabs(self):
        for task_id, left_mm, count in (
                (task_id, left_mm, count)
                for task_id, left_mm in (('task3-2', 2200), ('task3-4', 100), ('task3-5', 400))
                for count in (0, 1, 2, 3, None)):
            with self.subTest(task=task_id, count=count):
                robot, starts, events = self.run_routes(
                    StrategyPlan('measured refill', (TaskStep('task2-1'), TaskStep(task_id))),
                    0, refill_grabs=1, refill_count=count)
                self.assertEqual(starts, ['task2-1', 'task0-3', 'task1-0', task_id])
                self.assertEqual(events.count(('refill_grab',)), 1)
                self.assertEqual(robot.check_carried_cube_count.call_count, 2)
                selected = 2 if count is None else count
                self.assertEqual([e for e in events if e[0] == 'build'],
                                 [('build', selected)] if selected else [])
                self.assertIn((task_id, 'backward', 200), events)
                self.assertIn((task_id, 'left', left_mm), events)
                self.assertLess(events.index(('count_end', 'task1-0')),
                                events.index(('start', task_id)))
                self.assertEqual(events.count(('task1-0', 'backward', 400)), 1)

    def test_completed_task1_refill_keeps_three_cube_build_without_extra_inspection(self):
        robot, starts, events = self.run_routes(
            StrategyPlan('completed refill', (TaskStep('task2-2'), TaskStep('task3-1'))),
            1, refill_grabs=2)
        self.assertEqual(starts, ['task2-2', 'task0-3', 'task1-0', 'task3-1'])
        self.assertEqual(events.count(('refill_grab',)), 2)
        robot.check_carried_cube_count.assert_called_once()
        self.assertEqual([e for e in events if e[0] == 'build'], [('build', 3)])
        robot.actions.build.assert_called_once()

    def test_build_count_handoff_is_validated_consumed_and_not_reused(self):
        for invalid in (-1, 4, True, 1.5, '2'):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                BuildApproach(37, 'task1-0', invalid)
        robot = robot_fixture()
        context = TaskContext(robot, current_task='task1-0')
        context.publish_build_approach(37, carried_cube_count=1)
        task = Task3Program(robot, context=context)
        task._preflight()
        self.assertEqual(task._carried_cube_count, 1)
        self.assertIsNone(context.build_approach)
        context.current_task = 'task2-2'
        context.publish_build_approach(42)
        next_task = Task3Program(robot, context=context)
        next_task._preflight()
        self.assertIsNone(next_task._carried_cube_count)
        self.assertIsNone(context.build_approach)

    def test_quantity_parameters_and_excluded_ids(self):
        for missing in (1, 2, 3):
            self.assertEqual(dict(refill_steps('task1-1', missing)[0].parameters),
                             {'orange_target_count': missing})
            self.assertEqual(dict(refill_steps('task2-2', missing)[1].parameters),
                             {'target_cube_count': missing})
        for task_id in ('task1-0', 'task2-0', 'task1-3', 'task0-3'):
            self.assertEqual(refill_steps(task_id, 3), ())


if __name__ == '__main__':
    unittest.main()
