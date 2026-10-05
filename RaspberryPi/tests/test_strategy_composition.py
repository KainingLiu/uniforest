"""Task ordering, handoff and cancellation tests; no hardware is opened."""

import contextlib
from dataclasses import dataclass
import io
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from Strategy.context import BuildApproach, TaskContext
from Strategy.plans import PLANS, StrategyPlan, validate_plan
from Strategy.results import TaskResult, TaskStatus
from Strategy.runner import resolve_selection, run_plan
from Strategy.tasks import TASK_LIBRARY, TaskDefinition, TaskStep
from Strategy.task1 import Task1Program, Task1_2Program, Task1_3Program
from Strategy.task2 import Task2Program, Task2_2Program
from Strategy.task3 import (Task3Program, Task3_2Program, Task3_3Program,
                            Task3_4Program, Task3_5Program)


def robot_fixture():
    """Only records requested calls; does not simulate robot motion."""
    robot = SimpleNamespace(
        telem=SimpleNamespace(yaw_deg=-143.0, uptime_ms=1000),
        has_vision=True, has_field_localization=True,
        strategy_lock=threading.Lock(),
        set_collection_context=Mock(), diagnostics=SimpleNamespace(write=Mock()),
        reset_field_localization_filter=Mock(), reset_vision_filter=Mock(),
        set_cube_detection_profile=Mock(),
    )
    robot.transport = SimpleNamespace(connected=True, emergency_stop_generation=0)
    def emergency_stop():
        robot.transport.emergency_stop_generation += 1
    robot.transport.emergency_stop = Mock(side_effect=emergency_stop)
    robot.telemetry_age = 0.0
    robot.link_generation = 0
    robot.inspection_link_snapshot = lambda: (
        robot.telem, time.monotonic() - robot.telemetry_age, time.monotonic(), robot.link_generation)
    robot.actions = SimpleNamespace(_cancel_event=None)
    robot.actions.set_cancel_event = lambda event: setattr(robot.actions, '_cancel_event', event)
    robot.chassis = SimpleNamespace(turn=Mock(), set_speeds=Mock(return_value=True))
    return robot


@dataclass(frozen=True)
class RecordConfig:
    value: int = 0


class CompositionTests(unittest.TestCase):
    def setUp(self):
        self.output = contextlib.redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.addCleanup(self.output.__exit__, None, None, None)
        self.robot = robot_fixture()

    def test_default_and_legacy_sequences(self):
        expected = ['task0-1', 'task1-1', 'task2-1', 'task3-1',
                    'task1-2', 'task2-2', 'task3-2',
                    'task1-3', 'task2-2', 'task3-3']
        self.assertEqual([s.task_id for s in PLANS['PlanA'].steps], expected)
        plan_c = ['task0-1', 'task1-1', 'task0-3', 'task1-2',
                  'task2-1', 'task3-4', 'task2-2', 'task3-2',
                  'task1-3', 'task2-2', 'task3-3']
        for selection in ('PlanC', 'planc'):
            self.assertEqual([s.task_id for s in resolve_selection(selection).steps], plan_c)
        from agent.tools import RobotToolExecutor
        self.assertEqual(RobotToolExecutor(dry_run=True).run_strategy('PlanC').value['tasks'], plan_c)
        for plan in PLANS.values():
            validate_plan(plan)
        for selection in ('all', 'classic', 'PlanA', 'plana'):
            self.assertIs(resolve_selection(selection), PLANS['PlanA'])
        self.assertIs(resolve_selection(), PLANS['PlanA'])
        self.assertEqual(resolve_selection('task0').steps[0].task_id, 'task0-1')
        self.assertEqual([s.task_id for s in resolve_selection('task2-r2').steps],
                         ['task0-2', 'task2-2', 'task3-2'])
        for variant in (1, 2):
            self.assertEqual([s.task_id for s in PLANS[f'collect-build-{variant}'].steps],
                             ['task0-2', f'task2-{variant}', f'task3-{variant}'])
        self.assertEqual([s.task_id for s in resolve_selection('task2-2').steps], ['task2-2'])

    def test_invalid_compositions_and_parameters_fail_before_execution(self):
        for steps in (('task3-1',), ('task2-1', 'task1-1', 'task3-1'),
                      ('task2-1', 'task3-1', 'task3-2'), ('missing',)):
            with self.subTest(steps=steps), self.assertRaises(ValueError):
                run_plan(self.robot, StrategyPlan('bad', tuple(map(TaskStep, steps))))
        with self.assertRaises(TypeError):
            run_plan(self.robot, StrategyPlan('bad-parameter', (
                TaskStep('task2-1'), TaskStep('task3-1', {'wrong_parameter': 1}))))
        self.robot.transport.emergency_stop.assert_not_called()
        self.robot.set_collection_context.assert_not_called()

    def test_reusing_task_creates_fresh_instances_and_separate_parameters(self):
        instances, observations = [], []
        class RecordTask:
            def __init__(self, robot, config, *, context):
                instances.append(self)
                self.config, self.context = config, context
            def run(self):
                observations.append(self.config.value)
                return 0
        definition = TaskDefinition(RecordTask, RecordConfig)
        plan = StrategyPlan('reused', (TaskStep('record', {'value': 4}), TaskStep('record')))
        context = TaskContext(self.robot)
        self.assertEqual(run_plan(self.robot, plan, task_library={'record': definition}, context=context), 0)
        self.assertEqual(observations, [4, 0])
        self.assertIsNot(instances[0], instances[1])
        self.assertIs(instances[0].context, instances[1].context)
        self.assertTrue(context.closed)
        self.assertIsNone(self.robot.actions._cancel_event)
        self.assertFalse(self.robot.strategy_lock.locked())

    def test_failures_never_start_next_task_or_return_false_success(self):
        for failure in ('code', 'structured', 'exception', 'interrupt', 'stop', 'stale', 'reboot', 'disconnect', 'reconnect'):
            with self.subTest(failure=failure):
                robot = robot_fixture()
                calls = []
                class FailingTask:
                    def __init__(self, robot, config, *, context):
                        self.robot = robot
                    def run(self):
                        calls.append('first')
                        if failure == 'code': return 9
                        if failure == 'structured': return TaskResult(TaskStatus.HARDWARE_FAULT)
                        if failure == 'exception': raise RuntimeError('failed')
                        if failure == 'interrupt': raise KeyboardInterrupt()
                        if failure == 'stop': self.robot.transport.emergency_stop()
                        if failure == 'stale': self.robot.telemetry_age = 1.0
                        if failure == 'reboot': self.robot.telem.uptime_ms = 0
                        if failure == 'disconnect': self.robot.transport.connected = False
                        if failure == 'reconnect': self.robot.link_generation += 1
                        return 0
                library = {'first': TaskDefinition(FailingTask, RecordConfig)}
                plan = StrategyPlan('failure', (TaskStep('first'), TaskStep('first')))
                context = TaskContext(robot)
                if failure in ('code', 'structured'):
                    self.assertNotEqual(run_plan(robot, plan, context=context, task_library=library), 0)
                else:
                    error = KeyboardInterrupt if failure == 'interrupt' else RuntimeError
                    with self.assertRaises(error):
                        run_plan(robot, plan, context=context, task_library=library)
                self.assertEqual(calls, ['first'])
                self.assertTrue(context.closed)
                self.assertIsNone(context.build_approach)
                self.assertFalse(robot.strategy_lock.locked())

    def test_stop_before_background_execution_and_closed_context_cannot_restart(self):
        context = TaskContext(self.robot)
        context.cancel_event.set()
        with patch.object(Task2Program, 'run') as run:
            with self.assertRaises(RuntimeError):
                run_plan(self.robot, PLANS['collect-build-1'], context=context)
            run.assert_not_called()
            context.cancel_event.clear()
            with self.assertRaises(RuntimeError):
                run_plan(self.robot, PLANS['collect-build-1'], context=context)
            run.assert_not_called()

    def test_concurrent_strategy_rejected_without_stopping_the_owner(self):
        self.robot.strategy_lock.acquire()
        try:
            with self.assertRaisesRegex(RuntimeError, 'already running'):
                run_plan(self.robot, PLANS['PlanA'])
            self.robot.transport.emergency_stop.assert_not_called()
        finally:
            self.robot.strategy_lock.release()

    def test_fresh_flow_ids_for_separate_runs(self):
        with patch.object(TASK_LIBRARY['task1-1'].program_type, 'run', return_value=0):
            run_plan(self.robot, resolve_selection('task1-1'))
            run_plan(self.robot, resolve_selection('task1-1'))
        flows = [c.kwargs['flow_id'] for c in self.robot.set_collection_context.call_args_list
                 if 'flow_id' in c.kwargs]
        self.assertEqual(len(set(flows)), 2)

    def test_task1_finishes_after_lateral_move_without_turning_back_to_zero(self):
        for task_type, lateral, before, after in (
                (Task1Program, 100, 'right', 'left'),
                (Task1_2Program, 400, 'right', 'left'),
                (Task1_3Program, 500, 'left', 'right')):
            with self.subTest(task=task_type.TASK_LABEL):
                robot = robot_fixture()
                task = task_type(robot, context=TaskContext(robot))
                task._cube_lateral_displacement_mm = 125
                moves, turns = [], []
                task._checked_move = lambda direction, mm, speed, **kw: moves.append((direction, mm, speed))
                task._turn_to_heading = lambda heading, **kw: turns.append(heading)
                task._drive_until_wall = Mock()
                task._align_delivery_tag_or_continue = Mock()
                robot.actions.hatch_open = Mock()
                robot.actions.hatch_close = Mock()
                task._run_delivery_route()
                self.assertEqual(turns, [90, 180])
                self.assertEqual(moves, [('backward', 400, 400), ('forward', 2675, 1000),
                                         (before, lateral, 400), ('backward', 300, 400),
                                         (after, lateral, 400)])
                self.assertEqual(task._heading_zero_deg, 37)
                robot.actions.hatch_open.assert_called_once_with(settle_ms=300)
                robot.actions.hatch_close.assert_called_once_with(settle_ms=0)

    def test_task2_entry_180_preserves_absolute_heading_for_next_turn(self):
        for task_type in (Task2Program, Task2_2Program):
            with self.subTest(task=task_type.TASK_LABEL):
                robot = robot_fixture()
                task = task_type(robot, context=TaskContext(robot))
                task._preflight()
                self.assertEqual(task._heading_zero_deg, 37)
                self.assertEqual(task._heading_error(180), 0)
                task._turn_to_heading(-90)
                robot.chassis.turn.assert_called_once_with(
                    90, 120, hold_ms=0, settle_cycles=1)

    def test_task2_stops_before_tag6_and_exports_calibrated_heading(self):
        for task_type, distance in ((Task2Program, 2500), (Task2_2Program, 2350)):
            for grabbed in (True, False):
                for overlap in (True, False):
                    with self.subTest(task=task_type.__name__, grabbed=grabbed, overlap=overlap):
                        robot = robot_fixture()
                        context = TaskContext(robot, current_task=task_type.TASK_LABEL)
                        task = task_type(robot, context=context)
                        moves, turns, post_purple = [], [], []
                        task._checked_move = lambda direction, mm, speed, **kw: moves.append((direction, mm))
                        task._turn_to_heading = turns.append
                        task._drive_until_wall = Mock()
                        task._capture_lateral_origin = lambda: 'origin'
                        task._measure_lateral_displacement_mm = lambda origin: 125
                        def purple_route(origin):
                            post_purple.append(origin)
                            # Represents the existing wall-based zero recalibration.
                            task._heading_zero_deg = 37.0
                        task._run_post_purple_route = purple_route
                        def purple(*, chassis_followup):
                            if grabbed: chassis_followup()
                            return grabbed
                        task._try_grab_purple = purple
                        def collect(count, grab, *, chassis_followup):
                            self.assertEqual(count, 2 if grabbed else 3)
                            if overlap: chassis_followup()
                        task._collect_orange_with_count_check = collect
                        task._align_delivery_tag = Mock(side_effect=AssertionError('Tag6 is in Task3'))
                        self.assertEqual(task.run(), 0)
                        self.assertEqual(moves, [('backward', distance), ('forward', 250),
                                                 ('backward', 100), ('right', 425), ('forward', 2750)])
                        self.assertEqual(turns, [-90, 180])
                        self.assertEqual(post_purple, ['origin'])
                        self.assertEqual(context.build_approach.heading_zero_deg, 37.0)
                        task._align_delivery_tag.assert_not_called()

    def test_task3_consumes_heading_and_keeps_build_overlap_for_all_variants(self):
        for task_type, direction, lateral, exit_route in (
                (Task3Program, 'right', 100,
                 [('turn', 180), ('left', 2500, 1000), ('wall', 'left')]),
                (Task3_2Program, 'right', 400,
                 [('turn', 180), ('left', 2200, 1000), ('wall', 'left')]),
                (Task3_3Program, 'left', 500,
                 [('turn', 180), ('left', 3000, 1000), ('wall', 'left')]),
                (Task3_4Program, 'right', 100, [('left', 100, 400)]),
                (Task3_5Program, 'right', 400, [('left', 400, 400)])):
            with self.subTest(task=task_type.__name__):
                robot = robot_fixture()
                context = TaskContext(robot, build_approach=BuildApproach(37.0, 'task2'))
                task = task_type(robot, context=context)
                events = []
                @contextlib.contextmanager
                def monitor(check):
                    events.append('monitor_enter')
                    check()
                    yield
                    check()
                    events.append('monitor_exit')
                robot.chassis.monitor_action = monitor
                robot.chassis.turn = lambda angle, speed, **kw: events.append(('turn', angle))
                def build(*, chassis_followup):
                    events.append('build_release')
                    chassis_followup(lambda: events.append('action_check'))
                    events.append('build_done')
                robot.actions.build = build
                task._align_delivery_tag = Mock(side_effect=lambda **kw: events.append('tag6'))
                task._align_building_or_continue = Mock(return_value=True)
                task._checked_move = lambda direction, mm, speed, **kw: events.append((direction, mm, speed))
                task._drive_until_wall = lambda **kw: events.append(('wall', kw['direction']))
                self.assertEqual(task.run(), 0)
                self.assertEqual(task._heading_zero_deg, 37.0)
                self.assertIsNone(context.build_approach)
                self.assertEqual(events, ['tag6', (direction, lateral, 400), 'build_release',
                    'monitor_enter', 'action_check', ('backward', 200, 400)] + exit_route
                    + ['action_check', 'monitor_exit', 'build_done'])
                self.assertNotIn('translation_only_completion', task._align_delivery_tag.call_args.kwargs)
                self.assertTrue(task._align_delivery_tag.call_args.kwargs['independent_heading'])
                self.assertEqual(task._align_delivery_tag.call_args.kwargs['tag_id'], 6)

    def test_task3_missing_or_consumed_handoff_never_commands_motion(self):
        context = TaskContext(self.robot)
        task = Task3Program(self.robot, context=context)
        with patch.object(task, '_run_mission') as mission:
            with self.assertRaisesRegex(RuntimeError, 'handoff'):
                task.run()
            mission.assert_not_called()
        for value in (float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                run_plan(self.robot, resolve_selection('task3-1'), heading_zero_deg=value)

    def test_build_failure_does_not_turn_into_a_successful_return_route(self):
        robot = robot_fixture()
        task = Task3Program(robot)
        task._checked_move = Mock()
        task._align_building_or_continue = Mock(return_value=True)
        robot.actions.build = Mock(side_effect=RuntimeError('action failed'))
        with patch.object(task, '_run_post_build_route') as followup:
            with self.assertRaisesRegex(RuntimeError, 'action failed'):
                task._run_build_phase()
            followup.assert_not_called()

    def test_executor_hands_off_actual_task2_zero_to_fresh_task3_instance(self):
        received = []
        def collect(task):
            task._heading_zero_deg = 37.0
            task.context.publish_build_approach(task._heading_zero_deg)
        def build(task):
            received.append((task._heading_zero_deg, task.context.current_task))
        with patch.object(Task2Program, '_run_collection_route', collect), \
             patch.object(TASK_LIBRARY['task0-2'].program_type, 'run', return_value=0), \
             patch.object(Task3Program, '_run_mission', build):
            self.assertEqual(run_plan(self.robot, PLANS['collect-build-1']), 0)
        self.assertEqual(received, [(37.0, 'task3-1')])

    def test_agent_catalog_and_dry_run_use_the_same_library(self):
        from agent.tools import RobotToolExecutor, tool_definitions
        from Strategy.runner import SELECTION_CHOICES
        schema = next(t for t in tool_definitions() if t['name'] == 'run_strategy')
        self.assertEqual(schema['parameters']['properties']['selection']['enum'], list(SELECTION_CHOICES))
        agent = RobotToolExecutor(dry_run=True)
        self.assertEqual(agent.run_strategy('collect-build-2').value['tasks'],
                         ['task0-2', 'task2-2', 'task3-2'])
        self.assertEqual(agent.run_strategy('task2-1').value['tasks'], ['task2-1'])
        for task_id in ('task3-1', 'task3-4', 'task3-5'):
            with self.subTest(task=task_id):
                with self.assertRaises(ValueError):
                    agent.run_strategy(task_id)
                self.assertEqual(agent.run_strategy(task_id, heading_zero_deg=37).value['tasks'], [task_id])

    def test_readonly_cli_and_missing_handoff_never_construct_robot(self):
        import main
        for args in (['--list-tasks'], ['--strategy', 'classic', '--show-plan'],
                     ['--task', 'task3-2', '--show-plan'],
                     ['--task', 'task1-3', '--show-plan'],
                     ['--task', 'task3-3', '--show-plan'],
                     ['--task', 'task3-4', '--show-plan'],
                     ['--task', 'task3-5', '--show-plan']):
            with self.subTest(args=args), patch('sys.argv', ['main.py', *args]), \
                 patch('main.Robot') as robot:
                self.assertEqual(main.main(), 0)
                robot.assert_not_called()
        for args in (['--task', 'task3-1'], ['--task', 'task3-3'],
                     ['--task', 'task3-4'], ['--task', 'task3-5'],
                     ['--task', 'task3-2', '--heading-zero-deg', 'nan']):
            with self.subTest(args=args), patch('sys.argv', ['main.py', *args]), \
                 patch('main.Robot') as robot, contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(main.main(), 2)
                robot.assert_not_called()


if __name__ == '__main__':
    unittest.main()
