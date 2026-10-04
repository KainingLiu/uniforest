"""PlanD round identity, final vision evidence and unload commit boundaries."""

import contextlib
import io
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from control.carried_cube_inspection import inspect_carried_cubes
from protocol.commands import (ACTION_IDLE, ACTION_RUNNING, ACTION_CANCELLED,
                               ACTION_TIMEOUT, ACTION_REJECTED, ACTION_CHASSIS_READY)
from Strategy.competition import CompetitionProgram, SearchRangeExhausted, TaskControl
from Strategy.context import TaskContext
from Strategy.plan_d_state import CargoResult, PlanDState
from Strategy.plans import PLANS, StrategyPlan
from Strategy.runner import run_plan
from Strategy.task2 import Task2Program
from Strategy.tasks import TaskDefinition, TaskStep
from tests.test_strategy_composition import RecordConfig, robot_fixture
from tests.test_visual_fallback import CountReplay


class PlanDCargoTests(unittest.TestCase):
    def setUp(self):
        quiet = contextlib.redirect_stdout(io.StringIO())
        quiet.__enter__()
        self.addCleanup(quiet.__exit__, None, None, None)

    def run_cargo(self, counts, *, exhausted=(), refill_counts=None,
                  refill_grabs=0, fault=None, both_sites=False, plan_name='PlanD'):
        """Use real collection/exit/unload/runner logic with motion substituted."""
        robot = robot_fixture()
        context = TaskContext(robot)
        events = []
        self.last_robot, self.last_context, self.last_events = robot, context, events
        count_iters = {source: iter(values) for source, values in counts.items()}
        refill_iters = {source: iter(values) for source, values in (refill_counts or {}).items()}
        robot.has_vision = robot.has_field_localization = False
        robot.move_chassis = Mock(return_value=SimpleNamespace(timed_out=False, cancelled=False))

        def fail_if(point):
            if fault is None or fault[0] != point:
                return
            if fault[1] == 'stop':
                robot.transport.emergency_stop_generation += 1
            elif fault[1] == 'stale':
                robot.telemetry_age = 1
            elif fault[1] == 'disconnect':
                robot.transport.connected = False
            elif fault[1] == 'cancel':
                context.cancel_event.set()
            else:
                raise RuntimeError('injected hardware failure')

        def inspect(**kwargs):
            task = context.current_task
            source = (context.plan_d.refill_source_task
                      if task == 'task2-0' and context.plan_d is not None else task)
            fail_if('inspection')
            callback = kwargs.get('chassis_followup')
            if callback is not None:
                callback()
            result = next((refill_iters if task == 'task2-0' else count_iters)[source])
            events.append(('count', task, source, result,
                           kwargs.get('inspect_while_moving', False)))
            if context.plan_d is not None:
                self.assertIsNone(context.plan_d.sites['A' if source == 'task1-1' else 'B']
                                  .unload_completed_at)
            return result

        robot.check_carried_cube_count = Mock(side_effect=inspect)

        def hatch_open(**kwargs):
            events.append(('open', context.current_task))
            if context.plan_d is not None:
                site = 'A' if context.current_task == 'task1-1' else 'B'
                self.assertIsNone(context.plan_d.base_height(site))
            fail_if('open')

        def hatch_close(**kwargs):
            events.append(('close', context.current_task))
            fail_if('close')

        robot.actions.hatch_open = Mock(side_effect=hatch_open)
        robot.actions.hatch_close = Mock(side_effect=hatch_close)

        def diagnostics(kind, **kwargs):
            if kind == 'task_start':
                events.append(('start', kwargs['task']))
            elif kind == 'plan_d_unload':
                events.append(('commit', kwargs['site_id'], kwargs['deposited_count']))
        robot.diagnostics.write = Mock(side_effect=diagnostics)

        def move(task, direction, distance, speed, **kwargs):
            events.append(('move', task.TASK_LABEL, direction, distance))
            if task.TASK_LABEL == 'task2-0':
                fail_if('refill_move')

        def initial_grab(task):
            if task.TASK_LABEL in exhausted:
                raise SearchRangeExhausted()
            events.append(('grab', task.TASK_LABEL))

        def refill_grab(task):
            if task.completed_grabs >= refill_grabs:
                raise SearchRangeExhausted()
            events.append(('refill_grab',))

        def align(task, **kwargs):
            events.append(('tag6', task.TASK_LABEL, task._heading_zero_deg))
            fail_if('tag6')
            task._check_active()
            return False

        def calibrate(task, **kwargs):
            task._heading_zero_deg = 31.5 if task.TASK_LABEL == 'task2-0' else 15

        steps = [TaskStep('task1-1')]
        if both_sites:
            steps += [TaskStep('task0-3'), TaskStep('task1-2')]
        with patch.object(TaskControl, '_checked_move', move), \
             patch.object(TaskControl, '_drive_until_wall'), \
             patch.object(TaskControl, '_turn_to_heading'), \
             patch.object(TaskControl, '_capture_lateral_origin', return_value=0), \
             patch.object(TaskControl, '_measure_lateral_displacement_mm', return_value=125), \
             patch.object(TaskControl, '_recalibrate_heading_zero', calibrate), \
             patch.object(TaskControl, '_align_delivery_tag_or_continue', align), \
             patch.object(CompetitionProgram, '_grab_task1_orange', initial_grab), \
             patch.object(Task2Program, '_grab_task2_orange', refill_grab):
            self.assertEqual(run_plan(robot, StrategyPlan(plan_name, tuple(steps)),
                                      context=context), 0)
        robot.transport.emergency_stop.assert_not_called()
        return robot, context, events

    def test_normal_count_commits_after_close_with_round_identity(self):
        robot, context, events = self.run_cargo({'task1-1': [3], 'task1-2': [3]},
                                                both_sites=True)
        self.assertEqual(robot.check_carried_cube_count.call_count, 2)
        self.assertEqual([e[1] for e in events if e[0] == 'start'],
                         ['task1-1', 'task0-3', 'task1-2'])
        for source, site_id, round_id in (('task1-1', 'A', 3), ('task1-2', 'B', 4)):
            site = context.plan_d.sites[site_id]
            self.assertEqual(site.cargo, CargoResult(round_id, site_id, 3))
            self.assertEqual(context.plan_d.base_height(site_id), 3)
            self.assertGreater(events.index(('commit', site_id, 3)),
                               events.index(('close', source)))
            self.assertIn('stack geometry unverified', site.height_evidence)
        self.assertTrue(context.closed)
        self.assertTrue(context.plan_d.closed)

    def test_refill_exit_overwrites_prior_count_without_grab_arithmetic(self):
        for observed in (0, 1, 2, 3, None):
            for grabs in (0, 1, 2):
                with self.subTest(observed=observed, grabs=grabs):
                    robot, context, events = self.run_cargo(
                        {'task1-1': [1] + ([None] if observed is None else [])},
                        exhausted=('task1-1',), refill_counts={'task1-1': [observed]},
                        refill_grabs=grabs)
                    site = context.plan_d.sites['A']
                    self.assertEqual(site.observed_cargo_count, observed)
                    self.assertEqual(site.deposited_count, observed)
                    self.assertEqual(site.height_status, 'unknown' if observed is None else 'known')
                    self.assertTrue(site.cargo.refill_used)
                    self.assertEqual(site.cargo.round_id, 3)
                    self.assertEqual([e[1] for e in events if e[0] == 'start'],
                                     ['task1-1', 'task2-0'])
                    self.assertEqual(events.count(('refill_grab',)), grabs)
                    self.assertIn(('count', 'task2-0', 'task1-1', observed, True), events)
                    self.assertIn(('tag6', 'task1-1', 31.5), events)
                    robot.actions.hatch_open.assert_called_once()
                    self.assertIsNone(context.plan_d.refill_source_task)

    def test_two_sites_keep_independent_final_counts_across_transfer(self):
        _, context, events = self.run_cargo(
            {'task1-1': [0], 'task1-2': [1]}, both_sites=True,
            exhausted=('task1-1', 'task1-2'),
            refill_counts={'task1-1': [2], 'task1-2': [1]})
        self.assertEqual(context.plan_d.base_height('A'), 2)
        self.assertEqual(context.plan_d.base_height('B'), 1)
        self.assertEqual(context.plan_d.sites['B'].cargo.round_id, 4)
        self.assertEqual([e[1] for e in events if e[0] == 'start'],
                         ['task1-1', 'task2-0', 'task0-3', 'task1-2', 'task2-0'])

    def test_full_exhausted_exit_needs_no_detour(self):
        robot, context, events = self.run_cargo({'task1-1': [3]},
                                               exhausted=('task1-1',))
        self.assertEqual(context.plan_d.base_height('A'), 3)
        self.assertEqual([e[1] for e in events if e[0] == 'start'], ['task1-1'])
        robot.check_carried_cube_count.assert_called_once()

    def test_unknown_receives_one_observation_retry_without_more_refill(self):
        for retry in (0, 1, 2, 3, None):
            with self.subTest(retry=retry):
                robot, context, events = self.run_cargo({'task1-1': [None, retry]})
                self.assertEqual(robot.check_carried_cube_count.call_count, 2)
                self.assertEqual(context.plan_d.base_height('A'), retry)
                self.assertTrue(context.plan_d.sites['A'].unknown_recheck_used)
                self.assertEqual([e[1] for e in events if e[0] == 'start'], ['task1-1'])
                self.assertEqual(len([e for e in events if e[0] == 'grab']), 3)
                self.assertEqual(robot.check_carried_cube_count.call_args.kwargs,
                                 {'allow_visual_failure': True, 'allow_idle': True})

    def test_unknown_original_detours_once_and_preserves_unknown_after_retry(self):
        robot, context, events = self.run_cargo(
            {'task1-1': [None, None]}, exhausted=('task1-1',),
            refill_counts={'task1-1': [None]}, refill_grabs=1)
        self.assertEqual(robot.check_carried_cube_count.call_count, 3)
        self.assertEqual([e[1] for e in events if e[0] == 'start'], ['task1-1', 'task2-0'])
        self.assertIsNone(context.plan_d.base_height('A'))
        self.assertEqual(context.plan_d.sites['A'].height_status, 'unknown')

    def test_failures_never_commit_or_continue_to_next_round(self):
        for point in ('inspection', 'refill_move', 'tag6', 'open', 'close'):
            for failure in ('hardware', 'stop', 'stale', 'disconnect', 'cancel'):
                with self.subTest(point=point, failure=failure):
                    with self.assertRaises(RuntimeError):
                        self.run_cargo({'task1-1': [1], 'task1-2': [3]},
                                       exhausted=('task1-1',), both_sites=True,
                                       refill_counts={'task1-1': [2]},
                                       fault=(point, failure))
                    state = self.last_context.plan_d
                    self.assertIsNone(state.sites['A'].unload_completed_at)
                    self.assertEqual(state.sites['A'].height_status, 'pending')
                    self.assertTrue(state.closed)
                    self.assertNotIn(('start', 'task0-3'), self.last_events)
                    self.assertTrue(self.last_robot.transport.emergency_stop.called)

    def test_ordinary_strategy_does_not_create_state_or_reinspect_refill(self):
        robot, context, events = self.run_cargo(
            {'task1-1': [1]}, exhausted=('task1-1',), plan_name='PlanB')
        self.assertIsNone(context.plan_d)
        robot.check_carried_cube_count.assert_called_once()
        self.assertEqual([e[1] for e in events if e[0] == 'start'], ['task1-1', 'task2-0'])

    def test_run_state_cannot_resume_and_new_context_starts_empty(self):
        robot, context, _ = self.run_cargo({'task1-1': [3]})
        with self.assertRaisesRegex(ValueError, 'cannot be resumed'):
            run_plan(robot, StrategyPlan('PlanD', (TaskStep('task1-1'),)), context=context)
        fresh = TaskContext(robot)
        self.assertIsNone(fresh.plan_d)
        state = PlanDState()
        self.assertIsNone(state.base_height('A'))
        self.assertIsNone(state.base_height('B'))

        snapshots = []

        class SnapshotTask:
            def __init__(self, robot, config, *, context):
                self.context = context

            def run(self):
                snapshots.append(self.context.plan_d)
                self.context.plan_d.record_cargo('task1-1', 1)
                self.context.plan_d.commit_unload('task1-1')
                return 0

        plan = StrategyPlan('PlanD', (TaskStep('snapshot'),))
        library = {'snapshot': TaskDefinition(SnapshotTask, RecordConfig)}
        self.assertEqual(run_plan(robot, plan, context=fresh, task_library=library), 0)
        self.assertEqual(run_plan(robot, plan, task_library=library), 0)
        self.assertIsNot(snapshots[0], snapshots[1])
        self.assertTrue(all(state.closed for state in snapshots))

    def test_malformed_task5_calibration_fails_before_any_plan_movement(self):
        robot = robot_fixture()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'malformed.json'
            path.write_text('{"schema_version": 99}', encoding='utf-8')
            plan = StrategyPlan('PlanD', tuple(
                TaskStep('task5', {'building_profiles_path': str(path)})
                if step.task_id == 'task5' else step for step in PLANS['PlanD'].steps))
            with self.assertRaises(ValueError):
                run_plan(robot, plan)
        robot.set_collection_context.assert_not_called()
        robot.transport.emergency_stop.assert_not_called()
        robot.chassis.turn.assert_not_called()
        self.assertFalse(robot.strategy_lock.locked())

    def test_state_validates_evidence_rejects_duplicate_and_closes(self):
        for invalid in (-1, 4, True, 1.5, '2'):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                CargoResult(3, 'A', invalid)
        with self.assertRaises(ValueError):
            CargoResult(4, 'A', 2)
        state = PlanDState()
        with self.assertRaisesRegex(RuntimeError, 'without cargo'):
            state.commit_unload('task1-1')
        state.record_cargo('task1-1', 2)
        state.begin_refill('task1-1')
        with self.assertRaisesRegex(RuntimeError, 'final cargo'):
            state.require_refill_result('task1-1')
        state.record_cargo('task1-1', 2, refill_used=True)
        self.assertEqual(state.require_refill_result('task1-1').count, 2)
        state.finish_refill('task1-1')
        with self.assertRaisesRegex(RuntimeError, 'one refill'):
            state.begin_refill('task1-1')
        self.assertIsNone(state.base_height('A'))
        state.commit_unload('task1-1')
        with self.assertRaisesRegex(RuntimeError, 'already unloaded'):
            state.commit_unload('task1-1')
        with self.assertRaisesRegex(RuntimeError, 'already unloaded'):
            state.record_cargo('task1-1', 3)
        state.mark_topping('A', 'done')
        state.mark_topping('A', 'done')
        with self.assertRaises(RuntimeError):
            state.mark_topping('A', 'skipped')
        state.close()
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            state.record_cargo('task1-2', 1)
        self.assertEqual(state.base_height('A'), 2)

    def test_stationary_unknown_retry_accepts_idle_but_rejects_failed_or_active_actions(self):
        for action_state in (ACTION_IDLE, ACTION_RUNNING, ACTION_CANCELLED,
                             ACTION_TIMEOUT, ACTION_REJECTED, ACTION_CHASSIS_READY):
            with self.subTest(action_state=action_state):
                replay = CountReplay('none')
                replay.chassis.set_speeds = Mock(return_value=True)
                replay.transport.get_action_status = lambda: (
                    SimpleNamespace(state=action_state, uptime_ms=0), replay.now)
                with patch('control.carried_cube_inspection.time', replay.clock):
                    if action_state == ACTION_IDLE:
                        self.assertIsNone(inspect_carried_cubes(
                            replay, allow_visual_failure=True, allow_idle=True))
                        replay.chassis.set_speeds.assert_called_once_with([0, 0, 0, 0])
                        replay.transport.emergency_stop.assert_not_called()
                    else:
                        with self.assertRaisesRegex(RuntimeError, 'not complete'):
                            inspect_carried_cubes(replay, allow_idle=True)
                        replay.transport.emergency_stop.assert_called_once()
                self.assertEqual(replay.servos, [])

    def test_allow_idle_preserves_stationary_wheel_and_stepper_guards(self):
        for fault in ('moving', 'busy', 'default_idle'):
            with self.subTest(fault=fault):
                replay = CountReplay('none')
                replay.transport.get_action_status = lambda: (
                    SimpleNamespace(state=ACTION_IDLE, uptime_ms=0), replay.now)
                original_snapshot = replay.inspection_link_snapshot

                def snapshot():
                    telem, received, pong, epoch = original_snapshot()
                    telem.stepper_busy = int(fault == 'busy')
                    if fault == 'moving':
                        for motor in telem.motors:
                            motor.speed_rpm = 100
                    return telem, received, pong, epoch

                replay.inspection_link_snapshot = snapshot
                with patch('control.carried_cube_inspection.time', replay.clock):
                    with self.assertRaises(RuntimeError):
                        inspect_carried_cubes(replay, allow_visual_failure=True,
                                              allow_idle=fault != 'default_idle')
                self.assertEqual(replay.servos, [])
                replay.transport.emergency_stop.assert_called_once()


if __name__ == '__main__':
    unittest.main()
