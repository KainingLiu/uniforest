"""Replay the first-row -85 mm target and zero-pickup inspection failures.

Plant values are synthetic; these tests do not certify physical clearance.
"""

from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from protocol.commands import (
    ACTION_IDLE, ACTION_DONE, ACTION_RUNNING, ACTION_CHASSIS_READY,
    ACTION_CANCELLED, ACTION_TIMEOUT, ACTION_REJECTED,
)
from Strategy.optimizations.trial import trial_configuration
from Strategy.flows import operations
from Strategy.settings import HighlandCollectionConfig
from control.carried_cube_inspection import begin_carried_inspection, inspect_carried_cubes
from tests.test_fast_alignment import InertialPlant, profile
from tests.test_inspection_session import InspectionReplay
from tests.test_registered_pickup_transitions import Fixture, action
from tests.test_functional_operations import operation_fixture, spec


class AlignmentOriginTests(unittest.TestCase):
    def test_stationary_target_is_not_rejected_by_search_origin(self):
        plant = InertialPlant(position=-160., target=-160., speed=0.)
        plant._search_position_mm = 0.
        self.assertTrue(plant.fast(profile=trial_configuration().alignment('ground-1', 'orange')))
        self.assertTrue(all(command[1] == 0 for command in plant.commands))

    def test_recorded_left_target_at_collection_origin_can_align(self):
        plant = InertialPlant(position=0., target=-85., speed=0.)
        selected = trial_configuration().alignment('ground-1', 'orange')
        self.assertTrue(plant.fast(profile=selected))
        self.assertLess(abs(plant.target-plant.position), 10.)
        self.assertTrue(any(command[1] < 0 for command in plant.commands))
        self.assertTrue(plant.commands[-1][3])

    def test_existing_profile_allows_left_alignment_without_new_parameters(self):
        plant = InertialPlant(position=0., target=-85., speed=0.)
        self.assertTrue(plant.fast(profile=profile()))
        self.assertLess(abs(plant.target-plant.position), 8.)

    def test_purple_opposite_search_direction_gets_symmetric_correction(self):
        plant = InertialPlant(position=0., target=85., speed=0., config=HighlandCollectionConfig())
        plant.color = 'purple'
        selected = trial_configuration().alignment('highland-1', 'purple')
        self.assertTrue(plant.fast(profile=selected))
        self.assertLess(abs(plant.target-plant.position), 10.)

    def test_alignment_travel_limit_applies_in_both_directions(self):
        # Match the synthetic plant's actual 200 mm/s2 acceleration/braking.
        selected = profile(max_travel_mm=150., max_duration_s=8.)
        for direction in (-1, 1):
            plant = InertialPlant(position=0., target=direction*300., speed=0.)
            self.assertFalse(plant.fast(profile=selected))
            for _ in range(30): plant.sleep(.02)
            self.assertLessEqual(max(abs(x) for _, x in plant.history), 150.)
            self.assertGreater(abs(plant.position), 100.)

    def test_reverse_alignment_keeps_consumed_forward_search_budget(self):
        plant = InertialPlant(position=0., target=-85., speed=0.)
        selected = profile(max_duration_s=8., max_travel_mm=500.)
        consumed = plant.config.search_max_distance_mm-1.
        plant._search_position_mm = consumed
        self.assertTrue(plant.fast(profile=selected))
        self.assertEqual(plant._search_position_mm, consumed)
        plant.target = 65.
        self.assertTrue(plant.fast(profile=selected))
        self.assertEqual(plant._search_position_mm, consumed)
        plant.target = None
        self.assertFalse(plant.run(alignment=selected))
        self.assertGreaterEqual(plant._search_position_mm, consumed)

    def test_negative_position_remains_valid_after_moving_visual_handoff(self):
        plant = InertialPlant(position=-60., target=-85., speed=20.)
        plant._search_position_mm = 20.
        selected = trial_configuration().alignment('ground-1', 'orange')
        self.assertTrue(plant.run(alignment=selected))
        self.assertLess(abs(plant.target-plant.position), 10.)
        self.assertGreaterEqual(plant._search_position_mm, 20.)


def idle_replay(state=ACTION_IDLE):
    replay = InspectionReplay(count=0)
    replay.transport.get_action_status = lambda: (
        SimpleNamespace(state=state, uptime_ms=0), replay.now)
    return replay


class IdleInspectionTests(unittest.TestCase):
    def test_idle_requires_explicit_zero_pickup_permission(self):
        replay = idle_replay()
        with replay.patched(), self.assertRaisesRegex(RuntimeError, 'before inspection: 0'):
            begin_carried_inspection(replay).inspect()
        self.assertEqual(replay.servo_commands, [])

    def test_idle_zero_pickup_inspection_returns_real_empty_count(self):
        replay = idle_replay()
        with replay.patched():
            self.assertEqual(inspect_carried_cubes(replay, allow_idle=True), 0)
        self.assertFalse(replay.actions._action_lock.locked())
        replay.transport.emergency_stop.assert_not_called()
        self.assertEqual(replay.servo_commands[-1], (0, 97.2))

    def test_idle_cannot_bypass_busy_motors_or_steppers(self):
        for kind in ('motor', 'stepper', 'frozen'):
            replay = idle_replay()
            original = replay.inspection_link_snapshot
            def snapshot():
                telem, received, pong, epoch = original()
                if kind == 'motor': telem.motors[0].speed_rpm = 50
                if kind == 'stepper': telem.stepper_busy = 1
                if kind == 'frozen': telem.uptime_ms = 100000
                return telem, received, pong, epoch
            replay.inspection_link_snapshot = snapshot
            with self.subTest(kind=kind), replay.patched(), self.assertRaisesRegex(RuntimeError, 'stationary timeout'):
                begin_carried_inspection(replay, allow_idle=True).inspect()
            self.assertEqual(replay.servo_commands, [])
            replay.transport.emergency_stop.assert_called_once()

    def test_idle_permission_never_accepts_active_or_failed_actions(self):
        for state in (ACTION_RUNNING, ACTION_CHASSIS_READY, ACTION_CANCELLED, ACTION_TIMEOUT, ACTION_REJECTED):
            replay = idle_replay(state)
            with self.subTest(state=state), replay.patched(), self.assertRaisesRegex(RuntimeError, 'before inspection'):
                begin_carried_inspection(replay, allow_idle=True).inspect()
            self.assertEqual(replay.servo_commands, [])
            replay.transport.emergency_stop.assert_called_once()

    def test_idle_permission_keeps_communication_and_cancellation_guards(self):
        for kind in ('stale', 'cancel', 'stop', 'reconnect'):
            replay = idle_replay()
            with self.subTest(kind=kind), replay.patched():
                session = begin_carried_inspection(replay, allow_idle=True)
                if kind == 'stale': replay.stale = True
                if kind == 'cancel': replay.cancelled = True
                if kind == 'stop': replay.transport.emergency_stop_generation += 1
                if kind == 'reconnect': replay.epoch += 1
                with self.assertRaises(RuntimeError): session.inspect()
            self.assertEqual(replay.servo_commands, [])


class ZeroPickupFlowTests(unittest.TestCase):
    def test_all_failed_acquisitions_can_inspect_idle_board_and_try_bounded_refill(self):
        fixture = Fixture()
        fixture.env.data['collection']['acquired'] = False
        fixture.env.transition_config = replace(fixture.env.transition_config,
            alignments={'ground-1/orange':profile()})
        replay = idle_replay()
        fixture.robot.begin_carried_cube_inspection = lambda **kw: begin_carried_inspection(replay, **kw)
        steps = []
        for index in (1, 2, 3):
            steps += [action('acquire_cube', f'find.{index}', index=index),
                      action('grab_cube', f'grab.{index}', method='grap3', index=index)]
        steps.append(action('inspect_cargo', 'inspect', method='grap3'))
        with replay.patched(), patch('Strategy.flows.operations._fast_align', return_value=False):
            fixture.run(steps)
        self.assertEqual(fixture.sessions, [])
        self.assertEqual(fixture.env.data['collection']['pickups'], 0)
        self.assertEqual(fixture.env.data['carried_count'], 0)
        self.assertFalse(replay.actions._action_lock.locked())
        replay.transport.emergency_stop.assert_not_called()

    def test_completed_pickup_keeps_done_requirement_in_registered_path(self):
        fixture = Fixture()
        fixture.env.data['collection']['pickups'] = 1
        replay = idle_replay()
        fixture.robot.begin_carried_cube_inspection = lambda **kw: begin_carried_inspection(replay, **kw)
        with replay.patched(), self.assertRaisesRegex(RuntimeError, 'before inspection: 0'):
            fixture.run([action('inspect_cargo', 'inspect', method='grap3')])
        self.assertEqual(replay.servo_commands, [])

    def test_blocking_operation_only_allows_idle_before_first_completed_pickup(self):
        for pickups in (0, 1):
            env, control = operation_fixture()
            operations.begin_collection(env, spec('begin_collection'))
            env.data['collection']['pickups'] = pickups
            replay = idle_replay()
            env.robot.check_carried_cube_count = lambda **kw: inspect_carried_cubes(replay, **kw)
            with replay.patched(), patch('Strategy.flows.operations.acquire_cube', return_value=False):
                if pickups == 0:
                    self.assertIsNone(operations.inspect_cargo(env, spec('inspect_cargo', method='grap3')))
                    self.assertEqual(env.data['carried_count'], 0)
                else:
                    with self.assertRaisesRegex(RuntimeError, 'before inspection: 0'):
                        operations.inspect_cargo(env, spec('inspect_cargo', method='grap3'))


if __name__ == '__main__':
    unittest.main()
