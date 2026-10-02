"""Exercise registered Inspect -> Navigate phase replacement in real flows."""

import contextlib
import io
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from Strategy.context import ExecutionContext
from Strategy.flows import operations
from Strategy.flows.factory import ActionEnvironment
from Strategy.flows.model import ActionSpec
from Strategy.plans import StrategyPlan
from Strategy.runner import run_plan
from Strategy.transition_config import TransitionConfig
from Strategy.transition_switches import TransitionSwitches
from tests.test_strategy_composition import robot_fixture


class InspectionAdapter:
    """Explicit restore/ownership fake; the actual servo session has its own tests."""

    def __init__(self, owner, count, index):
        self.owner, self.count, self.index = owner, count, index
        self.closed = self.restored = False
        self.finish_calls = self.close_calls = 0
        if not owner.robot.actions._action_lock.acquire(blocking=False):
            raise RuntimeError('mechanism already owned')
        owner.events.append(('inspect.begin', index))

    def inspect(self):
        self.owner.context.check_active()
        self.owner.events.append(('inspect.count', self.index, self.count))
        return self.count

    def check_restore(self):
        try:
            self.owner.context.check_active()
        except BaseException:
            self.abort()
            raise
        self.owner.events.append(('inspect.check', self.index))
        return self.restored

    def finish_restore(self):
        self.owner.context.check_active()
        self.finish_calls += 1
        self.restored = True
        self.owner.events.append(('inspect.finish', self.index))

    def close(self):
        if self.closed:
            return
        if not self.restored:
            raise AssertionError('inspection owner released before restoration')
        self.close_calls += 1
        self.closed = True
        self.owner.robot.actions._action_lock.release()
        self.owner.events.append(('inspect.close', self.index))

    def abort(self):
        if not self.closed:
            self.closed = True
            self.owner.robot.transport.emergency_stop()
            self.owner.robot.actions._action_lock.release()
            self.owner.events.append(('inspect.abort', self.index))


class RefillAdapter:
    def __init__(self, owner):
        self.owner = owner
        self.done = self.closed = self.chassis_ready = False
        if not owner.robot.actions._action_lock.acquire(blocking=False):
            raise AssertionError('refill began while inspection still owned mechanism')
        owner.events.append('refill.begin')

    def wait_chassis_ready(self, parallel_step=None):
        self.chassis_ready = True

    def check(self):
        self.owner.context.check_active()

    def wait_done(self):
        self.done = True

    def close(self):
        if not self.closed:
            self.closed = True
            self.owner.robot.actions._action_lock.release()
            self.owner.events.append('refill.close')

    def abort(self):
        if not self.closed:
            self.closed = True
            self.owner.robot.actions._action_lock.release()
            self.owner.robot.transport.emergency_stop()


class RegisteredInspectionReplay:
    def __init__(self, counts, *, profile='ground-1', early_departure=None,
                 route_fault=None, refill_available=True):
        self.robot = robot_fixture()
        self.robot.actions._action_lock = threading.Lock()
        self.robot.actions.pickup_full_lift_validated = False
        self.events, self.sessions = [], []
        self.context = ExecutionContext(self.robot, heading_zero_deg=37)
        self.env = ActionEnvironment(self.robot, self.context, transition_config=TransitionConfig(
            switches=TransitionSwitches(overrides={'inspect-departure': True})))
        self.control = self.env.control(profile)
        self.profile, self.early_departure = profile, early_departure
        self.route_fault, self.refill_available = route_fault, refill_available
        self.counts = iter(counts)
        self.control._capture_lateral_origin = Mock(return_value=(0, 0, 0, 0))
        self.control._checked_move = Mock(side_effect=lambda *a, **kw:
                                         self.events.append(('move', a, kw)))
        self.control._drive_until_wall = Mock(side_effect=lambda **kw:
                                            self.events.append(('wall', kw)))
        self.control._recalibrate_heading_zero = Mock(side_effect=lambda:
                                                     self.events.append('recalibrate'))
        self.control._grab_press_step = Mock(return_value=lambda check: True)
        self.robot.actions.begin = Mock(side_effect=lambda action_id: RefillAdapter(self))
        self.robot.begin_carried_cube_inspection = Mock(side_effect=self.begin)
        self.robot.check_carried_cube_count = Mock(side_effect=AssertionError('blocking fallback used'))
        self.exit_route = ('ground_delivery_reverse' if profile.startswith('ground')
                           else 'orange_depart_reverse')
        self.route = ('ground_to_delivery' if profile.startswith('ground')
                      else 'orange_to_build')
        method = 'grap3' if profile.startswith('ground') else 'grap1'
        self.plan = StrategyPlan('inspection-transfer', (
            ActionSpec('begin_collection', 'batch.begin', profile, {'color': 'orange'}),
            ActionSpec('inspect_cargo', 'batch.inspect', profile,
                       {'method': method, 'exit_route': self.exit_route}),
            ActionSpec('navigate', 'batch.depart', profile, {'route': self.route}),
        ))
        self.env.run_route = Mock(side_effect=self.run_route)

    def begin(self, **kwargs):
        if kwargs != {'allow_visual_failure': True,
                      'allow_idle': self.env.data['collection']['pickups'] == 0}:
            raise AssertionError('competition inspection must preserve visual fallback')
        session = InspectionAdapter(self, next(self.counts), len(self.sessions))
        self.sessions.append(session)
        return session

    def run_route(self, route, profile):
        self.events.append(('route', route, self.robot.actions._action_lock.locked()))
        if self.route_fault == 'exception':
            raise RuntimeError('route failed')
        if self.route_fault == 'stale':
            self.robot.telemetry_age = 1

    def seed(self, env, spec):
        result = operations.begin_collection(env, spec)
        if self.early_departure is not None:
            env.transitions.early_departure = (self.profile, self.early_departure, self.exit_route)
            flag = 'ground_reverse_done' if self.profile.startswith('ground') else 'orange_reverse_done'
            env.data[flag] = True
        return result

    def acquire(self, env, spec):
        if self.robot.actions._action_lock.locked():
            raise AssertionError('refill acquisition began before inspection close')
        self.events.append('refill.acquire')
        env.data['collection']['acquired'] = self.refill_available
        env.data['collection']['exhausted'] = not self.refill_available
        return self.refill_available

    def run(self):
        with patch('Strategy.runner.ActionEnvironment', return_value=self.env), \
             patch.dict(operations.OPERATIONS, {'begin_collection': self.seed}), \
             patch('Strategy.flows.transitions.operations.acquire_cube', side_effect=self.acquire), \
             contextlib.redirect_stdout(io.StringIO()):
            return run_plan(self.robot, self.plan, context=self.context)


class RegisteredInspectionTransitionTests(unittest.TestCase):
    def test_full_and_unknown_overlap_only_reverse_then_close_before_route_once(self):
        for profile in ('ground-1', 'highland-1'):
            for count in (3, None):
                with self.subTest(profile=profile, count=count):
                    replay = RegisteredInspectionReplay([count], profile=profile)
                    self.assertEqual(replay.run(), 0)
                    events = replay.events
                    self.assertLess(events.index(('route', replay.exit_route, True)),
                                    events.index(('inspect.finish', 0)))
                    self.assertLess(events.index(('inspect.close', 0)),
                                    events.index(('route', replay.route, False)))
                    self.assertEqual(replay.env.run_route.call_args_list, [
                        call(replay.exit_route, profile), call(replay.route, profile)])
                    self.assertEqual((replay.sessions[0].finish_calls,
                                      replay.sessions[0].close_calls), (1, 1))
                    self.assertEqual(replay.env.transitions.inspections, {})
                    self.assertFalse(replay.robot.actions._action_lock.locked())
                    replay.robot.check_carried_cube_count.assert_not_called()
                    replay.robot.transport.emergency_stop.assert_not_called()

    def test_partial_count_restores_unlocks_then_refills_before_next_inspection(self):
        for count in (0, 1, 2):
            with self.subTest(count=count):
                replay = RegisteredInspectionReplay([count, 3])
                self.assertEqual(replay.run(), 0)
                events = replay.events
                self.assertLess(events.index(('inspect.finish', 0)),
                                events.index(('inspect.close', 0)))
                self.assertLess(events.index(('inspect.close', 0)), events.index('refill.acquire'))
                self.assertEqual(events.count('refill.begin'), 3-count)
                self.assertEqual(events.count('refill.close'), 3-count)
                self.assertLess(max(i for i,e in enumerate(events) if e == 'refill.close'),
                                events.index(('inspect.begin', 1)))
                self.assertEqual(replay.env.data['collection']['pickups'], 3-count)
                self.assertEqual([(s.finish_calls,s.close_calls) for s in replay.sessions], [(1,1),(1,1)])
                self.assertEqual(replay.env.run_route.call_args_list, [
                    call(replay.exit_route, replay.profile),
                    call(replay.route, replay.profile)])
                self.assertLess(events.index(('inspect.close', 1)),
                                events.index(('route', replay.route, False)))

    def test_early_departure_returns_actual_distance_and_reanchors_before_refill(self):
        for profile, distance in (('ground-1', 382.5), ('highland-1', 91.75)):
            with self.subTest(profile=profile):
                replay = RegisteredInspectionReplay([2, 3], profile=profile,
                                                    early_departure=distance)
                self.assertEqual(replay.run(), 0)
                replay.control._checked_move.assert_called_once_with('forward', distance, 400)
                replay.control._drive_until_wall.assert_called_once_with(
                    context='Return for verified cargo refill')
                events = replay.events
                self.assertLess(events.index(('inspect.close', 0)),
                                events.index(('move', ('forward', distance, 400), {})))
                self.assertLess(events.index(('move', ('forward', distance, 400), {})),
                                events.index(('wall', {'context': 'Return for verified cargo refill'})))
                self.assertLess(events.index('recalibrate'), events.index('refill.acquire'))
                flag = 'ground_reverse_done' if profile.startswith('ground') else 'orange_reverse_done'
                self.assertFalse(replay.env.data[flag])
                self.assertIsNone(replay.env.transitions.early_departure)

    def test_refill_search_exhaustion_releases_inspection_and_runs_route_serially(self):
        replay = RegisteredInspectionReplay([2], refill_available=False)
        self.assertEqual(replay.run(), 0)
        self.assertIn(('route', replay.route, False), replay.events)
        self.assertNotIn('refill.begin', replay.events)
        self.assertEqual(replay.sessions[0].finish_calls, 1)
        self.assertEqual(replay.env.transitions.inspections, {})

    def test_route_fault_cancels_registered_overlap_and_cleans_owned_session(self):
        for fault in ('exception', 'stale'):
            with self.subTest(fault=fault):
                replay = RegisteredInspectionReplay([3], route_fault=fault)
                with self.assertRaisesRegex(RuntimeError, 'route failed|stale'):
                    replay.run()
                self.assertEqual(replay.env.run_route.call_count, 1)
                self.assertIn(('inspect.abort', 0), replay.events)
                self.assertEqual(replay.sessions[0].finish_calls, 0)
                self.assertFalse(replay.robot.actions._action_lock.locked())
                self.assertEqual(replay.env.transitions.inspections, {})
                self.assertTrue(replay.context.closed)

    def test_unregistered_following_route_waits_for_restore_before_motion(self):
        replay = RegisteredInspectionReplay([3])
        replay.plan = StrategyPlan('inspection-unregistered-followup', (
            *replay.plan.steps[:-1],
            ActionSpec('navigate', 'batch.offset', replay.profile, {'route': 'ground_tag_offset'}),
        ))
        self.assertEqual(replay.run(), 0)
        self.assertIn(('route', 'ground_tag_offset', False), replay.events)
        self.assertLess(replay.events.index(('inspect.close', 0)),
                        replay.events.index(('route', 'ground_tag_offset', False)))
        self.assertEqual(replay.sessions[0].finish_calls, 1)

    def test_invalid_adapter_count_aborts_before_any_route_or_refill(self):
        for count in (True, 1.0, -1, 4):
            with self.subTest(count=count):
                replay = RegisteredInspectionReplay([count])
                with self.assertRaisesRegex(RuntimeError, 'invalid carried cube count'):
                    replay.run()
                replay.env.run_route.assert_not_called()
                self.assertNotIn('refill.acquire', replay.events)
                self.assertFalse(replay.robot.actions._action_lock.locked())
                self.assertEqual(replay.env.transitions.inspections, {})


if __name__ == '__main__':
    unittest.main()
