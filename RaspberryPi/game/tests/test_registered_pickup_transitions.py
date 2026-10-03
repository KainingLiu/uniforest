"""Registered phase replacements with real factory/runtime and fake hardware.

Mechanism milestones and moving vision are deterministic substitutes here;
these tests verify ownership/sequence contracts, not physical motion calibration.
"""

import contextlib
from dataclasses import replace
from types import SimpleNamespace
import time
import unittest
from unittest.mock import Mock, patch

from Strategy.context import ExecutionContext
from Strategy.execution import ExecutionRuntime
from Strategy.execution.blind import BlindMotionProfile, BlindStatus, run_blind_transition
from Strategy.execution.pickup_motion import PickupMotionProfile
from Strategy.flows.factory import ActionEnvironment
from Strategy.flows.model import ActionSpec
from Strategy.plans import StrategyPlan
from Strategy.transition_config import PickupCalibration, TransitionConfig
from Strategy.transition_switches import TransitionSwitches


def calibration(**changes):
    # All values are synthetic; no enabled field profile is created by this test.
    blind = BlindMotionProfile(1, 50, 100, 100, 100, 2, 5, .1, .1, .02, .01, True)
    acquire = PickupMotionProfile(100, 100, 1000, 5, 5, .1, .1, .02, .01, 1, True)
    params = dict(arm_restore_s=0.0, next_blind=blind, next_acquire=acquire)
    params.update(changes)
    return PickupCalibration(**params)


def action(kind, name, profile='ground-1', **parameters):
    return ActionSpec(kind, name, profile, parameters)


class Mechanism:
    def __init__(self, fixture, token, action_id):
        self.fixture, self.token, self.action_id = fixture, token, action_id
        self.done = self.chassis_ready = False
        self.closed = self.aborted = False

    def check(self):
        self.fixture.context.check_active()

    def wait_chassis_ready(self, press):
        self.fixture.events.append(('wait_lift', self.token))
        press()
        self.check()
        self.chassis_ready = True
        self.fixture.events.append(('full_lift', self.token))

    def wait_done(self):
        self.check()
        self.fixture.events.append(('wait_done', self.token))
        self.done = True

    def close(self):
        self.fixture.events.append(('close_grab', self.token))
        self.closed = True

    def abort(self):
        self.fixture.events.append(('abort_grab', self.token))
        self.aborted = True


class Inspection:
    def __init__(self, fixture, count):
        self.fixture, self.count = fixture, count
        self.closed = self.restored = False

    def inspect(self, *, chassis_followup=None):
        try:
            self.fixture.context.check_active()
            if self.closed:
                raise RuntimeError('inspection is closed')
            if chassis_followup is not None:
                with self.fixture.robot.chassis.monitor_action(self.fixture.context.check_active):
                    chassis_followup()
            self.fixture.context.check_active()
            if self.count is not None and (type(self.count) is not int or not 0 <= self.count <= 3):
                raise RuntimeError(f'invalid carried cube count: {self.count}')
            self.fixture.events.append(('inspect', self.count))
            return self.count
        except BaseException:
            self.abort()
            raise

    def check_restore(self):
        self.fixture.context.check_active()

    def finish_restore(self):
        self.fixture.context.check_active()
        self.restored = True
        self.fixture.events.append(('restore_inspection', self.count))

    def close(self):
        if self.closed:
            return
        if not self.restored:
            raise AssertionError('inspection closed before restoration')
        self.closed = True
        self.fixture.events.append(('close_inspection', self.count))

    def abort(self):
        if self.closed:
            return
        self.closed = True
        self.fixture.events.append(('abort_inspection', self.count))


class Fixture:
    def __init__(self, *, profile='ground-1', method='grap3', policy=None,
                 counts=(3,), color='orange', enabled=True):
        self.events, self.sessions = [], []
        self.counts = iter(counts)
        self.profile, self.method = profile, method
        self.forward = 0.0
        self.pose = 0
        self.robot = SimpleNamespace(
            has_vision=True, telem=SimpleNamespace(yaw_deg=12.0, uptime_ms=1000),
            transport=SimpleNamespace(connected=True, emergency_stop_generation=0,
                                      emergency_stop=Mock()),
            actions=SimpleNamespace(begin=self.begin, pickup_full_lift_validated=enabled),
            reset_vision_filter=Mock(), set_cube_detection_profile=Mock(),
            set_collection_context=Mock(), begin_cube_camera_pose_change=self.begin_pose,
            end_cube_camera_pose_change=self.end_pose,
            begin_carried_cube_inspection=self.inspect,
            diagnostics=SimpleNamespace(write=Mock()),
        )
        self.robot.chassis = SimpleNamespace(
            capture_motor_positions=lambda: (self.forward,) * 4,
            lateral_displacement_mm=lambda origin: 200.0,
            forward_displacement_mm=lambda origin: self.forward - origin[0],
            measured_body_velocity=lambda: SimpleNamespace(vy_mm_s=40.0),
            lateral_distance_scale=1.0,
            mecanum_rpm=lambda x, y, yaw: (x, y, yaw),
            set_speeds=self.speeds, monitor_action=self.monitor,
        )
        self.robot.inspection_link_snapshot = lambda: (
            self.robot.telem, time.monotonic(), time.monotonic(), 0)
        self.robot.vision_result = SimpleNamespace(captured_monotonic=time.monotonic(), pose_epoch=0)
        self.context = ExecutionContext(self.robot, heading_zero_deg=0.0)
        policy = policy or calibration()
        config = TransitionConfig(True, 'synthetic-test', {f'{profile}/{method}': policy},
                                  switches=TransitionSwitches(overrides={
                                      'next-cube': policy.next_blind is not None,
                                      'last-departure': policy.last_departure,
                                      'purple-departure': policy.departure,
                                      'inspect-departure': True,
                                  }))
        self.env = ActionEnvironment(self.robot, self.context, transition_config=config)
        self.control = self.env.control(profile)
        self.control._grab_press_step = Mock(return_value=lambda: self.events.append(('press',)))
        self.control._find_cube = Mock(return_value=SimpleNamespace(x=0, z=150))
        self.control._align_orange = Mock(return_value=True)
        self.control._align_cube = Mock(return_value=True)
        self.control._fine_align_orange = Mock(return_value=True)
        self.control._checked_move = Mock(side_effect=self.move)
        self.control._drive_until_wall = Mock(side_effect=lambda **kw: self.events.append(('wall',)))
        self.control._recalibrate_heading_zero = Mock(side_effect=lambda *a: self.events.append(('rebase',)))
        self.control._turn_to_heading = Mock(side_effect=lambda *a, **kw: self.events.append(('turn',)))
        self.env.data['collection'] = dict(profile=profile, color=color, origin=(0,) * 4,
                                           acquired=True, exhausted=False, pickups=0,
                                           detector_profile='default')
        self.env._enter = Mock(wraps=self.env._enter)
        self.env._complete = Mock(wraps=self.env._complete)
        self.env.record_transition = Mock()
        self.runtime = ExecutionRuntime(guard=self.context.check_active, stop=self.env.stop,
                                        emergency_stop=self.robot.transport.emergency_stop,
                                        close=self.env.abort)

    def begin(self, action_id):
        session = Mechanism(self, len(self.sessions) + 1, action_id)
        self.sessions.append(session)
        self.events.append(('begin_grab', session.token))
        return session

    def begin_pose(self, reason):
        self.pose += 1
        self.events.append(('begin_pose', self.pose))
        return self.pose

    def end_pose(self, pose, *, settle_s):
        self.events.append(('end_pose', pose))
        self.robot.vision_result = SimpleNamespace(captured_monotonic=time.monotonic(), pose_epoch=pose)
        return True

    def inspect(self, **kwargs):
        return Inspection(self, next(self.counts))

    def speeds(self, rpm):
        self.events.append(('speed', tuple(rpm)))
        return True

    def move(self, direction, distance, speed, **kwargs):
        self.context.check_active()
        self.events.append(('move', direction, distance))
        self.forward += distance if direction == 'forward' else -distance if direction == 'backward' else 0

    @contextlib.contextmanager
    def monitor(self, check):
        check()
        yield
        check()

    def blind(self, profile, **kwargs):
        # Real runtime body must already have awaited the complete-lift milestone.
        if not self.sessions[-1].chassis_ready:
            raise AssertionError('blind move started before full lift')
        self.events.append(('blind',))
        first = kwargs['read_sample']()
        if not first.lifted or first.arm_reset:
            raise AssertionError('unexpected full-lift/arm-reset sample')
        kwargs['command_speed'](40.0)
        self.sessions[-1].done = True
        second = kwargs['read_sample']()
        if not second.arm_reset or second.epoch != self.pose:
            raise AssertionError('handoff lacks restored camera epoch')
        if kwargs['handoff'](40.0) is False:
            raise AssertionError('bounded vision failure rejected handoff ownership')
        return SimpleNamespace(status=BlindStatus.HANDED_OFF)

    def run(self, steps, *, warm=True, blind=None):
        self.warm = Mock(return_value=warm)
        with patch('Strategy.flows.transitions.run_blind_transition', side_effect=blind or self.blind), \
             patch('Strategy.flows.transitions.acquire_after_blind', self.warm):
            return self.runtime.run(self.env.compile(StrategyPlan('synthetic', tuple(steps))))


class RegisteredPickupTransitionTests(unittest.TestCase):
    def next_pair(self, fixture, *, follow_grab=False):
        p, m = fixture.profile, fixture.method
        steps = [action('grab_cube', 'grab.1', p, method=m, index=1),
                 action('acquire_cube', 'find.2', p, index=2)]
        if follow_grab:
            steps.append(action('grab_cube', 'grab.2', p, method=m, index=2))
        return steps

    def test_registered_next_cube_replaces_exit_enter_and_skips_duplicate_search(self):
        for profile_name, method in [('ground-1', 'grap3'), ('ground-2', 'grap3'),
                                     ('ground-3', 'grap3'), ('highland-1', 'grap1'),
                                     ('highland-2', 'grap1')]:
            with self.subTest(profile=profile_name):
                f = Fixture(profile=profile_name, method=method)
                steps = self.next_pair(f)
                f.run(steps)
                self.assertLess(f.events.index(('full_lift', 1)), f.events.index(('blind',)))
                self.assertEqual(f.events.count(('close_grab', 1)), 1)
                self.assertEqual(f.events.count(('end_pose', 1)), 1)
                self.assertEqual([call.args[0].name for call in f.env._enter.call_args_list],
                                 ['grab.1', 'find.2'])
                self.assertEqual([call.args[0].name for call in f.env._complete.call_args_list],
                                 ['grab.1', 'find.2'])
                f.control._find_cube.assert_not_called()
                self.assertTrue(f.env.data['collection']['acquired'])
                self.assertEqual(f.env.data['collection']['pickups'], 1)
                self.assertEqual(f.warm.call_args.kwargs['initial_speed_mm_s'], 40.0)
                self.assertEqual(f.warm.call_args.kwargs['phase_origin'], (0,) * 4)
                f.robot.reset_vision_filter.assert_not_called()
                names = [event.phase for event in f.runtime.trace]
                self.assertIn('transition:grab_to_next_cube', names)

    def test_bounded_warm_failure_falls_back_to_ordinary_acquisition_once(self):
        f = Fixture()
        f.run(self.next_pair(f), warm=False)
        f.control._find_cube.assert_called_once()
        f.control._align_orange.assert_called_once()
        self.assertFalse(f.runtime.closed)
        self.assertTrue(f.env.data['collection']['acquired'])
        f.robot.transport.emergency_stop.assert_not_called()

    def test_real_blind_reverse_recovery_finishes_arm_and_acquires_next_cube(self):
        f = Fixture()
        f.robot.diagnostics = Mock()
        f.control._measure_lateral_displacement_mm = lambda origin: -1.5
        f.robot.chassis.measured_body_velocity = lambda: SimpleNamespace(vy_mm_s=-20.)
        f.control._find_cube.side_effect = lambda **kw: (
            f.events.append(('visual_search',)) or SimpleNamespace(x=0, z=150))
        f.run(self.next_pair(f, follow_grab=True), blind=run_blind_transition)
        self.assertEqual(len(f.sessions), 2)
        self.assertTrue(all(s.done and s.closed and not s.aborted for s in f.sessions))
        self.assertLess(f.events.index(('close_grab', 1)), f.events.index(('visual_search',)))
        self.assertLess(f.events.index(('visual_search',)), f.events.index(('begin_grab', 2)))
        f.control._align_orange.assert_called_once()
        f.warm.assert_not_called()
        f.robot.transport.emergency_stop.assert_not_called()
        f.env.record_transition.assert_called_once_with('grab_to_next_cube', 'grab.1', 'direction_recovery')
        feedback = next(call for call in f.robot.diagnostics.write.call_args_list
                        if call.args == ('blind_feedback',))
        self.assertEqual(feedback.kwargs['velocity_mm_s'], -20.)
        self.assertEqual(feedback.kwargs['displacement_mm'], -1.5)
        self.assertEqual(feedback.kwargs['outcome'], 'direction_recovery')

    def test_unsupported_firmware_finishes_grab_before_ordinary_search(self):
        f = Fixture(enabled=False)
        f.run(self.next_pair(f))
        f.warm.assert_not_called()
        f.control._find_cube.assert_called_once()
        self.assertNotIn(('blind',), f.events)
        self.assertTrue(f.sessions[0].done)
        self.assertTrue(f.sessions[0].closed)

    def test_cancellation_during_handoff_never_starts_next_gripper(self):
        f = Fixture()
        def cancel(profile, **kwargs):
            self.assertTrue(f.sessions[0].chassis_ready)
            f.context.cancel_event.set()
            kwargs['guard']()
        with self.assertRaises(RuntimeError):
            f.run(self.next_pair(f, follow_grab=True), blind=cancel)
        self.assertEqual(len(f.sessions), 1)
        self.assertTrue(f.sessions[0].aborted)
        self.assertTrue(f.runtime.closed)
        f.control._find_cube.assert_not_called()
        f.robot.transport.emergency_stop.assert_called_once()

    def test_purple_departure_runs_once_after_lift_before_mechanism_finishes(self):
        f = Fixture(profile='highland-1', method='grap2', color='purple',
                    policy=calibration(next_blind=None, next_acquire=None, departure=True))
        steps = [action('grab_cube', 'purple.grab', f.profile, method='grap2',
                        followup_route='purple_to_orange'),
                 action('navigate', 'purple.leave', f.profile, route='purple_to_orange')]
        def route(env, profile):
            self.assertTrue(f.sessions[0].chassis_ready)
            self.assertFalse(f.sessions[0].done)
            f.events.append(('purple_route',))
        with patch.dict('Strategy.flows.factory.ROUTES', {'purple_to_orange': route}):
            f.run(steps)
        self.assertEqual(f.events.count(('purple_route',)), 1)
        self.assertTrue(f.env.data['purple_grabbed'])
        self.assertEqual(f.env.data['collection']['pickups'], 1)

    def test_last_orange_departs_through_skipped_third_slot(self):
        f = Fixture(profile='highland-1', method='grap1',
                    policy=calibration(last_departure=True))
        f.env.data['purple_grabbed'] = True
        f.env.transition_config = replace(f.env.transition_config, switches=TransitionSwitches(
            overrides={'last-departure': True, 'inspect-departure': False}))
        steps = [action('grab_cube', 'orange.grab.2', f.profile, method='grap1',
                        index=2, conditional_on_purple=True),
                 action('acquire_cube', 'orange.find.3', f.profile, index=3,
                        conditional_on_purple=True),
                 action('grab_cube', 'orange.grab.3', f.profile, method='grap1',
                        index=3, conditional_on_purple=True),
                 action('inspect_cargo', 'orange.inspect', f.profile, method='grap1',
                        exit_route='orange_depart_reverse')]
        f.run(steps)
        self.assertEqual(len(f.sessions), 1)
        f.control._find_cube.assert_not_called()
        self.assertEqual([event[:2] for event in f.events if event[0] == 'move'],
                         [('move', 'backward')])
        self.assertTrue(f.env.data['orange_reverse_done'])
        names = [event.phase for event in f.runtime.trace]
        self.assertIn('transition:last_grab_skips_unused_slot', names)
        self.assertNotIn(('blind',), f.events)

    def test_inspection_priority_returns_partial_cargo_for_refill_then_departs_again(self):
        f = Fixture(policy=calibration(last_departure=True), counts=(2, 3))
        steps = [action('grab_cube', 'last.grab', method='grap3', index=3),
                 action('inspect_cargo', 'inspect', method='grap3',
                        exit_route='ground_delivery_reverse'),
                 action('navigate', 'delivery', route='ground_to_delivery')]
        f.run(steps)
        moves = [event for event in f.events if event[0] == 'move']
        self.assertEqual(moves[:3], [('move', 'backward', 400.0),
                                    ('move', 'forward', 400.0),
                                    ('move', 'backward', 400.0)])
        self.assertLess(f.events.index(('restore_inspection', 2)),
                        f.events.index(('move', 'forward', 400.0)))
        self.assertLess(f.events.index(('wall',)), f.events.index(('begin_grab', 2)))
        self.assertEqual(len(f.sessions), 2)
        self.assertEqual(f.env.data['collection']['origin'], (0,) * 4)
        self.assertIsNone(f.env.transitions.early_departure)
        self.assertTrue(f.env.data['ground_reverse_done'])
        self.assertEqual(f.events.count(('close_inspection', 2)), 1)
        self.assertEqual(f.events.count(('close_inspection', 3)), 1)
        phases = [event.phase for event in f.runtime.trace]
        self.assertNotIn('transition:last_grab_to_inspect', phases)
        self.assertIn('transition:inspect_to_route', phases)

    def test_full_cargo_after_early_departure_does_not_repeat_reverse(self):
        f = Fixture(policy=calibration(last_departure=True))
        f.env.transition_config = replace(f.env.transition_config, switches=TransitionSwitches(
            overrides={'last-departure': True, 'inspect-departure': False}))
        f.run([action('grab_cube', 'last.grab', method='grap3', index=3),
               action('inspect_cargo', 'inspect', method='grap3', exit_route='ground_delivery_reverse'),
               action('navigate', 'delivery', route='ground_to_delivery')])
        reverse = [e for e in f.events if e[:2] == ('move', 'backward')]
        self.assertEqual(reverse, [('move', 'backward', 400.0)])
        self.assertEqual(len(f.sessions), 1)
        self.assertIn('transition:last_grab_to_inspect', [e.phase for e in f.runtime.trace])

    def test_blind_exhaustion_consumes_travel_even_without_visual_handoff(self):
        f = Fixture()
        f.control._search_position_mm = 400.0
        moved = [0.0]
        # A prior left recovery made net displacement smaller than consumed
        # rightward budget. Preserve that distinction throughout the blind leg.
        blind_origin = ('blind',) * 4
        f.control._capture_lateral_origin = lambda: blind_origin
        f.control._measure_lateral_displacement_mm = lambda origin: (
            moved[0] if origin == blind_origin else 300.0 + moved[0])
        def stopped_blind(profile, **kwargs):
            kwargs['read_sample']()
            moved[0] = 60.0
            kwargs['read_sample']()
            kwargs['stop']()
            return SimpleNamespace(status=BlindStatus.BOUND_REACHED,
                                   displacement_mm=60.0, duration_s=0.2)
        seen = []
        f.control._find_cube.side_effect = lambda **kw: (
            seen.append(f.control._search_position_mm) or SimpleNamespace(x=0, z=150))
        f.run(self.next_pair(f), blind=stopped_blind)
        self.assertEqual(seen, [460.0])
        f.warm.assert_not_called()

    def test_invalid_cargo_count_aborts_without_any_refill(self):
        for count in (-1, 4, True):
            with self.subTest(count=count):
                f = Fixture(counts=(count,))
                with self.assertRaisesRegex(RuntimeError, 'invalid carried cube count'):
                    f.run([action('inspect_cargo', 'inspect', method='grap3',
                                  exit_route='ground_delivery_reverse')])
                self.assertEqual(f.sessions, [])
                self.assertTrue(f.runtime.closed)
                f.robot.transport.emergency_stop.assert_called_once()

    def test_blind_profile_is_limited_by_remaining_phase_budget(self):
        f = Fixture()
        f.control._search_position_mm = 980.0
        seen = []
        def bounded(profile, **kwargs):
            seen.append(profile.max_distance_mm)
            kwargs['stop']()
            return SimpleNamespace(status=BlindStatus.BOUND_REACHED,
                                   displacement_mm=0.0, duration_s=0.0)
        f.run(self.next_pair(f), blind=bounded)
        self.assertEqual(seen, [20.0])

    def test_next_cube_expected_100mm_keeps_the_smaller_safety_limit(self):
        for name,method in (('ground-1','grap3'),('ground-2','grap3'),('ground-3','grap3'),
                            ('highland-1','grap1'),('highland-2','grap1')):
            for hard_limit in (120.0,60.0):
                with self.subTest(profile=name,hard_limit=hard_limit):
                    policy=calibration()
                    policy=replace(policy,next_blind=replace(policy.next_blind,max_distance_mm=hard_limit))
                    fixture=Fixture(profile=name,method=method,policy=policy)
                    seen=[]
                    def bounded(config,**kwargs):
                        seen.append(config.max_distance_mm)
                        kwargs['stop']()
                        return SimpleNamespace(status=BlindStatus.BOUND_REACHED)
                    fixture.run(self.next_pair(fixture),blind=bounded)
                    self.assertEqual(seen,[min(hard_limit,100.0+policy.next_blind.braking_margin_mm)])

    def test_invalid_yaw_never_reaches_warm_controller_or_next_grip(self):
        f = Fixture()
        f.robot.telem.yaw_deg = float('nan')
        with self.assertRaises((RuntimeError, ValueError)):
            f.run(self.next_pair(f, follow_grab=True))
        f.warm.assert_not_called()
        self.assertEqual(len(f.sessions), 1)
        self.assertTrue(f.sessions[0].aborted)


if __name__ == '__main__':
    unittest.main()
