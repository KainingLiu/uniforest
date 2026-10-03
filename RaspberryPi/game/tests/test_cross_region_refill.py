"""Real recovery/route orchestration with non-moving hardware endpoints."""

import contextlib
import io
from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from Strategy.context import ExecutionContext
from Strategy.errors import SearchRangeExhausted
from Strategy.execution import ExecutionRuntime
from Strategy.flows.factory import ActionEnvironment, validate_spec
from Strategy.flows.model import ActionSpec
from Strategy.flows import operations, refill, refill_routes
from Strategy.plans import PLANS, StrategyPlan
from Strategy.settings import PROFILES
from Strategy.refill_policy import BothOrangeAreasExhausted
from Strategy.transition_config import TransitionConfig
from control.trajectory import BodyVelocity
from tests.test_functional_operations import operation_fixture


class Recovery:
    def __init__(self, profile='ground-1', *, counts=(2, 2, 3, 3), pickups=2,
                 purple=False, alternate_empty=False, planning=False):
        base, _ = operation_fixture(profile)
        self.robot = base.robot
        self.robot.chassis.capture_motor_positions = Mock(side_effect=object)
        self.events = []
        self.robot.actions.pickup_full_lift_validated = False
        self.robot.diagnostics = SimpleNamespace(write=Mock())
        self.robot.check_carried_cube_count = Mock(side_effect=counts)
        self.context = ExecutionContext(self.robot, heading_zero_deg=0,
            anchor='ground_area' if profile.startswith('ground') else 'upper_orange_area')
        self.env = ActionEnvironment(self.robot, self.context,
            transition_config=TransitionConfig(motion_planning_enabled=planning))
        self.env.motion_planning.run = Mock(side_effect=AssertionError('nested optimized route'))
        self.env.motion_planning.run_refill = Mock(wraps=self.env.motion_planning.run_refill)
        self.robot.chassis.measured_body_velocity = Mock(return_value=BodyVelocity())
        self.robot.chassis.follow_trajectory = Mock()
        for key in PROFILES:
            c = self.env.control(key)
            c._drive_until_wall = Mock()
            c._align_delivery_tag = Mock(return_value=True)
            c._align_delivery_tag_or_continue = Mock(return_value=True)
            c._turn_to_heading = Mock()
            c._checked_move = Mock(side_effect=lambda *a, **kw: self.context.check_active())
            c._grab_press_step = Mock(return_value=lambda: True)
            c._grab_with_wall_press = Mock(side_effect=lambda grab, **kw: grab())
            c._align_orange = Mock(return_value=True)
            c._align_cube = Mock(return_value=True)
            c._fine_align_orange = Mock(return_value=True)
            c._find_cube = Mock(side_effect=(SearchRangeExhausted('alternate empty')
                                           if alternate_empty else None),
                                return_value=SimpleNamespace(x=0, z=150))
        self.spec = ActionSpec('inspect_cargo', 'test.inspect', profile, {
            'method': 'grap3' if profile.startswith('ground') else 'grap1',
            'exit_route': 'ground_delivery_reverse' if profile.startswith('ground') else 'orange_depart_reverse',
            'cross_region_refill': True})
        operations.begin_collection(self.env, replace(self.spec, kind='begin_collection',
                                     parameters={'color': 'orange'}))
        self.env.data['collection'].update(exhausted=True, pickups=pickups)
        if profile.startswith('highland'):
            self.env.data['purple_grabbed'] = purple
        self.env.control(profile)._search_position_mm = 1800.
        self.env.data['carried_count'] = 3  # Stale data must never mask a shortage.

    def run(self, entry='phased'):
        if entry == 'phased':
            return self.env.transitions.inspect(self.spec)
        return operations.inspect_cargo(self.env, self.spec)


class CrossRegionRefillTests(unittest.TestCase):
    def setUp(self):
        output = contextlib.redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)

    def test_two_cubes_switch_once_grab_only_one_and_return_with_fresh_origin(self):
        for profile in ('ground-1', 'ground-2', 'ground-3', 'highland-1', 'highland-2'):
            for entry in ('phased', 'blocking'):
                for planning in (False, True):
                    with self.subTest(profile=profile, entry=entry, planning=planning):
                        r = Recovery(profile, planning=planning)
                        original_origin = r.env.data['collection']['origin']
                        original_anchor = r.context.anchor
                        with patch.object(refill_routes, 'transfer', wraps=refill_routes.transfer) as travel:
                            self.assertEqual(r.run(entry), 3)
                            self.assertEqual(travel.call_count, 2)
                            first, second = [call.args[1:] for call in travel.call_args_list]
                            self.assertEqual(first, tuple(reversed(second)))
                        method = 'grap1' if profile.startswith('ground') else 'grap3'
                        getattr(r.robot.actions, method).assert_called_once()
                        r.robot.actions.grap2.assert_not_called()
                        r.robot.actions.hatch_open.assert_not_called()
                        r.robot.actions.hatch_close.assert_not_called()
                        self.assertEqual(r.env.data['carried_count'], 3)
                        self.assertEqual(r.context.anchor, original_anchor)
                        self.assertEqual(r.env.data['collection']['profile'], profile)
                        self.assertIsNot(r.env.data['collection']['origin'], original_origin)
                        self.assertTrue(r.env.data['collection']['exhausted'])
                        region = 'ground' if profile.startswith('ground') else 'orange'
                        self.assertFalse(r.env.data[f'{region}_reverse_done'])
                        self.assertIsNone(r.env.data[f'{region}_lateral_mm'])
                        self.assertEqual(r.robot.check_carried_cube_count.call_count, 4)
                        r.env.motion_planning.run.assert_not_called()
                        self.assertEqual(r.env.motion_planning.run_refill.call_count, 2 if planning else 0)

    def test_existing_purple_is_preserved_and_only_orange_is_added(self):
        r = Recovery('highland-2', pickups=1, purple=True)
        self.assertEqual(r.run(), 3)
        self.assertTrue(r.env.data['purple_grabbed'])
        r.robot.actions.grap3.assert_called_once()
        r.robot.actions.grap2.assert_not_called()

    def test_compiled_inspection_resumes_original_transport_with_rebased_search_offset(self):
        for profile, route in (('ground-1', 'ground_to_delivery'),
                               ('highland-2', 'orange_to_build')):
            with self.subTest(profile=profile):
                r = Recovery(profile)
                # Actual binding must choose the phased inspection entry.
                r.robot.begin_carried_cube_inspection = Mock(
                    side_effect=AssertionError('exhausted recovery uses a serial fresh inspection'))
                transport = ActionSpec('navigate', 'resume.transport', profile,
                                       {'route': route}, ends_at='resume_destination')
                runtime = ExecutionRuntime(guard=r.context.check_active, stop=r.env.stop,
                    emergency_stop=r.robot.transport.emergency_stop, close=r.env.abort)
                runtime.run(r.env.compile(StrategyPlan('recover-and-resume', (r.spec, transport))))
                self.assertEqual(r.context.anchor, 'resume_destination')
                region = 'ground' if profile.startswith('ground') else 'orange'
                self.assertEqual(r.env.data[f'{region}_lateral_mm'], 0.)
                self.assertTrue(r.env.data[f'{region}_reverse_done'])
                self.assertEqual(r.env.data['carried_count'], 3)
                r.robot.transport.emergency_stop.assert_not_called()

    def test_exhaustion_during_same_area_refill_enters_alternate_recovery(self):
        r = Recovery(pickups=1, counts=(1, 1, 1, 2, 3, 3))
        r.env.data['collection']['exhausted'] = False
        source = r.env.control('ground-1')
        source._find_cube.side_effect = SearchRangeExhausted('source exhausted during refill')
        self.assertEqual(r.run('blocking'), 3)
        self.assertEqual(r.robot.actions.grap1.call_count, 2)
        r.robot.actions.grap3.assert_not_called()

    def test_zero_or_one_cube_has_a_finite_remaining_attempt_budget(self):
        for initial, counts in ((0, (0, 0, 1, 2, 3, 3)), (1, (1, 1, 2, 3, 3))):
            r = Recovery(pickups=initial, counts=counts)
            self.assertEqual(r.run(), 3)
            self.assertEqual(r.robot.actions.grap1.call_count, 3-initial)

    def test_full_cargo_never_transfers(self):
        r = Recovery(pickups=3, counts=(3,))
        with patch.object(refill_routes, 'transfer') as travel:
            self.assertEqual(r.run(), 3)
            travel.assert_not_called()
        r.robot.actions.grap1.assert_not_called()

    def test_both_areas_exhausted_stops_in_pending_without_partial_return(self):
        r = Recovery(counts=(2, 2, 2), alternate_empty=True)
        with patch.object(refill_routes, 'transfer', wraps=refill_routes.transfer) as travel:
            with self.assertRaises(BothOrangeAreasExhausted) as pending:
                r.run()
            self.assertEqual(pending.exception.count, 2)
            self.assertEqual(travel.call_count, 1)
        r.robot.actions.grap1.assert_not_called()
        self.assertEqual(r.robot.check_carried_cube_count.call_count, 3)
        r.context.cancel_event.set()
        with self.assertRaises(RuntimeError):
            r.run()

    def test_unknown_or_invalid_count_prevents_departure(self):
        for count in (None, True, 2.0, -1, 4):
            with self.subTest(count=count):
                r = Recovery(counts=(count,))
                with patch.object(refill_routes, 'transfer') as travel:
                    with self.assertRaises(RuntimeError):
                        r.run()
                    travel.assert_not_called()
                r.robot.actions.grap1.assert_not_called()

    def test_pending_stops_compiled_plan_before_transport(self):
        r = Recovery(counts=(2, 2, 2), alternate_empty=True)
        later = Mock()
        from Strategy.execution import Action, ActionFlow
        runtime = ExecutionRuntime(guard=r.context.check_active, stop=r.env.stop,
            emergency_stop=r.robot.transport.emergency_stop, close=r.env.abort)
        flow = ActionFlow('pending', (Action('inspect_cargo', 'inspect', body=r.run),
                                     Action('navigate', 'later', body=later)))
        with self.assertRaises(BothOrangeAreasExhausted):
            runtime.run(flow)
        later.assert_not_called()
        r.robot.actions.hatch_open.assert_not_called()
        r.robot.transport.emergency_stop.assert_called_once()

    def test_inspected_count_drives_refill_even_when_attempts_differ(self):
        for counts, grabs in (((2, 1, 2, 3, 3), 2), ((1, 1, 1, 2, 3, 3), 3)):
            with self.subTest(counts=counts):
                r = Recovery(pickups=counts[0], counts=counts)
                self.assertEqual(r.run(), 3)
                self.assertEqual(r.robot.actions.grap1.call_count, grabs)

    def test_return_count_change_does_not_resume_original_task(self):
        r = Recovery(counts=(2, 2, 3, 2))
        with self.assertRaisesRegex(RuntimeError, 'cargo changed during return'):
            r.run()

    def test_early_departure_closes_camera_and_returns_before_cross_region_count(self):
        r = Recovery()
        events = []
        r.env.transitions.inspections[r.spec.name] = SimpleNamespace(
            finish_restore=lambda: events.append('restore'), close=lambda: events.append('close'))
        r.env.transitions.early_departure = ('ground-1', 382.5, 'ground_delivery_reverse')
        control = r.env.control('ground-1')
        control._checked_move = Mock(side_effect=lambda *a, **kw: events.append(('move', *a)))
        original_count = r.robot.check_carried_cube_count
        r.robot.check_carried_cube_count = lambda **kw: (events.append('count'), original_count(**kw))[1]
        self.assertEqual(r.run(), 3)
        self.assertEqual(events[:4], ['restore', 'close', ('move', 'forward', 382.5, 400.), 'count'])
        self.assertFalse(r.env.transitions.inspections)

    def test_transfer_fault_aborts_runtime_and_prevents_later_action(self):
        for fault in ('cancel', 'stop', 'disconnect', 'exception'):
            with self.subTest(fault=fault):
                r = Recovery()
                def fail(*args, **kwargs):
                    if fault == 'cancel': r.context.cancel_event.set()
                    elif fault == 'stop': r.robot.transport.emergency_stop_generation += 1
                    elif fault == 'disconnect': r.robot.transport.connected = False
                    else: raise RuntimeError('transfer failed')
                    r.context.check_active()
                later = Mock()
                from Strategy.execution import Action, ActionFlow
                runtime = ExecutionRuntime(guard=r.context.check_active, stop=r.env.stop,
                    emergency_stop=r.robot.transport.emergency_stop, close=r.env.abort)
                flow = ActionFlow('recovery', (Action('inspect_cargo', 'inspect', body=r.run),
                                              Action('unload', 'later', body=later)))
                with patch.object(refill_routes, 'transfer', side_effect=fail):
                    with self.assertRaises(RuntimeError): runtime.run(flow)
                later.assert_not_called()
                r.robot.actions.grap1.assert_not_called()
                r.robot.transport.emergency_stop.assert_called_once()

    def test_real_route_leg_order_preserves_loaded_exit_geometry(self):
        expected = {
            'ground': ['ground_to_delivery', 'ground_tag_offset', 'ground_delivery_depart',
                       'to_purple', 'purple_to_orange'],
            'highland': ['orange_to_build', 'unload_approach', 'unload_depart', 'return_orange'],
        }
        for source, target in (('ground', 'highland'), ('highland', 'ground')):
            r = Recovery('ground-1' if source == 'ground' else 'highland-1')
            with patch.object(refill_routes, '_route', wraps=refill_routes._route) as route:
                refill_routes.transfer(r.env, source, target, ground_profile='ground-1',
                                       highland_profile='highland-1')
                self.assertEqual([c.args[1] for c in route.call_args_list], expected[source])
            transit = r.env.control('ground-1' if source == 'ground' else 'unload-1')
            self.assertTrue(any(c.args[:2] == ('backward', 300.)
                                for c in transit._checked_move.call_args_list))
            if source == 'ground':
                r.env.control('ground-1')._align_delivery_tag.assert_called_once()
                ramp = r.env.control('highland-1')._checked_move.call_args_list[0]
                self.assertTrue(ramp.kwargs['ramp_straight'])
            r.robot.actions.hatch_open.assert_not_called()

    def test_failed_anchor_or_tag_never_proceeds_to_pickup_area(self):
        for method in ('_drive_until_wall', '_align_delivery_tag'):
            r = Recovery()
            getattr(r.env.control('ground-1'), method).side_effect = RuntimeError('anchor lost')
            with self.assertRaisesRegex(RuntimeError, 'anchor lost'): r.run()
            r.robot.actions.grap1.assert_not_called()

    def test_plans_enable_refill_but_invalid_profile_or_method_is_rejected(self):
        for name in ('PlanA', 'PlanB'):
            for step in PLANS[name].steps:
                if step.kind == 'inspect_cargo':
                    self.assertTrue(step.parameters['cross_region_refill'])
                    validate_spec(step)
        r = Recovery()
        for changes in ({'method': 'grap2'}, {'exit_route': 'orange_depart_reverse'},
                        {'cross_region_refill': 1}):
            invalid = replace(r.spec, parameters={**r.spec.parameters, **changes})
            with self.assertRaises(ValueError): validate_spec(invalid)


if __name__ == '__main__':
    unittest.main()
