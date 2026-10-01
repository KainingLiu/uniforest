"""Continuous route geometry/guard contracts, without moving hardware."""
from dataclasses import replace
import math
import unittest
from unittest.mock import Mock, patch

from control.trajectory import BodyVelocity, Waypoint
from Strategy.flows import ActionSpec
from Strategy.flows.curves import CURVE_ROUTES
from Strategy.flows.factory import ActionEnvironment, ROUTE_PROFILES
from Strategy.transition_config import CurveCalibration, TransitionConfig
from tests.test_continuous_trajectory import profile as simulation_profile
from tests.test_functional_operations import operation_fixture


def point(**values):
    return dict(x_mm=0, y_mm=0, yaw_deg=0, dx_scale=0, dy_scale=0, dyaw_scale=0) | values


def calibration():
    # Simulation-only fixture, with an explicit off-chord intermediate point.
    return CurveCalibration((point(), point(y_mm=80, yaw_deg=5, dx_scale=.4,
                                            dy_scale=.6, dyaw_scale=.3),
                             point(dx_scale=1, dy_scale=1, dyaw_scale=1)),
                            simulation_profile())


def fixture(route, profile, *, heading_cw=0, lateral=0, config_overrides=None):
    original, control = operation_fixture(profile)
    original.robot.telem.yaw_deg = -heading_cw
    if config_overrides:
        control.config = replace(control.config, **config_overrides)
    robot, context = original.robot, original.context
    events = []
    robot.chassis.measured_body_velocity = Mock(return_value=BodyVelocity(0, 35, 0))
    result = object()
    def curve(points, settings, *, check, initial_velocity):
        check()
        events.append(('curve', points))
        return result
    robot.chassis.follow_trajectory = Mock(side_effect=curve)
    control._measure_lateral_displacement_mm = Mock(return_value=lateral)
    control._drive_until_wall = Mock(side_effect=lambda **kw: events.append(('wall', kw['direction'])))
    control._checked_move = Mock(side_effect=lambda *a, **kw: events.append(('move', *a)))
    control._recalibrate_heading_zero = Mock(side_effect=lambda: events.append(('rebase',)))
    env = ActionEnvironment(robot, context, transition_config=TransitionConfig(
        curves={f'{profile}/{route}': calibration()}))
    env._controllers[profile] = control
    env.record_transition = Mock()
    env.data.update(ground_origin=object(), purple_origin=object(), orange_origin=object())
    return env, control, events, result


class CurveRouteTests(unittest.TestCase):
    def endpoint(self, env):
        return env.robot.chassis.follow_trajectory.call_args.args[0][-1]

    def assertPoint(self, actual, x, y, yaw):
        self.assertAlmostEqual(actual.x_mm, x, delta=1e-6)
        self.assertAlmostEqual(actual.y_mm, y, delta=1e-6)
        self.assertAlmostEqual(actual.yaw_deg, yaw, delta=1e-6)

    def test_ground_delivery_uses_live_displacement_and_current_heading(self):
        env, c, events, result = fixture('ground_to_delivery', 'ground-1',
                                         heading_cw=30, lateral=340)
        self.assertIs(env.run_route('ground_to_delivery', 'ground-1'), result)
        self.assertPoint(self.endpoint(env), 830, 2460*math.sqrt(3)/2, 150)
        self.assertEqual(env.data['ground_lateral_mm'], 340)
        self.assertTrue(env.data['ground_reverse_done'])
        c._measure_lateral_displacement_mm.assert_called_once_with(env.data['ground_origin'])
        args = env.robot.chassis.follow_trajectory.call_args
        self.assertEqual(args.kwargs['initial_velocity'], BodyVelocity(0, 35, 0))
        self.assertEqual(args.kwargs['check'], env.context.check_active)
        middle = args.args[0][1]
        self.assertPoint(middle, .4*830, 80+.6*2460*math.sqrt(3)/2, 5+.3*150)
        self.assertEqual([item[0] for item in events], ['curve'])

    def test_ground_preexecuted_reverse_and_frozen_measurement_are_preserved(self):
        env, c, _, _ = fixture('ground_to_delivery', 'ground-1',
                                heading_cw=30, lateral=999)
        env.data.update(ground_reverse_done=True, ground_lateral_mm=340)
        env.run_route('ground_to_delivery', 'ground-1')
        self.assertPoint(self.endpoint(env), 1230, 2460*math.sqrt(3)/2, 150)
        c._measure_lateral_displacement_mm.assert_not_called()

    def test_purple_route_keeps_wall_barriers_and_does_not_repeat_callback_route(self):
        env, c, events, _ = fixture('purple_to_orange', 'highland-1',
                                     heading_cw=270, lateral=-120,
                                     config_overrides={'left_wall_approach_enabled': True})
        env.run_route('purple_to_orange', 'highland-1')
        self.assertPoint(self.endpoint(env), -100, 470, 90)
        self.assertEqual(events[1:], [('wall', 'left'), ('wall', 'forward'), ('rebase',)])
        self.assertTrue(env.data['purple_route_done'])
        env.run_route('purple_to_orange', 'highland-1')
        self.assertEqual(env.robot.chassis.follow_trajectory.call_count, 1)
        self.assertEqual(c._drive_until_wall.call_count, 2)

    def test_optional_purple_left_wall_and_signed_orange_displacement(self):
        env, _, events, _ = fixture('purple_to_orange', 'highland-1',
                                      config_overrides={'left_wall_approach_enabled': False})
        env.run_route('purple_to_orange', 'highland-1')
        self.assertEqual(events[1:], [('wall', 'forward'), ('rebase',)])
        env, c, _, _ = fixture('orange_to_build', 'highland-1', lateral=700)
        env.run_route('orange_to_build', 'highland-1')
        self.assertPoint(self.endpoint(env), -2850, -150, 180)
        self.assertTrue(env.data['orange_reverse_done'])
        env.robot.chassis.follow_trajectory.reset_mock()
        c._measure_lateral_displacement_mm.reset_mock()
        env.run_route('orange_to_build', 'highland-1')
        self.assertPoint(self.endpoint(env), -2750, -150, 180)
        c._measure_lateral_displacement_mm.assert_not_called()

    def test_build_return_preserves_explicit_cw_half_turn_or_larger_turn(self):
        for turn, expected in ((180, (-100, 2500, 180)), (270, (-2600, 0, 270))):
            with self.subTest(turn=turn):
                env, _, events, _ = fixture('build_return', 'building-1', heading_cw=17,
                                             config_overrides={'post_build_turn_cw_deg': turn})
                env.run_route('build_return', 'building-1')
                self.assertPoint(self.endpoint(env), *expected)
                self.assertEqual(events[1:], [('wall', 'left')])

    def test_staged_building_retains_wall_before_second_load_approach(self):
        for route, expected, tail in (
                ('staged_initial', (0, -700, 0), [('wall', 'left')]),
                ('staged_to_build', (-250, 940, 0), []),
                ('staged_return_first', (-100, -840, 0),
                 [('wall', 'left'), ('move', 'right', 300, 400)]),
                ('staged_return_final', (-100, -440, 0), [])):
            with self.subTest(route=route):
                env, _, events, _ = fixture(route, 'staged-building')
                env.run_route(route, 'staged-building')
                self.assertPoint(self.endpoint(env), *expected)
                self.assertEqual(events[1:], tail)

    def test_departures_and_single_axis_routes_preserve_destinations(self):
        cases = (
            ('depart_a', 'depart-a', (1200, 0, 0)),
            ('depart_b', 'depart-b', (900, 2700, 180)),
            ('return_orange', 'return-orange', (0, 2600, 180)),
            ('ground_tag_offset', 'ground-1', (0, 100, 0)),
            ('ground_delivery_depart', 'ground-1', (0, -100, 0)),
            ('build_offset', 'building-1', (0, 100, 0)),
            ('build_offset', 'building-3', (0, -500, 0)),
            ('unload_depart', 'unload-1', (0, 800, 0)),
        )
        for route, profile, expected in cases:
            with self.subTest(route=route, profile=profile):
                heading = 180 if route == 'return_orange' else 0
                env, _, events, _ = fixture(route, profile, heading_cw=heading)
                env.run_route(route, profile)
                self.assertPoint(self.endpoint(env), *expected)
                self.assertEqual(events[1:], [('wall', 'left')] if route == 'return_orange' else [])

    def test_curve_and_wall_failures_never_commit_route_completion_or_anchor(self):
        for fault in ('curve', 'wall'):
            with self.subTest(fault=fault):
                env, c, _, _ = fixture('purple_to_orange', 'highland-1')
                target = (env.robot.chassis.follow_trajectory if fault == 'curve'
                          else c._drive_until_wall)
                target.side_effect = RuntimeError(f'{fault} fault')
                action = env.bind(ActionSpec('navigate', 'test.navigate', 'highland-1',
                                            {'route': 'purple_to_orange'}, ends_at='orange'))
                with self.assertRaisesRegex(RuntimeError, fault):
                    action.enter()
                self.assertFalse(env.data.get('purple_route_done', False))
                self.assertEqual(env.context.anchor, 'start')
                env.record_transition.assert_not_called()

    def test_late_cancellation_cannot_record_success_or_commit_reverse_done(self):
        env, _, _, _ = fixture('ground_to_delivery', 'ground-1')
        def finished_then_cancelled(*args, **kwargs):
            env.context.cancel_event.set()
        env.robot.chassis.follow_trajectory.side_effect = finished_then_cancelled
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            env.run_route('ground_to_delivery', 'ground-1')
        self.assertFalse(env.data.get('ground_reverse_done', False))
        env.record_transition.assert_not_called()

    def test_invalid_endpoint_or_excessive_search_distance_never_start_motion(self):
        env, _, _, _ = fixture('ground_to_delivery', 'ground-1', lateral=3000)
        with self.assertRaisesRegex(RuntimeError, 'displacement'):
            env.run_route('ground_to_delivery', 'ground-1')
        env.robot.chassis.follow_trajectory.assert_not_called()
        env, _, _, _ = fixture('depart_a', 'depart-a')
        broken = replace(calibration(), points=(point(), point(dx_scale=.8)))
        env.transition_config = TransitionConfig(curves={'depart-a/depart_a': broken})
        with self.assertRaisesRegex(ValueError, 'endpoint'):
            env.run_route('depart_a', 'depart-a')
        env.robot.chassis.follow_trajectory.assert_not_called()

    def test_only_registered_calibrated_routes_replace_legacy_routes(self):
        env, _, _, _ = fixture('depart_a', 'depart-a')
        env.transition_config = TransitionConfig()
        with patch('Strategy.flows.factory.ROUTES', {'depart_a': Mock()}) as routes:
            env.run_route('depart_a', 'depart-a')
            routes['depart_a'].assert_called_once_with(env, 'depart-a')
        env.robot.chassis.follow_trajectory.assert_not_called()
        self.assertNotIn('to_purple', CURVE_ROUTES)  # Tag alignment stays a barrier.
        self.assertNotIn('unload_approach', CURVE_ROUTES)  # Two wall contacts stay explicit.
        for key in ('ground-1/to_purple', 'ground-1/depart_a'):
            with self.assertRaisesRegex(ValueError, 'calibration contract'):
                ActionEnvironment(env.robot, env.context,
                                  transition_config=TransitionConfig(curves={key: calibration()}))
        self.assertTrue(CURVE_ROUTES <= ROUTE_PROFILES.keys())


if __name__ == '__main__':
    unittest.main()
