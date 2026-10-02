"""Local route replacement uses real recipes and the actual wheel-loop follower."""
from dataclasses import replace
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from control.trajectory import BodyVelocity, Waypoint, CubicRoute
from Strategy.flows.curves import CURVE_ROUTES
from Strategy.flows.factory import ROUTE_PROFILES
from Strategy.flows.routes import ROUTES
from Strategy.optimizations.local_routes import RouteRecorder, LocalRoutes
from tests.test_curve_routes import fixture
from simulation.catalog import RouteHarness, _assumptions
from simulation.core import MecanumPlant


def environment(route, profile, *, heading=None, lateral=None, reverse=None):
    values = _assumptions(route, profile)
    if heading is not None: values['heading_cw_deg'] = heading
    if lateral is not None: values['search_lateral_mm'] = lateral
    if reverse is not None: values['reverse_already_done'] = reverse
    env,c,events,_ = fixture(route,profile,heading_cw=values['heading_cw_deg'],lateral=values['search_lateral_mm'])
    env.transition_config=replace(env.transition_config,curves={},navigation=None)
    from Strategy.optimizations.motion_planning import MotionPlanning
    env.motion_planning=MotionPlanning(optimizers=[LocalRoutes()])
    env.robot.chassis.measured_body_velocity=Mock(return_value=BodyVelocity())
    env.robot.field_pose=None  # No camera/world pose is required.
    if values['reverse_already_done']:
        env.data.update(ground_reverse_done=True,orange_reverse_done=True,
                        ground_lateral_mm=values['search_lateral_mm'],orange_lateral_mm=values['search_lateral_mm'])
    return env,c,events,values


class LocalRouteTests(unittest.TestCase):
    def test_all_registered_profiles_record_exact_original_geometry_without_motion(self):
        for route in sorted(CURVE_ROUTES):
            for profile in sorted(ROUTE_PROFILES[route]):
                with self.subTest(route=route,profile=profile):
                    env,c,_,values=environment(route,profile)
                    recorder=RouteRecorder(env,profile)
                    ROUTES[route](recorder,profile)
                    baseline=RouteHarness({'profile':profile,'start_assumptions':values})
                    ROUTES[route](baseline,profile)
                    self.assertAlmostEqual(recorder.pose.x_mm,baseline.pose.x_mm)
                    self.assertAlmostEqual(recorder.pose.y_mm,baseline.pose.y_mm)
                    self.assertAlmostEqual(recorder.pose.yaw_deg,baseline.pose.yaw_deg)
                    env.robot.chassis.follow_trajectory.assert_not_called()
                    c._checked_move.assert_not_called()
                    c._drive_until_wall.assert_not_called()

    def test_every_registered_route_selects_local_follower_without_json_or_tag(self):
        for route in sorted(CURVE_ROUTES):
            profile=sorted(ROUTE_PROFILES[route])[0]
            env,_,_,_=environment(route,profile)
            with self.subTest(route=route), patch('Strategy.flows.factory.ROUTES',{route:Mock()}) as classic:
                env.run_route(route,profile)
                classic[route].assert_not_called()
                self.assertGreater(env.robot.chassis.follow_trajectory.call_count,0)
                for call in env.robot.chassis.follow_trajectory.call_args_list:
                    points,settings=call.args
                    self.assertEqual(points[0],Waypoint(0,0,0))
                    self.assertTrue(settings.route_derived)
                    self.assertFalse(settings.validated)

    def test_clearance_retreat_finishes_before_turning_and_wall_tail_is_preserved(self):
        env,c,events,_=environment('build_return','building-1')
        seen=[]
        def follow(points,settings,**kwargs):
            seen.append(points)
            events.append(('planned',))
        env.robot.chassis.follow_trajectory.side_effect=follow
        env.run_route('build_return','building-1')
        self.assertEqual(len(seen),2)
        self.assertEqual(seen[0][-1],Waypoint(-c.config.post_build_reverse_mm,0,0))
        self.assertEqual(events[-1],('wall','left'))
        self.assertAlmostEqual(seen[1][-1].yaw_deg,c.config.post_build_turn_cw_deg)

    def test_dynamic_measurement_and_completed_reverse_are_not_repeated(self):
        env,c,_,_=environment('ground_to_delivery','ground-1',heading=30,lateral=340,reverse=True)
        env.run_route('ground_to_delivery','ground-1')
        calls=env.robot.chassis.follow_trajectory.call_args_list
        self.assertEqual(len(calls),1)
        end=calls[0].args[0][-1]
        self.assertAlmostEqual(end.x_mm,1230)
        self.assertAlmostEqual(end.y_mm,2460*math.sqrt(3)/2)
        self.assertAlmostEqual(end.yaw_deg,150)
        c._measure_lateral_displacement_mm.assert_not_called()

    def test_failure_does_not_commit_or_run_tail_or_replay_original(self):
        env,c,_,_=environment('purple_to_orange','highland-1')
        env.robot.chassis.follow_trajectory.side_effect=RuntimeError('lost telemetry')
        with patch('Strategy.flows.factory.ROUTES',{'purple_to_orange':Mock()}) as classic:
            with self.assertRaisesRegex(RuntimeError,'lost telemetry'):
                env.run_route('purple_to_orange','highland-1')
            classic['purple_to_orange'].assert_not_called()
        self.assertFalse(env.data.get('purple_route_done',False))
        c._drive_until_wall.assert_not_called()

    def test_departure_and_multileg_route_reach_endpoints_in_four_wheel_model(self):
        for route,profile in ((route,profile) for route in sorted(CURVE_ROUTES)
                              for profile in sorted(ROUTE_PROFILES[route])):
            env,_,_,values=environment(route,profile)
            baseline=RouteHarness({'profile':profile,'start_assumptions':values})
            ROUTES[route](baseline,profile)
            # Local compilation now also follows post-wall move spans. The
            # fake wall does not move the plant, so compare the full recipe end.
            goal=dict(x_mm=baseline.pose.x_mm,y_mm=baseline.pose.y_mm,yaw_deg=baseline.pose.yaw_deg)
            plant=MecanumPlant()
            env.robot.chassis.measured_body_velocity=plant.measured_velocity
            env.robot.chassis.follow_trajectory=lambda points,settings,**kwargs: plant.follow_local(
                points,profile=settings,initial_velocity=kwargs['initial_velocity'])
            with self.subTest(route=route):
                env.run_route(route,profile)
                self.assertLess(math.hypot(plant.x-goal['x_mm'],plant.y-goal['y_mm']),12)
                self.assertLess(abs(plant.yaw-goal['yaw_deg']),2)
                self.assertEqual(plant.targets,[0,0,0,0])
                self.assertEqual(plant.emergency_stops,0)


if __name__=='__main__': unittest.main()
