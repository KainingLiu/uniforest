"""Offline local-controller comparisons, excluded from competition dispatch."""
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
from control.chassis import Chassis, COUNTS_PER_CM, LATERAL_DISTANCE_SCALE
from protocol.commands import TelemBatch, MotorFeedback


def classic_move(plant, direction, distance, speed, **kwargs):
    """Replay the actual original straight controller on the same wheel plant."""
    link=SimpleNamespace(connected=True,emergency_stop_generation=0)
    link.set_chassis_speed=lambda rpms:setattr(plant,'targets',list(rpms)) or True
    link.emergency_stop=plant.emergency_stop
    chassis=Chassis(link)
    def feedback():
        chassis.update_telem(TelemBatch(motors=[MotorFeedback(cumulative_pos=round(c),speed_rpm=round(r))
            for c,r in zip(plant.counts,plant.rpms)],yaw_deg=-plant.yaw,uptime_ms=round(plant.now*1000)))
        return chassis.telem
    def advance(dt):
        plant.sleep(dt)
        feedback()
    signs={'forward':(-1,1,1,-1),'backward':(1,-1,-1,1),'right':(1,1,-1,-1),'left':(-1,-1,1,1)}[direction]
    scale=LATERAL_DISTANCE_SCALE if direction in ('left','right') else 1.
    with patch('control.chassis.time.monotonic',side_effect=lambda:plant.now):
        feedback()
        return chassis._move_linear(int(distance*scale*COUNTS_PER_CM/10),list(signs),
            chassis._mm_s_to_rpm(speed*scale),link,feedback,distance,sleep_fn=advance,
            hold_ms=kwargs.get('hold_ms',0),accel_ms=kwargs.get('accel_ms') or 300,
            route_mode=True,distance_scale=scale)


def environment(route, profile, *, heading=None, lateral=None, reverse=None):
    values = _assumptions(route, profile)
    if heading is not None: values['heading_cw_deg'] = heading
    if lateral is not None: values['search_lateral_mm'] = lateral
    if reverse is not None: values['reverse_already_done'] = reverse
    env,c,events,_ = fixture(route,profile,heading_cw=values['heading_cw_deg'],lateral=values['search_lateral_mm'])
    env.transition_config=replace(env.transition_config,curves={},navigation=None)
    from Strategy.optimizations.motion_planning import MotionPlanning
    env.motion_planning=MotionPlanning()
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

    def test_routes_keep_straights_and_smooth_compound_turns_without_map_or_tag(self):
        for route in sorted(CURVE_ROUTES):
            profile=sorted(ROUTE_PROFILES[route])[0]
            env,_,_,_=environment(route,profile)
            with self.subTest(route=route):
                env.run_route(route,profile)
                self.assertGreater(env.robot.chassis.follow_trajectory.call_count+
                    env.control(profile)._checked_move.call_count+env.robot.move_chassis.call_count,0)
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
        self.assertEqual(len(seen),1)
        self.assertEqual(events[0][:3],('move','backward',c.config.post_build_reverse_mm))
        self.assertEqual(events[-1],('wall','left'))
        self.assertAlmostEqual(seen[0][-1].yaw_deg,c.config.post_build_turn_cw_deg)

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
            env,c,_,values=environment(route,profile)
            baseline=RouteHarness({'profile':profile,'start_assumptions':values})
            ROUTES[route](baseline,profile)
            # Local compilation now also follows post-wall move spans. The
            # fake wall does not move the plant, so compare the full recipe end.
            goal=dict(x_mm=baseline.pose.x_mm,y_mm=baseline.pose.y_mm,yaw_deg=baseline.pose.yaw_deg)
            plant=MecanumPlant()
            c._checked_move=lambda *args,**kwargs:classic_move(plant,*args,**kwargs)
            env.robot.move_chassis=lambda *args,**kwargs:classic_move(plant,*args,**kwargs)
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
