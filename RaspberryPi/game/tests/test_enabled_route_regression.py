"""Regression coverage for the explicitly injected legacy local backend.

Default PlanA/PlanB full-planner dispatch is covered by test_recipe_navigation.
"""
from dataclasses import replace
import math
import unittest
from unittest.mock import Mock
import main
from Strategy.cli import load_execution_config
from Strategy.runner import resolve_selection
from control.trajectory import BodyVelocity, CubicRoute, Waypoint
from simulation.core import MecanumPlant
from tests.test_local_routes import environment


USER_ARGS=['--strategy','PlanA','--trial-optimizations',
           '--enable-transition','next-cube','--enable-transition','last-departure',
           '--enable-transition','purple-departure','--enable-transition','inspect-departure',
           '--enable-transition','build-return','--enable-motion-planning','--enable-fast-alignment']


class EnabledRouteRegression(unittest.TestCase):
    def fixture(self):
        env,c,events,_=environment('orange_to_build','highland-1',heading=180)
        env.transition_config=load_execution_config(main.parse_args(USER_ARGS),resolve_selection('PlanA'))
        c._align_delivery_tag_or_continue=Mock(return_value=True)
        c._turn_to_heading=Mock()
        return env,c,events

    def test_user_entry_optimizes_purple_transit_without_standalone_turn(self):
        env,c,_=self.fixture()
        env.run_route('to_purple','highland-1')
        self.assertTrue(env.transition_config.motion_planning_enabled)
        self.assertGreater(env.robot.chassis.follow_trajectory.call_count,0)
        c._turn_to_heading.assert_not_called()
        c._checked_move.assert_not_called()

    def test_tag_is_a_live_boundary_and_post_tag_motion_starts_after_it(self):
        env,c,_=self.fixture(); calls=[]
        c.config=replace(c.config,tag3_alignment_enabled=True,post_tag_lateral_mm=100)
        env.robot.chassis.follow_trajectory.side_effect=lambda *a,**kw:calls.append('move')
        env.robot.reset_field_localization_filter.side_effect=lambda:calls.append('reset')
        c._align_delivery_tag_or_continue.side_effect=lambda **kw:calls.append('tag')
        env.run_route('to_purple','highland-1')
        self.assertEqual(calls,['move','reset','tag','move'])
        c._turn_to_heading.assert_not_called()

    def test_classic_switch_preserves_original_sequence(self):
        env,c,_=self.fixture()
        env.transition_config=replace(env.transition_config,motion_planning_enabled=False)
        env.run_route('to_purple','highland-1')
        env.robot.chassis.follow_trajectory.assert_not_called()
        c._turn_to_heading.assert_called_once()
        self.assertGreater(c._checked_move.call_count,0)

    def test_purple_transit_preserves_endpoint_and_rotates_during_translation(self):
        env,c,_=self.fixture()
        env.run_route('to_purple','highland-1')
        self.assertGreater(env.robot.chassis.follow_trajectory.call_count,0)
        plant=MecanumPlant()
        moving_turn=False
        for call in env.robot.chassis.follow_trajectory.call_args_list:
            points,profile=call.args
            planned=CubicRoute(points,profile)
            for i in range(101):
                p,v=planned.sample(planned.duration_s*i/100)
                if abs(v.yaw_deg_s)>5 and math.hypot(v.vx_mm_s,v.vy_mm_s)>30:moving_turn=True
                if p.x_mm>-c.config.initial_distance_mm+c.config.wall_premove_mm:
                    self.assertAlmostEqual(p.yaw_deg,0,places=5)
                    self.assertAlmostEqual(v.yaw_deg_s,0,places=5)
            plant.follow_local(points,profile=profile)
        self.assertTrue(moving_turn)
        self.assertEqual(plant.emergency_stops,0)
        self.assertEqual(plant.controller_runs,1)
        self.assertTrue(all(math.hypot(p['vx'],p['vy'])>20 for p in plant.trace if abs(p['wz'])>5))
        self.assertLess(math.hypot(plant.x+c.config.initial_distance_mm,plant.y-c.config.wall_premove_mm),12)
        self.assertLess(abs(plant.yaw-90),2)
        settings=env.robot.chassis.follow_trajectory.call_args.args[1]
        self.assertEqual(settings.max_speed_mm_s,c.config.initial_speed_mm_s)
        self.assertEqual(settings.segment_speeds_mm_s[-1],c.config.wall_premove_speed_mm_s)

    def test_failed_tag_never_executes_post_tag_trajectory(self):
        env,c,_=self.fixture()
        c.config=replace(c.config,tag3_alignment_enabled=True)
        c._align_delivery_tag_or_continue.side_effect=RuntimeError('vision fault')
        with self.assertRaisesRegex(RuntimeError,'vision fault'):env.run_route('to_purple','highland-1')
        self.assertEqual(env.robot.chassis.follow_trajectory.call_count,1)
