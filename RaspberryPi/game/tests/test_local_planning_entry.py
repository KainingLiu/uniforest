"""Original actions, public launch, and one-shot moving Tag6 handoff contracts."""
from dataclasses import replace
import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import main
import task2_main
from Strategy.cli import load_execution_config
from Strategy.flows.model import ActionSpec
from Strategy.execution import ExecutionRuntime
from Strategy.optimizations.motion_planning import MotionPlanning
from Strategy.optimizations.local_routes import local_points
from Strategy.plans import PLANS, StrategyPlan
from control.trajectory import Waypoint
from control.trajectory import CubicRoute
import math
from tests.test_local_routes import environment


class LocalPlanningEntryTests(unittest.TestCase):
    def test_depart_b_cuts_corner_inside_corridor_and_preserves_goal(self):
        env,_,_,_=environment('depart_b','depart-b')
        env.run_route('depart_b','depart-b')
        points,settings=env.robot.chassis.follow_trajectory.call_args.args
        self.assertEqual(points[-1],Waypoint(900,2700,180))
        self.assertNotIn(Waypoint(900,0,0),points)
        curve=CubicRoute(points,settings)
        trace=[curve.sample(curve.duration_s*i/1000)[0] for i in range(1001)]
        # First straight and rightward leg define the original L corridor.
        deviation=[min(math.hypot(p.x_mm-max(0,min(900,p.x_mm)),p.y_mm),
                       math.hypot(p.x_mm-900,p.y_mm-max(0,min(2700,p.y_mm)))) for p in trace]
        self.assertGreater(max(deviation),150)
        self.assertLessEqual(max(deviation),250)
        length=sum(math.hypot(b.x_mm-a.x_mm,b.y_mm-a.y_mm) for a,b in zip(trace,trace[1:]))
        self.assertLess(length,3400)  # Original polyline is 3600 mm.

    def test_plan_b_first_collection_target_remains_purple_after_departure(self):
        steps=PLANS['PlanB'].steps
        self.assertEqual([(s.kind,s.parameters.get('route')) for s in steps[:4]],
                         [('navigate','depart_b'),('rebase_heading',None),
                          ('navigate','to_purple'),('anchor_wall',None)])
        first=next(s for s in steps if s.kind=='begin_collection')
        self.assertEqual(first.parameters['color'],'purple')

    def test_turn_blending_preserves_long_straight_and_terminal_tag_view(self):
        origin=Waypoint(0,0,0)
        events=[('turn',Waypoint(0,0,90),(),{}),
                ('move',Waypoint(0,1200,90),(),{})]
        points=local_points(events,origin)
        self.assertEqual(points,(origin,Waypoint(0,250,90),Waypoint(0,1200,90)))
        events=[('move',Waypoint(1200,0,0),(),{}),
                ('turn',Waypoint(1200,0,180),(),{})]
        points=local_points(events,origin,terminal_tag_window_mm=900)
        self.assertEqual(points,(origin,Waypoint(300,0,0),Waypoint(615,0,180),Waypoint(1200,0,180)))

    def test_single_1200mm_departure_keeps_original_controller_and_strict_timeout(self):
        env,c,_,_=environment('depart_a','depart-a')
        env.robot.field_pose=None
        env.run_route('depart_a','depart-a')
        env.robot.move_chassis.assert_called_once_with('forward',1200.,2000.,hold_ms=0,accel_ms=1600,route_mode=True)
        env.robot.chassis.follow_trajectory.assert_not_called()
        env.robot.move_chassis.return_value.timed_out=True
        with self.assertRaisesRegex(RuntimeError,'departure position move'):
            env.run_route('depart_a','depart-a')

    def test_relative_turn_sequence_has_equivalent_endpoint_and_final_heading(self):
        env,c,_,_=environment('depart_b','depart-b')
        def route(owner,profile):
            r=owner.control(profile)
            r.turn(90,120); r._checked_move('forward',600,400)
            r.turn(-90,120); r._checked_move('forward',900,400)
            r.turn(-90,120)
        with patch.dict('Strategy.flows.routes.ROUTES',{'depart_b':route}):
            env.run_route('depart_b','depart-b')
        points=env.robot.chassis.follow_trajectory.call_args.args[0]
        self.assertAlmostEqual(points[-1].x_mm,900)
        self.assertAlmostEqual(points[-1].y_mm,600)
        self.assertEqual(points[-1].yaw_deg,-90)
        env.robot.chassis.turn.assert_not_called()

    def test_enabled_entry_has_no_global_map_or_tag_precondition(self):
        for entry in (main,task2_main):
            flags=['--enable-motion-planning','--show-plan']
            with patch('sys.argv',['entry.py',*flags]),patch.object(entry,'Robot') as robot, \
                 contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(entry.main(),0)
            robot.assert_not_called()
            self.assertIn('local route smoothing',output.getvalue())
            self.assertNotIn('field_navigation',output.getvalue())
        config=load_execution_config(main.parse_args(['--enable-motion-planning','--no-field-localization']),PLANS['PlanA'])
        self.assertTrue(config.motion_planning_enabled)

    def test_only_actual_adjacent_tag6_and_offset_pairs_are_fused(self):
        planner=MotionPlanning(moving_tag6_enabled=True)
        planner.start(None,PLANS['PlanA'])
        self.assertEqual(len(planner._routes.approaches),6)
        planner.start(None,PLANS['PlanB'])
        self.assertEqual(len(planner._routes.approaches),2)
        planner.start(None,PLANS['collect-mixed-1'])
        self.assertEqual(planner._routes.approaches,{})

    def approach_environment(self):
        env,c,events,_=environment('ground_to_delivery','ground-1',heading=0,lateral=200,reverse=True)
        source=ActionSpec('navigate','go','ground-1',{'route':'ground_to_delivery'})
        tag=ActionSpec('align_tag','tag','ground-1',{'purpose':'delivery'})
        offset=ActionSpec('navigate','offset','ground-1',{'route':'ground_tag_offset'})
        plan=StrategyPlan('approach',(source,tag,offset))
        env.motion_planning=MotionPlanning(moving_tag6_enabled=True)
        env.motion_planning.start(env,plan)
        env.context.current_action='go'
        return env,c,events,source,tag,offset

    def test_confirmed_guidance_consumes_exact_following_actions_once(self):
        env,c,_,source,tag,offset=self.approach_environment()
        c._align_delivery_tag_or_continue=Mock(return_value=True)
        def follow(points,settings,**kw):
            g=kw['guidance']
            self.assertEqual(g.offset_mm,100)
            g.completed=g.active=True
            g.accepted_frames=4
        env.robot.chassis.follow_trajectory.side_effect=follow
        env.run_route('ground_to_delivery','ground-1')
        self.assertTrue(env._align_tag(tag))
        c._align_delivery_tag_or_continue.assert_not_called()
        env.bind(offset).enter()
        self.assertEqual(env.robot.chassis.follow_trajectory.call_count,1)
        c._checked_move.assert_not_called()
        # A later standalone request must perform its own alignment again.
        env._align_tag(tag)
        c._align_delivery_tag_or_continue.assert_called_once()

    def test_missing_visual_confirmation_keeps_original_tag_and_offset(self):
        env,c,_,source,tag,offset=self.approach_environment()
        c._align_delivery_tag_or_continue=Mock(return_value=True)
        env.run_route('ground_to_delivery','ground-1')
        env._align_tag(tag)
        c._align_delivery_tag_or_continue.assert_called_once()
        env.bind(offset).enter()
        c._checked_move.assert_called_once_with('right',100.,400.)

    def test_motion_failure_cannot_skip_alignment_or_commit_route(self):
        env,c,_,source,tag,offset=self.approach_environment()
        env.data['ground_reverse_done']=False
        env.robot.chassis.follow_trajectory.side_effect=RuntimeError('link fault')
        with self.assertRaisesRegex(RuntimeError,'link fault'):
            env.run_route('ground_to_delivery','ground-1')
        self.assertFalse(env.motion_planning.consume_completed(tag.name))
        self.assertFalse(env.motion_planning.consume_completed(offset.name))
        self.assertFalse(env.data['ground_reverse_done'])

    def test_inspection_retreat_under_next_action_name_cannot_start_tag_guidance(self):
        env,c,_,source,tag,offset=self.approach_environment()
        env.data['ground_reverse_done']=False
        env.run_route('ground_delivery_reverse','ground-1')
        env.robot.chassis.follow_trajectory.assert_not_called()
        c._checked_move.assert_called_once_with('backward',400.,400.)
        self.assertFalse(env.motion_planning.consume_completed(tag.name))

    def test_optimized_building_approach_requires_actual_visual_confirmation(self):
        env,c,_,_=environment('build_offset','building-1')
        c._align_building=Mock(side_effect=RuntimeError('building missing'))
        step=ActionSpec('align_building','align','building-1')
        env.robot.actions.begin=Mock()
        plan=StrategyPlan('terminal-confirmation',(step,ActionSpec('build','build','building-1')))
        runtime=ExecutionRuntime(guard=env.context.check_active,stop=env.stop,
            emergency_stop=env.robot.transport.emergency_stop,close=env.abort)
        with self.assertRaisesRegex(RuntimeError,'building missing'):
            runtime.run(env.compile(plan))
        env.robot.actions.begin.assert_not_called()
        env.robot.transport.emergency_stop.assert_called_once()


if __name__=='__main__':unittest.main()
