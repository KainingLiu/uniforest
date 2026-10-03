"""PlanB's public compiled first leg, including its logical heading reference."""
from dataclasses import replace
import math
import unittest
from unittest.mock import Mock
from Strategy.execution import ExecutionRuntime
from Strategy.plans import PLANS, StrategyPlan
from Strategy.optimizations.route_chain import grouped_steps
from control.trajectory import CubicRoute, BodyVelocity
from types import SimpleNamespace
from simulation.core import MecanumPlant
from tests.test_local_routes import environment


class RouteChainTests(unittest.TestCase):
    def test_unsettled_ramp_entry_aborts_before_climb(self):
        env,plan,runtime=self.setup_chain()
        env.robot.chassis.measured_body_velocity.side_effect=[BodyVelocity(),BodyVelocity(0,100,0)]
        with self.assertRaisesRegex(RuntimeError,'ramp entry is not settled'):
            runtime.run(env.compile(plan))
        env.robot.chassis.follow_trajectory.assert_called_once()
        self.assertEqual(env.context.anchor,'start')
        self.assertTrue(env.robot.transport.emergency_stop.called)

    def test_ramp_fault_does_not_start_upper_platform_turn(self):
        env,plan,runtime=self.setup_chain()
        env.robot.chassis.follow_trajectory.side_effect=[None,RuntimeError('ramp tracking fault')]
        with self.assertRaisesRegex(RuntimeError,'ramp tracking fault'):
            runtime.run(env.compile(plan))
        self.assertEqual(env.robot.chassis.follow_trajectory.call_count,2)
        env.robot.chassis.turn.assert_not_called()
        self.assertEqual(env.context.anchor,'start')

    def test_classic_ramp_timeout_at_95_percent_cannot_allow_next_turn(self):
        env,_,_=self.setup_chain()
        c=env.control('highland-1')
        env.robot.move_chassis.return_value=SimpleNamespace(cancelled=False,timed_out=True,
            estimated_chassis_distance_mm=2375.)
        with self.assertRaisesRegex(RuntimeError,'ramp move did not complete'):
            c._checked_move('backward',2500,1000,accel_ms=1000,ramp_straight=True)

    def setup_chain(self, steps=3):
        env,_,_,_=environment('depart_b','depart-b')
        env.context.anchor='start'
        plan=StrategyPlan('first-purple',PLANS['PlanB'].steps[:steps])
        runtime=ExecutionRuntime(guard=env.context.check_active,stop=env.stop,
            emergency_stop=env.robot.transport.emergency_stop,close=env.abort)
        return env,plan,runtime

    def test_public_compiler_preserves_fusion_but_isolates_straight_ramp(self):
        env,plan,runtime=self.setup_chain()
        compiled=env.compile(plan)
        self.assertEqual(len(compiled.flow.actions),1)
        runtime.run(compiled)
        self.assertEqual(env.robot.chassis.follow_trajectory.call_count,3)
        env.robot.chassis.turn.assert_not_called()
        env.robot.move_chassis.assert_not_called()
        env.robot.chassis.set_speeds.assert_called_once_with([0,0,0,0])
        self.assertEqual(env.context.anchor,'purple_area')
        calls=env.robot.chassis.follow_trajectory.call_args_list
        entry,_=calls[0].args
        self.assertAlmostEqual(entry[-1].x_mm,900)
        self.assertAlmostEqual(entry[-1].y_mm,2700)
        points,profile=calls[1].args
        self.assertAlmostEqual(points[-1].x_mm,2500)
        self.assertAlmostEqual(points[-1].y_mm,0)
        self.assertEqual(points[-1].yaw_deg,0)
        curve=CubicRoute(points,profile)
        for i in range(201):
            p,_=curve.sample(curve.duration_s*i/200)
            self.assertAlmostEqual(p.y_mm,0)
            self.assertEqual(p.yaw_deg,0)
        exit_points,_=calls[2].args
        self.assertAlmostEqual(exit_points[-1].x_mm,0)
        self.assertAlmostEqual(exit_points[-1].y_mm,-250)
        self.assertEqual(exit_points[-1].yaw_deg,-90)

    def test_four_wheel_model_stops_before_straight_climb_without_half_turn(self):
        env,plan,runtime=self.setup_chain()
        plant=MecanumPlant()
        env.robot.chassis.measured_body_velocity=plant.measured_velocity
        env.robot.chassis.follow_trajectory=lambda p,s,**kw:plant.follow_local(p,profile=s,initial_velocity=kw['initial_velocity'])
        runtime.run(env.compile(plan))
        self.assertEqual(plant.controller_runs,3)
        self.assertEqual(plant.emergency_stops,0)
        self.assertLess(math.hypot(plant.x-3400,plant.y-2450),12)
        self.assertLess(abs(plant.yaw+90),2)
        through=[p for p in plant.trace if 2400<p['y']<2800 and 1100<p['x']<3200]
        self.assertTrue(through)
        self.assertTrue(all(math.hypot(p['vx'],p['vy'])>20 for p in through))
        self.assertTrue(all(abs(p['yaw'])<1 for p in through))
        self.assertLess(max(p['y'] for p in through)-min(p['y'] for p in through),2)
        self.assertTrue(any(abs(p['x']-900)<12 and abs(p['y']-2700)<12
                            and math.hypot(p['vx'],p['vy'])<21 for p in plant.trace))

    def test_logical_rebase_is_transported_from_entry_gyro_not_final_pose(self):
        env,plan,runtime=self.setup_chain()
        env.robot.telem.yaw_deg=37.
        runtime.run(env.compile(plan))
        self.assertEqual(env.context.heading_zero_deg,37.)
        env.robot.telem.yaw_deg=127.
        self.assertAlmostEqual(env.control('highland-1')._heading_error(270),0.)

    def test_fault_restores_reference_and_does_not_enter_wall_or_repeat(self):
        env,plan,runtime=self.setup_chain(4)
        before=env.context.heading_zero_deg
        env.robot.telem.yaw_deg=37.
        wall=env.control('highland-1')._drive_until_wall=Mock()
        env.robot.chassis.follow_trajectory.side_effect=RuntimeError('stale telemetry')
        with self.assertRaisesRegex(RuntimeError,'stale telemetry'):runtime.run(env.compile(plan))
        self.assertEqual(env.context.heading_zero_deg,before)
        self.assertEqual(env.context.anchor,'start')
        wall.assert_not_called()
        env.robot.chassis.follow_trajectory.assert_called_once()
        env.robot.transport.emergency_stop.assert_called_once()

    def test_optional_tag3_boundary_still_precedes_terminal_move(self):
        env,plan,runtime=self.setup_chain()
        c=env.control('highland-1')
        c.config=replace(c.config,tag3_alignment_enabled=True,post_tag_lateral_mm=0)
        events=[]
        env.robot.chassis.follow_trajectory.side_effect=lambda *a,**kw:events.append('curve')
        env.robot.reset_field_localization_filter.side_effect=lambda:events.append('reset')
        c._align_delivery_tag_or_continue=Mock(side_effect=lambda **kw:events.append('tag'))
        c._checked_move=Mock(side_effect=lambda *a,**kw:events.append('terminal'))
        runtime.run(env.compile(plan))
        self.assertEqual(events,['curve','curve','reset','tag','terminal'])
        c._checked_move.assert_called_once_with('forward',250.,400.,accel_ms=300)

    def test_classic_and_standalone_routes_keep_their_original_action_boundaries(self):
        env,plan,_=self.setup_chain()
        env.transition_config=replace(env.transition_config,motion_planning_enabled=False)
        self.assertEqual(len(env.compile(plan).flow.actions),3)
        self.assertTrue(all(len(g)==1 for g in grouped_steps(PLANS['PlanA'].steps,enabled=True)))
        self.assertEqual([len(g) for g in grouped_steps(PLANS['PlanB'].steps[1:3],enabled=True)],[1,1])


if __name__=='__main__':unittest.main()
