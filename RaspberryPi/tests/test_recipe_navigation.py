"""Original route endpoints use the full planner/timing/PID at competition entry."""
from dataclasses import replace
import math
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from control.chassis import Chassis, COUNTS_PER_CM, LATERAL_DISTANCE_SCALE
from control.trajectory import BodyVelocity, Waypoint
from Strategy.navigation import Pose, NavigationError
from Strategy.navigation.control import MotionFeedback, execute_route
from Strategy.navigation.odometry import RouteOdometry
from Strategy.navigation.recipe import RecipeRoutes, default_planner, endpoint_from_recipe
from Strategy.navigation.robot_adapter import RobotNavigation
from Strategy.optimizations.motion_planning import MotionPlanning
from Strategy.optimizations.local_routes import RouteRecorder
from Strategy.flows.routes import ROUTES
from Strategy.plans import PLANS
from simulation.core import MecanumPlant
from tests.test_curve_routes import fixture


def environment(route='ground_tag_offset', profile='ground-1', **kwargs):
    fixture_route={'ground_delivery_reverse':'ground_to_delivery',
        'orange_depart_reverse':'orange_to_build','to_purple':'orange_to_build',
        'unload_approach':'unload_depart'}.get(route,route)
    env,c,events,_ = fixture(fixture_route,profile,**kwargs)
    env.transition_config = replace(env.transition_config,curves={},navigation=None)
    env.motion_planning = MotionPlanning()
    backend = next(x for x in env.motion_planning.optimizers if isinstance(x,RecipeRoutes))
    backend.planner = default_planner()
    state = SimpleNamespace(pose=Pose(3700,5600,0))
    state.snapshot = lambda: MotionFeedback(state.pose,BodyVelocity(),time.monotonic())
    backend.odometry = state
    env.robot.field_pose = None
    env.robot.diagnostics = SimpleNamespace(write=Mock())
    env.robot.actions._action_lock = threading.Lock()
    env.robot.telem.stepper_busy = 0
    env.robot.chassis.measured_body_velocity = Mock(return_value=BodyVelocity())
    return env,c,state,backend


class RecipeNavigationTests(unittest.TestCase):
    def test_both_full_plans_start_default_backend_without_camera_or_new_configuration(self):
        import main
        from Strategy.cli import load_execution_config
        for name in ('PlanA','PlanB'):
            env,_,state,backend=environment()
            plan=PLANS[name]
            env.transition_config=load_execution_config(main.parse_args(
                ['--strategy',name,'--enable-motion-planning']),plan)
            env.robot.begin_route_odometry=Mock(return_value=state)
            env.robot.end_route_odometry=Mock()
            env.motion_planning.start(env,plan)
            env.robot.begin_route_odometry.assert_called_once_with(Pose(700,6500,270))
            self.assertIs(backend.odometry,state)
            env.motion_planning.close(env)
            env.robot.end_route_odometry.assert_called_once_with(state)

    def test_every_plan_route_uses_full_backend_and_preserves_original_endpoint(self):
        sites={(s.parameters[k],s.profile) for p in (PLANS['PlanA'],PLANS['PlanB'])
               for s in p.steps for k in ('route','exit_route','followup_route') if k in s.parameters}
        self.assertEqual({r for r,p in sites},set(ROUTES))
        for route,profile in sorted(sites):
            with self.subTest(route=route,profile=profile):
                env,c,state,_=environment(route,profile,lateral=110)
                # Test only geometry/dispatch; the real planner/model is exercised below.
                recorder=RouteRecorder(env,profile)
                ROUTES[route](recorder,profile)
                initial=state.pose
                expected=endpoint_from_recipe(initial,Waypoint(0,0,0),recorder.pose)
                def prepare(adapter,start,goal):
                    return SimpleNamespace(start=start,goal=goal,nominal_length_mm=1,candidate_count=2)
                def execute(adapter,planned):
                    state.pose=planned.goal
                    return {'status':'arrived'}
                with patch.object(RobotNavigation,'prepare_between',autospec=True,side_effect=prepare) as plan, \
                     patch.object(RobotNavigation,'execute',autospec=True,side_effect=execute):
                    env.run_route(route,profile)
                self.assertGreater(plan.call_count,0)
                for actual,wanted in zip(vars(state.pose).values(),vars(expected).values()):
                    self.assertAlmostEqual(actual,wanted,places=6)
                env.robot.chassis.follow_trajectory.assert_not_called()
                env.robot.chassis.turn.assert_not_called()
                c._checked_move.assert_not_called()
                env.robot.diagnostics.write.assert_any_call('motion_backend_selected',route=route,
                    profile=profile,backend='field_curve:recipe_endpoints')

    def test_old_intermediate_points_are_not_imposed_on_full_path_search(self):
        env,_,state,_=environment('depart_b','depart-b')
        with patch.object(RobotNavigation,'prepare_between',return_value=SimpleNamespace(
                nominal_length_mm=1,candidate_count=1)) as prepare, \
             patch.object(RobotNavigation,'execute'):
            env.run_route('depart_b','depart-b')
        prepare.assert_called_once_with(state.pose,Pose(4600,8300,180))

    def test_real_planner_and_real_velocity_pid_execute_on_virtual_wheels_without_tag(self):
        env,_,_,backend=environment(config_overrides={'post_tag_lateral_right_mm':80})
        plant=MecanumPlant()
        robot=env.robot
        env.context=SimpleNamespace(check_active=plant.guard,close=Mock())
        robot.chassis.lateral_distance_scale=LATERAL_DISTANCE_SCALE
        robot.chassis.mecanum_rpm=Chassis.mecanum_rpm
        robot.chassis.measured_body_velocity=plant.measured_velocity
        def snapshot():
            return SimpleNamespace(stepper_busy=0),plant.now,plant.now,0
        robot.inspection_link_snapshot=snapshot
        robot.chassis.set_speeds=Mock(side_effect=lambda values:setattr(plant,'targets',list(values)) or True)
        robot.transport.emergency_stop=Mock(side_effect=plant.emergency_stop)
        backend.odometry=SimpleNamespace(snapshot=lambda:MotionFeedback(
            Pose(3700+plant.x,5600+plant.y,plant.yaw),plant.measured_velocity(),plant.now))
        with patch('Strategy.navigation.robot_adapter.time.monotonic',side_effect=lambda:plant.now), \
             patch('Strategy.navigation.robot_adapter.time.sleep',side_effect=plant.sleep), \
             patch('Strategy.navigation.control.PositionTracker', wraps=__import__(
                 'Strategy.navigation.tracking',fromlist=['PositionTracker']).PositionTracker) as pid:
            result=env.run_route('ground_tag_offset','ground-1')
        pid.assert_called_once()
        self.assertEqual(result['status'],'arrived')
        self.assertLess(math.hypot(plant.x,plant.y-80),8)
        self.assertEqual(plant.targets,[0,0,0,0])
        robot.chassis.follow_trajectory.assert_not_called()
        robot.transport.emergency_stop.assert_not_called()

    def test_planning_fault_never_falls_back_or_runs_contact_tail(self):
        env,c,_,_=environment('purple_to_orange','highland-1')
        with patch.object(RobotNavigation,'prepare_between',side_effect=NavigationError('collision')):
            with self.assertRaisesRegex(NavigationError,'collision'):
                env.run_route('purple_to_orange','highland-1')
        self.assertFalse(env.data.get('purple_route_done',False))
        c._drive_until_wall.assert_not_called()
        env.robot.chassis.follow_trajectory.assert_not_called()

    def test_tag_remains_a_live_boundary_before_remaining_route(self):
        env,c,state,_=environment('to_purple','highland-1',heading_cw=180,
            config_overrides={'tag3_alignment_enabled':True,'post_tag_lateral_mm':100})
        events=[]
        c._align_delivery_tag_or_continue=Mock(side_effect=lambda **kw:events.append('tag'))
        env.robot.reset_field_localization_filter.side_effect=lambda:events.append('reset')
        with patch.object(RobotNavigation,'prepare_between',return_value=SimpleNamespace(
                nominal_length_mm=1,candidate_count=1)), \
             patch.object(RobotNavigation,'execute',side_effect=lambda p:events.append('planned')):
            env.run_route('to_purple','highland-1')
        self.assertEqual(events,['planned','reset','tag','planned'])

    def test_new_telemetry_published_during_read_is_not_rejected_as_future(self):
        planner=default_planner();pose=Pose(3700,5600,0)
        route=planner.plan_between(pose,pose)
        clock=[0.]
        def feedback():
            clock[0]+=.001
            return MotionFeedback(pose,BodyVelocity(),clock[0])
        result=execute_route(planner,route,read_feedback=feedback,read_observations=lambda:[],
            send_velocity=lambda v:True,check=lambda:True,stop=lambda:None,
            clock=lambda:clock[0],sleep=lambda dt:clock.__setitem__(0,clock[0]+dt))
        self.assertEqual(result['status'],'arrived')


class OdometryTests(unittest.TestCase):
    def test_robot_feeds_every_telemetry_frame_and_releases_only_the_owner(self):
        from robot import Robot
        robot=Robot(port='unused-test-port')
        robot._on_telem(self.frame())
        owner=robot.begin_route_odometry(Pose(1000,2000,0))
        counts=round(COUNTS_PER_CM*10)
        robot._on_telem(self.frame((-counts,counts,counts,-counts),uptime=20))
        self.assertAlmostEqual(owner.snapshot().pose.x,1100,delta=.1)
        robot.end_route_odometry(object())
        self.assertIs(robot._route_odometry,owner)
        robot.end_route_odometry(owner)
        self.assertIsNone(robot._route_odometry)
    def frame(self, counts=(0,0,0,0), yaw=0, uptime=0):
        return SimpleNamespace(motors=[SimpleNamespace(cumulative_pos=c,speed_rpm=0) for c in counts],
                               yaw_deg=yaw,uptime_ms=uptime)

    def test_tracks_intermediate_frames_while_other_actions_turn_then_move(self):
        odo=RouteOdometry(Pose(1000,2000,0),LATERAL_DISTANCE_SCALE,3,5)
        odo.update(self.frame(),0,3,5)
        odo.update(self.frame(yaw=-90,uptime=20),.02,3,5)
        counts=round(COUNTS_PER_CM*10)
        odo.update(self.frame((-counts,counts,counts,-counts),-90,40),.04,3,5)
        result=odo.snapshot(.04).pose
        self.assertAlmostEqual(result.x,1000,places=5)
        self.assertAlmostEqual(result.y,2100,delta=.1)
        self.assertEqual(result.yaw,90)

    def test_link_loss_restart_stale_frame_and_emergency_stop_latch_fault(self):
        for fault in ('link','stop','gap','restart','invalid'):
            odo=RouteOdometry(Pose(1000,2000,0),LATERAL_DISTANCE_SCALE,3,5)
            odo.update(self.frame(uptime=100),0,3,5)
            frame=self.frame(uptime=120)
            if fault=='restart':frame.uptime_ms=0
            if fault=='invalid':frame.yaw_deg=float('nan')
            odo.update(frame,.3 if fault=='gap' else .02,4 if fault=='link' else 3,6 if fault=='stop' else 5)
            odo.update(self.frame(uptime=140),.04,3,5)
            with self.subTest(fault=fault),self.assertRaises(NavigationError):odo.snapshot(.04)

    def test_unsigned_encoder_wrap_and_duplicate_frame_do_not_jump(self):
        odo=RouteOdometry(Pose(1000,2000,0),LATERAL_DISTANCE_SCALE,0,0)
        first=(0x7fffffff,0x7fffffff,0x7fffffff,0x7fffffff)
        odo.update(self.frame(first),0,0,0)
        odo.update(self.frame(tuple(-0x80000000 for _ in range(4)),uptime=20),.02,0,0)
        self.assertEqual(odo.snapshot(.02).pose,Pose(1000,2000,0))


if __name__=='__main__':unittest.main()
