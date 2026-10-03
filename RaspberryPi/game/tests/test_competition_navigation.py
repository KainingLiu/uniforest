"""Standalone field-planner adapter checks; competition entry uses local smoothing."""
import contextlib
from dataclasses import replace
import io
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import main
import task2_main
from control.chassis import Chassis, LATERAL_DISTANCE_SCALE
from control.trajectory import BodyVelocity, Waypoint
from Strategy.flows.curves import RouteGeometry, CURVE_ROUTES
from Strategy.navigation import Pose, NavigationError
from Strategy.navigation.competition import CompetitionNavigationConfig, CompetitionRoutes, measured_pose
from Strategy.navigation.robot_adapter import NavigationCalibration, RobotNavigation
from Strategy.plans import PLANS
from Strategy.optimizations.motion_planning import MotionPlanning
from simulation.core import MecanumPlant
from tests.test_curve_routes import fixture

EXAMPLE = Path(__file__).resolve().parents[1] / 'Strategy/navigation/competition.example.json'


def synthetic_config():
    # This approval belongs solely to the fake robot/virtual field in this test.
    return replace(CompetitionNavigationConfig.load(EXAMPLE),
                   calibration=NavigationCalibration(True, 'synthetic test only', 0))


def field_pose(pose=Pose(3700, 5600, 0), captured=None):
    class Frame(SimpleNamespace):
        @property
        def captured_monotonic(self):
            return time.monotonic() if self._captured is None else self._captured

        @captured_monotonic.setter
        def captured_monotonic(self, value):
            self._captured = value
    return Frame(valid=True, calibrated=True, _captured=captured,
        tag_solutions=[SimpleNamespace(tag_id=5, x_m=(pose.x-2400)/1000,
            y_m=(3600-pose.y)/1000, yaw_deg=-pose.yaw, area_px=600., reprojection_error_px=.5)])


def environment():
    env, control, events, result = fixture('ground_tag_offset', 'ground-1',
        config_overrides={'post_tag_lateral_right_mm': 80., 'post_tag_lateral_direction': 'right'})
    env.transition_config = replace(env.transition_config, curves={}, navigation=synthetic_config())
    backend = CompetitionRoutes()
    env.motion_planning = SimpleNamespace(run=lambda owner, name, selected, **kw:
        backend.prepare(owner,name,selected).execute())
    env.robot.field_pose = field_pose()
    env.robot.diagnostics = SimpleNamespace(write=Mock())
    env.robot.actions._action_lock = threading.Lock()
    env.robot.telem.stepper_busy = 0
    env.robot.chassis.measured_body_velocity = Mock(return_value=BodyVelocity())
    return env


class CompetitionPlanningTests(unittest.TestCase):
    def test_every_plana_planb_eligible_route_dispatches_to_navigation(self):
        config = synthetic_config()
        sites = {(step.parameters['route'], step.profile)
                 for plan in (PLANS['PlanA'], PLANS['PlanB']) for step in plan.steps
                 if step.kind == 'navigate' and step.parameters['route'] in CURVE_ROUTES}
        for route, profile in sites:
            with self.subTest(route=route, profile=profile):
                env, _, _, _ = fixture(route, profile)
                env.transition_config = replace(env.transition_config, curves={}, navigation=config)
                backend = CompetitionRoutes()
                env.motion_planning = SimpleNamespace(run=lambda owner, name, selected, **kw:
                    backend.prepare(owner,name,selected).execute())
                env.robot.field_pose = field_pose()
                env.robot.diagnostics = SimpleNamespace(write=Mock())
                planned = SimpleNamespace(nominal_length_mm=100., candidate_count=1)
                with patch.object(RobotNavigation, 'prepare_between', return_value=planned) as prepare, \
                        patch.object(RobotNavigation, 'execute', return_value='arrived') as execute, \
                        patch('Strategy.flows.factory.ROUTES', {route: Mock()}) as classic:
                    env.run_route(route, profile)
                    prepare.assert_called_once()
                    execute.assert_called_once_with(planned)
                    classic[route].assert_not_called()
        self.assertEqual({route for route, _ in sites}, CURVE_ROUTES)

    def test_recipe_dispatch_uses_real_planner_and_preserves_endpoint(self):
        env = environment()
        with patch.object(RobotNavigation, 'execute', return_value='arrived') as execute, \
                patch('Strategy.flows.factory.ROUTES', {'ground_tag_offset': Mock()}) as classic:
            self.assertEqual(env.run_route('ground_tag_offset', 'ground-1'), 'arrived')
        route = execute.call_args.args[0]
        self.assertEqual(route.start, Pose(3700, 5600, 0))
        self.assertEqual(route.goal, Pose(3700, 5680, 0))
        self.assertGreater(len(route.samples), 2)
        self.assertGreater(route.candidate_count, 0)
        self.assertGreaterEqual(float(route.samples[:, 4].min()), 0)
        classic['ground_tag_offset'].assert_not_called()
        env.robot.chassis.follow_trajectory.assert_not_called()
        env.record_transition.assert_called_with('field_navigation', 'ground_tag_offset', 'complete')

    def test_default_off_does_not_consult_localization_or_run_planner(self):
        env = environment()
        env.transition_config = replace(env.transition_config, motion_planning_enabled=False)
        env.robot.field_pose = None
        with patch('Strategy.flows.factory.ROUTES', {'ground_tag_offset': Mock(return_value='classic')}) as routes:
            self.assertEqual(env.run_route('ground_tag_offset', 'ground-1'), 'classic')
            routes['ground_tag_offset'].assert_called_once()

    def test_invalid_pose_or_goal_never_starts_motion(self):
        for fault in ('uncalibrated', 'stale', 'missing', 'collision'):
            env = environment()
            if fault == 'uncalibrated': env.robot.field_pose.calibrated = False
            if fault == 'stale': env.robot.field_pose.captured_monotonic -= 2
            if fault == 'missing': env.robot.field_pose = None
            if fault == 'collision': env.robot.field_pose = field_pose(Pose(200, 400, 0))
            with self.subTest(fault=fault), patch.object(RobotNavigation, 'execute') as execute, \
                    patch('Strategy.flows.factory.ROUTES', {'ground_tag_offset': Mock()}) as classic:
                with self.assertRaises(NavigationError): env.run_route('ground_tag_offset', 'ground-1')
                execute.assert_not_called()
                classic['ground_tag_offset'].assert_not_called()

    def test_planning_error_or_execution_error_never_replays_old_route_or_commits(self):
        for stage in ('prepare_between', 'execute'):
            env = environment()
            complete = Mock()
            geometry = RouteGeometry(Waypoint(0, 80, 0), complete)
            with patch('Strategy.flows.curves.prepare_route_geometry', return_value=geometry), \
                    patch.object(RobotNavigation, stage, side_effect=NavigationError('injected fault')), \
                    patch('Strategy.flows.factory.ROUTES', {'ground_tag_offset': Mock()}) as classic:
                with self.assertRaisesRegex(NavigationError, 'injected fault'):
                    env.run_route('ground_tag_offset', 'ground-1')
                classic['ground_tag_offset'].assert_not_called()
                complete.assert_not_called()

    def test_start_changed_while_planning_is_rejected(self):
        env = environment()
        with patch('Strategy.navigation.competition.measured_pose',
                   side_effect=[Pose(3700,5600,0), Pose(3750,5600,0)]), \
                patch.object(RobotNavigation, 'execute') as execute:
            with self.assertRaisesRegex(NavigationError, 'start changed'):
                env.run_route('ground_tag_offset', 'ground-1')
            execute.assert_not_called()

    def test_active_overlap_uses_explicit_classic_reason_without_planning(self):
        env = environment()
        env.transitions.inspections['synthetic'] = object()
        with patch('Strategy.flows.routes.ROUTES', {'ground_tag_offset': Mock(return_value='overlap')}):
            self.assertEqual(env.run_route('ground_tag_offset', 'ground-1'), 'overlap')
        env.robot.chassis.follow_trajectory.assert_not_called()

    def test_hardware_entry_requires_idle_mechanism_and_stationary_chassis(self):
        for fault in ('moving', 'busy'):
            env = environment()
            if fault == 'moving': env.robot.chassis.measured_body_velocity.return_value = BodyVelocity(30,0,0)
            else: env.robot.telem.stepper_busy = 1
            with self.subTest(fault=fault), patch.object(RobotNavigation, 'execute') as execute:
                with self.assertRaisesRegex(ValueError, 'stationary|mechanism'):
                    env.run_route('ground_tag_offset','ground-1')
                execute.assert_not_called()

    def test_raw_tag_disagreement_rejects_fused_pose(self):
        env = environment()
        other = SimpleNamespace(**vars(env.robot.field_pose.tag_solutions[0]))
        other.x_m += .5
        env.robot.field_pose.tag_solutions.append(other)
        with self.assertRaisesRegex(NavigationError, 'disagree'):
            measured_pose(env.robot, env.transition_config.navigation.calibration)

    def test_camera_snapshot_is_taken_before_freshness_clock(self):
        env = environment()
        with patch('Strategy.navigation.competition.time.monotonic', side_effect=[100., 100.001]):
            self.assertEqual(measured_pose(env.robot, env.transition_config.navigation.calibration),
                             Pose(3700,5600,0))

    def test_real_adapter_drives_four_wheel_plant_to_recipe_destination(self):
        env = environment()
        plant = MecanumPlant()
        robot = env.robot
        env.context = SimpleNamespace(check_active=plant.guard, close=Mock())
        robot.field_pose = field_pose(captured=0.)
        robot.chassis.lateral_distance_scale = LATERAL_DISTANCE_SCALE
        robot.chassis.mecanum_rpm = Chassis.mecanum_rpm
        robot.chassis.measured_body_velocity = plant.measured_velocity
        def speeds(values):
            plant.guard()
            plant.targets = list(values)
            return True
        robot.chassis.set_speeds = Mock(side_effect=speeds)
        robot.transport.emergency_stop = Mock(side_effect=plant.emergency_stop)
        def snapshot():
            telem = SimpleNamespace(stepper_busy=0, yaw_deg=-plant.yaw,
                motors=[SimpleNamespace(cumulative_pos=round(c)) for c in plant.counts])
            return telem, plant.now, plant.now, 0
        robot.inspection_link_snapshot = snapshot
        with patch('Strategy.navigation.robot_adapter.time.monotonic', side_effect=lambda: plant.now), \
                patch('Strategy.navigation.robot_adapter.time.sleep', side_effect=plant.sleep):
            result = env.run_route('ground_tag_offset', 'ground-1')
        self.assertEqual(result['status'], 'arrived')
        self.assertLess(math.hypot(plant.x, plant.y-80), 8)
        self.assertGreater(robot.chassis.set_speeds.call_count, 2)
        self.assertEqual(plant.targets, [0,0,0,0])
        robot.transport.emergency_stop.assert_not_called()


class CompetitionLaunchTests(unittest.TestCase):
    def launch(self, entry, flags, expected):
        with patch('sys.argv', ['entry.py', *flags]), patch.object(entry, 'Robot') as robot, \
                contextlib.redirect_stdout(io.StringIO()) as output, contextlib.redirect_stderr(io.StringIO()) as errors:
            self.assertEqual(entry.main(), expected, errors.getvalue())
            robot.assert_not_called()
        return output.getvalue(), errors.getvalue()

    def test_local_planning_previews_without_configuration_or_tag_before_robot_creation(self):
        for entry in (main, task2_main):
            output, _ = self.launch(entry, ['--enable-motion-planning', '--show-plan'], 0)
            self.assertIn('local route smoothing', output)

    def test_local_preview_and_standalone_map_data_remain_distinct(self):
        for plan in ('PlanA', 'PlanB'):
            flags = ['--strategy', plan, '--enable-motion-planning']
            output, _ = self.launch(main, [*flags, '--show-plan'], 0)
            self.assertIn('No global-map start', output)
            for step in PLANS[plan].steps:
                if step.kind == 'navigate':
                    route = step.parameters['route']
                    self.assertIn(f'{step.profile}/{route}: local_route:existing_recipe', output)
        with self.assertRaises(ValueError):
            CompetitionNavigationConfig.load(EXAMPLE).calibration.validate()


if __name__ == '__main__':
    unittest.main()
