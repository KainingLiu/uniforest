"""The optional moving Tag6 module cannot change plain local planning."""
from dataclasses import replace
import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import main
import task2_main
from Strategy.cli import load_execution_config, print_plan
from Strategy.flows.factory import ActionEnvironment
from Strategy.flows.route_source import RouteRecorder
from Strategy.flows.routes import ROUTES
from Strategy.optimizations.motion_planning import MotionPlanning
from Strategy.plans import PLANS
from Strategy.transition_config import TransitionConfig
from tests.test_local_routes import environment


class MovingTag6SwitchTests(unittest.TestCase):
    def test_cli_independent_default_and_explicit_enable_disable(self):
        for entry in (main, task2_main):
            for extra,expected in (([],False),(['--disable-moving-tag6'],False),
                                   (['--enable-moving-tag6'],True)):
                with self.subTest(entry=entry.__name__,extra=extra):
                    config=load_execution_config(entry.parse_args(['--enable-motion-planning',*extra]),PLANS['PlanA'])
                    self.assertTrue(config.motion_planning_enabled)
                    self.assertIs(config.moving_tag6_enabled,expected)
                    output=io.StringIO()
                    with contextlib.redirect_stdout(output):print_plan(PLANS['PlanA'],config)
                    self.assertIn('Moving Tag6: '+('enabled' if expected else 'disabled'),output.getvalue())
        config=load_execution_config(main.parse_args(['--trial-optimizations','--enable-transitions',
            '--enable-fast-alignment','--enable-motion-planning']),PLANS['PlanA'])
        self.assertFalse(config.moving_tag6_enabled)

    def test_moving_feedback_requires_planning_before_robot_creation(self):
        for entry in (main, task2_main):
            with patch('sys.argv',['entry.py','--enable-moving-tag6','--show-plan']), \
                    patch.object(entry,'Robot') as robot,contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(entry.main(),2)
            robot.assert_not_called()
        with self.assertRaisesRegex(ValueError,'requires motion planning'):
            MotionPlanning(enabled=False,moving_tag6_enabled=True)

    def test_config_file_supports_opt_in_but_cli_still_requires_explicit_enable(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'config.json'
            for enabled in (False,True):
                path.write_text(json.dumps(dict(version=1,motion_planning_enabled=True,
                    moving_tag6_enabled=enabled)),encoding='utf-8')
                self.assertIs(TransitionConfig.load(path).moving_tag6_enabled,enabled)
                config=load_execution_config(main.parse_args(['--transition-config',str(path),
                    '--enable-motion-planning','--disable-moving-tag6']),PLANS['PlanA'])
                self.assertFalse(config.moving_tag6_enabled)
            path.write_text('{"version":1}',encoding='utf-8')
            self.assertFalse(TransitionConfig.load(path).moving_tag6_enabled)
        for invalid in (1,None,'true'):
            with self.assertRaisesRegex(ValueError,'must be boolean'):
                TransitionConfig(motion_planning_enabled=True,moving_tag6_enabled=invalid)

    def test_all_disabled_approaches_preserve_original_goal_alignment_and_offset(self):
        cases=0
        for plan in (PLANS['PlanA'],PLANS['PlanB']):
            for source,tag,offset in zip(plan.steps,plan.steps[1:],plan.steps[2:]):
                if source.kind!='navigate' or tag.kind!='align_tag' or offset.kind!='navigate':continue
                route=source.parameters['route']
                if route not in ('ground_to_delivery','orange_to_build'):continue
                with self.subTest(plan=plan.name,source=source.name):
                    env,c,_,_=environment(route,source.profile,heading=0,lateral=200,reverse=True)
                    env=ActionEnvironment(env.robot,env.context,transition_config=replace(
                        env.transition_config,moving_tag6_enabled=False))
                    env._controllers[source.profile]=c
                    env.data.update(ground_origin=object(),orange_origin=object(),
                        ground_reverse_done=True,orange_reverse_done=True,
                        ground_lateral_mm=200,orange_lateral_mm=200)
                    env.motion_planning.start(env,plan);env.context.current_action=source.name
                    recorder=RouteRecorder(env,source.profile);ROUTES[route](recorder,source.profile)
                    env.robot.field_pose=SimpleNamespace(valid=True,tag_solutions=['must not read'])
                    with patch('Strategy.optimizations.tag_approach.TagApproach',side_effect=AssertionError('moving Tag6 loaded')):
                        env.run_route(route,source.profile)
                    args=env.robot.chassis.follow_trajectory.call_args
                    self.assertIsNone(args.kwargs.get('guidance'))
                    self.assertEqual(args.args[0][-1],recorder.pose)
                    self.assertEqual(env.motion_planning._routes.approaches,{})
                    self.assertFalse(env.motion_planning.consume_completed(tag.name))
                    control=env.control(tag.profile)
                    control._align_delivery_tag_or_continue=Mock(return_value=True)
                    control._checked_move=Mock()
                    env._align_tag(tag)
                    env.bind(offset).enter()
                    control._align_delivery_tag_or_continue.assert_called_once()
                    cfg=control.config;build=tag.parameters['purpose']=='build'
                    control._checked_move.assert_called_once_with(
                        cfg.post_tag6_lateral_direction if build else cfg.post_tag_lateral_direction,
                        cfg.post_tag6_lateral_right_mm if build else cfg.post_tag_lateral_right_mm,
                        cfg.post_tag6_lateral_speed_mm_s if build else cfg.post_tag_lateral_speed_mm_s)
                    cases+=1
        self.assertEqual(cases,8)

    def test_enabled_configuration_attaches_module_and_disabled_configuration_clears_it(self):
        env,c,_,_=environment('ground_to_delivery','ground-1')
        enabled=ActionEnvironment(env.robot,env.context,transition_config=replace(
            env.transition_config,moving_tag6_enabled=True))
        enabled.motion_planning.start(enabled,PLANS['PlanA'])
        self.assertEqual(len(enabled.motion_planning._routes.approaches),6)
        routes=enabled.motion_planning._routes
        routes.completed_actions.add('old-tag')
        routes.configure(PLANS['PlanA'],moving_tag6_enabled=False)
        self.assertEqual(routes.approaches,{})
        self.assertFalse(routes.consume_completed('old-tag'))


if __name__=='__main__':unittest.main()
