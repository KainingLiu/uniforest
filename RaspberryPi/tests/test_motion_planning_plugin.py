"""Exercise backend selection at the actual competition route dispatch seam."""
from dataclasses import replace
from types import SimpleNamespace
import contextlib
import io
import unittest
from unittest.mock import Mock, patch
from Strategy.optimizations.motion_planning import MotionPlanning, PreparedMotion, FieldNavigation
from Strategy.transition_config import TransitionConfig
from Strategy.navigation import Location, Pose
from tests.test_curve_routes import fixture


class MotionPluginTests(unittest.TestCase):
    def test_explicit_enable_and_master_off_calls_original_route_without_optimizer(self):
        env,_,_,_=fixture('depart_a','depart-a')
        self.assertTrue(env.transition_config.motion_planning_enabled)
        env.run_route('depart_a','depart-a')
        env.robot.chassis.follow_trajectory.assert_called_once()
        env,_,_,_=fixture('depart_a','depart-a')
        env.transition_config=replace(env.transition_config,motion_planning_enabled=False)
        with patch('Strategy.flows.factory.ROUTES',{'depart_a':Mock(return_value='classic')}) as routes:
            self.assertEqual(env.run_route('depart_a','depart-a'),'classic')
            routes['depart_a'].assert_called_once_with(env,'depart-a')
        env.robot.chassis.follow_trajectory.assert_not_called()

    def test_replacement_backend_is_used_and_failure_never_replays_classic(self):
        env,_,_,_=fixture('depart_a','depart-a')
        for fault in (False,True):
            run=Mock(return_value='new',side_effect=RuntimeError('active failure') if fault else None)
            backend=SimpleNamespace(prepare=Mock(return_value=PreparedMotion('replacement',run)))
            env.motion_planning=MotionPlanning(optimizers=[backend])
            with patch('Strategy.flows.factory.ROUTES',{'depart_a':Mock()}) as routes:
                if fault:
                    with self.assertRaisesRegex(RuntimeError,'active failure'):env.run_route('depart_a','depart-a')
                else:self.assertEqual(env.run_route('depart_a','depart-a'),'new')
                routes['depart_a'].assert_not_called()
            run.assert_called_once()

    def test_field_backend_prepares_actual_pose_before_any_motion(self):
        env,_,_,_=fixture('depart_a','depart-a')
        adapter=Mock(); adapter.prepare.return_value='planned'; adapter.execute.return_value='done'
        source,target,pose=Location('start'),Location('orange_highland'),Pose(650,6500,270)
        backend=FieldNavigation({'depart-a/depart_a':lambda env:(source,target,pose)},lambda env:adapter)
        prepared=backend.prepare(env,'depart_a','depart-a')
        adapter.execute.assert_not_called()
        adapter.prepare.assert_called_once_with(source,target,actual_start=pose)
        self.assertEqual(prepared.execute(),'done')
        adapter.execute.assert_called_once_with('planned')
        self.assertIsNone(backend.prepare(env,'unknown','depart-a'))

    def test_cli_switch_is_visible_and_does_not_construct_hardware(self):
        import main
        self.assertFalse(main.parse_args([]).classic_motion)
        with patch('sys.argv',['main.py','--strategy','PlanB','--classic-motion','--show-plan']),patch('main.Robot') as robot,contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(main.main(),0)
        self.assertIn('classic (optimizers disabled)',output.getvalue())
        robot.assert_not_called()

    def test_default_config_and_cli_disable_planning_even_with_loaded_curves(self):
        import main
        from Strategy.cli import load_execution_config
        from Strategy.plans import PLANS
        self.assertFalse(TransitionConfig().motion_planning_enabled)
        configured = TransitionConfig(motion_planning_enabled=True, curves={'fixture': object()})
        with patch('Strategy.cli.TransitionConfig.load', return_value=configured):
            config = load_execution_config(main.parse_args(['--transition-config', 'synthetic.json']), PLANS['PlanA'])
        self.assertFalse(config.motion_planning_enabled)
        self.assertTrue(config.curves)

    def test_both_entry_points_enable_planning_and_reject_conflicting_flags(self):
        import main
        import task2_main
        for entry in (main, task2_main):
            for flags, expected in (([], 'classic (optimizers disabled)'),
                                    (['--enable-motion-planning'], 'field curve planner; existing route endpoints; CompetitionMotion + PositionTracker; CAD map + encoder/IMU')):
                with patch('sys.argv', ['entry.py', *flags, '--show-plan']), \
                        patch.object(entry, 'Robot') as robot, contextlib.redirect_stdout(io.StringIO()) as output:
                    self.assertEqual(entry.main(), 0)
                    self.assertIn('Motion planning: ' + expected, output.getvalue())
                    robot.assert_not_called()
            with patch('sys.argv', ['entry.py', '--classic-motion', '--enable-motion-planning']), \
                    patch.object(entry, 'Robot') as robot, contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    entry.main()
                self.assertEqual(error.exception.code, 2)
                robot.assert_not_called()

    def test_explicit_curve_backend_obeys_cli_master_switch(self):
        import main
        from Strategy.cli import load_execution_config
        from Strategy.plans import PLANS
        for flags in ([], ['--enable-motion-planning']):
            env, _, _, _ = fixture('depart_a', 'depart-a')
            configured = replace(env.transition_config, motion_planning_enabled=False)
            with patch('Strategy.cli.TransitionConfig.load', return_value=configured):
                env.transition_config = load_execution_config(
                    main.parse_args(['--transition-config', 'synthetic.json', *flags]), PLANS['PlanA'])
            with patch('Strategy.flows.factory.ROUTES', {'depart_a': Mock(return_value='classic')}) as routes:
                env.run_route('depart_a', 'depart-a')
                self.assertEqual(env.robot.chassis.follow_trajectory.call_count, int(bool(flags)))
                self.assertEqual(routes['depart_a'].call_count, int(not flags))

    def test_config_rejects_non_boolean_master_switch(self):
        for bad in (1,'false',None):
            with self.assertRaises(ValueError):TransitionConfig(motion_planning_enabled=bad)
