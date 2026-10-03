"""Launch overrides exercised through real entry points and action execution."""

import contextlib
from dataclasses import replace
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import main
import task2_main
from Strategy.cli import load_execution_config
from Strategy.context import ExecutionContext
from Strategy.execution import ExecutionRuntime
from Strategy.flows.factory import ActionEnvironment
from Strategy.flows.model import ActionSpec
from Strategy.plans import PLANS, StrategyPlan
from Strategy.transition_config import TransitionConfig
from Strategy.transition_switches import TRANSITION_NAMES, TransitionSwitches, transition_enabled, transition_sites
from tests import test_transition_config as calibration_tests
from tests.test_registered_pickup_transitions import Fixture, action, calibration
from tests.test_registered_inspection_transitions import RegisteredInspectionReplay
from tests.test_strategy_composition import FakeActionSession, robot_fixture



class TransitionCliTests(unittest.TestCase):
    def configure(self, flags, plan='PlanA', document=None):
        with tempfile.TemporaryDirectory() as directory:
            if document is not None:
                path = Path(directory) / 'synthetic.json'
                path.write_text(json.dumps(document), encoding='utf-8')
                flags = ['--transition-config', str(path), *flags]
            args = main.parse_args([*flags, '--show-plan'])
            return load_execution_config(args, PLANS[plan])

    def preview(self, entry, args, expected=0):
        output, errors = io.StringIO(), io.StringIO()
        with patch('sys.argv', ['entry.py', *args]), patch.object(entry, 'Robot') as robot, \
                contextlib.redirect_stdout(output), contextlib.redirect_stderr(errors):
            self.assertEqual(entry.main(), expected, errors.getvalue())
            robot.assert_not_called()
        return output.getvalue(), errors.getvalue()

    def test_defaults_disable_every_transition_even_with_calibration_or_trial_parameters(self):
        document = calibration_tests.TransitionConfigTests().document()
        configs = (TransitionConfig(), self.configure([]), self.configure([], document=document),
                   self.configure(['--trial-optimizations']))
        for config in configs:
            self.assertFalse(config.alignments)
            self.assertFalse(config.motion_planning_enabled)
            for plan in ('PlanA', 'PlanB'):
                for name, source in transition_sites(PLANS[plan].steps):
                    with self.subTest(plan=plan, name=name, profile=source.profile, trial=config.trial_run):
                        self.assertFalse(transition_enabled(config, name, source))

    def test_each_of_seven_optimizations_can_be_enabled_alone(self):
        cases = [(name, ['--enable-transition', name]) for name in TRANSITION_NAMES]
        cases += [('motion-planning', ['--enable-motion-planning']),
                  ('fast-alignment', ['--enable-fast-alignment'])]
        for entry in (main, task2_main):
            plan = PLANS['PlanA' if entry is main else 'collect-mixed-1']
            for selected, flags in cases:
                with self.subTest(entry=entry.__name__, selected=selected):
                    config = load_execution_config(entry.parse_args(['--trial-optimizations', *flags, '--show-plan']), plan)
                    self.assertEqual(config.motion_planning_enabled, selected == 'motion-planning')
                    self.assertEqual(bool(config.alignments), selected == 'fast-alignment')
                    for name, source in transition_sites(plan.steps):
                        self.assertEqual(transition_enabled(config, name, source), name == selected)

    def test_all_seven_and_individual_disable_keep_other_choices(self):
        all_flags = ['--trial-optimizations', '--enable-transitions',
                     '--enable-motion-planning', '--enable-fast-alignment']
        config = self.configure(all_flags)
        self.assertTrue(config.motion_planning_enabled and config.alignments)
        self.assertTrue(all(transition_enabled(config, name, source)
                            for name, source in transition_sites(PLANS['PlanA'].steps)))
        for name in TRANSITION_NAMES:
            config = self.configure([*all_flags, '--disable-transition', name])
            self.assertTrue(config.motion_planning_enabled and config.alignments)
            for other, source in transition_sites(PLANS['PlanA'].steps):
                self.assertEqual(transition_enabled(config, other, source), other != name)
        for enable, disable in (('--enable-motion-planning', '--disable-motion-planning'),
                                ('--enable-fast-alignment', '--disable-fast-alignment')):
            config = self.configure([flag for flag in all_flags if flag != enable] + [disable])
            self.assertEqual(config.motion_planning_enabled, enable != '--enable-motion-planning')
            self.assertEqual(bool(config.alignments), enable != '--enable-fast-alignment')
            self.assertTrue(all(transition_enabled(config, name, source)
                                for name, source in transition_sites(PLANS['PlanA'].steps)))

    def test_alignment_conflicts_reject_before_hardware(self):
        for entry in (main, task2_main):
            with patch('sys.argv', ['entry.py', '--enable-fast-alignment', '--disable-fast-alignment']), \
                    patch.object(entry, 'Robot') as robot, contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    entry.main()
                self.assertEqual(error.exception.code, 2)
                robot.assert_not_called()
            self.preview(entry, ['--enable-fast-alignment', '--show-plan'], expected=2)

    def test_both_entry_points_preview_all_seven(self):
        for entry in (main, task2_main):
            output, _ = self.preview(entry, ['--trial-optimizations', '--enable-transitions',
                                            '--enable-motion-planning',
                                            '--enable-fast-alignment', '--show-plan'])
            self.assertIn('Motion planning: local route smoothing', output)
            self.assertIn('Fast pickup alignment: ground-1/orange', output)
            self.assertNotIn(': disabled', output.split('Action transitions (enabled still requires runtime conditions):')[1])

    def test_enable_all_enables_every_site_in_both_plans_and_entry_points(self):
        for plan in ('PlanA', 'PlanB'):
            config = self.configure(['--trial-optimizations', '--enable-transitions'], plan)
            sites = tuple(transition_sites(PLANS[plan].steps))
            self.assertEqual({name for name, _ in sites}, set(TRANSITION_NAMES))
            self.assertTrue(all(transition_enabled(config, name, source) for name, source in sites))
        for entry, flags in ((main, ['--strategy', 'PlanA']), (main, ['--strategy', 'PlanB']),
                             (task2_main, ['--variant', '1']), (task2_main, ['--variant', '2'])):
            output, _ = self.preview(entry, [*flags, '--trial-optimizations',
                                            '--enable-transitions', '--show-plan'])
            choices = output.split('Action transitions (enabled still requires runtime conditions):')[1]
            self.assertIn(': enabled', choices)
            self.assertNotIn(': disabled', choices)

    def test_enable_all_with_calibration_and_scoped_overrides(self):
        document = calibration_tests.TransitionConfigTests().document()
        flags = ['--enable-transitions', '--disable-transition', 'next-cube',
                 '--enable-transition', 'next-cube@ground-1']
        config = self.configure(flags, 'collect-orange-1', document)
        self.assertTrue(all(transition_enabled(config, name, source)
                            for name, source in transition_sites(PLANS['collect-orange-1'].steps)))
        config = self.configure(['--trial-optimizations', '--enable-transitions',
                                 '--disable-transition', 'next-cube',
                                 '--enable-transition', 'next-cube@ground-1',
                                 '--disable-transition', 'inspect-departure@ground-2'])
        for name, source in transition_sites(PLANS['PlanA'].steps):
            expected = not ((name == 'next-cube' and source.profile != 'ground-1') or
                            (name == 'inspect-departure' and source.profile == 'ground-2'))
            self.assertEqual(transition_enabled(config, name, source), expected)

    def test_conflicting_group_flags_reject_before_robot_creation(self):
        for entry in (main, task2_main):
            with patch('sys.argv', ['entry.py', '--enable-transitions', '--disable-transitions']), \
                    patch.object(entry, 'Robot') as robot, contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    entry.main()
                self.assertEqual(error.exception.code, 2)
                robot.assert_not_called()

    def test_group_off_then_global_and_scoped_overrides_are_order_independent(self):
        flags = ['--disable-transitions', '--enable-transition', 'inspect-departure',
                 '--disable-transition', 'inspect-departure@ground-2',
                 '--enable-transition', 'build-return@building-1']
        config = self.configure(flags)
        enabled = {(name, source.profile) for name, source in transition_sites(PLANS['PlanA'].steps)
                   if transition_enabled(config, name, source)}
        self.assertEqual(enabled, {('inspect-departure', p) for p in
                                  ('ground-1', 'ground-3', 'highland-1', 'highland-2')}
                         | {('build-return', 'building-1')})
        reordered = self.configure(flags[5:] + flags[:5])
        self.assertEqual(config.switches, reordered.switches)

    def test_cli_enable_overrides_false_flag_but_preserves_calibration(self):
        document = calibration_tests.TransitionConfigTests().document()
        document['pickups']['ground-1/grap3']['last_departure'] = False
        config = self.configure(['--enable-transition', 'last-departure@ground-1'],
                                'collect-orange-1', document)
        site = next(s for n, s in transition_sites(PLANS['collect-orange-1'].steps)
                    if n == 'last-departure')
        self.assertTrue(transition_enabled(config, 'last-departure', site))
        self.assertFalse(config.pickup('ground-1', 'grap3').last_departure)
        self.assertTrue(config.pickup('ground-1', 'grap3').next_blind.validated)

    def test_missing_calibration_and_bad_selectors_reject_before_robot_creation(self):
        flags = [
            ['--enable-transition', name] for name in ('next-cube', 'last-departure', 'purple-departure')
        ] + [
            ['--enable-transitions'],
            ['--disable-transition', 'unknown'],
            ['--disable-transition', 'build-return@ground-1'],
            ['--disable-transition', 'next-cube@'],
            ['--disable-transition', 'inspect-departure', '--enable-transition', 'inspect-departure'],
        ]
        for values in flags:
            with self.subTest(values=values):
                self.preview(main, [*values, '--show-plan'], expected=2)

    def test_explicit_next_cube_requires_both_motion_profiles(self):
        document = calibration_tests.TransitionConfigTests().document()
        del document['pickups']['ground-1/grap3']['next_cube']
        with self.assertRaisesRegex(ValueError, 'blind/acquire'):
            self.configure(['--enable-transition', 'next-cube@ground-1'], 'collect-orange-1', document)

    def test_disabling_transitions_preserves_curve_and_alignment_choices(self):
        from Strategy.optimizations.trial import trial_configuration
        with patch('Strategy.optimizations.trial.trial_configuration', wraps=trial_configuration):
            config = self.configure(['--trial-optimizations', '--enable-fast-alignment', '--disable-transitions'])
        self.assertTrue(config.trial_run)
        self.assertTrue(config.alignments)
        self.assertTrue(config.pickups)
        self.assertFalse(any(transition_enabled(config, name, source)
                             for name, source in transition_sites(PLANS['PlanA'].steps)))
        document = calibration_tests.TransitionConfigTests().document()
        config = self.configure(['--disable-transitions', '--enable-motion-planning'], document=document)
        self.assertTrue(config.curves)
        self.assertTrue(config.motion_planning_enabled)

    def test_trial_configuration_keeps_its_explicit_gate_and_accepts_overrides(self):
        output, _ = self.preview(main, ['--trial-optimizations', '--enable-transitions', '--disable-transition',
                                       'next-cube@ground-1', '--show-plan'])
        self.assertIn('FIELD TRIAL', output)
        self.assertIn('next-cube@ground-1: disabled', output)
        self.assertIn('next-cube@ground-2: enabled', output)

    def test_both_entry_points_preview_effective_selections_without_hardware(self):
        for entry, base in ((main, ['--strategy', 'PlanB']),
                            (task2_main, ['--variant', '2'])):
            with self.subTest(entry=entry.__name__):
                output, _ = self.preview(entry, [*base, '--disable-transitions',
                                                '--enable-transition', 'inspect-departure@highland-2',
                                                '--show-plan'])
                self.assertIn('inspect-departure@highland-2: enabled', output)
                self.assertIn('purple-departure@highland-2: disabled', output)

    def test_both_entry_points_preview_defaults_without_hardware(self):
        for entry in (main, task2_main):
            output, _ = self.preview(entry, ['--show-plan'])
            choices = output.split('Action transitions (enabled still requires runtime conditions):')[1]
            self.assertIn(': disabled', choices)
            self.assertNotIn(': enabled', choices)

    def test_task2_forwards_choices_to_runner_and_preflight_still_skips_execution(self):
        for preflight in (False, True):
            args = ['--variant', '2', '--disable-transitions'] + (['--preflight-only'] if preflight else [])
            with patch('sys.argv', ['task2_main.py', *args]), patch('task2_main.Robot') as constructor, \
                    patch('task2_main.run_selection', return_value=0) as run:
                robot = constructor.return_value
                robot.connect.return_value = True
                robot.hardware_preflight.return_value.ok = True
                self.assertEqual(task2_main.main(), 0)
                if preflight:
                    run.assert_not_called()
                    self.assertFalse(constructor.call_args.kwargs['execution_extensions'])
                else:
                    self.assertEqual(run.call_args.args[1], 'collect-mixed-2')
                    self.assertTrue(run.call_args.kwargs['transition_config'].switches.disable_all)
                robot.stop.assert_called_once()


class TransitionExecutionSwitchTests(unittest.TestCase):
    def select(self, env, name, enabled, *, disable_all=False):
        env.transition_config = replace(env.transition_config,
            switches=TransitionSwitches() if enabled is None else
            TransitionSwitches(disable_all, {name: enabled}))

    def test_next_cube_off_waits_for_grab_and_runs_ordinary_search_once(self):
        for enabled in (None, False, True):
            f = Fixture()
            self.select(f.env, 'next-cube@ground-1', enabled, disable_all=True)
            steps = [action('grab_cube', 'grab.1', method='grap3', index=1),
                     action('acquire_cube', 'find.2', index=2)]
            f.run(steps)
            if enabled:
                f.warm.assert_called_once()
                f.control._find_cube.assert_not_called()
            else:
                f.warm.assert_not_called()
                f.control._find_cube.assert_called_once()
                phases = [(e.action, e.phase) for e in f.runtime.trace if e.status == 'completed']
                self.assertLess(phases.index(('grab.1', 'exit')), phases.index(('find.2', 'enter')))
                self.assertIn(('grab.1', 'stop'), phases)
            self.assertTrue(f.sessions[0].done and f.sessions[0].closed)

    def test_last_departure_overrides_flag_and_handles_skipped_third_slot(self):
        for enabled in (False, True):
            f = Fixture(profile='highland-1', method='grap1', policy=calibration(last_departure=not enabled))
            f.env.data['purple_grabbed'] = True
            self.select(f.env, 'last-departure', enabled, disable_all=True)
            f.run([
                action('grab_cube', 'grab.2', f.profile, method='grap1', index=2, conditional_on_purple=True),
                action('acquire_cube', 'find.3', f.profile, index=3, conditional_on_purple=True),
                action('grab_cube', 'grab.3', f.profile, method='grap1', index=3, conditional_on_purple=True),
                action('inspect_cargo', 'inspect', f.profile, method='grap1', exit_route='orange_depart_reverse'),
            ])
            self.assertEqual(len(f.sessions), 1)
            self.assertEqual(bool([e for e in f.events if e[:2] == ('move', 'backward')]), enabled)
            self.assertTrue(f.sessions[0].done and f.sessions[0].closed)

    def test_purple_departure_switch_changes_order_without_repeating_route(self):
        for enabled in (False, True):
            f = Fixture(profile='highland-1', method='grap2', color='purple',
                        policy=calibration(next_blind=None, next_acquire=None, departure=not enabled))
            self.select(f.env, 'purple-departure', enabled)
            steps = [action('grab_cube', 'purple.grab', f.profile, method='grap2', followup_route='purple_to_orange'),
                     action('navigate', 'purple.leave', f.profile, route='purple_to_orange')]
            route = Mock(side_effect=lambda *args: self.assertEqual(f.sessions[0].done, not enabled))
            with patch.dict('Strategy.flows.factory.ROUTES', {'purple_to_orange': route}):
                f.run(steps)
            route.assert_called_once()
            self.assertTrue(f.sessions[0].done and f.sessions[0].closed)

    def test_inspection_off_restores_before_route_on_overlaps_only_retreat(self):
        for enabled in (False, True):
            for count in (3, None):
                replay = RegisteredInspectionReplay([count])
                self.select(replay.env, 'inspect-departure', enabled, disable_all=True)
                self.assertEqual(replay.run(), 0)
                self.assertIn(('route', replay.route, False), replay.events)
                self.assertEqual(len([e for e in replay.events if e[0] == 'route']), 2 if enabled else 1)
                if enabled:
                    self.assertIn(('route', replay.exit_route, True), replay.events)
                else:
                    self.assertLess(replay.events.index(('inspect.close', 0)),
                                    replay.events.index(('route', replay.route, False)))

    def test_build_switch_waits_for_done_when_off_and_always_joins(self):
        for enabled in (None, False, True):
            robot = robot_fixture()
            context = ExecutionContext(robot, heading_zero_deg=37)
            env = ActionEnvironment(robot, context)
            self.select(env, 'build-return', enabled, disable_all=True)
            session = FakeActionSession()
            robot.actions.begin = Mock(return_value=session)
            env.run_route = Mock(side_effect=lambda *args: self.assertEqual(session.done, not enabled))
            plan = StrategyPlan('build-followup', (
                ActionSpec('build', 'build', 'building-1'),
                ActionSpec('navigate', 'return', 'building-1', {'route': 'build_return', 'after_build': True}),
            ))
            runtime = ExecutionRuntime(guard=context.check_active, stop=env.stop,
                                       emergency_stop=robot.transport.emergency_stop, close=env.abort)
            runtime.run(env.compile(plan))
            env.run_route.assert_called_once()
            self.assertTrue(session.done and session.closed)

    def test_default_inspection_restores_before_transport_without_reverse_overlap(self):
        replay = RegisteredInspectionReplay([3])
        self.select(replay.env, 'inspect-departure', None)
        self.assertEqual(replay.run(), 0)
        self.assertEqual(replay.env.run_route.call_count, 1)
        self.assertLess(replay.events.index(('inspect.close', 0)),
                        replay.events.index(('route', replay.route, False)))

    def test_disabled_overlap_does_not_mask_route_fault_or_restart(self):
        replay = RegisteredInspectionReplay([3], route_fault='exception')
        self.select(replay.env, 'inspect-departure', False)
        with self.assertRaisesRegex(RuntimeError, 'route failed'):
            replay.run()
        self.assertEqual(replay.env.run_route.call_count, 1)
        self.assertIn(('inspect.close', 0), replay.events)
        self.assertTrue(replay.context.closed)
        replay.robot.transport.emergency_stop.assert_called()


if __name__ == '__main__':
    unittest.main()
