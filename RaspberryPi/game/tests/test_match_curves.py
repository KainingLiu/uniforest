"""Whole-plan continuous candidates use real route lowering and wheel control."""
from copy import deepcopy
import math
import unittest

from Strategy.plans import PLAN_A, PLAN_B
from simulation.core import SimulationSettings
from simulation.match import load_scenario, run_match


class MatchCurveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.settings = SimulationSettings()
        cls.scenario = load_scenario()
        cls.matches = {(p, m): run_match(p, route_mode=m, settings=cls.settings,
                                        scenario=deepcopy(cls.scenario))
                       for p in ('PlanA', 'PlanB') for m in ('stop_turn', 'continuous')}

    def test_four_whole_matches_retain_source_actions_and_expected_materials(self):
        for plan in (PLAN_A, PLAN_B):
            baseline, continuous = [self.matches[plan.name, m] for m in ('stop_turn', 'continuous')]
            for result in (baseline, continuous):
                self.assertEqual(result['status'], 'nominal_completed')
                self.assertEqual([e['name'] for e in result['events']], [s.name for s in plan.steps])
            self.assertEqual(baseline['initial_cubes'], continuous['initial_cubes'])
            self.assertEqual(baseline['start_pose'], continuous['start_pose'])
            self.assertEqual(baseline['metrics']['built_cube_count'], continuous['metrics']['built_cube_count'])
            self.assertEqual(baseline['metrics']['build_count'], continuous['metrics']['build_count'])
            self.assertEqual(baseline['metrics']['skipped_actions'], continuous['metrics']['skipped_actions'])

    def test_multi_leg_prefixes_have_real_moving_internal_nodes(self):
        for plan in ('PlanA', 'PlanB'):
            result = self.matches[plan, 'continuous']
            merged = [r for r in result['curve_runs'] if r['curve_enabled'] and r['multi_leg']]
            self.assertGreater(len(merged), 5)
            self.assertEqual(result['metrics']['curve_multi_leg_runs'], len(merged))
            for run in merged:
                self.assertGreater(len(run['source_motion_legs']), 1)
                self.assertGreater(len(run['waypoints']), 2)
                self.assertEqual(run['prefix_controller_runs'], 1)
                self.assertEqual(len(run['internal_nodes']), len(run['waypoints'])-2)
                self.assertTrue(all(n['moving'] for n in run['internal_nodes']))
            self.assertTrue(any(n['measured_speed_mm_s'] > 100 for r in merged for n in r['internal_nodes']))
        baseline = self.matches['PlanB', 'stop_turn']
        self.assertEqual(baseline['metrics']['curve_enabled_runs'], 0)

    def test_depart_b_combines_three_real_legs_without_changing_endpoint_semantics(self):
        baseline = self.matches['PlanB', 'stop_turn']['curve_runs'][0]
        curved = self.matches['PlanB', 'continuous']['curve_runs'][0]
        self.assertEqual(baseline['route'], 'depart_b')
        self.assertEqual(curved['controller_runs'], 1)
        self.assertEqual(baseline['controller_runs'], 3)
        self.assertEqual(curved['waypoints'][-1], {'x_mm': 900, 'y_mm': 2700, 'yaw_deg': 180})
        self.assertEqual(curved['start_pose'], baseline['start_pose'])
        self.assertLess(curved['prefix_elapsed_s'], baseline['t1']-baseline['t0'])

    def test_live_search_and_preexecuted_reverse_are_used_for_each_round(self):
        runs = self.matches['PlanA', 'continuous']['curve_runs']
        deliveries = [r for r in runs if r['route'] == 'ground_to_delivery']
        self.assertEqual(len(deliveries), 3)
        for run in deliveries:
            # Inspection already executes ground_delivery_reverse. A duplicated
            # retreat would add a fourth prefix leg and change this geometry.
            self.assertEqual([l['kind'] for l in run['source_motion_legs']], ['turn', 'move', 'turn'])
            self.assertEqual(run['prefix_controller_runs'], 1)
        distances = [next(l['distance_mm'] for l in r['source_motion_legs'] if l['kind']=='move')
                     for r in deliveries]
        self.assertGreater(max(distances)-min(distances), 100)
        purple = [r for r in runs if r['route'] == 'purple_to_orange']
        self.assertTrue(all([l['kind'] for l in r['source_motion_legs']] == ['move','turn','move'] for r in purple))

    def test_staged_wall_then_postwall_approach_remains_outside_curve_prefix(self):
        result = self.matches['PlanB', 'continuous']
        run = next(r for r in result['curve_runs'] if r['route'] == 'staged_return_first')
        self.assertEqual(run['prefix_controller_runs'], 1)
        self.assertGreaterEqual(run['controller_runs'], 2)
        self.assertEqual(run['waypoints'][-1], {'x_mm': -100, 'y_mm': -840, 'yaw_deg': 0})
        walls = [m for m in result['milestones'] if m['action']==run['action']
                 and m['stage']=='wall_target_reached']
        self.assertEqual(len(walls), 1)
        self.assertGreaterEqual(walls[0]['t'], run['prefix_end_s'])
        self.assertGreater(run['t1'], walls[0]['t'])

    def test_time_breakdown_is_exhaustive_and_keeps_nominal_corrections_separate(self):
        for result in self.matches.values():
            metrics = result['metrics']
            self.assertAlmostEqual(metrics['elapsed_s'], metrics['source_motion_elapsed_s']+
                                   metrics['nominal_correction_elapsed_s']+
                                   metrics['nominal_wait_elapsed_s'], places=4)
            self.assertGreater(metrics['nominal_correction_elapsed_s'], 0)
        for plan in ('PlanA', 'PlanB'):
            a, b = [self.matches[plan, m]['metrics'] for m in ('stop_turn', 'continuous')]
            self.assertAlmostEqual(a['nominal_wait_elapsed_s'], b['nominal_wait_elapsed_s'], places=4)

    def test_midcurve_failure_retains_actual_attempt_and_never_resumes(self):
        result = run_match('PlanB', route_mode='continuous', settings=SimulationSettings(fault_after_s=.15))
        self.assertEqual(result['status'], 'failed')
        curve = result['curve_runs'][0]
        self.assertTrue(curve['curve_enabled'])
        self.assertEqual(curve['status'], 'failed')
        self.assertEqual(curve['prefix_controller_runs'], 1)
        self.assertEqual(curve['controller_runs'], 1)
        self.assertEqual(len(result['curve_runs']), 1)
        self.assertTrue(all(e['status']=='skipped_after_failure' for e in result['events'][1:]))

    def test_modes_do_not_add_hidden_pose_resets_or_skip_anchor_travel(self):
        for plan in ('PlanA', 'PlanB'):
            result = self.matches[plan, 'continuous']
            for a, b in zip(result['points'], result['points'][1:]):
                dt = b['t']-a['t']
                self.assertGreaterEqual(dt, 0)
                self.assertLessEqual(math.hypot(b['x']-a['x'], b['y']-a['y']),
                                     self.settings.max_speed_mm_s*dt+1)
                self.assertLessEqual(abs(b['yaw']-a['yaw']), self.settings.max_yaw_speed_deg_s*dt+.2)
            for previous, following in zip(result['events'], result['events'][1:]):
                self.assertEqual(previous['end_pose'], following['start_pose'])
        with self.assertRaisesRegex(ValueError, 'route_mode'):
            run_match(route_mode='draw_only')


if __name__ == '__main__':
    unittest.main()
