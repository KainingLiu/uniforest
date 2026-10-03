"""Whole-match curve integration, checked through real production calls/output."""

from copy import deepcopy
import math
import unittest
from unittest.mock import patch

from Strategy.flows.curves import CURVE_ROUTES, run_curve as production_run_curve
from Strategy.plans import PLAN_A, PLAN_B
from Strategy.settings import PROFILES
from simulation.core import MecanumPlant, SimulationSettings
from simulation.match import load_scenario, run_match


class MatchCurveContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.settings = SimulationSettings()
        cls.scenario = load_scenario()
        cls.original_scenario = deepcopy(cls.scenario)
        cls.results,cls.curve_calls,cls.motion_calls = {},{},{}
        real_follow = MecanumPlant.follow_local
        for plan in (PLAN_A,PLAN_B):
            for mode in ('stop_turn','continuous'):
                calls = []
                def observed_follow(plant,points,*,profile=None,initial_velocity=None):
                    calls.append(dict(action=plant.action_index,stage=plant.stage,points=tuple(points)))
                    return real_follow(plant,points,profile=profile,initial_velocity=initial_velocity)
                with patch('simulation.match.run_curve',wraps=production_run_curve) as curve, \
                        patch.object(MecanumPlant,'follow_local',new=observed_follow):
                    result = run_match(plan.name,settings=cls.settings,scenario=cls.scenario,
                                       max_points=1200,route_mode=mode)
                key = (plan.name,mode)
                cls.results[key] = result
                cls.curve_calls[key] = list(curve.call_args_list)
                cls.motion_calls[key] = calls

    def test_curve_coverage_matches_actual_production_adapter_calls(self):
        for (name,mode),result in self.results.items():
            enabled = [r for r in result['curve_runs'] if r['curve_enabled']]
            with self.subTest(plan=name,mode=mode):
                self.assertEqual(result['status'],'nominal_completed',result['warnings'])
                self.assertEqual(len(self.curve_calls[(name,mode)]),len(enabled))
                self.assertEqual(result['metrics']['curve_enabled_runs'],len(enabled))
                self.assertEqual([call.args[1:3] for call in self.curve_calls[(name,mode)]],
                                 [(r['route'],r['profile']) for r in enabled])
                if mode == 'continuous':
                    self.assertGreater(len(enabled),0)
                    self.assertTrue(all(r['route'] in CURVE_ROUTES for r in enabled))
                else:
                    self.assertEqual(enabled,[])

    def test_departure_replaces_three_real_motion_calls_with_one_curve(self):
        # PlanB departure is three known source legs with no contact/vision
        # correction: this catches running the curve AND repeating old legs.
        stopped = [c for c in self.motion_calls[('PlanB','stop_turn')] if c['action'] == 0]
        curved = [c for c in self.motion_calls[('PlanB','continuous')] if c['action'] == 0]
        self.assertEqual(len(stopped),3)
        self.assertEqual(len(curved),1)
        self.assertEqual(len(curved[0]['points']),4)
        run = self.results[('PlanB','continuous')]['curve_runs'][0]
        self.assertEqual(run['route'],'depart_b')
        self.assertEqual(run['controller_runs'],1)
        self.assertEqual(run['prefix_controller_runs'],1)
        self.assertEqual(len(run['source_motion_legs']),3)

    def test_wall_barriers_and_postwall_staged_move_remain(self):
        for name in ('PlanA','PlanB'):
            baseline,continuous = self.results[(name,'stop_turn')],self.results[(name,'continuous')]
            def walls(match):
                return [(m['action'],m['direction']) for m in match['milestones']
                        if m['stage'] == 'wall_target_reached']
            self.assertEqual(walls(continuous),walls(baseline))
        result = self.results[('PlanB','continuous')]
        run = next(r for r in result['curve_runs'] if r['route'] == 'staged_return_first')
        calls = [c for c in self.motion_calls[('PlanB','continuous')] if c['action'] == run['action']]
        # The final source rightward motion remains after the left-wall barrier.
        tail = calls[-1]['points'][-1]
        self.assertAlmostEqual(tail.x_mm,0)
        self.assertAlmostEqual(tail.y_mm,PROFILES['staged-building'].second_load_right_mm)
        self.assertAlmostEqual(tail.yaw_deg,0)
        self.assertTrue(any(m['action']==run['action'] and m['stage']=='wall_target_reached'
                            and m['t'] >= run['prefix_end_s']-.001 for m in result['milestones']))

    def test_both_modes_share_scenario_and_do_not_mutate_caller_inputs(self):
        self.assertEqual(self.scenario,self.original_scenario)
        for name in ('PlanA','PlanB'):
            a,b = self.results[(name,'stop_turn')],self.results[(name,'continuous')]
            self.assertEqual(a['start_pose'],b['start_pose'])
            self.assertEqual(a['initial_cubes'],b['initial_cubes'])
            self.assertEqual(a['state_frames'][0],b['state_frames'][0])
            self.assertEqual([e['name'] for e in a['events']],[e['name'] for e in b['events']])
            self.assertEqual(a['metrics']['built_cube_count'],b['metrics']['built_cube_count'])

    def test_curve_mode_keeps_continuous_world_pose_across_actions(self):
        for name in ('PlanA','PlanB'):
            result = self.results[(name,'continuous')]
            for a,b in zip(result['events'],result['events'][1:]):
                self.assertEqual(a['end_pose'],b['start_pose'])
                self.assertEqual(a['t1'],b['t0'])
            for a,b in zip(result['points'],result['points'][1:]):
                dt = b['t']-a['t']
                self.assertGreaterEqual(dt,0)
                self.assertLessEqual(math.hypot(b['x']-a['x'],b['y']-a['y']),
                                     self.settings.max_speed_mm_s*dt+1)
                self.assertLessEqual(abs(b['yaw']-a['yaw']),self.settings.max_yaw_speed_deg_s*dt+.2)

    def test_total_elapsed_includes_nominal_corrections_and_waits(self):
        for key,result in self.results.items():
            with self.subTest(case=key):
                metrics = result['metrics']
                parts = (metrics['source_motion_elapsed_s'],metrics['nominal_correction_elapsed_s'],
                         metrics['nominal_wait_elapsed_s'])
                self.assertTrue(all(value >= 0 for value in parts))
                self.assertGreater(parts[1],0)
                self.assertGreater(parts[2],0)
                self.assertAlmostEqual(sum(parts),result['events'][-1]['t1'],places=4)
                self.assertAlmostEqual(metrics['elapsed_s'],result['points'][-1]['t'],places=2)
                self.assertAlmostEqual(sum(e['t1']-e['t0'] for e in result['events']),sum(parts),places=4)

    def test_curve_completion_never_claims_field_validation(self):
        for result in self.results.values():
            self.assertIs(result['simulation_only'],True)
            self.assertIs(result['field_validated'],False)
            self.assertIs(result['collision_checked'],False)
            self.assertTrue(any('collision' in warning.lower() for warning in result['warnings']))


if __name__ == '__main__':
    unittest.main()
