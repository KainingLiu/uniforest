"""Nominal full-match outcomes and failure scenarios, not field performance claims."""
from collections import Counter
from copy import deepcopy
import unittest
from unittest.mock import patch

from simulation.core import SimulationSettings
from simulation.match import load_scenario, run_match


class WholeMatchSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scenario = load_scenario()
        cls.matches = {name: run_match(name) for name in ('PlanA', 'PlanB')}

    def test_nominal_results_follow_actual_material_counts_and_layer_order(self):
        for name, buildings, remaining in (
                ('PlanA', 3, {'orange': 5}), ('PlanB', 2, {'orange': 10, 'purple': 1})):
            with self.subTest(plan=name):
                result = self.matches[name]
                self.assertEqual(result['status'], 'nominal_completed')
                lookup = {c['id']: c['color'] for c in result['initial_cubes']}
                final = result['state_frames'][-1]
                self.assertEqual(final['build_count'], buildings)
                self.assertEqual(Counter(lookup[i] for i in final['remaining']), Counter(remaining))
                self.assertEqual(final['cargo'], [])
                self.assertTrue(all(not p['cubes'] for p in final['placements']))
                for building in final['buildings']:
                    self.assertEqual([lookup[i] for i in building['layers']], ['orange']*5+['purple'])
                    self.assertTrue(building['assumed'])

    def test_building_grows_from_existing_three_cube_base_one_release_at_a_time(self):
        result = self.matches['PlanA']
        for event in (e for e in result['events'] if e['kind'] == 'build'):
            frames = result['state_frames'][event['state_before']:event['state_after']+1]
            before_ids = {b['id'] for b in frames[0]['buildings']}
            counts = []
            for frame in frames:
                for building in frame['buildings']:
                    if building['id'] not in before_ids:
                        count = len(building['layers'])
                        if not counts or counts[-1] != count:
                            counts.append(count)
            self.assertEqual(counts, [3, 4, 5, 6])
        self.assertEqual(sum(m['stage'] == 'build_cube_released' for m in result['milestones']), 9)

    def test_plan_b_load_retrieves_the_previous_mixed_deposit_order(self):
        result = self.matches['PlanB']
        frames = result['state_frames']
        for event in (e for e in result['events'] if e['kind'] == 'load_staged'):
            transfer = self.scenario['transfers']['PlanB'][event['name']]
            before = next(p for p in frames[event['state_before']]['placements']
                          if p['id'] == transfer['placement'])
            self.assertEqual(event['cargo_after'], before['cubes'])
            self.assertEqual(len(event['cargo_after']), 3)

    def test_empty_world_has_no_generated_cargo_or_buildings(self):
        scenario = deepcopy(self.scenario)
        scenario['resources'] = []
        result = run_match('PlanB', scenario=scenario)
        self.assertEqual(result['initial_cubes'], [])
        self.assertEqual(result['metrics']['build_count'], 0)
        for frame in result['state_frames']:
            self.assertEqual(frame['remaining']+frame['cargo']+frame['lost'], [])
            self.assertEqual(frame['buildings'], [])
            self.assertTrue(all(p['cubes'] == [] for p in frame['placements']))
        for event in result['events']:
            if event['kind'] in ('load_staged', 'build', 'grab_cube'):
                self.assertTrue(event['status'].startswith('skipped'), event)

    def test_missing_ground_bases_build_only_the_cargo_that_exists(self):
        scenario = deepcopy(self.scenario)
        scenario['resources'] = [c for c in scenario['resources'] if c['region'] != 'ground']
        result = run_match('PlanA', scenario=scenario)
        self.assertEqual(result['metrics']['built_cube_count'], 9)
        self.assertTrue(all(len(b['layers']) == 3 for b in result['state_frames'][-1]['buildings']))
        self.assertTrue(all(e['status'] == 'partial_build'
                            for e in result['events'] if e['kind'] == 'build'))

    def test_terminal_fault_preserves_entire_source_list_without_resuming(self):
        result = run_match('PlanA', settings=SimulationSettings(fault_after_s=.15))
        self.assertEqual(result['status'], 'failed')
        self.assertEqual(len(result['events']), 109)
        self.assertEqual(result['events'][0]['status'], 'failed')
        self.assertTrue(all(e['status'] == 'skipped_after_failure' for e in result['events'][1:]))
        self.assertTrue(all(e['t0'] == e['t1'] for e in result['events'][1:]))
        self.assertEqual(result['metrics']['build_count'], 0)
        self.assertEqual(result['state_frames'][-1]['remaining'], [c['id'] for c in result['initial_cubes']])

    def test_all_action_boundaries_survive_display_downsampling(self):
        for result in self.matches.values():
            self.assertLessEqual(len(result['points']), 1500)
            times = [p['t'] for p in result['points']]
            for event in result['events']:
                for boundary in (event['t0'], event['t1']):
                    self.assertTrue(any(abs(t-boundary) <= .00051 for t in times), event['name'])
            self.assertEqual(result['points'][0]['x'], result['start_pose']['x_mm'])
            self.assertEqual(result['points'][0]['y'], result['start_pose']['y_mm'])
            self.assertTrue(any(m['stage'] == 'assumed_anchor_correction' for m in result['milestones']))

    def test_no_hardware_construction_or_wall_clock_sleep(self):
        with patch('protocol.transport.Transport.__init__', side_effect=AssertionError('hardware')), \
                patch('serial.Serial', side_effect=AssertionError('serial')), \
                patch('time.sleep', side_effect=AssertionError('wall-clock sleep')):
            result = run_match('PlanB')
        self.assertEqual(result['status'], 'nominal_completed')


if __name__ == '__main__':
    unittest.main()
