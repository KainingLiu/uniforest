"""Production follower/routes exercised in the isolated virtual-time simulator."""
import json
import math
import unittest
from unittest.mock import patch

from control.trajectory import Waypoint
from Strategy.flows.curves import CURVE_ROUTES
from Strategy.flows.factory import ROUTE_PROFILES
from Strategy.plans import PLAN_A, PLAN_B
from simulation import SimulationSettings, build_catalog, run_case, run_catalog
from simulation.core import MecanumPlant


class MotionSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.catalog = build_catalog()
        cls.results = run_catalog(max_points=100)

    def case(self, key):
        return next(case for case in self.catalog if case['id'] == key)

    def result(self, key):
        return next(case for case in self.results['cases'] if case['id'] == key)

    def test_catalog_covers_every_legal_pair_and_every_plan_occurrence(self):
        expected = {f'{profile}/{route}' for route in CURVE_ROUTES
                    for profile in ROUTE_PROFILES[route]}
        self.assertEqual({c['id'] for c in self.catalog}, expected)
        self.assertEqual(self.results['curve_family_count'], 15)
        self.assertEqual(self.results['case_count'], len(expected))
        for plan in (PLAN_A, PLAN_B):
            expected_steps = {step.name for step in plan.steps if step.kind == 'navigate'
                              and step.parameters['route'] in CURVE_ROUTES}
            reported = {name for case in self.catalog for name in case['plan_steps'][plan.name]}
            self.assertEqual(reported, expected_steps)

    def test_all_modes_actually_run_and_export_bounded_finite_traces(self):
        self.assertEqual(self.results['layer_count'], len(self.catalog)*3)
        for case in self.results['cases']:
            self.assertEqual({l['id'] for l in case['layers']},
                             {'stop_turn', 'rounded_corners', 'endpoint_shortcut'})
            for layer in case['layers']:
                with self.subTest(case=case['id'], layer=layer['id']):
                    self.assertEqual(layer['status'], 'completed', layer['error'])
                    self.assertTrue(layer['simulation_only'])
                    self.assertFalse(layer['field_validated'])
                    self.assertGreater(len(layer['points']), 2)
                    self.assertLessEqual(len(layer['points']), 100)
                    self.assertGreater(layer['metrics']['elapsed_s'], 0)
                    self.assertGreater(layer['metrics']['controller_runs'], 0)
                    self.assertGreater(layer['metrics']['path_length_mm'], 0)
                    self.assertEqual(layer['points'][0]['t'], 0)
                    self.assertAlmostEqual(layer['points'][-1]['t'], layer['metrics']['elapsed_s'], places=3)
                    self.assertTrue(all(math.isfinite(value) for p in layer['points'] for value in p.values()))
                    self.assertTrue(all(b['t'] > a['t'] for a, b in zip(layer['points'], layer['points'][1:])))
                    self.assertLessEqual(layer['metrics']['max_wheel_rpm'], 3000)
        payload = json.dumps(self.results, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
        self.assertLess(len(payload.encode('utf-8')), 850_000)

    def test_simulation_never_constructs_transport_or_waits_for_wall_clock(self):
        with patch('protocol.transport.Transport.__init__', side_effect=AssertionError('hardware construction')), \
                patch('serial.Serial', side_effect=AssertionError('serial opened')), \
                patch('time.sleep', side_effect=AssertionError('wall-clock wait')):
            result = run_case(self.case('depart-b/depart_b'))
        self.assertTrue(all(l['status'] == 'completed' for l in result['layers']))

    def test_waypoint_candidates_and_stop_reference_are_distinct(self):
        case = self.result('depart-b/depart_b')
        layers = {l['id']: l for l in case['layers']}
        self.assertEqual([s['kind'] for s in case['original_steps']], ['move', 'move', 'turn'])
        self.assertEqual(len(layers['rounded_corners']['waypoints']), 4)
        self.assertEqual(len(layers['endpoint_shortcut']['waypoints']), 2)
        self.assertEqual(layers['stop_turn']['metrics']['controller_runs'], 3)
        self.assertEqual(layers['rounded_corners']['metrics']['controller_runs'], 1)
        self.assertGreater(layers['rounded_corners']['metrics']['path_length_mm'],
                           layers['endpoint_shortcut']['metrics']['path_length_mm'])
        self.assertIn('legacy PID timing not replayed', case['comparison_basis'])

    def test_wall_and_postwall_staged_legs_are_preserved_and_labelled_omitted(self):
        case = self.result('staged-building/staged_return_first')
        self.assertEqual([s['kind'] for s in case['original_steps']], ['move', 'move', 'wall', 'move'])
        self.assertEqual(case['nominal_endpoint'], {'x_mm': -100.0, 'y_mm': -840.0, 'yaw_deg': 0.0})
        self.assertEqual(case['nominal_final_pose'], {'x_mm': -100.0, 'y_mm': -540.0, 'yaw_deg': 0.0})
        for layer in case['layers']:
            walls = [b for b in layer['barriers'] if b['kind'] == 'wall']
            self.assertEqual(len(walls), 1)
            self.assertEqual(walls[0]['direction'], 'left')
            self.assertFalse(walls[0]['simulated_motion'])
            self.assertGreater(layer['metrics']['elapsed_s'], walls[0]['t'])

    def test_nominal_search_assumption_changes_live_endpoint_without_changing_source(self):
        key = 'ground-1/ground_to_delivery'
        default = self.case(key)
        changed = next(c for c in build_catalog(assumption_overrides={key: {
            'search_lateral_mm': 500.0, 'reverse_already_done': False}}) if c['id'] == key)
        self.assertAlmostEqual(default['nominal_endpoint']['x_mm'], 0, delta=1e-6)
        self.assertAlmostEqual(default['nominal_endpoint']['y_mm'], 2500, delta=1e-6)
        self.assertAlmostEqual(changed['nominal_endpoint']['x_mm'], -400, delta=1e-6)
        self.assertAlmostEqual(changed['nominal_endpoint']['y_mm'], 2300, delta=1e-6)
        self.assertEqual(changed['start_assumptions']['search_lateral_mm'], 500)
        self.assertIn('not measurements', changed['start_assumptions']['source'])

    def test_failure_keeps_partial_trace_and_aborts_all_three_independent_modes(self):
        result = run_case(self.case('depart-b/depart_b'), settings=SimulationSettings(fault_after_s=.15))
        for layer in result['layers']:
            self.assertEqual(layer['status'], 'failed')
            self.assertIn('injected simulation communication fault', layer['error'])
            self.assertGreater(len(layer['points']), 1)
            self.assertGreater(layer['metrics']['elapsed_s'], .1)
            self.assertEqual(layer['metrics']['emergency_stops'], 1)
            self.assertEqual(layer['metrics']['controller_runs'], 1)

    def test_timeout_remains_a_failed_layer_instead_of_aborting_catalog(self):
        result = run_case(self.case('ground-1/ground_tag_offset'),
                          settings=SimulationSettings(wheel_response_s=100, timeout_s=2))
        for layer in result['layers']:
            self.assertEqual(layer['status'], 'failed')
            self.assertTrue(layer['error'].startswith('TimeoutError:'), layer['error'])
            self.assertEqual(layer['metrics']['emergency_stops'], 1)

    def test_mecanum_odometry_uses_imu_wrap_and_slip_is_visible_only_in_truth(self):
        plant = MecanumPlant()
        plant.follow_local((Waypoint(0, 0, 0), Waypoint(0, 0, 270)))
        self.assertAlmostEqual(plant.yaw, 270, delta=1)
        self.assertAlmostEqual(plant.oyaw, 270, delta=1)
        ideal = MecanumPlant()
        slipping = MecanumPlant(SimulationSettings(lateral_traction=.75))
        for sim in (ideal, slipping):
            sim.follow_local((Waypoint(0, 0, 0), Waypoint(0, 600, 0)))
            self.assertAlmostEqual(sim.oy, 600, delta=6)
        self.assertAlmostEqual(ideal.y, 600, delta=6)
        self.assertAlmostEqual(slipping.y, 450, delta=6)
        self.assertGreater(slipping.metrics()['max_odometry_error_mm'], 140)
        self.assertLess(slipping.metrics()['max_tracking_error_mm'], 100)

    def test_results_are_deterministic_and_never_export_a_field_calibration(self):
        case = self.case('depart-a/depart_a')
        a, b = run_case(case), run_case(case)
        self.assertEqual(a, b)
        self.assertNotIn('curves', self.results)
        self.assertNotIn('pickups', self.results)
        self.assertFalse(self.results['field_validated'])
        self.assertTrue(self.results['simulation_only'])


if __name__ == '__main__':
    unittest.main()
