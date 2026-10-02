"""Independent whole-match contracts: continuity, provenance and source coverage.

These assertions describe observable replay results; they do not duplicate the
simulator's route expansion, inventory mutation or interpolation algorithms.
"""

from collections import Counter
import math
import unittest

from Strategy.plans import PLAN_A, PLAN_B
from simulation.core import SimulationSettings
from simulation.match import run_match


def pose(value):
    return tuple(value.get(long,value.get(short))
                 for long,short in (('x_mm','x'),('y_mm','y'),('yaw_deg','yaw')))


class MatchContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.settings = SimulationSettings()
        cls.matches = {plan.name:run_match(plan.name,settings=cls.settings,max_points=1200)
                       for plan in (PLAN_A,PLAN_B)}

    def test_each_source_action_has_exactly_one_ordered_event(self):
        for plan in (PLAN_A,PLAN_B):
            events = self.matches[plan.name]['events']
            with self.subTest(plan=plan.name):
                self.assertEqual([e['name'] for e in events],[s.name for s in plan.steps])
                self.assertEqual([e['kind'] for e in events],[s.kind for s in plan.steps])
                self.assertEqual([e['profile'] for e in events],[s.profile for s in plan.steps])
                self.assertEqual([e['index'] for e in events],list(range(len(plan.steps))))

    def test_action_boundaries_share_time_and_pose_without_relocation(self):
        for name,result in self.matches.items():
            events = result['events']
            for previous,following in zip(events,events[1:]):
                with self.subTest(plan=name,action=following['name']):
                    self.assertAlmostEqual(previous['t1'],following['t0'],places=5)
                    for before,after in zip(pose(previous['end_pose']),pose(following['start_pose'])):
                        self.assertAlmostEqual(before,after,places=5)
            for event in events:
                self.assertGreaterEqual(event['t1'],event['t0'])
                if event['t1'] == event['t0']:
                    self.assertEqual(pose(event['start_pose']),pose(event['end_pose']))

    def test_trace_motion_is_bounded_by_elapsed_virtual_time(self):
        # Chord distance cannot exceed travelled distance even after display
        # downsampling. The small allowance accounts for output rounding only.
        for name,result in self.matches.items():
            points = result['points']
            self.assertGreater(len(points),2)
            for a,b in zip(points,points[1:]):
                with self.subTest(plan=name,t=b['t']):
                    dt = b['t']-a['t']
                    self.assertGreaterEqual(dt,0)
                    travel = math.hypot(b['x']-a['x'],b['y']-a['y'])
                    turn = abs(b['yaw']-a['yaw'])
                    self.assertLessEqual(travel,self.settings.max_speed_mm_s*dt+1.0)
                    self.assertLessEqual(turn,self.settings.max_yaw_speed_deg_s*dt+.2)

    def test_every_cube_has_exactly_one_owner_in_every_inventory_frame(self):
        for name,result in self.matches.items():
            initial = [cube['id'] for cube in result['initial_cubes']]
            self.assertEqual(len(initial),len(set(initial)))
            for index,frame in enumerate(result['state_frames']):
                with self.subTest(plan=name,frame=index):
                    owned = list(frame['remaining'])+list(frame['cargo'])+list(frame['lost'])
                    owned += [cube for stack in frame['placements'] for cube in stack['cubes']]
                    owned += [cube for building in frame['buildings'] for cube in building['layers']]
                    self.assertEqual(Counter(owned),Counter(initial))
                    self.assertLessEqual(len(frame['cargo']),3)

    def test_event_cargo_comes_from_referenced_state_and_has_capacity(self):
        for name,result in self.matches.items():
            frames = result['state_frames']
            for event in result['events']:
                with self.subTest(plan=name,action=event['name']):
                    before,after = frames[event['state_before']],frames[event['state_after']]
                    self.assertEqual(event['cargo_before'],before['cargo'])
                    self.assertEqual(event['cargo_after'],after['cargo'])
                    self.assertLessEqual(len(event['cargo_before']),3)
                    self.assertLessEqual(len(event['cargo_after']),3)
                    if event['kind'] in ('navigate','anchor_wall','rebase_heading','align_tag',
                                         'align_building','begin_collection','acquire_cube','inspect_cargo'):
                        self.assertEqual(event['cargo_before'],event['cargo_after'])

    def test_pickup_and_unload_preserve_material_provenance(self):
        for name,result in self.matches.items():
            frames = result['state_frames']
            for event in result['events']:
                before,after = frames[event['state_before']],frames[event['state_after']]
                with self.subTest(plan=name,action=event['name']):
                    if event['kind'] == 'grab_cube':
                        new_cargo = set(after['cargo'])-set(before['cargo'])
                        self.assertLessEqual(len(new_cargo),1)
                        self.assertLessEqual(new_cargo,set(before['remaining']))
                        self.assertTrue(new_cargo.isdisjoint(after['remaining']))
                    elif event['kind'] == 'unload':
                        unloaded = set(before['cargo'])-set(after['cargo'])
                        placed_after = {c for p in after['placements'] for c in p['cubes']}
                        self.assertLessEqual(unloaded,placed_after)

    def test_successful_purple_pickup_skips_third_orange_slot(self):
        for plan in (PLAN_A,PLAN_B):
            purple_succeeded = False
            skipped = 0
            events = self.matches[plan.name]['events']
            for spec,event in zip(plan.steps,events):
                if spec.kind == 'begin_collection' and spec.parameters.get('color') == 'purple':
                    purple_succeeded = False
                if spec.kind == 'grab_cube' and spec.parameters.get('method') == 'grap2':
                    purple_succeeded = len(event['cargo_after']) > len(event['cargo_before'])
                if (purple_succeeded and spec.parameters.get('conditional_on_purple')
                        and spec.parameters.get('index') == 3):
                    with self.subTest(plan=plan.name,action=spec.name):
                        self.assertTrue(event['status'].startswith('skipped'),event['status'])
                        self.assertEqual(event['cargo_before'],event['cargo_after'])
                        self.assertEqual(pose(event['start_pose']),pose(event['end_pose']))
                        skipped += 1
            self.assertGreater(skipped,0,'nominal replay must exercise purple-dependent slots')

    def test_unmeasured_observation_and_mechanism_steps_declare_assumptions(self):
        assumed_kinds = {'anchor_wall','acquire_cube','grab_cube','inspect_cargo',
                         'align_tag','align_building','unload','load_staged','build'}
        for name,result in self.matches.items():
            self.assertIs(result['simulation_only'],True)
            self.assertTrue(result['assumptions'])
            for event in result['events']:
                if event['kind'] in assumed_kinds and not event['status'].startswith('skipped'):
                    with self.subTest(plan=name,action=event['name']):
                        self.assertTrue(event['assumptions'],'nominal evidence/timing must be identified')

    def test_load_and_build_transfer_existing_material_only(self):
        for name,result in self.matches.items():
            frames = result['state_frames']
            for event in result['events']:
                if event['kind'] not in ('load_staged','build'):
                    continue
                before,after = frames[event['state_before']],frames[event['state_after']]
                staged_before = {c for p in before['placements'] for c in p['cubes']}
                staged_after = {c for p in after['placements'] for c in p['cubes']}
                built_before = {c for b in before['buildings'] for c in b['layers']}
                built_after = {c for b in after['buildings'] for c in b['layers']}
                with self.subTest(plan=name,action=event['name']):
                    if event['kind'] == 'load_staged':
                        new_cargo = set(after['cargo'])-set(before['cargo'])
                        self.assertLessEqual(new_cargo,staged_before)
                        self.assertTrue(new_cargo.isdisjoint(staged_after))
                    else:
                        new_build = built_after-built_before
                        self.assertLessEqual(new_build,set(before['cargo'])|staged_before)
                        self.assertTrue(new_build.isdisjoint(set(after['cargo'])|staged_after))


if __name__ == '__main__':
    unittest.main()
