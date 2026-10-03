"""Navigation contracts at the same interfaces used by the lab and adapter."""
from dataclasses import replace
import itertools
import json
import math
import random
import unittest
import numpy as np
from Strategy.navigation import Location, Pose, NavigationError
from Strategy.navigation.localization import Observation, VisualOdometry, compose
from Strategy.navigation.robot_adapter import NavigationCalibration
from simulation.navigation import load_planner, simulate


class PlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.planner=load_planner()

    def test_every_directed_family_pair_and_random_along_wall_positions(self):
        p=self.planner; rng=random.Random(812)
        for left,right in itertools.product(p.LABELS,repeat=2):
            for u in (0,.5,1):
                a,b=Location(left,u),Location(right,1-u)
                route=p.plan(a,b)
                self.assertGreaterEqual(float(p.clearance(route.samples[:,:3]).min()),-1e-6,(left,right,u))
                self.assertLess(np.linalg.norm(route.samples[-1,:2]-[route.goal.x,route.goal.y]),.001)
        for _ in range(80):
            a,b=(Location(rng.choice(list(p.LABELS)),rng.random(),rng.uniform(0,120)) for _ in range(2))
            route=p.plan(a,b)
            self.assertGreaterEqual(float(p.clearance(route.samples[:,:3]).min()),-1e-6)

    def test_start_to_highland_does_not_turn_then_reverse(self):
        route=self.planner.plan(Location('start'),Location('orange_highland'))
        self.assertLess(np.ptp(route.samples[:,2]),.001)
        self.assertTrue(np.any((route.samples[:,1]>4200)&(route.samples[:,1]<5000)))
        ramp=route.samples[(route.samples[:,1]>4200)&(route.samples[:,1]<5000)]
        self.assertTrue(np.all((ramp[:,0]>3000)&(ramp[:,0]<4000)))
        prefix=route.samples[route.samples[:,0]<1800,:2]
        delta=np.diff(prefix,axis=0)
        tangent=np.unwrap(np.arctan2(delta[:,1],delta[:,0]))
        self.assertLess(np.degrees((np.maximum.accumulate(tangent)-tangent).max()),3)
        self.assertGreater(route.candidate_count,1)
        self.assertGreater(route.blend_radius_mm,260)

    def test_shape_envelope_catches_rotating_corner_and_bad_endpoints(self):
        p=self.planner; a=Pose(2800,3300,270)
        self.assertLess(p.clearance([(a.x,a.y,a.yaw)])[0],0)
        for bad in (Location('unknown'),):
            with self.assertRaises(ValueError): p.resolve(bad)
        for value in (-.01,1.01,math.nan,True):
            with self.assertRaises(ValueError): Location('building',value)
        with self.assertRaises(NavigationError):
            p.plan(Location('start'),Location('building'),actual_start=Pose(2400,3600,0))

    def test_zero_gap_has_free_side_arrival_within_tolerance(self):
        r=self.planner.plan(Location('start'),Location('purple',.4,0))
        self.assertGreater(r.samples[-1,4],2.9)
        self.assertLess(np.linalg.norm(r.samples[-1,:2]-[r.goal.x,r.goal.y]),5)

    def test_north_wall_rotation_retains_a_clear_turn_bay(self):
        for a,b in ((Location('orange_highland',.5),Location('purple',0)),
                    (Location('purple',0),Location('orange_highland',.5))):
            r=self.planner.plan(a,b)
            self.assertGreaterEqual(self.planner.clearance(r.samples[:,:3]).min(),-1e-6)
            self.assertGreater(np.ptp(r.samples[:,2]),80)

    def test_nearly_facing_start_and_ground_goal_have_no_forced_corridor_excursion(self):
        p=self.planner; r=p.plan(Location('start'),Location('orange_ground',0))
        self.assertLess(r.samples[:,0].max()-max(r.start.x,r.goal.x),200)
        self.assertLess(np.linalg.norm(np.diff(r.samples[:,:2],axis=0),axis=1).sum(),1250)
        self.assertTrue(np.all(np.diff(r.samples[:,1])<=.001))

    def test_real_robot_requires_a_measured_record(self):
        with self.assertRaises(ValueError): NavigationCalibration(False,'',0).validate()

    def test_grab_end_can_be_anywhere_in_its_wall_interval(self):
        p=self.planner; actual=p.resolve(Location('orange_ground',.97,30))
        route=p.plan(Location('orange_ground'),Location('building'),actual_start=actual)
        self.assertAlmostEqual(route.source.along,.97)
        self.assertAlmostEqual(route.source.gap_mm,30)
        self.assertEqual(route.start,actual)


class LocalizationTests(unittest.TestCase):
    def test_delayed_observation_uses_capture_pose_including_turn(self):
        e=VisualOdometry(); correction=Pose(15,-10,1)
        for i in range(31):
            t=i*.02; e.predict(t,Pose(800*t,50*t,100*t))
        old=Pose(800*.4,50*.4,100*.4)
        self.assertTrue(e.update(Observation(compose(correction,old),.4),.6))
        # One EMA update is exactly 35% of the odometry-to-field transform.
        self.assertAlmostEqual(e.offset.x,5.25,places=5)
        self.assertAlmostEqual(e.offset.y,-3.5,places=5)
        self.assertAlmostEqual(e.offset.yaw,.35,places=5)

    def test_bad_frames_and_duplicate_frames_never_unlock_building(self):
        e=VisualOdometry()
        for i in range(21): e.predict(i*.02,Pose(1000,5600,90))
        for o in (Observation(Pose(1600,5600,90),.3),
                  Observation(Pose(1000,5600,90),.5),
                  Observation(Pose(1000,5600,90),.3,calibrated=False)):
            self.assertFalse(e.update(o,.4))
        o=Observation(Pose(1000,5600,90),.32)
        self.assertTrue(e.update(o,.4))
        for _ in range(10): self.assertFalse(e.update(o,.4))
        self.assertFalse(e.tag_locked)
        self.assertFalse(e.terminal_ready(.4))


class ClosedLoopTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.planner=load_planner()

    def test_continuous_start_to_opposite_orange(self):
        r=simulate(Location('start'),Location('orange_highland'),planner=self.planner)
        self.assertEqual(r['status'],'arrived',r.get('reason'))
        self.assertEqual(r['metrics']['collisions'],0)
        self.assertEqual(r['metrics']['internal_stops'],0)
        self.assertGreater(r['metrics']['high_speed_min_clearance_mm'],100)
        self.assertLess(r['metrics']['position_error_mm'],self.planner.motion.arrival_mm)
        json.dumps(r,allow_nan=False)

    def test_tag_and_building_observations_correct_encoder_bias_while_moving(self):
        r=simulate(Location('orange_highland',.7),Location('building',.35),planner=self.planner,drift_mm=30)
        self.assertEqual(r['status'],'arrived',r.get('reason'))
        self.assertTrue(r['build_ready'])
        self.assertEqual(r['metrics']['internal_stops'],0)
        self.assertLess(r['metrics']['position_error_mm'],5)
        self.assertGreaterEqual(r['visual_updates']['tag6'],3)
        self.assertTrue(any(p['tag_locked'] and math.hypot(p['vx'],p['vy'])>30 for p in r['points']))

    def test_missing_tag_or_building_observation_inhibits_build(self):
        for vision in ('missing','no_building'):
            r=simulate(Location('orange_highland'),Location('building'),planner=self.planner,vision=vision)
            self.assertEqual(r['status'],'stopped')
            self.assertFalse(r['build_ready'])
            self.assertEqual(r['metrics']['collisions'],0)

    def test_contact_endpoint_and_opposite_directions(self):
        for source,target in ((Location('start',0,0),Location('purple',1,0)),
                              (Location('building',1,0),Location('start',0,0))):
            r=simulate(source,target,planner=self.planner,max_points=10)
            self.assertEqual(r['status'],'arrived',r.get('reason'))
            self.assertEqual(r['metrics']['collisions'],0)
            self.assertLess(r['metrics']['position_error_mm'],self.planner.motion.arrival_mm)

    def test_nearby_building_goal_finishes_turn_before_waiting_for_tag(self):
        r=simulate(Location('orange_ground',1),Location('building',0),planner=self.planner,max_points=10)
        self.assertEqual(r['status'],'arrived',r.get('reason'))
        self.assertTrue(r['build_ready'])

    def test_cruise_brakes_before_entering_close_wall_zone(self):
        pairs=[(s,Location('building',0)) for s in (Location('orange_highland'),Location('purple',0),Location('building',1))]
        pairs.append((Location('orange_ground',1),Location('tag6')))
        for source,target in pairs:
            r=simulate(source,target,planner=self.planner,max_points=8)
            self.assertEqual(r['status'],'arrived',r.get('reason'))
            self.assertGreaterEqual(r['metrics']['high_speed_min_clearance_mm'],100)
            self.assertEqual(r['metrics']['collisions'],0)
            self.assertLessEqual(r['metrics']['peak_wheel_rpm'],math.ceil(self.planner.motion.wheel_rpm))


if __name__=='__main__': unittest.main()
