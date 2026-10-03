"""Standalone field research and current capped classic-loop contracts.

The field experiment is not selected by competition MotionPlanning. Its timing
study used a 1 m/s profile; current 2 m/s transport is tested through LocalRoutes
in test_terrain_speeds. Feedback is now capped in the classic loop, so the old
15% timing-parity and relative peak-speed assertions no longer apply.
"""
from dataclasses import replace
import math
import unittest
from control.chassis import Chassis, FWD_BASE_DECEL_DIST, FWD_BASE_SPEED_RPM, COUNTS_PER_CM, MECANUM_RPM_PER_CM_S
from control.motion_law import route_feedforward
from Strategy.navigation.motion import CompetitionMotion
from Strategy.settings import PROFILES
from simulation.linear_comparison import compare


class MotionTests(unittest.TestCase):
    def test_existing_scalar_route_commands_are_unchanged(self):
        for speed in (400,1000,2000):
            for elapsed in (0,20,140,300,800,1300):
                for remaining in (-1,0,25,300,90000):
                    for accel in (0,300,800):
                        ramp=min(1,elapsed/accel) if accel else 1
                        ratio=min(1,max(0,remaining/90000))
                        expected=speed*min(Chassis._smoothstep(ramp),Chassis._smoothstep(ratio))
                        self.assertEqual(route_feedforward(speed,elapsed,accel,remaining,90000),expected)

    def test_parameters_have_competition_sources_and_only_cruise_is_raised(self):
        m=CompetitionMotion.competition()
        self.assertEqual(m.cruise_mm_s,PROFILES['depart-b'].speed_mm_s)
        self.assertEqual(m.accel_s,PROFILES['depart-b'].accel_ms/1000)
        self.assertEqual(m.ramp_mm_s,PROFILES['highland-1'].build_route_speed_mm_s)
        self.assertEqual(m.approach_mm_s,PROFILES['ground-1'].near_wall_speed_mm_s)
        self.assertEqual(m.travel_mm_s,PROFILES['depart-b'].speed_mm_s*1.1)
        legacy=FWD_BASE_DECEL_DIST*(1000*MECANUM_RPM_PER_CM_S/10/FWD_BASE_SPEED_RPM)*10/COUNTS_PER_CM
        self.assertAlmostEqual(m.braking_distance(1000),legacy)
        lowered=replace(m,cruise_multiplier=1, wheel_rpm=None)
        self.assertEqual(lowered.travel_mm_s,PROFILES['depart-b'].speed_mm_s)
        self.assertEqual(lowered.ramp_mm_s,m.ramp_mm_s)
        self.assertAlmostEqual(m.wheel_rpm/lowered.wheel_rpm,1.1)
        for bad in (0,True,float('nan'),1.21):
            with self.assertRaises(ValueError): replace(m,cruise_multiplier=bad)

    def test_historical_field_experiment_and_capped_classic_both_reach_goal(self):
        motion=replace(CompetitionMotion.competition(),cruise_mm_s=1000,
                       accel_s=.8,wheel_rpm=None)
        for distance in (300,1200,2500):
            for direction in ('forward','backward','right','left'):
                with self.subTest(distance=distance,direction=direction):
                    r=compare(distance,direction,motion=motion)
                    old,new=r['classic'],r['optimized']
                    self.assertFalse(old['timed_out'])
                    self.assertEqual(new['status'],'arrived')
                    self.assertLess(new['error_mm'],8)
                    self.assertLess(old['error_mm'],8)
                    ceiling=motion.short_mm_s if distance<motion.short_distance_mm else motion.cruise_mm_s
                    self.assertLessEqual(old['peak_speed_mm_s'],ceiling+1)
                    self.assertLessEqual(new['peak_speed_mm_s'],motion.command_mm_s+1)
                    # No terminal reverse correction hidden by a total-time metric.
                    axis='vx' if direction in ('forward','backward') else 'vy'
                    sign=1 if direction in ('forward','right') else -1
                    self.assertGreater(min(p[axis]*sign for p in new['points']),-25)
