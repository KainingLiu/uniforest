"""Flat/ramp transport limits through original routes and fused execution."""
import math
import unittest
from control.trajectory import CubicRoute
from simulation.core import MecanumPlant
from Strategy.settings import PROFILES
from tests import test_route_chain
from tests.test_route_speed_profiles import route_fixture


class TerrainSpeedTests(unittest.TestCase):
    def test_flat_long_routes_and_ramps_have_independent_caps(self):
        for key in ('depart-a','depart-b'):
            self.assertEqual(PROFILES[key].speed_mm_s,2000)
            self.assertEqual(PROFILES[key].accel_ms,1600)
        for key in ('highland-1','highland-2'):
            self.assertEqual(PROFILES[key].initial_speed_mm_s,1000)
            self.assertEqual(PROFILES[key].build_route_speed_mm_s,1000)
            self.assertEqual(PROFILES[key].ramp_accel_ms,1000)
        self.assertEqual(PROFILES['ground-1'].delivery_forward_speed_mm_s,2000)
        self.assertEqual(PROFILES['building-1'].post_build_route_speed_mm_s,2000)
        self.assertEqual(PROFILES['staged-building'].long_route_speed_mm_s,2000)
        self.assertEqual(PROFILES['ground-1'].delivery_reverse_speed_mm_s,400)
        self.assertEqual(PROFILES['ground-1'].far_wall_speed_mm_s,300)

    def test_two_metre_cruise_uses_long_acceleration_in_real_controller(self):
        env,c=route_fixture()
        for direction in ('forward','backward','right','left'):
            c._checked_move(direction,2000,2000)
            env.robot.move_chassis.assert_called_with(direction,2000,2000,
                hold_ms=0,accel_ms=1600,route_mode=True)

    def test_fused_plan_keeps_ramp_reference_and_physical_model_below_one_metre(self):
        env,plan,runtime=test_route_chain.RouteChainTests().setup_chain()
        captured=[];plant=MecanumPlant()
        env.robot.chassis.measured_body_velocity=plant.measured_velocity
        def follow(points,settings,**kw):
            captured.append((points,settings))
            return plant.follow_local(points,profile=settings,initial_velocity=kw['initial_velocity'])
        env.robot.chassis.follow_trajectory=follow
        runtime.run(env.compile(plan))
        points,settings=captured[0]
        self.assertEqual(settings.max_speed_mm_s,2000)
        self.assertIn(2000,settings.segment_speeds_mm_s)
        # Ramp is a separate local straight from (0,0) to (2500,0).
        ramp_points,ramp_settings=captured[1]
        self.assertEqual(len(ramp_points),2)
        self.assertAlmostEqual(ramp_points[-1].x_mm,2500)
        self.assertAlmostEqual(ramp_points[-1].y_mm,0)
        self.assertEqual(ramp_settings.max_speed_mm_s,1000)
        ramp=[math.hypot(p['vx'],p['vy']) for p in plant.trace
              if 1700<p['x']<3100 and abs(p['y']-2700)<60]
        self.assertTrue(ramp)
        self.assertLessEqual(max(ramp),1000.5)
        self.assertLess(math.hypot(plant.x-3400,plant.y-2450),12)
        self.assertEqual(plant.emergency_stops,0)

    def test_isolated_fast_straight_reference_respects_acceleration_budget(self):
        from tests.test_continuous_trajectory import profile
        from control.trajectory import Waypoint
        settings=profile(route_derived=True,max_speed_mm_s=2000,max_accel_mm_s2=1875,timeout_s=20)
        curve=CubicRoute((Waypoint(0,0,0),Waypoint(3000,0,0)),settings)
        dt=.001
        speeds=[curve.sample(i*dt)[1].vx_mm_s for i in range(int(curve.duration_s/dt)+1)]
        self.assertLessEqual(max(abs(b-a)/dt for a,b in zip(speeds,speeds[1:])),1875.1)


if __name__=='__main__':unittest.main()
