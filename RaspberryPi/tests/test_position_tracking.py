"""Behavior of moving-reference PID under bias, clipping and pose correction."""
from dataclasses import replace
import unittest
import numpy as np
from Strategy.navigation.tracking import PositionTracker,TrackingGains


class PositionPIDTests(unittest.TestCase):
    def test_integral_rejects_constant_velocity_bias_on_moving_target(self):
        def run(gains):
            pid=PositionTracker(gains); x=0.; velocity=150.; target=0.
            for _ in range(3000):
                error=target-x
                correction=pid.step([error,0,0],[150-velocity,0,0],.02)
                velocity=150+correction[0]-12
                pid.applied(correction); x+=velocity*.02; target+=150*.02
            return abs(target-x)
        g=TrackingGains()
        self.assertLess(run(g),run(replace(g,xy_ki=0))*.2)

    def test_wheel_or_wall_clipping_does_not_wind_up_integral(self):
        p=PositionTracker()
        for _ in range(2000):
            p.step([20,-20,3],[0,0,0],.02)
            p.applied([0,0,0])
        np.testing.assert_allclose(p.integral,0)
        p.step([20,-20,3],[0,0,0],.02)
        self.assertGreater(p.integral[0],0)
        p.reset(); np.testing.assert_allclose(p.integral,0)

    def test_relocalization_has_no_derivative_kick_and_yaw_uses_short_arc(self):
        p=PositionTracker(replace(TrackingGains(),xy_kp=0,xy_ki=0,yaw_ki=0))
        p.step([0,0,0],[0,0,0],.02)
        correction=p.step([120,-120,359],[0,0,0],.02)
        np.testing.assert_allclose(correction[:2],0)
        self.assertAlmostEqual(correction[2],-2.5)

    def test_invalid_inputs_and_local_saturation(self):
        for value in (float('nan'),-1,True):
            with self.assertRaises(ValueError):TrackingGains(xy_kp=value)
        p=PositionTracker(replace(TrackingGains(),xy_output_limit_mm_s=2))
        for _ in range(100):p.step([20,20,0],[0,0,0],.02)
        np.testing.assert_allclose(p.integral,0)
        for dt in (0,float('nan'),1):
            with self.assertRaises(ValueError):p.step([0,0,0],[0,0,0],dt)
