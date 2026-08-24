import os
import sys
import time
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot import HardwarePreflightReport, Robot


class RobotPreflightTests(unittest.TestCase):
    def test_report_requires_telemetry_and_protocol(self):
        report = HardwarePreflightReport(
            connected=True,
            telemetry_received=True,
            telemetry_age_s=0.01,
            telemetry_uptime_ms=10,
            rx_frames=2,
            tx_frames=3,
            rx_crc_errors=0,
            vision_active=False,
            localization_active=False,
            protocol_valid=True,
        )
        self.assertTrue(report.ok)

        stale = HardwarePreflightReport(**{
            **report.__dict__, 'telemetry_received': False})
        self.assertFalse(stale.ok)

    def test_preflight_does_not_require_actuators(self):
        robot = object.__new__(Robot)
        robot.transport = SimpleNamespace(
            connected=True, rx_frames=5, tx_frames=7, rx_crc_errors=0)
        robot._telem = SimpleNamespace(uptime_ms=123)
        robot._telem_received_at = time.monotonic()
        robot._telem_lock = __import__('threading').Lock()
        robot._vision = None
        robot._localizer = None

        report = robot.hardware_preflight(timeout_s=0.0)
        self.assertTrue(report.ok)
        self.assertEqual(report.telemetry_uptime_ms, 123)


if __name__ == '__main__':
    unittest.main()
