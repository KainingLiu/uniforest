import struct
import os
import sys
import threading
import types
import unittest
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Importing the protocol package normally loads the serial transport. Position
# math tests do not open hardware, so provide only the symbols needed at import.
if 'serial' not in sys.modules:
    serial_stub = types.ModuleType('serial')
    serial_stub.Serial = object
    serial_stub.SerialException = Exception
    serial_stub.EIGHTBITS = 8
    serial_stub.PARITY_NONE = 'N'
    serial_stub.STOPBITS_ONE = 1
    sys.modules['serial'] = serial_stub

from control.chassis import Chassis, COUNTS_PER_CM, LATERAL_DISTANCE_SCALE
from protocol.commands import TelemBatch
from tools.chassis_distance_test import PositionKeyTest


def make_telem_payload(motor_positions, motor_speeds=(0, 0, 0, 0)):
    payload = bytearray()
    for i in range(4):
        payload.extend(struct.pack(
            '>HhhB', 100 + i, motor_speeds[i], 0, 30 + i))
    payload.extend(struct.pack('>ff', 0.0, 0.0))
    payload.extend(struct.pack('>6H', *([1024] * 6)))
    payload.extend(bytes(4))
    payload.extend(struct.pack('>I', 1234))
    payload.extend(struct.pack('>2i', 10, -20))
    payload.extend(struct.pack('>4i', *motor_positions))
    return bytes(payload)


class ChassisPositionTests(unittest.TestCase):
    def test_80_byte_telemetry_decodes_cumulative_positions(self):
        positions = (330000, -331000, 332000, -333000)
        payload = make_telem_payload(positions)

        self.assertEqual(len(payload), 80)
        telem = TelemBatch.unpack(payload)
        self.assertEqual(
            tuple(m.cumulative_pos for m in telem.motors), positions)

    def test_forward_projection_uses_action_local_origin(self):
        origin = (100000, -200000, 300000, -400000)
        delta = 25 * COUNTS_PER_CM
        positions = (
            origin[0] - delta,
            origin[1] + delta,
            origin[2] + delta,
            origin[3] - delta,
        )
        telem = TelemBatch.unpack(make_telem_payload(positions))

        wheels, projected = Chassis._project_wheel_positions(
            telem, origin, [-1, 1, 1, -1])

        self.assertEqual(wheels, (delta, delta, delta, delta))
        self.assertEqual(projected, delta)

    def test_right_projection_exposes_individual_wheel_slip(self):
        origin = (0, 0, 0, 0)
        positions = (10000, 9500, -9000, -10500)
        telem = TelemBatch.unpack(make_telem_payload(positions))

        wheels, projected = Chassis._project_wheel_positions(
            telem, origin, [1, 1, -1, -1])

        self.assertEqual(wheels, (10000, 9500, 9000, 10500))
        self.assertEqual(projected, 9750)

    def test_lateral_odometry_reports_calibrated_signed_displacement(self):
        origin = (100000, -200000, 300000, -400000)
        chassis_mm = 465.0
        wheel_counts = round(
            chassis_mm * LATERAL_DISTANCE_SCALE * COUNTS_PER_CM / 10.0)
        positions = (
            origin[0] + wheel_counts,
            origin[1] + wheel_counts,
            origin[2] - wheel_counts,
            origin[3] - wheel_counts,
        )
        chassis = Chassis.__new__(Chassis)
        chassis.lateral_distance_scale = LATERAL_DISTANCE_SCALE
        chassis._telem = TelemBatch.unpack(make_telem_payload(positions))

        self.assertEqual(chassis.capture_motor_positions(), positions)
        self.assertAlmostEqual(
            chassis.lateral_displacement_mm(origin), chassis_mm, delta=0.1)

        reversed_positions = tuple(
            2 * origin[i] - positions[i] for i in range(4))
        chassis._telem = TelemBatch.unpack(
            make_telem_payload(reversed_positions))
        self.assertAlmostEqual(
            chassis.lateral_displacement_mm(origin), -chassis_mm, delta=0.1)

    def test_legacy_payload_is_rejected(self):
        with self.assertRaises(ValueError):
            TelemBatch.unpack(bytes(64))

    def test_temporary_keyboard_mapping(self):
        self.assertEqual(PositionKeyTest.MOVES['w'], ('forward', 1000.0))
        self.assertEqual(PositionKeyTest.MOVES['s'], ('forward', -1000.0))
        self.assertEqual(PositionKeyTest.MOVES['a'], ('right', -1000.0))
        self.assertEqual(PositionKeyTest.MOVES['d'], ('right', 1000.0))

    def test_lateral_calibration_matches_field_measurement(self):
        self.assertAlmostEqual(LATERAL_DISTANCE_SCALE, 500.0 / 465.0)
        result = Chassis.__new__(Chassis)
        result.lateral_distance_scale = LATERAL_DISTANCE_SCALE
        self.assertAlmostEqual(465.0 * result.lateral_distance_scale, 500.0)

    def test_settle_requires_position_and_low_wheel_speed(self):
        stopped = TelemBatch.unpack(make_telem_payload((0, 0, 0, 0)))
        moving = TelemBatch.unpack(make_telem_payload(
            (0, 0, 0, 0), (25, -25, 25, -25)))

        self.assertTrue(Chassis._linear_settled(stopped, 400))
        self.assertFalse(Chassis._linear_settled(stopped, 401))
        self.assertFalse(Chassis._linear_settled(moving, 0))

    def test_cancelled_move_sends_only_zero_speed(self):
        class FakeTransport:
            def __init__(self):
                self.commands = []

            def set_chassis_speed(self, rpm):
                self.commands.append(tuple(rpm))

        telem = TelemBatch.unpack(make_telem_payload((0, 0, 0, 0)))
        transport = FakeTransport()
        chassis = Chassis(transport)
        cancel = threading.Event()
        cancel.set()

        result = chassis._move_linear(
            1000, [-1, 1, 1, -1], 500.0, transport,
            lambda: telem, 10.0, cancel_event=cancel)

        self.assertTrue(result.cancelled)
        self.assertEqual(transport.commands, [(0, 0, 0, 0)])

    def test_turn_tracks_yaw_across_wrap_and_holds(self):
        class FakeClock:
            def __init__(self):
                self.now = 0.0
                self.yaw_updates = iter((-175.0, 179.0, 90.0, 10.0))
                self.chassis = None

            def time(self):
                return self.now

            def sleep(self, seconds):
                self.now += seconds
                next_yaw = next(self.yaw_updates, None)
                if next_yaw is not None:
                    self.chassis._telem.yaw_deg = next_yaw

        class FakeTransport:
            def __init__(self):
                self.commands = []
                self.stop_count = 0

            def set_chassis_speed(self, rpm):
                self.commands.append(tuple(rpm))

            def emergency_stop(self):
                self.stop_count += 1

        clock = FakeClock()
        transport = FakeTransport()
        chassis = Chassis(transport)
        chassis._telem = SimpleNamespace(yaw_deg=-170.0)
        clock.chassis = chassis

        with patch('control.chassis.time.time', clock.time), \
                patch('control.chassis.time.sleep', clock.sleep):
            chassis.turn(180.0, 90.0, hold_ms=500, settle_cycles=1)

        self.assertGreaterEqual(clock.now, 0.5)
        self.assertLess(clock.now, 1.0)
        self.assertEqual(transport.stop_count, 1)
        self.assertEqual(transport.commands[-1], (0, 0, 0, 0))


if __name__ == '__main__':
    unittest.main()
