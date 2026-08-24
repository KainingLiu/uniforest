import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Strategy.task0 import Task0Config, Task0Program, Task0State


class Task0Tests(unittest.TestCase):
    def test_full_mission_initial_move_parameters(self):
        moves = []
        emergency_stops = []
        robot = SimpleNamespace(
            telem=SimpleNamespace(yaw_deg=12.0),
            has_vision=True,
            has_field_localization=True,
            move_chassis=lambda direction, distance, speed, **kwargs: (
                moves.append((direction, distance, speed, kwargs))
                or SimpleNamespace(timed_out=False, cancelled=False)),
            transport=SimpleNamespace(
                emergency_stop=lambda: emergency_stops.append(True)),
        )

        program = Task0Program(robot)

        self.assertEqual(program.run(), 0)
        self.assertEqual(program.state, Task0State.FINISHED)
        self.assertEqual(moves, [
            ('forward', 1200.0, 750.0,
             {'hold_ms': 0, 'accel_ms': 800}),
        ])
        self.assertEqual(emergency_stops, [])

    def test_preflight_failure_stops_without_moving(self):
        moves = []
        emergency_stops = []
        robot = SimpleNamespace(
            telem=SimpleNamespace(yaw_deg=0.0),
            has_vision=False,
            has_field_localization=True,
            move_chassis=lambda *args, **kwargs: moves.append(args),
            transport=SimpleNamespace(
                emergency_stop=lambda: emergency_stops.append(True)),
        )
        program = Task0Program(robot, Task0Config(telemetry_wait_s=0.0))

        with self.assertRaisesRegex(RuntimeError, 'vision subsystem'):
            program.run()

        self.assertEqual(program.state, Task0State.FAULT)
        self.assertEqual(moves, [])
        self.assertEqual(emergency_stops, [True])


if __name__ == '__main__':
    unittest.main()
