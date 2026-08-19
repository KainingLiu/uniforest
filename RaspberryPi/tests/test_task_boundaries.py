import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Strategy.task0 import Task0Config, Task0Program, Task0State
from Strategy.task1 import (
    Task1Config,
    Task1Program,
    Task1Round2Config,
    Task1Round2Program,
    Task1State,
)
from Strategy.task2 import (
    Task2DebugProgram,
    Task2Round2Config,
    Task2Round2Program,
)


class TaskBoundaryTests(unittest.TestCase):
    def test_task0_has_explicit_api(self):
        self.assertIsNotNone(Task0Config)
        self.assertIsNotNone(Task0Program)
        self.assertIsNotNone(Task0State)

    def test_task1_has_explicit_api(self):
        self.assertIsNotNone(Task1Config)
        self.assertIsNotNone(Task1Program)
        self.assertIsNotNone(Task1State)
        self.assertIsNotNone(Task1Round2Config)
        self.assertIsNotNone(Task1Round2Program)

    def test_task2_round2_has_explicit_api(self):
        self.assertIsNotNone(Task2Round2Config)
        self.assertIsNotNone(Task2Round2Program)

    def test_task2_debug_preflight_sends_no_commands(self):
        robot = SimpleNamespace(
            telem=SimpleNamespace(yaw_deg=0.0),
            has_vision=True,
            has_field_localization=True,
        )

        self.assertEqual(Task2DebugProgram(robot).run(), 0)


if __name__ == '__main__':
    unittest.main()
