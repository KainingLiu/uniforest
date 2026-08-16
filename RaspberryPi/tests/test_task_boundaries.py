import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Strategy.task1 import Task1Config, Task1Program, Task1State
from Strategy.task2 import Task2DebugProgram


class TaskBoundaryTests(unittest.TestCase):
    def test_task1_has_explicit_api(self):
        self.assertIsNotNone(Task1Config)
        self.assertIsNotNone(Task1Program)
        self.assertIsNotNone(Task1State)

    def test_task2_debug_preflight_sends_no_commands(self):
        robot = SimpleNamespace(
            telem=SimpleNamespace(yaw_deg=0.0),
            has_vision=True,
            has_field_localization=True,
        )

        self.assertEqual(Task2DebugProgram(robot).run(), 0)


if __name__ == '__main__':
    unittest.main()
