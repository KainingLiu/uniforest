import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Strategy.runner import run_tasks


class TaskRunnerTests(unittest.TestCase):
    @staticmethod
    def _factory(label, events, result=0):
        class FakeProgram:
            def __init__(self, robot):
                events.append(('create', label, robot))

            def run(self):
                events.append(('run', label))
                return result

        return FakeProgram

    def test_all_runs_task1_then_task2_on_same_robot(self):
        events = []
        robot = object()

        result = run_tasks(
            robot, 'all',
            task1_factory=self._factory('task1', events),
            task2_factory=self._factory('task2', events))

        self.assertEqual(result, 0)
        self.assertEqual(events, [
            ('create', 'task1', robot),
            ('run', 'task1'),
            ('create', 'task2', robot),
            ('run', 'task2'),
        ])

    def test_single_task_selection_remains_available(self):
        events = []

        result = run_tasks(
            object(), 'task2',
            task1_factory=self._factory('task1', events),
            task2_factory=self._factory('task2', events))

        self.assertEqual(result, 0)
        self.assertEqual([event[:2] for event in events], [
            ('create', 'task2'),
            ('run', 'task2'),
        ])

    def test_task1_failure_prevents_task2_start(self):
        events = []

        result = run_tasks(
            object(), 'all',
            task1_factory=self._factory('task1', events, result=7),
            task2_factory=self._factory('task2', events))

        self.assertEqual(result, 7)
        self.assertEqual([event[:2] for event in events], [
            ('create', 'task1'),
            ('run', 'task1'),
        ])


if __name__ == '__main__':
    unittest.main()
