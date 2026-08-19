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

    def test_all_runs_task0_then_both_rounds_on_same_robot(self):
        events = []
        robot = object()

        result = run_tasks(
            robot, 'all',
            task0_factory=self._factory('task0', events),
            task1_factory=self._factory('task1-r1', events),
            task2_factory=self._factory('task2-r1', events),
            task1_round2_factory=self._factory('task1-r2', events),
            task2_round2_factory=self._factory('task2-r2', events))

        self.assertEqual(result, 0)
        self.assertEqual(events, [
            ('create', 'task0', robot),
            ('run', 'task0'),
            ('create', 'task1-r1', robot),
            ('run', 'task1-r1'),
            ('create', 'task2-r1', robot),
            ('run', 'task2-r1'),
            ('create', 'task1-r2', robot),
            ('run', 'task1-r2'),
            ('create', 'task2-r2', robot),
            ('run', 'task2-r2'),
        ])

    def test_round_selections_skip_task0(self):
        for selection, expected in (
                ('round1', ['task1-r1', 'task2-r1']),
                ('round2', ['task1-r2', 'task2-r2'])):
            with self.subTest(selection=selection):
                events = []
                result = run_tasks(
                    object(), selection,
                    task0_factory=self._factory('task0', events),
                    task1_factory=self._factory('task1-r1', events),
                    task2_factory=self._factory('task2-r1', events),
                    task1_round2_factory=self._factory(
                        'task1-r2', events),
                    task2_round2_factory=self._factory(
                        'task2-r2', events))

                self.assertEqual(result, 0)
                self.assertEqual(
                    [event[1] for event in events if event[0] == 'run'],
                    expected)

    def test_round_specific_task_selections(self):
        for selection, expected in (
                ('task1', 'task1-r1'),
                ('task2', 'task2-r1'),
                ('task1-r1', 'task1-r1'),
                ('task2-r1', 'task2-r1'),
                ('task1-r2', 'task1-r2'),
                ('task2-r2', 'task2-r2')):
            with self.subTest(selection=selection):
                events = []
                result = run_tasks(
                    object(), selection,
                    task0_factory=self._factory('task0', events),
                    task1_factory=self._factory('task1-r1', events),
                    task2_factory=self._factory('task2-r1', events),
                    task1_round2_factory=self._factory(
                        'task1-r2', events),
                    task2_round2_factory=self._factory(
                        'task2-r2', events))

                self.assertEqual(result, 0)
                self.assertEqual(
                    [event[1] for event in events if event[0] == 'run'],
                    [expected])

    def test_single_task_selection_remains_available(self):
        events = []

        result = run_tasks(
            object(), 'task2',
            task0_factory=self._factory('task0', events),
            task1_factory=self._factory('task1', events),
            task2_factory=self._factory('task2', events))

        self.assertEqual(result, 0)
        self.assertEqual([event[:2] for event in events], [
            ('create', 'task2'),
            ('run', 'task2'),
        ])

    def test_single_task1_does_not_run_task0(self):
        events = []

        result = run_tasks(
            object(), 'task1',
            task0_factory=self._factory('task0', events),
            task1_factory=self._factory('task1', events),
            task2_factory=self._factory('task2', events))

        self.assertEqual(result, 0)
        self.assertEqual([event[:2] for event in events], [
            ('create', 'task1'),
            ('run', 'task1'),
        ])

    def test_task1_failure_prevents_task2_start(self):
        events = []

        result = run_tasks(
            object(), 'all',
            task0_factory=self._factory('task0', events),
            task1_factory=self._factory('task1', events, result=7),
            task2_factory=self._factory('task2', events))

        self.assertEqual(result, 7)
        self.assertEqual([event[:2] for event in events], [
            ('create', 'task0'),
            ('run', 'task0'),
            ('create', 'task1'),
            ('run', 'task1'),
        ])

    def test_task0_failure_prevents_task1_start(self):
        events = []

        result = run_tasks(
            object(), 'all',
            task0_factory=self._factory('task0', events, result=8),
            task1_factory=self._factory('task1', events),
            task2_factory=self._factory('task2', events))

        self.assertEqual(result, 8)
        self.assertEqual([event[:2] for event in events], [
            ('create', 'task0'),
            ('run', 'task0'),
        ])


if __name__ == '__main__':
    unittest.main()
