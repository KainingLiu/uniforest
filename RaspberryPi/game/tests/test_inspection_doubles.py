"""Keep inspection doubles aligned with the production callback contract."""

import inspect
import unittest
from unittest.mock import Mock

from control.carried_cube_inspection import CarriedInspectionSession
from tests.test_registered_inspection_transitions import RegisteredInspectionReplay, InspectionAdapter
from tests.test_registered_pickup_transitions import Fixture, Inspection


class InspectionDoubleTests(unittest.TestCase):
    def sessions(self, count):
        owner = RegisteredInspectionReplay([count])
        owner.seed(owner.env, owner.plan.steps[0])
        yield owner, owner.begin(allow_visual_failure=True, allow_idle=True)
        owner = Fixture(counts=(count,))
        yield owner, owner.inspect()

    def test_keyword_interface_matches_production(self):
        expected = inspect.signature(CarriedInspectionSession.inspect)
        for double in (InspectionAdapter, Inspection):
            self.assertEqual(inspect.signature(double.inspect), expected)

    def test_retreat_runs_once_before_count_and_before_restoration(self):
        for count in (0, 1, 2, 3, None):
            for owner, session in self.sessions(count):
                with self.subTest(kind=type(session).__name__, count=count):
                    def retreat():
                        self.assertFalse(session.restored)
                        if hasattr(owner.robot.actions, '_action_lock'):
                            self.assertTrue(owner.robot.actions._action_lock.locked())
                        owner.events.append('retreat')
                    callback = Mock(side_effect=retreat)
                    self.assertEqual(session.inspect(chassis_followup=callback), count)
                    callback.assert_called_once_with()
                    count_index = next(i for i, event in enumerate(owner.events)
                                       if isinstance(event, tuple) and event[0] in ('inspect.count', 'inspect'))
                    self.assertLess(owner.events.index('retreat'), count_index)
                    self.assertFalse(session.restored)
                    session.finish_restore()
                    session.close()
                    self.assertTrue(session.closed)

    def test_callback_exception_aborts_without_publishing_count(self):
        for owner, session in self.sessions(3):
            with self.subTest(kind=type(session).__name__):
                callback = Mock(side_effect=RuntimeError('retreat failed'))
                with self.assertRaisesRegex(RuntimeError, 'retreat failed'):
                    session.inspect(chassis_followup=callback)
                self.assertTrue(session.closed)
                self.assertFalse(session.restored)
                self.assertFalse(any(isinstance(e, tuple) and e[0] in ('inspect.count', 'inspect')
                                     for e in owner.events))
                if hasattr(owner.robot.actions, '_action_lock'):
                    self.assertFalse(owner.robot.actions._action_lock.locked())

    def test_cancel_during_callback_is_observed_before_count(self):
        for owner, session in self.sessions(3):
            with self.subTest(kind=type(session).__name__):
                with self.assertRaises(RuntimeError):
                    session.inspect(chassis_followup=owner.context.cancel_event.set)
                self.assertTrue(session.closed)
                self.assertFalse(session.restored)


if __name__ == '__main__':
    unittest.main()
