"""Action phase replacement and fault boundaries; no hardware is opened."""

from dataclasses import FrozenInstanceError
import itertools
import threading
import unittest
from unittest.mock import Mock

from Strategy.execution import (
    Action, ActionFlow, CompiledPlan, ExecutionBusy, ExecutionCancelled, ExecutionError,
    ExecutionRuntime, Transition, TransitionConflict, TransitionRegistry, compile_flow,
)


class ExecutionTests(unittest.TestCase):
    def setUp(self):
        self.calls = []
        self.emergency = Mock()
        self.close = Mock()
        self.state = {'camera_ready': True}
        self.runtime = ExecutionRuntime(
            guard=lambda: None, stop=lambda: self.calls.append('stop'),
            emergency_stop=self.emergency, close=self.close, context=self.state)

    def action(self, name, **kwargs):
        callbacks = {
            'enter': lambda: self.calls.append(f'{name}.enter'),
            'body': lambda: self.calls.append(f'{name}.body') or name,
            'exit': lambda: self.calls.append(f'{name}.exit'),
            'precondition': lambda: self.calls.append(f'{name}.ready'),
        }
        callbacks.update(kwargs)
        return Action(name, name, **callbacks)

    def transition(self, name='overlap', **kwargs):
        callbacks = {
            'execute': lambda a, b, ctx: self.calls.append(name),
            'matches': lambda a, b, ctx: ctx['camera_ready'],
        }
        callbacks.update(kwargs)
        return Transition(name, 'grab', 'navigate', **callbacks)

    def flow(self, **kwargs):
        return ActionFlow('collect', (self.action('grab'), self.action('navigate', **kwargs)))

    def test_unregistered_edge_exits_stops_then_checks_entry_and_stops_at_end(self):
        self.assertEqual(self.runtime.run(self.flow()), ('grab', 'navigate'))
        self.assertEqual(self.calls, [
            'grab.ready', 'grab.enter', 'grab.body', 'grab.exit', 'stop',
            'navigate.ready', 'navigate.enter', 'navigate.body', 'navigate.exit', 'stop'])
        self.emergency.assert_not_called()
        self.assertFalse(self.runtime.closed)

    def test_transition_replaces_both_phases_without_double_execution(self):
        registry = TransitionRegistry((self.transition(),))
        plan = compile_flow(self.flow(), registry)
        self.assertEqual(plan.preview()[1]['transition_candidates'], ('overlap',))
        self.runtime.run(plan)
        self.assertEqual(self.calls, [
            'grab.ready', 'grab.enter', 'grab.body', 'overlap', 'navigate.ready',
            'navigate.body', 'navigate.exit', 'stop'])
        phases = [(event.action, event.phase) for event in self.runtime.trace]
        self.assertNotIn(('grab', 'exit'), phases)
        self.assertNotIn(('navigate', 'enter'), phases)

    def test_compiled_plan_rechecks_live_conditions_and_freezes_registry(self):
        registry = TransitionRegistry((self.transition(),))
        plan = compile_flow(self.flow(), registry)
        registry.register(self.transition('late', priority=10,
                                          matches=lambda a, b, ctx: True))
        self.state['camera_ready'] = False
        self.runtime.run(plan)
        self.assertIn('grab.exit', self.calls)
        self.assertNotIn('overlap', self.calls)
        self.assertNotIn('late', self.calls)

    def test_matches_use_source_target_context(self):
        source = self.action('grab', context={'direction': 'right', 'arm': 'grap3'})
        target = self.action('navigate', context={'zone': 'orange'})
        registry = TransitionRegistry((self.transition(matches=lambda a, b, ctx: (
            a.context['arm'] == 'grap3' and b.context['zone'] == 'orange'
            and ctx['camera_ready'])),))
        self.runtime.run(ActionFlow('known edge', (source, target)), registry=registry)
        self.assertIn('overlap', self.calls)

    def test_failed_transition_precondition_never_starts_overlap(self):
        registry = TransitionRegistry((self.transition(
            precondition=lambda a, b, ctx: False),))
        with self.assertRaises(ExecutionError):
            self.runtime.run(self.flow(), registry=registry)
        self.assertNotIn('overlap', self.calls)
        self.assertNotIn('navigate.body', self.calls)
        self.emergency.assert_called_once()
        self.close.assert_called_once()

    def test_target_precondition_is_checked_after_transition_before_body(self):
        registry = TransitionRegistry((self.transition(),))
        with self.assertRaises(ExecutionError):
            self.runtime.run(self.flow(precondition=lambda: False), registry=registry)
        self.assertIn('overlap', self.calls)
        self.assertNotIn('navigate.body', self.calls)
        self.emergency.assert_called_once()

    def test_fallback_target_precondition_prevents_unsafe_enter(self):
        with self.assertRaises(ExecutionError):
            self.runtime.run(self.flow(precondition=lambda: False))
        self.assertEqual(self.calls[-1], 'stop')
        self.assertNotIn('navigate.enter', self.calls)
        self.emergency.assert_called_once()

    def test_cancellation_in_body_rejects_remaining_phases_and_cannot_resume(self):
        cancel = threading.Event()
        self.runtime.cancel_event = cancel
        flow = ActionFlow('cancel', (self.action('grab', body=cancel.set), self.action('navigate')))
        with self.assertRaises(ExecutionCancelled):
            self.runtime.run(flow)
        self.assertNotIn('grab.exit', self.calls)
        self.assertNotIn('navigate.enter', self.calls)
        cancel.clear()
        with self.assertRaises(ExecutionCancelled):
            self.runtime.run(self.flow())
        self.emergency.assert_called_once()
        self.close.assert_called_once()

    def test_guard_failure_after_callback_stops_before_next_callback(self):
        healthy = [True]
        self.runtime._guard = lambda: healthy[0]
        flow = ActionFlow('fault', (self.action('grab', body=lambda: healthy.__setitem__(0, False)),))
        with self.assertRaises(ExecutionError):
            self.runtime.run(flow)
        self.assertNotIn('grab.exit', self.calls)
        self.assertTrue(self.runtime.closed)
        self.emergency.assert_called_once()

    def test_ambiguous_matching_profiles_abort_instead_of_arbitrary_choice(self):
        registry = TransitionRegistry((self.transition('left'), self.transition('right')))
        with self.assertRaises(TransitionConflict):
            self.runtime.run(self.flow(), registry=registry)
        self.assertNotIn('left', self.calls)
        self.assertNotIn('right', self.calls)
        self.emergency.assert_called_once()

    def test_higher_priority_matching_profile_is_selected(self):
        registry = TransitionRegistry((self.transition('generic'),
                                       self.transition('specific', priority=1)))
        self.runtime.run(self.flow(), registry=registry)
        self.assertIn('specific', self.calls)
        self.assertNotIn('generic', self.calls)

    def test_duplicate_profile_names_and_missing_handlers_are_rejected_early(self):
        registry = TransitionRegistry((self.transition(),))
        with self.assertRaises(ValueError):
            registry.register(self.transition())
        with self.assertRaises(TypeError):
            Action('grab', 'missing', None)
        with self.assertRaises(TypeError):
            Transition('bad', 'grab', 'navigate', None)
        with self.assertRaises(TypeError):
            self.runtime.run([self.action('grab')])
        with self.assertRaises(ValueError):
            CompiledPlan(self.flow(), ())
        with self.assertRaises(TypeError):
            CompiledPlan(self.flow(), ((None,),))
        with self.assertRaises(ValueError):
            CompiledPlan(self.flow(), ((Transition('wrong', 'build', 'navigate', lambda: None),),))
        self.assertEqual(self.calls, [])
        self.emergency.assert_not_called()
        self.assertFalse(self.runtime.closed)

    def test_nested_run_rejection_does_not_stop_current_owner(self):
        def body():
            with self.assertRaises(ExecutionBusy):
                self.runtime.run(self.flow())
            self.calls.append('outer continues')
        self.runtime.run(ActionFlow('outer', (self.action('grab', body=body),)))
        self.assertIn('outer continues', self.calls)
        self.emergency.assert_not_called()

    def test_concurrent_run_rejection_does_not_interrupt_current_owner(self):
        entered, finish = threading.Event(), threading.Event()
        errors = []
        def body():
            entered.set()
            if not finish.wait(2):
                raise RuntimeError('test owner timed out')
        def run_owner():
            try:
                self.runtime.run(ActionFlow('owner', (self.action('grab', body=body),)))
            except BaseException as error:
                errors.append(error)
        thread = threading.Thread(target=run_owner)
        thread.start()
        try:
            self.assertTrue(entered.wait(2))
            with self.assertRaises(ExecutionBusy):
                self.runtime.run(self.flow())
            self.emergency.assert_not_called()
        finally:
            finish.set()
            thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])

    def test_fault_cleanup_preserves_original_exception_and_still_closes(self):
        self.emergency.side_effect = IOError('link unavailable')
        def fault():
            raise ValueError('original failure')
        with self.assertRaisesRegex(ValueError, 'original failure'):
            self.runtime.run(ActionFlow('fault', (self.action('grab', body=fault),)))
        self.close.assert_called_once()
        self.assertEqual(len(self.runtime.cleanup_errors), 1)
        self.assertTrue(self.runtime.closed)

    def test_terminal_stop_failure_aborts(self):
        self.runtime._stop = lambda: False
        with self.assertRaises(ExecutionError):
            self.runtime.run(ActionFlow('terminal', (self.action('grab'),)))
        self.emergency.assert_called_once()

    def test_trace_sink_failure_during_fault_does_not_hide_original_exception(self):
        def sink(event):
            if event.status == 'failed':
                raise IOError('diagnostics unavailable')
        def fault():
            raise ValueError('original failure')
        self.runtime._trace_sink = sink
        with self.assertRaisesRegex(ValueError, 'original failure'):
            self.runtime.run(ActionFlow('fault', (self.action('grab', body=fault),)))
        self.assertEqual(self.runtime.trace[-1].status, 'failed')
        self.emergency.assert_called_once()
        self.assertEqual(len(self.runtime.cleanup_errors), 1)

    def test_metadata_is_immutable_and_copied_from_caller(self):
        metadata = {'route': {'points': [1, 2]}}
        action = self.action('grab', context=metadata)
        metadata['route']['points'].append(3)
        self.assertEqual(action.context['route']['points'], (1, 2))
        with self.assertRaises(TypeError):
            action.context['route']['new'] = 0
        with self.assertRaises(FrozenInstanceError):
            action.body = lambda: None

    def test_trace_records_ordered_timed_phases_and_failure(self):
        self.runtime._clock = lambda ticks=itertools.count(): next(ticks) * 0.01
        self.runtime.run(ActionFlow('timed', (self.action('grab'),)))
        events = self.runtime.trace
        self.assertEqual([event.sequence for event in events], list(range(len(events))))
        self.assertEqual([event.status for event in events], ['started', 'completed'] * 5)
        self.assertTrue(all(event.duration_s > 0 for event in events if event.status == 'completed'))
        self.assertEqual({event.flow for event in events}, {'timed'})
        failed = ActionFlow('failed', (self.action('navigate', precondition=lambda: False),))
        with self.assertRaises(ExecutionError):
            self.runtime.run(failed)
        self.assertEqual(self.runtime.trace[-1].status, 'failed')


if __name__ == '__main__':
    unittest.main()
