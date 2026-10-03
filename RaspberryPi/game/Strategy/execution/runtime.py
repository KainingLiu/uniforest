"""Single-owner synchronous action executor with guarded phase handoffs."""

from __future__ import annotations

import threading
import time

from .compiler import CompiledPlan, compile_flow
from .model import (ExecutionBusy, ExecutionCancelled, ExecutionError,
                    TraceEvent, TransitionConflict, noop)


class ExecutionRuntime:
    """Run blocking action methods without introducing background workers.

    Long callbacks must use their existing context guard/cancel checks; the
    runtime guards their boundaries. ``cancel_event`` may be the same Event
    supplied to the hardware clients. Clearing it cannot revive a failed run.
    A runtime is reusable after successful windows, and permanently closed on
    any execution failure. The caller retains the robot-wide strategy lock.
    """

    def __init__(self, *, guard, stop, emergency_stop, close=noop,
                 context=None, trace=None, cancel_event=None, clock=time.monotonic):
        close = noop if close is None else close
        for name, callback in (('guard', guard), ('stop', stop),
                               ('emergency_stop', emergency_stop), ('close', close),
                               ('clock', clock)):
            if not callable(callback):
                raise TypeError(f'{name} must be callable')
        if trace is not None and not callable(trace):
            raise TypeError('trace must be callable')
        self._guard = guard
        self._stop = stop
        self._emergency_stop = emergency_stop
        self._close = close
        self.context = {} if context is None else context
        self.cancel_event = cancel_event if cancel_event is not None else threading.Event()
        self._trace_sink = trace
        self._clock = clock
        self._owner = threading.Lock()
        self.closed = False
        self.trace = []
        self.cleanup_errors = []
        self._flow_name = ''

    def cancel(self):
        """Request cancellation; hardware stop and long waits use their owner API."""
        self.cancel_event.set()

    def _check_active(self):
        if self.closed or self.cancel_event.is_set():
            raise ExecutionCancelled('execution cancelled or closed; explicit new run required')
        if self._guard() is False:
            raise ExecutionError('execution guard rejected the current state')
        # The external guard may observe and propagate a cancellation itself.
        if self.closed or self.cancel_event.is_set():
            raise ExecutionCancelled('execution cancelled during guard')

    def _emit(self, action, phase, status, started=None, detail=''):
        now = self._clock()
        event = TraceEvent(len(self.trace), self._flow_name, action, phase,
                           status, now, 0.0 if started is None else now - started,
                           detail)
        self.trace.append(event)
        if self._trace_sink is not None:
            self._trace_sink(event)
        return now

    def _invoke(self, action, phase, callback, *args, check_result=False):
        started = self._emit(action, phase, 'started')
        try:
            self._check_active()
            result = callback(*args)
            self._check_active()
            if check_result and result is False:
                raise ExecutionError(f'{action}: {phase} rejected the current state')
        except BaseException as error:
            try:
                self._emit(action, phase, 'failed', started, type(error).__name__)
            except BaseException as trace_error:
                # Diagnostics must not replace the fault that caused the abort.
                self.cleanup_errors.append(trace_error)
            raise
        self._emit(action, phase, 'completed', started)
        return result

    def _enter(self, action):
        self._invoke(action.name, 'precondition', action.precondition, check_result=True)
        self._invoke(action.name, 'enter', action.enter)

    def _select(self, source, target, candidates):
        matches = []
        for transition in candidates:
            if self._invoke(source.name, f'match:{transition.name}',
                            transition.matches, source, target, self.context):
                matches.append(transition)
        if not matches:
            return None
        top = max(transition.priority for transition in matches)
        winners = [transition for transition in matches if transition.priority == top]
        if len(winners) != 1:
            raise TransitionConflict(
                f'{source.name} -> {target.name}: ambiguous transitions '
                + ', '.join(transition.name for transition in winners))
        return winners[0]

    def _abort(self):
        self.closed = True
        for callback in (self._emergency_stop, self._close):
            try:
                callback()
            except BaseException as error:
                # Preserve the original fault while still attempting both cleanup steps.
                self.cleanup_errors.append(error)

    def run(self, flow_or_plan, *, registry=None):
        if not self._owner.acquire(blocking=False):
            raise ExecutionBusy('execution runtime already has an owner')
        try:
            # Rejected/invalid plans have no side effects. Runtime failure cleanup
            # starts only after complete structural validation and ownership.
            if self.closed:
                raise ExecutionCancelled('execution is closed; create a new runtime')
            if isinstance(flow_or_plan, CompiledPlan):
                if registry is not None:
                    raise ValueError('a compiled plan already contains its registry snapshot')
                plan = flow_or_plan
            else:
                plan = compile_flow(flow_or_plan, registry)
            if len(plan.edges) != max(0, len(plan.flow.actions) - 1):
                raise ValueError('compiled edge count does not match its flow')
            self._flow_name = plan.flow.name
            try:
                self._check_active()
                actions = plan.flow.actions
                results = []
                if actions:
                    self._enter(actions[0])
                for index, action in enumerate(actions):
                    results.append(self._invoke(action.name, 'body', action.body))
                    if index + 1 == len(actions):
                        self._invoke(action.name, 'exit', action.exit)
                        self._invoke(action.name, 'stop', self._stop, check_result=True)
                        continue
                    target = actions[index + 1]
                    transition = self._select(action, target, plan.edges[index])
                    if transition is None:
                        self._invoke(action.name, 'exit', action.exit)
                        self._invoke(action.name, 'stop', self._stop, check_result=True)
                        self._enter(target)
                    else:
                        self._invoke(action.name, f'transition_precondition:{transition.name}',
                                     transition.precondition, action, target, self.context,
                                     check_result=True)
                        self._invoke(action.name, f'transition:{transition.name}',
                                     transition.execute, action, target, self.context)
                        self._invoke(target.name, 'precondition', target.precondition,
                                     check_result=True)
                self._check_active()
                return tuple(results)
            except BaseException:
                self._abort()
                raise
        finally:
            self._owner.release()
