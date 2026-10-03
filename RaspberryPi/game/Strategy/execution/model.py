"""Hardware-independent action descriptions; callbacks retain hardware ownership."""

from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Callable, Mapping


def noop():
    """An explicitly empty phase."""


def _freeze(value):
    if isinstance(value, Mapping):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    if isinstance(value, (set, frozenset)):
        return frozenset(_freeze(item) for item in value)
    if value is None or isinstance(value, (str, bytes, bool, int, float)):
        return value
    raise TypeError('action context must contain immutable scalar metadata or containers')


@dataclass(frozen=True)
class Action:
    """One functional action, with three independently executable phases.

    ``precondition`` raises (or returns False) when this action cannot safely
    enter. On an optimized edge it validates the state delivered by T(A, B),
    immediately before B.body. Phase callbacks return data or raise on failure.
    """

    kind: str
    name: str
    body: Callable
    enter: Callable = noop
    exit: Callable = noop
    precondition: Callable = noop
    context: Mapping = field(default_factory=dict)

    def __post_init__(self):
        if not self.kind or not self.name:
            raise ValueError('action kind and name must be nonempty')
        for phase in ('enter', 'body', 'exit', 'precondition'):
            if not callable(getattr(self, phase)):
                raise TypeError(f'{self.name}: missing callable {phase}')
        if not isinstance(self.context, Mapping):
            raise TypeError('action context must be a mapping')
        object.__setattr__(self, 'context', _freeze(self.context))


@dataclass(frozen=True)
class ActionFlow:
    """A finite window; a policy may construct another window after its results."""

    name: str
    actions: tuple[Action, ...]

    def __post_init__(self):
        if not self.name:
            raise ValueError('flow name must be nonempty')
        object.__setattr__(self, 'actions', tuple(self.actions))
        if any(not isinstance(action, Action) for action in self.actions):
            raise TypeError('every flow item must be an Action')


@dataclass(frozen=True)
class TraceEvent:
    sequence: int
    flow: str
    action: str
    phase: str
    status: str
    monotonic_s: float
    duration_s: float = 0.0
    detail: str = ''


class ExecutionError(RuntimeError):
    """An execution contract failed."""


class ExecutionBusy(ExecutionError):
    """Another call already owns this runtime."""


class ExecutionCancelled(ExecutionError):
    """An execution was cancelled and cannot be resumed."""


class TransitionConflict(ExecutionError):
    """Several equally preferred transitions match one edge."""
