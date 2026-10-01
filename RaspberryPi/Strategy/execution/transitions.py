"""Explicit contextual action-pair transitions, without default motion profiles."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


def _always(_source, _target, _context):
    return True


def _ready(_source, _target, _context):
    return None


@dataclass(frozen=True)
class Transition:
    """Replaces source.exit and target.enter, exactly once.

    ``matches`` selects a context; it must be free of hardware side effects.
    ``precondition`` checks physical prerequisites before execute starts.
    ``execute`` owns the entire overlap and must join any existing supervised
    hardware activity before returning. Exceptions abort the whole window.
    """

    name: str
    source_kind: str
    target_kind: str
    execute: Callable
    matches: Callable = _always
    precondition: Callable = _ready
    priority: int = 0

    def __post_init__(self):
        if not self.name or not self.source_kind or not self.target_kind:
            raise ValueError('transition name and action kinds must be nonempty')
        for field in ('execute', 'matches', 'precondition'):
            if not callable(getattr(self, field)):
                raise TypeError(f'{self.name}: missing callable {field}')
        if not isinstance(self.priority, int):
            raise TypeError('transition priority must be an integer')


class TransitionRegistry:
    def __init__(self, transitions=()):
        self._transitions = {}
        for transition in transitions:
            self.register(transition)

    def register(self, transition):
        if not isinstance(transition, Transition):
            raise TypeError('register expects a Transition')
        if transition.name in self._transitions:
            raise ValueError(f'duplicate transition name: {transition.name}')
        self._transitions[transition.name] = transition
        return transition

    def candidates(self, source, target):
        return tuple(sorted(
            (transition for transition in self._transitions.values()
             if transition.source_kind == source.kind
             and transition.target_kind == target.kind),
            key=lambda transition: (-transition.priority, transition.name)))
