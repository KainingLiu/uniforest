"""Compile immutable edge candidates; live conditions remain runtime checks."""

from dataclasses import dataclass

from .model import ActionFlow
from .transitions import Transition, TransitionRegistry


@dataclass(frozen=True)
class CompiledPlan:
    flow: ActionFlow
    edges: tuple

    def __post_init__(self):
        if not isinstance(self.flow, ActionFlow):
            raise TypeError('compiled flow must be an ActionFlow')
        object.__setattr__(self, 'edges', tuple(tuple(edge) for edge in self.edges))
        if len(self.edges) != max(0, len(self.flow.actions) - 1):
            raise ValueError('compiled edge count does not match its flow')
        for index, edge in enumerate(self.edges):
            source, target = self.flow.actions[index:index + 2]
            for transition in edge:
                if not isinstance(transition, Transition):
                    raise TypeError('compiled edge candidates must be Transitions')
                if (transition.source_kind != source.kind
                        or transition.target_kind != target.kind):
                    raise ValueError('compiled transition has incompatible action kinds')

    def preview(self):
        """Describe choices, without claiming a live transition will match."""
        rows = []
        for index, action in enumerate(self.flow.actions):
            candidates = () if index == 0 else self.edges[index - 1]
            rows.append({
                'name': action.name,
                'kind': action.kind,
                'transition_candidates': tuple(item.name for item in candidates),
                'fallback': 'previous.exit -> stop -> precondition -> enter'
                            if index else 'precondition -> enter',
            })
        return tuple(rows)


def compile_flow(flow, registry=None):
    if not isinstance(flow, ActionFlow):
        raise TypeError('compile_flow expects an ActionFlow')
    if registry is None:
        registry = TransitionRegistry()
    if not isinstance(registry, TransitionRegistry):
        raise TypeError('registry must be a TransitionRegistry')
    return CompiledPlan(flow, tuple(
        registry.candidates(source, target)
        for source, target in zip(flow.actions, flow.actions[1:])))
