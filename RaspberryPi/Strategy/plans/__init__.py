"""Named strategy packages made from the common task library."""

from dataclasses import dataclass
from typing import Tuple

from ..tasks import TASK_LIBRARY, TaskStep


@dataclass(frozen=True)
class StrategyPlan:
    name: str
    steps: Tuple[TaskStep, ...]
    description: str = ''

    def __post_init__(self):
        object.__setattr__(self, 'steps', tuple(self.steps))
        if not self.name or not self.steps:
            raise ValueError('a strategy needs a name and at least one task')


def validate_plan(plan, *, task_library=TASK_LIBRARY, initial_handoff=False):
    """Check all IDs, parameters and handoff dependencies before any motion."""
    available = 'build_approach' if initial_handoff else None
    for step in plan.steps:
        if step.task_id not in task_library:
            raise ValueError(f'unknown task: {step.task_id}')
        definition = task_library[step.task_id]
        definition.configuration(step)
        if definition.requires is not None and definition.requires != available:
            raise ValueError(
                f'{step.task_id} requires the preceding Task2 exit heading; '
                'standalone Task3/Task4 needs an explicit --heading-zero-deg '
                'at its 180-degree entry pose')
        available = definition.provides


from .plan_a import PLAN_A, SUPPORT_PLANS
from .plan_b import PLAN_B
from .plan_c import PLAN_C
from .plan_d import PLAN_D

PLANS = {plan.name: plan for plan in (PLAN_A, PLAN_B, PLAN_C, PLAN_D, *SUPPORT_PLANS)}

PLAN_IDS = tuple(PLANS)

__all__ = ['StrategyPlan', 'validate_plan', 'PLANS', 'PLAN_IDS']
