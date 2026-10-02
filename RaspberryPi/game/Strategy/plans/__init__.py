"""Declarative strategy choices, with no hardware program classes."""
from dataclasses import dataclass
import math
from ..flows.model import ActionSpec

@dataclass(frozen=True)
class StrategyPlan:
    name: str
    steps: tuple[ActionSpec, ...]
    entry_anchor: str = 'start'
    entry_heading_deg: float = 0.0
    needs_heading_zero: bool = False
    description: str = ''

    def __post_init__(self):
        object.__setattr__(self, 'steps', tuple(self.steps))
        if not self.name or not self.steps:
            raise ValueError('a strategy requires named functional actions')
        if any(not isinstance(s, ActionSpec) for s in self.steps):
            raise TypeError('strategy steps must be ActionSpec values')
        if len({s.name for s in self.steps}) != len(self.steps):
            raise ValueError('action names must be unique within a plan')
        if not math.isfinite(self.entry_heading_deg):
            raise ValueError('entry heading must be finite')


def validate_plan(plan, *, heading_zero_deg=None, initial_anchor=None):
    from ..flows.factory import validate_spec
    if heading_zero_deg is not None and not math.isfinite(heading_zero_deg):
        raise ValueError('heading zero must be finite')
    if plan.needs_heading_zero and heading_zero_deg is None:
        raise ValueError('this entry requires a known heading zero at the build approach')
    anchor = plan.entry_anchor if initial_anchor is None else initial_anchor
    collection_profile = collection_color = None
    previous = None
    for spec in plan.steps:
        validate_spec(spec)
        if spec.kind == 'begin_collection':
            collection_profile,collection_color=spec.profile,spec.parameters['color']
        if spec.kind in ('acquire_cube','grab_cube','inspect_cargo'):
            if collection_profile != spec.profile:
                raise ValueError(f'{spec.name} requires begin_collection with the same profile')
            if spec.kind == 'inspect_cargo' and collection_color != 'orange':
                raise ValueError(f'{spec.name} requires an orange collection session')
        if spec.kind == 'grab_cube' and (previous is None or previous.kind != 'acquire_cube'
                                         or previous.profile != spec.profile):
            raise ValueError(f'{spec.name} requires an immediately preceding acquisition')
        if spec.requires_anchor is not None and spec.requires_anchor != anchor:
            raise ValueError(f'{spec.name} requires anchor {spec.requires_anchor}, got {anchor}')
        if spec.ends_at is not None:
            anchor = spec.ends_at
        previous = spec

from .plan_a import PLAN_A, SUPPORT_PLANS
from .plan_b import PLAN_B
from .segments import SEGMENTS
PLANS = {p.name: p for p in (PLAN_A, PLAN_B, *SUPPORT_PLANS, *SEGMENTS)}
PLAN_IDS = tuple(PLANS)
