"""Compatibility imports; the original strategy is defined in plan_a."""

from .plan_a import PLAN_A, SET1, SET2, SUPPORT_PLANS

PLANS = {'classic': PLAN_A, **{plan.name: plan for plan in SUPPORT_PLANS}}
