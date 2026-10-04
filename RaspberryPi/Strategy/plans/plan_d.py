"""PlanB route with measured base heights and height-aware Task5 topping."""

from . import StrategyPlan
from .plan_b import PLAN_B


PLAN_D = StrategyPlan(
    'PlanD', PLAN_B.steps,
    'PlanB route; remember each orange base and top it from its next layer')
