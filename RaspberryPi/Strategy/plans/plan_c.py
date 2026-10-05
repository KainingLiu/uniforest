"""PlanC: the first two Task1 routes followed by collection/build cycles."""

from . import StrategyPlan
from ..tasks import TaskStep


PLAN_C = StrategyPlan('PlanC', tuple(TaskStep(task_id) for task_id in (
    'task0-1', 'task1-1', 'task0-3', 'task1-2',
    'task2-1', 'task3-4', 'task2-2', 'task3-2',
    'task1-3', 'task2-2', 'task3-3')),
    'Task1 variants 1/2 with Task0-3 transfer, then three collection/build cycles')
