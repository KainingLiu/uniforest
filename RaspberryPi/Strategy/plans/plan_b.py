"""Task2/Task4 unloading, both Task1 routes, then Task5 building cycles."""

from . import StrategyPlan
from ..tasks import TaskStep


PLAN_B = StrategyPlan('PlanB', tuple(TaskStep(task_id) for task_id in (
    'task0-2', 'task2-1', 'task4-1', 'task2-2', 'task4-2',
    'task0-3', 'task1-1', 'task0-3', 'task1-2', 'task5')),
    'Task2/Task4 unloading, both Task1 collection routes and Task5 building')
