"""PlanA: original competition followed by the third collection/build cycle."""

from . import StrategyPlan
from ..tasks import TaskStep


SET1 = (TaskStep('task1-1'), TaskStep('task2-1'), TaskStep('task3-1'))
SET2 = (TaskStep('task1-2'), TaskStep('task2-2'), TaskStep('task3-2'))
SET3 = (TaskStep('task1-3'), TaskStep('task2-2'), TaskStep('task3-3'))

PLAN_A = StrategyPlan('PlanA', (TaskStep('task0-1'), *SET1, *SET2, *SET3),
                      'Three collection/build cycles with reusable task variants')
SUPPORT_PLANS = (
    StrategyPlan('set1', (TaskStep('task0-1'), *SET1), 'Task variant 1 sequence'),
    StrategyPlan('set2', (TaskStep('task0-1'), *SET2), 'Task variant 2 sequence'),
    StrategyPlan('collect-build-1', SET1[1:], 'Task2-1 followed by Task3-1'),
    StrategyPlan('collect-build-2', SET2[1:], 'Task2-2 followed by Task3-2'),
)
