"""PlanA explicitly alternates resource collection and building."""
from . import StrategyPlan
from .recipes import navigate, ground_delivery, mixed_collection, build_and_return


def round_steps(index):
    highland = 1 if index == 1 else 2
    return (*ground_delivery(f'a.{index}.ground', f'ground-{index}'),
            *mixed_collection(f'a.{index}.mixed', f'highland-{highland}'),
            *build_and_return(f'a.{index}.tower', f'building-{index}'))

SET1 = round_steps(1)
SET2 = round_steps(2)
SET3 = round_steps(3)
DEPART = navigate('a.depart', 'depart-a', 'depart_a', requires='start', ends='ground_area')
PLAN_A = StrategyPlan('PlanA', (DEPART, *SET1, *SET2, *SET3),
                      description='Three rounds of collection, unloading and building')
SUPPORT_PLANS = (
    StrategyPlan('set1', (DEPART, *SET1)),
    StrategyPlan('set2', (DEPART, *SET2)),
)
