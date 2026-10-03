"""Named functional flow selections for field diagnostics."""
from . import StrategyPlan
from .recipes import (navigate, ground_delivery, mixed_collection,
                      build_and_return, wall_unload, return_orange, staged_building)

SEGMENTS = [
    StrategyPlan('depart-a', (navigate('depart.a','depart-a','depart_a',ends='ground_area'),)),
    StrategyPlan('depart-b', (navigate('depart.b','depart-b','depart_b',ends='delivery_exit'),)),
    StrategyPlan('return-orange', return_orange('return.orange'), 'delivery_exit', 180),
    StrategyPlan('build-staged', staged_building('staged'), 'delivery_exit', 180),
]
for i in (1,2,3):
    SEGMENTS.append(StrategyPlan(f'collect-orange-{i}', ground_delivery(f'ground.{i}',f'ground-{i}'), 'ground_area'))
    SEGMENTS.append(StrategyPlan(f'build-{i}', build_and_return(f'build.{i}',f'building-{i}'), 'build_approach',180,True))
for i in (1,2):
    SEGMENTS.append(StrategyPlan(f'collect-mixed-{i}',mixed_collection(f'mixed.{i}',f'highland-{i}'),'delivery_exit',180))
    SEGMENTS.append(StrategyPlan(f'unload-{i}',wall_unload(f'unload.{i}',f'unload-{i}'),'build_approach',180,True))
    SEGMENTS.append(StrategyPlan(f'collect-build-{i}', (
        *mixed_collection(f'mixed.{i}',f'highland-{i}'),
        *build_and_return(f'build.{i}',f'building-{i}')),'delivery_exit',180))
SEGMENTS = tuple(SEGMENTS)
