"""PlanB gathers/stages resources before the two building cycles."""
from . import StrategyPlan
from .recipes import (navigate, ground_delivery, mixed_collection,
                      wall_unload, return_orange, staged_building)

PLAN_B = StrategyPlan('PlanB', (
    navigate('b.depart', 'depart-b', 'depart_b', requires='start', ends='delivery_exit'),
    *mixed_collection('b.mixed.1', 'highland-1'),
    *wall_unload('b.stage.1', 'unload-1'),
    *mixed_collection('b.mixed.2', 'highland-2'),
    *wall_unload('b.stage.2', 'unload-2'),
    *return_orange('b.return.1'),
    *ground_delivery('b.ground.1', 'ground-1'),
    *return_orange('b.return.2'),
    *ground_delivery('b.ground.2', 'ground-2'),
    *staged_building('b.towers'),
), description='Fixed collection batches first, then staged building')
