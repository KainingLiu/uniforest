"""Stable Task1 strategy API.

The implementation remains in ``competition.py`` for compatibility with
existing deployments. New code should import the explicit Task1 names here.
"""

from dataclasses import dataclass

from .competition import (
    CompetitionProgram,
    CompetitionState,
    FirstTaskConfig,
)

Task1Program = CompetitionProgram
Task1State = CompetitionState
Task1Config = FirstTaskConfig


@dataclass(frozen=True)
class Task1Round2Config(FirstTaskConfig):
    delivery_forward_base_mm: float = 2500.0
    post_tag_lateral_right_mm: float = 400.0
    pre_final_turn_lateral_left_mm: float = 400.0


class Task1Round2Program(CompetitionProgram):
    TASK_LABEL = 'Task1-R2'

    def __init__(self, robot,
                 config: Task1Round2Config = Task1Round2Config()):
        super().__init__(robot, config)


__all__ = [
    'Task1Config',
    'Task1Program',
    'Task1Round2Config',
    'Task1Round2Program',
    'Task1State',
]
