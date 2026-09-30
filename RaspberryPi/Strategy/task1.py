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
class Task1_2Config(FirstTaskConfig):
    post_tag_lateral_right_mm: float = 400.0
    pre_final_turn_lateral_left_mm: float = 400.0


class Task1_2Program(CompetitionProgram):
    TASK_LABEL = 'task1-2'

    def __init__(self, robot,
                 config: Task1_2Config = Task1_2Config(), *, context=None):
        super().__init__(robot, config, context=context)


@dataclass(frozen=True)
class Task1_3Config(FirstTaskConfig):
    post_tag_lateral_right_mm: float = 500.0
    post_tag_lateral_direction: str = 'left'
    pre_final_turn_lateral_left_mm: float = 500.0
    pre_final_turn_lateral_direction: str = 'right'


class Task1_3Program(CompetitionProgram):
    TASK_LABEL = 'task1-3'

    def __init__(self, robot,
                 config: Task1_3Config = Task1_3Config(), *, context=None):
        super().__init__(robot, config, context=context)


__all__ = [
    'Task1Config',
    'Task1Program',
    'Task1_2Config',
    'Task1_2Program',
    'Task1_3Config',
    'Task1_3Program',
    'Task1State',
]

# Compatibility imports only; user-facing IDs are task1-1, task1-2 and task1-3.
Task1Round2Config = Task1_2Config
Task1Round2Program = Task1_2Program
