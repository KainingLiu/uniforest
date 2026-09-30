"""Shared task library. Plans refer to these IDs instead of copying routes."""

from dataclasses import dataclass, field, replace
from types import MappingProxyType
from typing import Mapping, Optional

from ..task0 import (Task0_1Config, Task0_1Program, Task0_2Config, Task0_2Program,
                     Task0_3Config, Task0_3Program)
from ..task1 import (Task1Config, Task1Program, Task1_2Config, Task1_2Program,
                     Task1_3Config, Task1_3Program)
from ..task2 import Task2Config, Task2Program, Task2_2Config, Task2_2Program
from ..task3 import (Task3Config, Task3Program, Task3_2Config, Task3_2Program,
                     Task3_3Config, Task3_3Program)
from ..task4 import Task4Config, Task4Program, Task4_2Config, Task4_2Program
from ..task5 import Task5Config, Task5Program


@dataclass(frozen=True)
class TaskStep:
    task_id: str
    parameters: Mapping = field(default_factory=dict)

    def __post_init__(self):
        object.__setattr__(self, 'parameters', MappingProxyType(dict(self.parameters)))


@dataclass(frozen=True)
class TaskDefinition:
    program_type: type
    config_type: type
    requires: Optional[str] = None
    provides: Optional[str] = None

    def configuration(self, step):
        # Each invocation gets a separate immutable configuration.
        return replace(self.config_type(), **step.parameters)

    def create(self, context, step):
        return self.program_type(context.robot, self.configuration(step), context=context)


TASK_LIBRARY = {
    'task0-1': TaskDefinition(Task0_1Program, Task0_1Config),
    'task0-2': TaskDefinition(Task0_2Program, Task0_2Config),
    'task0-3': TaskDefinition(Task0_3Program, Task0_3Config),
    'task1-1': TaskDefinition(Task1Program, Task1Config),
    'task1-2': TaskDefinition(Task1_2Program, Task1_2Config),
    'task1-3': TaskDefinition(Task1_3Program, Task1_3Config),
    'task2-1': TaskDefinition(Task2Program, Task2Config, provides='build_approach'),
    'task2-2': TaskDefinition(Task2_2Program, Task2_2Config, provides='build_approach'),
    'task3-1': TaskDefinition(Task3Program, Task3Config, requires='build_approach'),
    'task3-2': TaskDefinition(Task3_2Program, Task3_2Config, requires='build_approach'),
    'task3-3': TaskDefinition(Task3_3Program, Task3_3Config, requires='build_approach'),
    'task4-1': TaskDefinition(Task4Program, Task4Config, requires='build_approach'),
    'task4-2': TaskDefinition(Task4_2Program, Task4_2Config, requires='build_approach'),
    'task5': TaskDefinition(Task5Program, Task5Config),
}
TASK_IDS = tuple(TASK_LIBRARY)

__all__ = ['TaskStep', 'TaskDefinition', 'TASK_LIBRARY', 'TASK_IDS']
