"""Competition strategy package with task-specific entry points."""

from .task0 import Task0Config, Task0Program, Task0State
from .task1 import (
    Task1Config,
    Task1Program,
    Task1Round2Config,
    Task1Round2Program,
    Task1State,
)
from .task2 import (
    Task2Config,
    Task2DebugConfig,
    Task2DebugProgram,
    Task2Program,
    Task2Round2Config,
    Task2Round2Program,
    Task2State,
)

# Compatibility for existing tools; new code should use the Task1 names.
CompetitionProgram = Task1Program
CompetitionState = Task1State
FirstTaskConfig = Task1Config

__all__ = [
    'Task0Config',
    'Task0Program',
    'Task0State',
    'Task1Config',
    'Task1Program',
    'Task1Round2Config',
    'Task1Round2Program',
    'Task1State',
    'Task2DebugConfig',
    'Task2DebugProgram',
    'Task2Config',
    'Task2Program',
    'Task2Round2Config',
    'Task2Round2Program',
    'Task2State',
    'CompetitionProgram',
    'CompetitionState',
    'FirstTaskConfig',
]
