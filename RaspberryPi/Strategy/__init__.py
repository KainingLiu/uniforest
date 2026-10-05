"""Competition strategy package with task-specific entry points."""

from .task0 import (Task0Config, Task0Program, Task0State, Task0_1Config,
                    Task0_1Program, Task0_2Config, Task0_2Program,
                    Task0_3Config, Task0_3Program)
from .task1 import (
    Task1Config,
    Task1Program,
    Task1_0Config,
    Task1_0Program,
    Task1Round2Config,
    Task1Round2Program,
    Task1State,
    Task1_2Config,
    Task1_2Program,
    Task1_3Config,
    Task1_3Program,
)
from .task2 import (
    Task2Config,
    Task2DebugConfig,
    Task2DebugProgram,
    Task2Program,
    Task2Round2Config,
    Task2Round2Program,
    Task2State,
    Task2_2Config,
    Task2_2Program,
    Task2_0Config,
    Task2_0Program,
)
from .task3 import (Task3Config, Task3Program, Task3State, Task3_2Config, Task3_2Program,
                    Task3_3Config, Task3_3Program, Task3_4Config, Task3_4Program,
                    Task3_5Config, Task3_5Program)
from .task4 import Task4Config, Task4Program, Task4State, Task4_2Config, Task4_2Program
from .task5 import Task5Config, Task5Program, Task5State
from .context import TaskContext, BuildApproach

# Compatibility for existing tools; new code should use the Task1 names.
CompetitionProgram = Task1Program
CompetitionState = Task1State
FirstTaskConfig = Task1Config

__all__ = [
    'TaskContext', 'BuildApproach',
    'Task5Config', 'Task5Program', 'Task5State',
    'Task0_1Config', 'Task0_1Program', 'Task0_2Config', 'Task0_2Program',
    'Task0_3Config', 'Task0_3Program',
    'Task4Config', 'Task4Program', 'Task4State', 'Task4_2Config', 'Task4_2Program',
    'Task1_2Config', 'Task1_2Program', 'Task2_2Config', 'Task2_2Program',
    'Task1_0Config', 'Task1_0Program',
    'Task2_0Config', 'Task2_0Program',
    'Task1_3Config', 'Task1_3Program', 'Task3_3Config', 'Task3_3Program',
    'Task3Config', 'Task3Program', 'Task3State', 'Task3_2Config', 'Task3_2Program',
    'Task3_4Config', 'Task3_4Program', 'Task3_5Config', 'Task3_5Program',
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
