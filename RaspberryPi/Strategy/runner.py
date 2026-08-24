"""Task selection and sequencing for the official competition entry."""

from .task0 import Task0Program
from .task1 import Task1Program, Task1Round2Program
from .task2 import Task2Program, Task2Round2Program
from .results import TaskResult


TASK_CHOICES = (
    'all',
    'round1',
    'round2',
    'task1',
    'task2',
    'task1-r1',
    'task2-r1',
    'task1-r2',
    'task2-r2',
)


def run_tasks(robot, selection='all', *,
              task0_factory=Task0Program,
              task1_factory=Task1Program,
              task2_factory=Task2Program,
              task1_round2_factory=Task1Round2Program,
              task2_round2_factory=Task2Round2Program) -> int:
    if selection not in TASK_CHOICES:
        raise ValueError(f'unknown task selection: {selection}')

    round1 = [
        ('Task1-R1', task1_factory),
        ('Task2-R1', task2_factory),
    ]
    round2 = [
        ('Task1-R2', task1_round2_factory),
        ('Task2-R2', task2_round2_factory),
    ]
    selections = {
        'all': [('Task0', task0_factory), *round1, *round2],
        'round1': round1,
        'round2': round2,
        'task1': [round1[0]],
        'task2': [round1[1]],
        'task1-r1': [round1[0]],
        'task2-r1': [round1[1]],
        'task1-r2': [round2[0]],
        'task2-r2': [round2[1]],
    }
    sequence = selections[selection]

    for label, factory in sequence:
        print(f'[Competition] Starting {label}')
        diagnostics = getattr(robot, 'diagnostics', None)
        if diagnostics is not None:
            diagnostics.write('task_start', task=label,
                              selection=selection)
        raw_result = factory(robot).run()
        outcome = (raw_result if isinstance(raw_result, TaskResult)
                   else TaskResult.from_code(int(raw_result), task=label))
        result = outcome.code
        if not outcome.ok:
            print(f'[Competition] {label} failed with code {result} '
                  f'({outcome.status.value})')
            if diagnostics is not None:
                diagnostics.write(
                    'task_failed', task=label, code=result,
                    status=outcome.status.value,
                    message=outcome.message)
            return result
        print(f'[Competition] {label} complete')
        if diagnostics is not None:
            diagnostics.write('task_complete', task=label,
                              status=outcome.status.value)
    return 0


__all__ = ['TASK_CHOICES', 'run_tasks']
