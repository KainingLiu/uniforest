"""Task selection and sequencing for the official competition entry."""

from .task1 import Task1Program
from .task2 import Task2Program


TASK_CHOICES = ('all', 'task1', 'task2')


def run_tasks(robot, selection='all', *,
              task1_factory=Task1Program,
              task2_factory=Task2Program) -> int:
    if selection not in TASK_CHOICES:
        raise ValueError(f'unknown task selection: {selection}')

    sequence = []
    if selection in ('all', 'task1'):
        sequence.append(('Task1', task1_factory))
    if selection in ('all', 'task2'):
        sequence.append(('Task2', task2_factory))

    for label, factory in sequence:
        print(f'[Competition] Starting {label}')
        result = factory(robot).run()
        if result != 0:
            print(f'[Competition] {label} failed with code {result}')
            return result
        print(f'[Competition] {label} complete')
    return 0


__all__ = ['TASK_CHOICES', 'run_tasks']
