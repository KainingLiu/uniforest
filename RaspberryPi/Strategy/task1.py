"""Stable Task1 strategy API.

The implementation remains in ``competition.py`` for compatibility with
existing deployments. New code should import the explicit Task1 names here.
"""

from dataclasses import dataclass

from .competition import (
    CompetitionProgram,
    CompetitionState,
    FirstTaskConfig,
    SearchRangeExhausted,
)
from .common import report_visual_fallback

Task1Program = CompetitionProgram
Task1State = CompetitionState
Task1Config = FirstTaskConfig


@dataclass(frozen=True)
class Task1_0Config(FirstTaskConfig):
    # Requested new Grap3 operations, not a final carried-count target.
    target_cube_count: int = 3

    def __post_init__(self):
        if type(self.target_cube_count) is not int or not 1 <= self.target_cube_count <= 3:
            raise ValueError('task1-0 target_cube_count must be an integer from 1 to 3')


class Task1_0Program(CompetitionProgram):
    """Collect a requested orange count and stop immediately before Tag6 alignment."""

    TASK_LABEL = 'task1-0'

    def __init__(self, robot,
                 config: Task1_0Config = Task1_0Config(), *, context=None):
        super().__init__(robot, config, context=context)
        self.completed_grabs = 0
        self.search_exhausted = False

    def _collect_task1_orange(self, *, chassis_followup=None):
        # Preserve the requested pickup amount; inspect an exhausted exit only.
        # The common delivery approach performs the reverse after collection.
        self.completed_grabs = 0
        self.search_exhausted = False
        for index in range(self.config.target_cube_count):
            self._check_active()
            print(f'[{self.TASK_LABEL}] Orange pickup '
                  f'{index + 1}/{self.config.target_cube_count}')
            try:
                self._grab_task1_orange()
            except SearchRangeExhausted:
                self._check_active()
                self.search_exhausted = True
                report_visual_fallback(
                    self.robot, self.TASK_LABEL, 'orange collection',
                    'search budget exhausted; continue the route to the Tag6 approach')
                break
            self._check_active()
            self.completed_grabs += 1
        print(f'[{self.TASK_LABEL}] Grap3 operations completed: '
              f'{self.completed_grabs}/{self.config.target_cube_count}')

    def _run_delivery_route(self, *, reverse_done=False):
        build_count = None
        if self.search_exhausted:
            count = self._inspect_during_exit(
                lambda: self._run_delivery_approach(reverse_done=reverse_done),
                request_refill=False)
            # The confirmed visual-failure policy is one missing cube, not a stop.
            build_count = 2 if count is None else count
            print(f'[{self.TASK_LABEL}] Refill exit count={count}; '
                  f'next Task3 build count={build_count}; no further refill')
            diagnostics = getattr(self.robot, 'diagnostics', None)
            if diagnostics is not None:
                diagnostics.write('refill_build_count', task=self.TASK_LABEL,
                                  observed_count=count, build_count=build_count,
                                  visual_fallback=count is None)
        else:
            self._run_delivery_approach(reverse_done=reverse_done)
        self._check_active()
        if self.context is not None:
            # Both refill exits reach the Tag6 approach at heading 180.
            # Use the new wall-calibrated zero, never the pre-detour handoff.
            # Publish the count only after inspection AND arm restore finish.
            self.context.publish_build_approach(
                self._heading_zero_deg, carried_cube_count=build_count)


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
    'Task1_0Config',
    'Task1_0Program',
    'Task1_2Config',
    'Task1_2Program',
    'Task1_3Config',
    'Task1_3Program',
    'Task1State',
]

# Compatibility imports only; canonical variants are task1-0/1/2/3.
Task1Round2Config = Task1_2Config
Task1Round2Program = Task1_2Program
