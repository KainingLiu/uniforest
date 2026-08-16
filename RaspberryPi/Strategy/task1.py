"""Stable Task1 strategy API.

The implementation remains in ``competition.py`` for compatibility with
existing deployments. New code should import the explicit Task1 names here.
"""

from .competition import (
    CompetitionProgram,
    CompetitionState,
    FirstTaskConfig,
)

Task1Program = CompetitionProgram
Task1State = CompetitionState
Task1Config = FirstTaskConfig

__all__ = ['Task1Program', 'Task1State', 'Task1Config']
