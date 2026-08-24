"""Stable task result vocabulary while preserving legacy integer returns."""

from dataclasses import dataclass
from enum import Enum


class TaskStatus(str, Enum):
    SUCCESS = 'success'
    MOTION_DEGRADED = 'motion_degraded'
    HARDWARE_FAULT = 'hardware_fault'
    PERCEPTION_FAULT = 'perception_fault'
    STRATEGY_FAULT = 'strategy_fault'
    CANCELLED = 'cancelled'


def status_from_return_code(code: int) -> TaskStatus:
    return TaskStatus.SUCCESS if code == 0 else TaskStatus.STRATEGY_FAULT


@dataclass(frozen=True)
class TaskResult:
    """Structured task outcome; ``code`` preserves the legacy API."""
    status: TaskStatus
    code: int = 0
    task: str = ''
    message: str = ''

    @classmethod
    def from_code(cls, code: int, *, task: str = '', message: str = ''):
        return cls(status_from_return_code(code), code, task, message)

    @property
    def ok(self) -> bool:
        return self.status is TaskStatus.SUCCESS


__all__ = ['TaskStatus', 'TaskResult', 'status_from_return_code']
