"""Pure strategy-control helpers shared by Task1 and Task2."""

from __future__ import annotations


class TaskStateReporting:
    """Mirror existing state assignments into optional dataset metadata."""

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        self._state = value
        report = getattr(self.robot, 'set_collection_context', None)
        if report is not None:
            report(task=self.TASK_LABEL, phase=value.name)


def wrap_angle(angle_deg: float) -> float:
    """Normalize an angle to [-180, 180)."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def minimum_command(value: float, minimum: float) -> float:
    """Keep a non-zero control command above its actuator deadband."""
    if value == 0.0 or abs(value) >= minimum:
        return value
    return minimum if value > 0.0 else -minimum


def slew_command(target: float, current: float,
                 max_rate: float, dt: float) -> float:
    """Limit command acceleration while preserving its sign."""
    max_delta = max_rate * dt
    delta = max(-max_delta, min(max_delta, target - current))
    return current + delta


__all__ = ['TaskStateReporting', 'minimum_command', 'slew_command', 'wrap_angle']
