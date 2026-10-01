"""Shared control math and visual-fallback diagnostics for functional actions."""

from __future__ import annotations


class VisualAlignmentUnavailable(RuntimeError):
    """A visual target was lost or its alignment time budget expired."""


def report_visual_fallback(robot, task, stage, reason):
    print(f'[{task}] Visual fallback ({stage}): {reason}; continue mission')
    diagnostics = getattr(robot, 'diagnostics', None)
    if diagnostics is not None:
        diagnostics.write('visual_fallback', task=task, stage=stage, reason=str(reason))


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


__all__ = ['VisualAlignmentUnavailable', 'report_visual_fallback',
           'minimum_command', 'slew_command', 'wrap_angle']
