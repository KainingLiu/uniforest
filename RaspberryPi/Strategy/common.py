"""Pure strategy-control helpers shared by Task1 and Task2."""

from __future__ import annotations


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


__all__ = ['minimum_command', 'slew_command', 'wrap_angle']
