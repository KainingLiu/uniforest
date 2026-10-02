"""Pure wall-approach helpers shared by Task1 and Task2."""

from __future__ import annotations


def velocity_for_direction(direction: str, speed_mm_s: float):
    """Return mecanum vx/vy in cm/s for a named wall direction."""
    speed_cm_s = speed_mm_s / 10.0
    values = {
        'forward': (speed_cm_s, 0.0),
        'backward': (-speed_cm_s, 0.0),
        'left': (0.0, -speed_cm_s),
        'right': (0.0, speed_cm_s),
    }
    try:
        return values[direction]
    except KeyError as exc:
        raise ValueError(
            'wall direction must be forward, backward, left, or right') from exc


__all__ = ['velocity_for_direction']
