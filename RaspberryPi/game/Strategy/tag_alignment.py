"""Pure AprilTag translation helpers shared by delivery controllers."""

from __future__ import annotations

from statistics import median
from typing import Iterable, Optional, Tuple


Translation = Tuple[float, float]


def translation_jump(previous: Optional[Translation], current: Translation,
                     max_distance_mm: float,
                     max_lateral_mm: float) -> Optional[Translation]:
    """Return (distance jump, lateral jump), or None when within limits."""
    if previous is None:
        return None
    jump = (abs(current[0] - previous[0]),
            abs(current[1] - previous[1]))
    if jump[0] <= max_distance_mm and jump[1] <= max_lateral_mm:
        return None
    return jump


def median_translation(samples: Iterable[Translation]) -> Translation:
    """Compute a robust translation sample from recent valid observations."""
    values = list(samples)
    if not values:
        raise ValueError('at least one translation sample is required')
    return (median(item[0] for item in values),
            median(item[1] for item in values))


__all__ = ['Translation', 'median_translation', 'translation_jump']
