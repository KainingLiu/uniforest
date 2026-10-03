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


def missing_tag_details(pose, tag_id: int, *, now: float,
                        max_age_s: float, not_before: float) -> str:
    """Describe the rejected observation without changing any control gates."""
    if pose is None:
        return 'reason=no_frame'
    timestamp = getattr(pose, 'timestamp', 0.0)
    age = now - timestamp
    raw = getattr(pose, 'raw_tag_ids', getattr(pose, 'tag_ids', ()))
    accepted = tuple(item.tag_id for item in getattr(pose, 'tag_solutions', ()))
    capture_error = getattr(pose, 'capture_error', '')
    if capture_error:
        reason = 'camera_unavailable'
    elif timestamp <= not_before:
        reason = 'no_new_frame_since_alignment'
    elif age > max_age_s:
        reason = 'stale_frame'
    elif tag_id not in raw:
        reason = 'tag_not_decoded'
    else:
        reason = 'pose_rejected'
    rejected = getattr(pose, 'rejection_reasons', ())
    return (f'reason={reason}, age={age:.3f}s, raw_ids={raw}, '
            f'accepted_ids={accepted}, rejected={rejected}, capture_error={capture_error}')


__all__ = ['Translation', 'median_translation', 'translation_jump',
           'missing_tag_details']
