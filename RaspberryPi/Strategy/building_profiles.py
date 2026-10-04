"""Measured top-edge distance scales for PlanD's three base heights.

The one/two-layer defaults intentionally carry no scale.  A synthetic test
profile exercises selection and control only; it is not a field calibration.
"""

from dataclasses import dataclass
from datetime import date
import json
import math
from pathlib import Path
from typing import Optional


DEFAULT_BUILDING_PROFILES_PATH = Path(__file__).with_name('building_profiles.json')


@dataclass(frozen=True)
class BuildingProfile:
    base_height: int
    status: str
    z_scale_mm_px: Optional[float]
    calibration_date: str = ''
    measured_distance_mm: Optional[float] = None
    top_row_px: Optional[float] = None
    result: str = ''

    @property
    def available(self):
        return self.status in ('verified', 'legacy')


def _positive_number(value, name):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value <= 0):
        raise ValueError(f'{name} must be a positive finite number')
    return float(value)


def load_building_profiles(path=DEFAULT_BUILDING_PROFILES_PATH):
    """Validate an explicit per-height JSON calibration before chassis motion."""
    with Path(path).open(encoding='utf-8') as source:
        data = json.load(source)
    if (not isinstance(data, dict) or type(data.get('schema_version')) is not int
            or data['schema_version'] != 1):
        raise ValueError('building profiles require schema_version 1')
    profiles = data.get('profiles')
    if not isinstance(profiles, dict) or set(profiles) != {'1', '2', '3'}:
        raise ValueError('building profiles must contain heights 1, 2 and 3')
    result = {}
    for height in (1, 2, 3):
        entry = profiles[str(height)]
        if not isinstance(entry, dict):
            raise ValueError(f'building profile {height} must be an object')
        status = entry.get('status')
        scale = entry.get('z_scale_mm_px')
        if status == 'unverified':
            if scale is not None:
                raise ValueError(f'unverified building profile {height} must have a null scale')
            result[height] = BuildingProfile(height, status, None)
            continue
        if status == 'legacy' and height == 3:
            result[height] = BuildingProfile(
                height, status, _positive_number(scale, 'z_scale_mm_px'))
            continue
        if status != 'verified':
            raise ValueError(f'building profile {height} has an invalid status')
        scale = _positive_number(scale, 'z_scale_mm_px')
        distance = _positive_number(entry.get('measured_distance_mm'), 'measured_distance_mm')
        row = _positive_number(entry.get('top_row_px'), 'top_row_px')
        # Task3/5's camera intrinsics and top-edge coordinates are 640x480.
        if row >= 480:
            raise ValueError('top_row_px must be inside the calibrated 480 px frame')
        if not math.isclose(scale, distance * row, rel_tol=1e-6):
            raise ValueError('building scale must equal measured_distance_mm * top_row_px')
        measured_date = entry.get('calibration_date')
        if not isinstance(measured_date, str):
            raise ValueError('verified building profile requires calibration_date')
        if date.fromisoformat(measured_date).isoformat() != measured_date:
            raise ValueError('calibration_date must use YYYY-MM-DD')
        observation = entry.get('result')
        if not isinstance(observation, str) or not observation.strip():
            raise ValueError('verified building profile requires a field result')
        result[height] = BuildingProfile(
            height, status, scale, measured_date, distance, row, observation)
    return result


def validate_building_profile_targets(profiles, target_z_mm):
    """Reject image-unreachable alignment targets before picking up saved cargo.

    Task3/5 use a 480 px image and clamp the inverse-row denominator to 1.
    A mathematically valid single-distance calibration can still require an
    invisible top edge at the requested placement distance.
    """
    target = _positive_number(target_z_mm, 'building_target_z_mm')
    for height, profile in profiles.items():
        if not profile.available:
            continue
        scale = _positive_number(profile.z_scale_mm_px, 'z_scale_mm_px')
        target_row = scale / target
        if not math.isfinite(target_row) or not 1 <= target_row < 480:
            raise ValueError(
                f'building profile {height} cannot align at {target:g} mm: '
                f'target top row {target_row:g} must be >= 1 and < 480')


__all__ = ['BuildingProfile', 'DEFAULT_BUILDING_PROFILES_PATH', 'load_building_profiles',
           'validate_building_profile_targets']
