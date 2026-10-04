#!/usr/bin/env python3
"""Record a measured PlanD top-edge profile offline; never access the robot."""

import argparse
from datetime import date
import json
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from Strategy.building_profiles import (
    DEFAULT_BUILDING_PROFILES_PATH, load_building_profiles, validate_building_profile_targets,
)
from Strategy.task5 import Task5Config


def update_profile(path, *, layer, distance_mm, top_row_px, calibration_date, result,
                   write=False):
    """Validate a measured candidate, and only replace the file on explicit write."""
    path = Path(path)
    if type(layer) is not int or layer not in (1, 2):
        raise ValueError('PlanD calibration layer must be 1 or 2')
    load_building_profiles(path)
    data = json.loads(path.read_text(encoding='utf-8'))
    data['profiles'][str(layer)] = {
        'status': 'verified',
        'z_scale_mm_px': distance_mm * top_row_px,
        'calibration_date': calibration_date,
        'measured_distance_mm': distance_mm,
        'top_row_px': top_row_px,
        'result': result,
    }
    # Validate the exact serialized artifact before replacing any field data.
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', suffix='.json',
                                         dir=path.parent, delete=False) as candidate:
            temporary = Path(candidate.name)
            json.dump(data, candidate, ensure_ascii=False, indent=2, allow_nan=False)
            candidate.write('\n')
        profiles = load_building_profiles(temporary)
        validate_building_profile_targets(profiles, Task5Config().building_target_z_mm)
        if write:
            temporary.replace(path)
            temporary = None
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return data['profiles'][str(layer)]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profiles', type=Path, default=DEFAULT_BUILDING_PROFILES_PATH)
    parser.add_argument('--layer', type=int, choices=(1, 2), required=True)
    parser.add_argument('--distance-mm', type=float, required=True,
                        help='Measured horizontal robot-to-building distance in mm')
    parser.add_argument('--top-row-px', type=float, required=True,
                        help='Mean upper-edge row in the configured 640x480 cube image')
    parser.add_argument('--date', default=date.today().isoformat(),
                        help='Field measurement date (YYYY-MM-DD)')
    parser.add_argument('--result', required=True,
                        help='Actual field observation; do not claim untested landing results')
    parser.add_argument('--write', action='store_true',
                        help='Save the measured profile; default only previews it')
    args = parser.parse_args(argv)
    try:
        profile = update_profile(
            args.profiles, layer=args.layer, distance_mm=args.distance_mm,
            top_row_px=args.top_row_px, calibration_date=args.date,
            result=args.result, write=args.write)
    except (ValueError, OSError, TypeError) as exc:
        parser.error(str(exc))
    print(json.dumps({'layer': args.layer, 'saved': args.write, 'profile': profile},
                     ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
