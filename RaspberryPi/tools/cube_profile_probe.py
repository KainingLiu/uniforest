#!/usr/bin/env python3
"""Compare cube detection profiles on a saved frame without robot motion."""

import argparse
import json
import os
import sys

import cv2

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from vision.cube_detector import (  # noqa: E402
    DETECTION_PROFILE_OVERRIDES,
    color_profiles_for,
    detect_all_blocks,
    roi_top_ratio_for,
    morphology_for,
    max_front_aspect_for,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('image')
    parser.add_argument(
        '--profile', action='append', dest='profiles',
        choices=sorted(DETECTION_PROFILE_OVERRIDES),
        help='profile to test; repeat to compare (default: all)')
    args = parser.parse_args()

    frame = cv2.imread(args.image)
    if frame is None:
        parser.error(f'cannot read image: {args.image}')

    calib_path = os.path.join(PROJECT_ROOT, 'vision', 'camera_calib.json')
    with open(calib_path, 'r', encoding='utf-8') as stream:
        calib = json.load(stream)
    state = {
        'fx': float(calib['fx']),
        'fy': float(calib['fy']),
        'cx': frame.shape[1] / 2.0,
        'cy': frame.shape[0] / 2.0,
    }

    profiles = args.profiles or sorted(DETECTION_PROFILE_OVERRIDES)
    for profile_name in profiles:
        blocks = detect_all_blocks(
            frame, state, color_profiles_for(profile_name),
            roi_top_ratio=roi_top_ratio_for(profile_name),
            morph_kernel_size=morphology_for(profile_name)[0],
            morph_iterations=morphology_for(profile_name)[1],
            max_front_aspect_ratio=max_front_aspect_for(profile_name))
        print(f'{profile_name}: {len(blocks)} block(s)')
        for block in blocks:
            print(
                f'  {block.color_name}: x={block.x:+.1f} mm, '
                f'y={block.y:+.1f} mm, z={block.z:.1f} mm, '
                f'confidence={block.confidence:.1f}%, '
                f'h/w={block.height_width_ratio:.2f}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
