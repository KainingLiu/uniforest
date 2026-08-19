#!/usr/bin/env python3
"""Measure stacked-orange building observations without moving the robot."""

from __future__ import annotations

import argparse
import math
import os
from statistics import median
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision import CubeDetector


def quad_height_width_ratio(quad) -> float:
    if quad is None or len(quad) != 4:
        return 0.0
    width = (math.dist(quad[0], quad[1]) + math.dist(quad[3], quad[2])) / 2
    height = (math.dist(quad[0], quad[3]) + math.dist(quad[1], quad[2])) / 2
    return height / max(width, 1e-6)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', default='cube')
    parser.add_argument('--duration', type=float, default=5.0)
    args = parser.parse_args()

    detector = CubeDetector(camera_id=args.camera, show_gui=False)
    if not detector.start():
        return 1

    samples = []
    last_timestamp = None
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            result = detector.result
            if result is None or result.timestamp == last_timestamp:
                time.sleep(0.01)
                continue
            last_timestamp = result.timestamp
            oranges = [block for block in result.all_blocks
                       if block.color_name.casefold() == 'orange']
            if oranges:
                block = max(oranges, key=lambda item:
                            quad_height_width_ratio(item.quad))
                ratio = block.height_width_ratio
                samples.append((block.x, block.z, block.confidence, ratio))
                print(f'BUILDING_SAMPLE x={block.x:+.2f} z={block.z:.2f} '
                      f'confidence={block.confidence:.1f} '
                      f'height_width={ratio:.3f}')
            time.sleep(0.01)
    finally:
        detector.stop()

    if not samples:
        print('BUILDING_NOT_FOUND')
        return 1
    print('BUILDING_MEDIAN '
          f'x={median(item[0] for item in samples):+.2f} '
          f'z={median(item[1] for item in samples):.2f} '
          f'confidence={median(item[2] for item in samples):.1f} '
          f'height_width={median(item[3] for item in samples):.3f} '
          f'samples={len(samples)}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
