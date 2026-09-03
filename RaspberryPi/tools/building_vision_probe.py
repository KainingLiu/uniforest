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
from Strategy.task2 import Task2Config


_BUILD_CFG = Task2Config()


def top_left_u(quad) -> float:
    if quad is None or len(quad) != 4:
        return 320.0
    return (quad[0][0] + quad[1][0]) / 2.0


def quad_height_width_ratio(quad) -> float:
    if quad is None or len(quad) != 4:
        return 0.0
    width = (math.dist(quad[0], quad[1]) + math.dist(quad[3], quad[2])) / 2
    height = (math.dist(quad[0], quad[3]) + math.dist(quad[1], quad[2])) / 2
    return height / max(width, 1e-6)


def top_edge_row(quad) -> float:
    """Return the pixel row of the visible upper edge (quad rows 0/1)."""
    if quad is None or len(quad) != 4:
        return 0.0
    return (quad[0][1] + quad[1][1]) / 2.0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', default='cube')
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--profile', default='building',
                        help="cube detection profile; use 'building' to see "
                             'the bright top surface, or default/task2_orange')
    args = parser.parse_args()

    detector = CubeDetector(camera_id=args.camera, show_gui=False)
    if not detector.start():
        return 1
    detector.set_detection_profile(args.profile)

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
                row = top_edge_row(block.quad)
                z_model = _BUILD_CFG.building_z_scale_mm_px / max(row, 1.0)
                x_model = ((top_left_u(block.quad)
                            - _BUILD_CFG.building_reference_cx_px)
                           * z_model / _BUILD_CFG.building_reference_fx_px)
                samples.append((block.x, block.z, block.confidence, ratio,
                                row, z_model, x_model))
                print(f'BUILDING_SAMPLE x={block.x:+.2f} z={block.z:.2f} '
                      f'top_row={row:.2f} z_model={z_model:.2f} '
                      f'x_model={x_model:+.2f} '
                      f'confidence={block.confidence:.1f} '
                      f'height_width={ratio:.3f}')
            time.sleep(0.01)
    finally:
        detector.stop()

    if not samples:
        print('BUILDING_NOT_FOUND')
        return 1
    sample = lambda index: median(item[index] for item in samples)
    print('BUILDING_MEDIAN '
          f'x={sample(0):+.2f} z={sample(1):.2f} '
          f'top_row={sample(4):.2f} z_model={sample(5):.2f} '
          f'x_model={sample(6):+.2f} confidence={sample(2):.1f} '
          f'height_width={sample(3):.3f} samples={len(samples)}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
