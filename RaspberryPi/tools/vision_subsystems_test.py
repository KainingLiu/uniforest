#!/usr/bin/env python3
"""Run cube and tag vision together without connecting robot actuators."""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision import CubeDetector, FieldLocalizer


def main() -> int:
    parser = argparse.ArgumentParser(description="Concurrent dual-camera test")
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()
    cube = CubeDetector(camera_id="cube", show_gui=False)
    field = FieldLocalizer(camera="tag", show_gui=False)
    if not cube.start():
        return 1
    try:
        if not field.start():
            return 1
        started = time.monotonic()
        while time.monotonic() - started < args.duration:
            time.sleep(0.05)
        cube_result = cube.result
        field_result = field.result
        cube_fps = cube_result.fps if cube_result is not None else 0.0
        field_fps = field_result.fps if field_result is not None else 0.0
        print(f"DUAL_VISION_OK cube_fps={cube_fps:.1f} "
              f"tag_fps={field_fps:.1f} "
              f"field_valid={bool(field_result and field_result.valid)} "
              f"tag_ids={field_result.tag_ids if field_result else ()}")
        return 0 if cube_fps > 0.0 and field_fps > 0.0 else 1
    finally:
        field.stop()
        cube.stop()


if __name__ == "__main__":
    sys.exit(main())
