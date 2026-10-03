#!/usr/bin/env python3
"""Verify stable camera roles and capture frames without running strategy."""

import argparse
import os
import sys

import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision.camera_devices import resolve_camera_source


def main() -> int:
    parser = argparse.ArgumentParser(description="Test stable camera roles")
    parser.add_argument("roles", nargs="*", default=["cube", "tag"])
    parser.add_argument("--frames", type=int, default=10)
    args = parser.parse_args()

    resolved = {
        role: resolve_camera_source(role)
        for role in args.roles
    }
    real_paths = [os.path.realpath(str(source)) for source in resolved.values()]
    if len(real_paths) != len(set(real_paths)):
        print(f"CAMERA_ROLE_ERROR duplicate physical nodes: {resolved}")
        return 1

    failed = False
    for role, source in resolved.items():
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        ok = False
        frame = None
        try:
            for _ in range(max(1, args.frames)):
                ok, frame = cap.read()
                if ok:
                    break
        finally:
            cap.release()

        if not ok or frame is None:
            print(f"CAMERA_ROLE_FAIL role={role} source={source}")
            failed = True
            continue
        height, width = frame.shape[:2]
        print(f"CAMERA_ROLE_OK role={role} source={source} "
              f"node={os.path.realpath(str(source))} size={width}x{height}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
