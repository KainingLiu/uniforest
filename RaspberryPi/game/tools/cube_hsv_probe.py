#!/usr/bin/env python3
"""Capture a cube-camera frame for offline HSV threshold diagnosis."""

from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision.camera_devices import resolve_camera_source


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', default='cube')
    parser.add_argument('--output', default='/tmp/cube_hsv_probe.jpg')
    parser.add_argument('--warmup-frames', type=int, default=90)
    parser.add_argument('--roi',
                        help='Optional HSV statistics ROI: x0,y0,x1,y1')
    args = parser.parse_args()

    source = resolve_camera_source(args.camera)
    cap = cv2.VideoCapture(source)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        raise RuntimeError(f'cannot open cube camera {source}')

    frame = None
    try:
        for _ in range(max(1, args.warmup_frames)):
            ok, candidate = cap.read()
            if ok:
                frame = candidate
    finally:
        cap.release()
    if frame is None:
        raise RuntimeError('cube camera returned no frame')

    height, width = frame.shape[:2]
    if args.roi:
        x0, y0, x1, y1 = (int(value) for value in args.roi.split(','))
        if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height):
            raise ValueError(f'invalid ROI {args.roi!r} for {width}x{height}')
    else:
        x0, x1 = width // 4, 3 * width // 4
        y0, y1 = height // 4, 3 * height // 4
    roi = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    saturated = roi[:, :, 1] >= 50
    pixels = roi[saturated]
    if len(pixels):
        percentiles = np.percentile(pixels, [5, 25, 50, 75, 95], axis=0)
        print(f'ROI={x0},{y0},{x1},{y1} HSV_PERCENTILES '
              'p05,p25,p50,p75,p95')
        for percentile, values in zip((5, 25, 50, 75, 95), percentiles):
            print(f'  p{percentile:02d}: H={values[0]:.0f} '
                  f'S={values[1]:.0f} V={values[2]:.0f}')
    else:
        print('CENTER_HSV_PERCENTILES unavailable: no S>=50 pixels')

    if not cv2.imwrite(args.output, frame):
        raise RuntimeError(f'cannot write frame to {args.output}')
    print(f'FRAME={args.output} SIZE={width}x{height}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
