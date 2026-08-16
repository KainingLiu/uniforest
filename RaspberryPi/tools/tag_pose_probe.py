#!/usr/bin/env python3
"""Print raw AprilTag PnP candidates for field-map calibration."""

import argparse
from collections import Counter
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision.camera_devices import resolve_camera_source
from vision.field_localizer import (
    _candidate_to_world,
    _object_points,
    _reprojection_error,
    load_camera_model,
    load_field_config,
    solve_tag_pose,
    DEFAULT_CALIB_FILE,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect raw tag pose candidates")
    parser.add_argument("--camera", default="tag")
    parser.add_argument("--frames", type=int, default=30)
    args = parser.parse_args()

    config = load_field_config()
    width, height = map(int, config["camera"]["resolution"])
    source = resolve_camera_source(args.camera)
    cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, config["tag_family"]))
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    best = ([], None)
    successful_reads = 0
    detected_frames = 0
    tag_counts = Counter()
    try:
        for _ in range(max(1, args.frames)):
            ok, frame = cap.read()
            if not ok:
                continue
            successful_reads += 1
            corners, ids, _ = detector.detectMarkers(frame)
            if ids is not None:
                detected_frames += 1
                tag_counts.update(int(value) for value in ids.flatten())
            if ids is not None and len(ids) > len(best[0]):
                best = (list(zip(corners, ids.flatten())), frame.shape[:2])
    finally:
        cap.release()
    rate = (100.0 * detected_frames / successful_reads
            if successful_reads else 0.0)
    print(f"TAG_PROBE_STATS reads={successful_reads}/{max(1, args.frames)} "
          f"detected_frames={detected_frames} rate={rate:.1f}% "
          f"tag_counts={dict(sorted(tag_counts.items()))}")
    detections, shape = best
    if not detections or shape is None:
        print("TAG_PROBE no tags detected")
        return 1

    actual_h, actual_w = shape
    camera_matrix, dist_coeffs, calibrated = load_camera_model(
        DEFAULT_CALIB_FILE, actual_w, actual_h)
    obj_pts = _object_points(config["tag_size_m"])
    print(f"TAG_PROBE source={source} size={actual_w}x{actual_h} "
          f"calibrated={calibrated}")
    for raw_corners, raw_id in detections:
        tag_id = int(raw_id)
        points = raw_corners.reshape(4, 2).astype(np.float32)
        order = np.asarray(config["solver"]["corner_order"], dtype=np.int32)
        points = points[order]
        count, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            obj_pts, points.reshape(4, 1, 2), camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        print(f"tag={tag_id} corners={np.round(points, 1).tolist()} candidates={count}")
        tag = config["tags"].get(str(tag_id))
        if tag is None:
            print("  ignored: tag is not in field_map.json")
            continue
        for index, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
            (x_m, y_m, yaw_deg, height_m, distance_m,
             lateral_m, relative_yaw) = _candidate_to_world(
                rvec, tvec, tag, config)
            error = _reprojection_error(
                obj_pts, points.reshape(4, 1, 2), rvec, tvec,
                camera_matrix, dist_coeffs)
            print(f"  candidate={index} x={x_m:+.3f} y={y_m:+.3f} "
                  f"yaw={yaw_deg:+.1f} height={height_m:+.3f} "
                  f"distance={distance_m:+.3f} lateral={lateral_m:+.3f} "
                  f"relative_yaw={relative_yaw:+.1f} reproj={error:.3f}px")
        accepted = solve_tag_pose(
            points, tag_id, config, camera_matrix, dist_coeffs)
        print(f"  accepted={accepted}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
