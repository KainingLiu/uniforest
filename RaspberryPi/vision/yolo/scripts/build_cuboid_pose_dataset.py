#!/usr/bin/env python3
"""Build a geometry-assisted cuboid-keypoint dataset from reviewed masks.

The source camera is monocular RGB. Each reviewed visible mask is converted to
four image-plane reference corners. A known 100 mm cube and camera intrinsics
then produce a projected eight-corner cuboid for visualization and later PnP.
These are pose-assistance labels, not depth ground truth.
"""

from __future__ import annotations

from collections import Counter
import json
from pathlib import Path
import shutil

import cv2
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT / 'data/raw/collection_20260926'
SOURCE = ROOT / 'data/collection/collection_v1'
OUTPUT = ROOT / 'data/collection/cuboid_pose_v1'
REVIEWED = SOURCE / 'reviewed.jsonl'
SPLIT_MANIFEST = SOURCE / 'split_manifest.json'
CLASSES = {'orange_cube': 0, 'purple_cube': 1}
CUBE_MM = 100.0
OBJECT_FACE = np.asarray([
    [-CUBE_MM / 2, -CUBE_MM / 2, 0],
    [CUBE_MM / 2, -CUBE_MM / 2, 0],
    [CUBE_MM / 2, CUBE_MM / 2, 0],
    [-CUBE_MM / 2, CUBE_MM / 2, 0],
], dtype=np.float64)
OBJECT_CUBOID = np.asarray([
    [-CUBE_MM / 2, -CUBE_MM / 2, 0],
    [CUBE_MM / 2, -CUBE_MM / 2, 0],
    [CUBE_MM / 2, CUBE_MM / 2, 0],
    [-CUBE_MM / 2, CUBE_MM / 2, 0],
    [-CUBE_MM / 2, -CUBE_MM / 2, CUBE_MM],
    [CUBE_MM / 2, -CUBE_MM / 2, CUBE_MM],
    [CUBE_MM / 2, CUBE_MM / 2, CUBE_MM],
    [-CUBE_MM / 2, CUBE_MM / 2, CUBE_MM],
], dtype=np.float64)
EDGES = ((0, 1), (1, 2), (2, 3), (3, 0),
         (4, 5), (5, 6), (6, 7), (7, 4),
         (0, 4), (1, 5), (2, 6), (3, 7))


def order_corners(points):
    points = np.asarray(points, dtype=np.float32)
    rect = np.zeros((4, 2), dtype=np.float32)
    sums = points.sum(axis=1)
    diffs = np.diff(points, axis=1).ravel()
    rect[0] = points[np.argmin(sums)]
    rect[2] = points[np.argmax(sums)]
    rect[1] = points[np.argmin(diffs)]
    rect[3] = points[np.argmax(diffs)]
    return rect


def polygon_from_label(label, width, height):
    values = label.split()
    points = np.asarray([[float(values[i]) * width,
                          float(values[i + 1]) * height]
                         for i in range(1, len(values), 2)], dtype=np.float32)
    if len(points) < 3:
        raise ValueError('polygon has fewer than three points')
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [np.rint(points).astype(np.int32)], 255)
    return points, mask


def reference_quad(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise ValueError('mask has no contour')
    contour = max(contours, key=cv2.contourArea)
    hull = cv2.convexHull(contour)
    perimeter = cv2.arcLength(hull, True)
    for epsilon in (0.01, 0.02, 0.03, 0.04, 0.05):
        approx = cv2.approxPolyDP(hull, epsilon * perimeter, True)
        if len(approx) == 4:
            return order_corners(approx.reshape(4, 2)), 'contour_quad'
    rect = cv2.minAreaRect(hull)
    return order_corners(cv2.boxPoints(rect)), 'min_area_rect'


def camera_matrix(sidecar):
    calibration = sidecar['camera']['calibration']
    return np.asarray([[calibration['fx'], 0, calibration['cx']],
                       [0, calibration['fy'], calibration['cy']],
                       [0, 0, 1]], dtype=np.float64)


def fit_cuboid(quad, matrix):
    ok, rvec, tvec = cv2.solvePnP(
        OBJECT_FACE, quad.astype(np.float64), matrix, None,
        flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or float(tvec[2, 0]) <= 0:
        raise ValueError('PnP returned a camera-behind pose')
    projected_face, _ = cv2.projectPoints(OBJECT_FACE, rvec, tvec,
                                          matrix, None)
    error = float(np.linalg.norm(
        projected_face.reshape(4, 2) - quad, axis=1).mean())
    projected, _ = cv2.projectPoints(OBJECT_CUBOID, rvec, tvec,
                                     matrix, None)
    return projected.reshape(8, 2), error, rvec.reshape(3), tvec.reshape(3)


def bbox_from_mask(mask, width, height):
    ys, xs = np.where(mask > 0)
    if not len(xs):
        raise ValueError('empty mask')
    x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
    return ((x0 + x1) / 2 / width, (y0 + y1) / 2 / height,
            (x1 - x0 + 1) / width, (y1 - y0 + 1) / height)


def keypoint_line(class_id, bbox, quad, width, height):
    values = [str(class_id), *(f'{value:.6f}' for value in bbox)]
    for point in quad:
        visible = int(0 <= point[0] < width and 0 <= point[1] < height)
        normalized = np.clip(point / np.asarray([width, height]), 0.0, 1.0)
        values.extend((f'{normalized[0]:.6f}', f'{normalized[1]:.6f}',
                       str(2 if visible else 0)))
    return ' '.join(values)


def draw_overlay(frame, entries):
    view = frame.copy()
    colors = {0: (0, 165, 255), 1: (220, 80, 180)}
    for entry in entries:
        points = np.rint(entry['cuboid']).astype(np.int32)
        color = colors[entry['class_id']]
        for a, b in EDGES:
            cv2.line(view, tuple(points[a]), tuple(points[b]), color, 2,
                     cv2.LINE_AA)
        for index, point in enumerate(points):
            cv2.circle(view, tuple(point), 4, (0, 255, 0) if index < 4 else color,
                       -1, cv2.LINE_AA)
            cv2.putText(view, str(index), tuple(point + [3, -3]),
                        cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 255), 1,
                        cv2.LINE_AA)
        cv2.putText(view, f"{entry['class']} err={entry['reprojection_error']:.1f}px",
                    (max(0, int(points[:, 0].min())),
                     max(18, int(points[:, 1].min()) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)
    return view


def main():
    parser = __import__('argparse').ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=SOURCE)
    parser.add_argument('--raw', type=Path, default=RAW)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f'output already exists: {args.output}')
    reviewed = [json.loads(line) for line in
                (args.source / 'reviewed.jsonl').read_text(encoding='utf-8').splitlines()]
    split_manifest = json.loads((args.source / 'split_manifest.json').read_text())
    split_ids = {split: set(ids) for split, ids in split_manifest['splits'].items()}
    sidecars = {}
    for item in reviewed:
        sidecar_path = args.raw / item['image'].replace('.jpg', '.json')
        sidecars[item['id']] = json.loads(sidecar_path.read_text(encoding='utf-8'))
    usable = [item for item in reviewed if item['review_status'] != 'excluded'
              and item['id'] in set().union(*split_ids.values())]
    if len(usable) != sum(map(len, split_ids.values())):
        raise ValueError('reviewed and split manifests disagree')

    records = []
    for item in usable:
        source_image = args.raw / item['image']
        frame = cv2.imread(str(source_image))
        if frame is None:
            raise ValueError(f'cannot decode {source_image}')
        height, width = frame.shape[:2]
        image_entries, labels = [], []
        for label in item['labels']:
            class_name = item['classes'][len(labels)]
            _, mask = polygon_from_label(label, width, height)
            quad, quad_source = reference_quad(mask)
            cuboid, error, rvec, tvec = fit_cuboid(
                quad, camera_matrix(sidecars[item['id']]));
            bbox = bbox_from_mask(mask, width, height)
            entry = {'class': class_name, 'class_id': CLASSES[class_name],
                     'quad': quad.tolist(), 'cuboid': cuboid.tolist(),
                     'quad_source': quad_source,
                     'reprojection_error': error,
                     'rvec': rvec.tolist(), 'tvec_mm': tvec.tolist()}
            image_entries.append(entry)
            labels.append(keypoint_line(entry['class_id'], bbox, quad,
                                        width, height))
        split = next(name for name, ids in split_ids.items() if item['id'] in ids)
        name = f"{item['session_id']}_{Path(item['image']).stem}"
        image_dir = args.output / 'images' / split
        label_dir = args.output / 'labels' / split
        overlay_dir = args.output / 'overlays' / split
        for directory in (image_dir, label_dir, overlay_dir):
            directory.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_image, image_dir / f'{name}.jpg')
        (label_dir / f'{name}.txt').write_text('\n'.join(labels) +
                                               ('\n' if labels else ''), encoding='utf-8')
        cv2.imwrite(str(overlay_dir / f'{name}.jpg'), draw_overlay(frame, image_entries),
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
        records.append({**item, 'split': split, 'image_name': f'{name}.jpg',
                        'instances': image_entries})

    dataset = {'path': args.output.as_posix(), 'train': 'images/train',
               'val': 'images/val', 'test': 'images/test',
               'kpt_shape': [4, 3], 'flip_idx': [1, 0, 3, 2],
               'names': {class_id: name for name, class_id in CLASSES.items()}}
    (args.output / 'dataset.yaml').write_text(
        yaml.safe_dump(dataset, allow_unicode=True, sort_keys=False), encoding='utf-8')
    (args.output / 'cuboid_manifest.json').write_text(
        json.dumps({'source_split_manifest': split_manifest,
                    'cube_size_mm': CUBE_MM,
                    'keypoint_order': ['reference_tl', 'reference_tr',
                                       'reference_br', 'reference_bl'],
                    'label_semantics': ('four visible reference corners from the reviewed mask; '
                                        'the remaining cuboid corners are derived by PnP and are not labels'),
                    'records': records}, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    counts = Counter(record['split'] for record in records)
    errors = [instance['reprojection_error'] for record in records
              for instance in record['instances']]
    print(json.dumps({'images': dict(counts), 'instances': len(errors),
                      'reprojection_error_mean_px': float(np.mean(errors)) if errors else 0,
                      'reprojection_error_p95_px': float(np.percentile(errors, 95)) if errors else 0,
                      'output': str(args.output)}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
