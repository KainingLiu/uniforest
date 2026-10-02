#!/usr/bin/env python3
"""Evaluate and visualize six visible cube outline keypoints."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import torch
import ultralytics
from ultralytics import YOLO

from propose_visible_outline import EDGES


ROOT = Path(__file__).resolve().parents[1]
DATASET = ROOT / 'data/collection/visible_outline_v2'
COLORS = {0: (0, 145, 255), 1: (220, 80, 180)}


def sha256(path: Path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for part in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(part)
    return digest.hexdigest()


def truth_entries(label_path: Path, width: int, height: int):
    entries = []
    for line in label_path.read_text(encoding='utf-8').splitlines():
        values = line.split()
        if len(values) != 23:
            raise ValueError(f'expected six keypoints: {label_path}')
        box = np.asarray([float(x) for x in values[1:5]]) * [width, height, width, height]
        cx, cy, w, h = box
        xyxy = np.asarray([cx - w / 2, cy - h / 2,
                           cx + w / 2, cy + h / 2])
        points = np.asarray([[float(values[i]) * width,
                              float(values[i + 1]) * height]
                             for i in range(5, 23, 3)], dtype=np.float32)
        visibility = np.asarray([int(values[i + 2])
                                 for i in range(5, 23, 3)], dtype=np.uint8)
        entries.append({'class_id': int(values[0]), 'box': xyxy,
                        'points': points, 'visibility': visibility})
    return entries


def predicted_entries(result):
    if result.boxes is None or result.keypoints is None:
        return []
    boxes = result.boxes.xyxy.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)
    scores = result.boxes.conf.cpu().numpy()
    points = result.keypoints.xy.cpu().numpy()
    confidences = result.keypoints.conf
    confidences = (confidences.cpu().numpy() if confidences is not None
                   else np.ones(points.shape[:2], dtype=np.float32))
    return [{'class_id': int(cls), 'box': box, 'score': float(score),
             'points': point, 'visibility_score': conf}
            for cls, box, score, point, conf in zip(
                classes, boxes, scores, points, confidences, strict=True)]


def iou(first, second):
    x0, y0 = np.maximum(first[:2], second[:2])
    x1, y1 = np.minimum(first[2:], second[2:])
    overlap = max(0, x1 - x0) * max(0, y1 - y0)
    a = max(0, first[2] - first[0]) * max(0, first[3] - first[1])
    b = max(0, second[2] - second[0]) * max(0, second[3] - second[1])
    return float(overlap / max(a + b - overlap, 1e-9))


def match(truth, predicted):
    candidates = sorted(((iou(target['box'], estimate['box']), ti, pi)
                         for ti, target in enumerate(truth)
                         for pi, estimate in enumerate(predicted)
                         if target['class_id'] == estimate['class_id']),
                        reverse=True)
    used_truth, used_pred, pairs = set(), set(), []
    for overlap, ti, pi in candidates:
        if overlap < .1 or ti in used_truth or pi in used_pred:
            continue
        used_truth.add(ti)
        used_pred.add(pi)
        pairs.append((truth[ti], predicted[pi], overlap))
    return pairs


def valid_shape(entry, threshold):
    points = entry['points']
    confident = entry['visibility_score'] >= threshold
    if not confident.all():
        return None
    if (points[0, 0] >= points[1, 0]
            or points[3, 0] >= points[2, 0]
            or points[4, 0] >= points[5, 0]):
        return False
    back_y = points[:2, 1].mean()
    fold_y = points[2:4, 1].mean()
    bottom_y = points[4:, 1].mean()
    return bool(back_y + 5 < fold_y and fold_y + 5 < bottom_y)


def render(image, entries, label, threshold):
    view = image.copy()
    cv2.rectangle(view, (0, 0), (view.shape[1] - 1, 28), (12, 12, 12), -1)
    cv2.putText(view, label, (5, 20), cv2.FONT_HERSHEY_SIMPLEX,
                .55, (255, 255, 255), 1, cv2.LINE_AA)
    for entry in entries:
        points = np.rint(entry['points']).astype(np.int32)
        confidence = (entry.get('visibility_score')
                      if 'visibility_score' in entry else entry['visibility'])
        visible = np.asarray(confidence) >= (
            threshold if 'visibility_score' in entry else 1)
        color = COLORS[entry['class_id']]
        for first, second in EDGES:
            if visible[first] and visible[second]:
                cv2.line(view, tuple(points[first]), tuple(points[second]),
                         color, 2, cv2.LINE_AA)
        for index, point in enumerate(points):
            if visible[index]:
                cv2.circle(view, tuple(point), 4, (0, 255, 0), -1,
                           cv2.LINE_AA)
                cv2.putText(view, str(index), tuple(point + [3, -3]),
                            cv2.FONT_HERSHEY_SIMPLEX, .4,
                            (255, 255, 255), 1, cv2.LINE_AA)
    return view


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--split', choices=('val', 'test'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--imgsz', type=int, default=320)
    parser.add_argument('--keypoint-conf', type=float, default=.5,
                        help='Minimum keypoint visibility confidence for rendered edges')
    args = parser.parse_args()
    if not 0 < args.keypoint_conf < 1:
        parser.error('--keypoint-conf must lie between 0 and 1')
    if args.imgsz < 160 or args.imgsz % 32:
        parser.error('--imgsz must be at least 160 and divisible by 32')
    if args.output.exists():
        raise FileExistsError(f'evaluation already exists: {args.output}')
    args.output.mkdir(parents=True)
    model = YOLO(str(args.weights))
    metrics = model.val(data=str(DATASET / 'dataset.yaml'), split=args.split,
                        imgsz=args.imgsz, batch=16, device=0, workers=0, plots=True,
                        project=str(args.output.resolve()), name='ultralytics')
    images = sorted((DATASET / 'images' / args.split).glob('*.jpg'))
    predictions = model.predict(source=[str(path) for path in images],
                                imgsz=args.imgsz, conf=.25, iou=.5, device=0,
                                verbose=False)
    comparisons = args.output / 'comparisons'
    comparisons.mkdir()
    errors, per_image = [], []
    visibility_counts = Counter()
    for image_path, result in zip(images, predictions, strict=True):
        image = cv2.imread(str(image_path))
        height, width = image.shape[:2]
        truth = truth_entries(DATASET / 'labels' / args.split /
                              f'{image_path.stem}.txt', width, height)
        predicted = predicted_entries(result)
        pairs = match(truth, predicted)
        for target, estimate, _ in pairs:
            truth_visible = target['visibility'] > 0
            pred_visible = estimate['visibility_score'] >= args.keypoint_conf
            visibility_counts['true_positive'] += int(np.sum(truth_visible & pred_visible))
            visibility_counts['false_positive'] += int(np.sum(~truth_visible & pred_visible))
            visibility_counts['false_negative'] += int(np.sum(truth_visible & ~pred_visible))
            visibility_counts['true_negative'] += int(np.sum(~truth_visible & ~pred_visible))
            mask = truth_visible & pred_visible
            for first, second in zip(target['points'][mask],
                                     estimate['points'][mask], strict=True):
                errors.append(float(np.linalg.norm(first - second)))
        left = render(image, truth, 'reviewed visible edges', args.keypoint_conf)
        right = render(image, predicted, 'YOLO visible edges', args.keypoint_conf)
        cv2.imwrite(str(comparisons / f'{image_path.stem}.jpg'),
                    cv2.hconcat((left, right)),
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
        counts_truth = Counter(item['class_id'] for item in truth)
        counts_pred = Counter(item['class_id'] for item in predicted)
        per_image.append({'image': image_path.name,
                          'truth_counts': dict(counts_truth),
                          'predicted_counts': dict(counts_pred),
                          'matched': len(pairs),
                          'full_shape_validity': [valid_shape(item, args.keypoint_conf)
                                                  for item in predicted]})
    sheets = args.output / 'contact_sheets'
    sheets.mkdir()
    files = sorted(comparisons.glob('*.jpg'))
    for start in range(0, len(files), 8):
        cells = []
        for path in files[start:start + 8]:
            image = cv2.imread(str(path))
            image = cv2.resize(image, (640, 240), interpolation=cv2.INTER_AREA)
            cv2.putText(image, path.stem[-22:], (5, 239),
                        cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1,
                        cv2.LINE_AA)
            cells.append(image)
        while len(cells) < 8:
            cells.append(np.zeros((240, 640, 3), dtype=np.uint8))
        sheet = cv2.vconcat([cv2.hconcat(cells[i:i + 2])
                             for i in (0, 2, 4, 6)])
        cv2.imwrite(str(sheets / f'sheet_{start // 8 + 1:03d}.jpg'), sheet)
    summary = {
        'model_sha256': sha256(args.weights),
        'dataset_manifest_sha256': sha256(DATASET / 'split_manifest.json'),
        'torch_version': torch.__version__,
        'ultralytics_version': ultralytics.__version__,
        'split': args.split, 'images': len(images),
        'imgsz': args.imgsz,
        'keypoint_conf_threshold': args.keypoint_conf,
        'visibility_counts': dict(visibility_counts),
        'box': {'map50': float(metrics.box.map50),
                'map50_95': float(metrics.box.map)},
        'pose': {'map50': float(metrics.pose.map50),
                 'map50_95': float(metrics.pose.map)},
        'exact_count_images': sum(
            item['truth_counts'] == item['predicted_counts']
            for item in per_image),
        'matched_instances': sum(item['matched'] for item in per_image),
        'visible_keypoint_mean_error_px': float(np.mean(errors)) if errors else None,
        'visible_keypoint_p95_error_px': float(np.percentile(errors, 95)) if errors else None,
        'full_shape_invalid_predictions': sum(
            value is False for item in per_image
            for value in item['full_shape_validity']),
        'per_image': per_image,
    }
    (args.output / 'summary.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    print(json.dumps({key: summary[key] for key in
                      ('split', 'images', 'box', 'pose', 'exact_count_images',
                       'matched_instances', 'visible_keypoint_mean_error_px',
                       'visible_keypoint_p95_error_px',
                       'full_shape_invalid_predictions')},
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
