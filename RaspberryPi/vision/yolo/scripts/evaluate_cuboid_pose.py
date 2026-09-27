#!/usr/bin/env python3
"""Evaluate the cuboid reference-corner model and render PnP cuboids."""

from __future__ import annotations

from collections import Counter
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import torch
import ultralytics
from ultralytics import YOLO

from build_cuboid_pose_dataset import EDGES, camera_matrix, fit_cuboid


ROOT = Path(__file__).resolve().parents[1]
DATASET = ROOT / 'data/collection/cuboid_pose_v1'
SOURCE = ROOT / 'data/collection/collection_v1'
RAW = ROOT / 'data/raw/collection_20260926'
BEST = ROOT / 'runs/cuboid_pose_v1/stage2_finetune/weights/best.pt'
OUTPUT = ROOT / 'runs/cuboid_pose_v1/test_evaluation'


def sha256(path: Path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def read_labels(path: Path):
    result = []
    for line in path.read_text(encoding='utf-8').splitlines():
        values = line.split()
        points = np.asarray([[float(values[i]), float(values[i + 1])]
                             for i in range(5, len(values), 3)], dtype=np.float32)
        result.append({'class_id': int(values[0]), 'bbox': np.asarray(values[1:5], dtype=np.float32),
                       'keypoints': points})
    return result


def match_predictions(truth, predicted, width, height):
    pairs = []
    remaining = list(range(len(predicted)))
    for expected in truth:
        candidates = [index for index in remaining
                      if int(predicted[index]['class_id']) == expected['class_id']]
        if not candidates:
            continue
        center = expected['bbox'][:2] * np.asarray([width, height])
        index = min(candidates, key=lambda i: np.linalg.norm(
            predicted[i]['keypoints'].mean(axis=0) - center))
        remaining.remove(index)
        pairs.append((expected, predicted[index]))
    return pairs


def draw_prediction(image, predictions, matrix, draw_wireframe=False):
    view = image.copy()
    colors = {0: (0, 165, 255), 1: (220, 80, 180)}
    for prediction in predictions:
        color = colors[prediction['class_id']]
        keypoints = np.rint(prediction['keypoints']).astype(np.int32)
        try:
            cuboid, error, _, _ = fit_cuboid(prediction['keypoints'], matrix)
        except (ValueError, cv2.error):
            cuboid, error = None, float('nan')
        if cuboid is not None and draw_wireframe:
            points = np.rint(cuboid).astype(np.int32)
            for start, end in EDGES:
                cv2.line(view, tuple(points[start]), tuple(points[end]), color, 2,
                         cv2.LINE_AA)
        for index, point in enumerate(keypoints):
            cv2.circle(view, tuple(point), 4, (0, 255, 0), -1, cv2.LINE_AA)
            cv2.putText(view, str(index), tuple(point + [3, -3]),
                        cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 255, 255), 1,
                        cv2.LINE_AA)
        label = f"{prediction['name']} {prediction['confidence']:.2f}"
        if np.isfinite(error):
            label += f" pnp={error:.1f}px"
        cv2.putText(view, label,
                    (max(0, int(keypoints[:, 0].min())),
                     max(18, int(keypoints[:, 1].min()) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)
    return view


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wireframe', action='store_true',
                        help='draw the experimental PnP hidden edges')
    args = parser.parse_args()
    if not BEST.exists():
        raise FileNotFoundError(BEST)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(BEST))
    metrics = model.val(data=str(DATASET / 'dataset.yaml'), split='test',
                        imgsz=320, batch=16, device=0, workers=0, plots=True,
                        project=str(OUTPUT.resolve()), name='metrics', exist_ok=True)
    reviewed = {item['id']: item for item in
                (json.loads(line) for line in
                 (SOURCE / 'reviewed.jsonl').read_text(encoding='utf-8').splitlines())}
    split = json.loads((SOURCE / 'split_manifest.json').read_text(encoding='utf-8'))
    test_ids = split['splits']['test']
    paths = []
    for image_id in sorted(test_ids):
        item = reviewed[image_id]
        name = f"{item['session_id']}_{Path(item['image']).stem}.jpg"
        paths.append((image_id, item, DATASET / 'images/test' / name))
    results = model.predict(source=[str(path) for _, _, path in paths],
                            imgsz=320, conf=0.25, iou=0.5, device=0,
                            verbose=False)
    overlay_dir = OUTPUT / 'overlays'
    overlay_dir.mkdir(exist_ok=True)
    per_image, pixel_errors = [], []
    for (image_id, item, image_path), result in zip(paths, results, strict=True):
        image = cv2.imread(str(image_path))
        height, width = image.shape[:2]
        truth = read_labels(DATASET / 'labels/test' / (image_path.stem + '.txt'))
        predicted = []
        if result.boxes is not None and result.keypoints is not None:
            classes = result.boxes.cls.cpu().numpy().astype(int)
            confidence = result.boxes.conf.cpu().numpy()
            points = result.keypoints.xy.cpu().numpy()
            for class_id, score, keypoints in zip(classes, confidence, points):
                predicted.append({'class_id': int(class_id),
                                  'name': result.names[int(class_id)],
                                  'confidence': float(score),
                                  'keypoints': keypoints})
        pairs = match_predictions(truth, predicted, width, height)
        for expected, actual in pairs:
            pixel_errors.append(float(np.sqrt(np.mean(
                (expected['keypoints'] * [width, height] - actual['keypoints']) ** 2))))
        matrix = camera_matrix(json.loads(
            (RAW / item['image'].replace('.jpg', '.json')).read_text(encoding='utf-8')))
        overlay = draw_prediction(image, predicted, matrix,
                                  draw_wireframe=args.wireframe)
        cv2.imwrite(str(overlay_dir / f'{image_id:03d}.jpg'), overlay,
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
        truth_counts = Counter(int(entry['class_id']) for entry in truth)
        predicted_counts = Counter(entry['class_id'] for entry in predicted)
        per_image.append({'id': image_id, 'image': image_path.name,
                          'truth_counts': dict(truth_counts),
                          'predicted_counts': dict(predicted_counts),
                          'matched_instances': len(pairs),
                          'truth_instances': len(truth),
                          'predicted_instances': len(predicted)})
    overlay_files = sorted(overlay_dir.glob('[0-9][0-9][0-9].jpg'))
    sheets = overlay_dir / 'contact_sheets'
    sheets.mkdir(exist_ok=True)
    for start in range(0, len(overlay_files), 16):
        cells = []
        for path in overlay_files[start:start + 16]:
            cell = cv2.resize(cv2.imread(str(path)), (320, 240),
                              interpolation=cv2.INTER_AREA)
            cv2.putText(cell, path.stem, (4, 18), cv2.FONT_HERSHEY_SIMPLEX,
                        .5, (255, 255, 255), 1, cv2.LINE_AA)
            cells.append(cell)
        while len(cells) < 16:
            cells.append(np.zeros((240, 320, 3), dtype=np.uint8))
        cv2.imwrite(str(sheets / f'sheet_{start // 16 + 1:03d}.jpg'),
                    cv2.vconcat([cv2.hconcat(cells[i:i + 4])
                                 for i in (0, 4, 8, 12)]))
    summary = {
        'model_sha256': sha256(BEST),
        'source_split_manifest_sha256': sha256(SOURCE / 'split_manifest.json'),
        'pytorch_version': torch.__version__,
        'ultralytics_version': ultralytics.__version__,
        'test_images': len(paths),
        'box': {'map50': float(metrics.box.map50), 'map50_95': float(metrics.box.map),
                'precision': float(metrics.box.mp), 'recall': float(metrics.box.mr)},
        'pose': {'map50': float(metrics.pose.map50), 'map50_95': float(metrics.pose.map),
                 'precision': float(metrics.pose.mp), 'recall': float(metrics.pose.mr)},
        'exact_instance_count_images': sum(
            x['truth_counts'] == x['predicted_counts'] for x in per_image),
        'matched_instances': sum(x['matched_instances'] for x in per_image),
        'reference_keypoint_rmse_mean_px': float(np.mean(pixel_errors)) if pixel_errors else None,
        'reference_keypoint_rmse_p95_px': float(np.percentile(pixel_errors, 95)) if pixel_errors else None,
        'label_semantics': 'four visible reference corners; PnP cuboid is geometry-assisted, not depth ground truth',
        'per_image': per_image,
    }
    (OUTPUT / 'summary.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({key: summary[key] for key in
                      ('test_images', 'box', 'pose', 'exact_instance_count_images',
                       'matched_instances', 'reference_keypoint_rmse_mean_px',
                       'reference_keypoint_rmse_p95_px')}, ensure_ascii=False,
                     indent=2))


if __name__ == '__main__':
    main()
