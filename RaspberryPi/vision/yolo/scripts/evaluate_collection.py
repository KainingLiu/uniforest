#!/usr/bin/env python3
"""Evaluate the selected model once on the held-out collection_v1 session."""

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


ROOT = Path(__file__).resolve().parents[1]
DATASET = ROOT / 'data/collection/collection_v1'
BEST = ROOT / 'runs/collection_v1/stage1_frozen/weights/best.pt'
OUTPUT = ROOT / 'runs/collection_v1/test_evaluation'


def sha256(path: Path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def render_contact_sheets(results, mismatches):
    cells = []
    for result in results:
        image = result.plot()
        image = cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)
        stem = Path(result.path).stem
        label = stem[-24:]
        mismatch = stem in mismatches
        cv2.rectangle(image, (0, 0), (319, 26), (0, 0, 170) if mismatch else (0, 55, 0), -1)
        cv2.putText(image, label, (5, 18), cv2.FONT_HERSHEY_SIMPLEX,
                    0.48, (255, 255, 255), 1, cv2.LINE_AA)
        cells.append(image)
    while len(cells) % 16:
        cells.append(np.zeros((240, 320, 3), dtype=np.uint8))
    for start in range(0, len(cells), 16):
        rows = [cv2.hconcat(cells[start + row:start + row + 4])
                for row in (0, 4, 8, 12)]
        cv2.imwrite(str(OUTPUT / f'test_sheet_{start // 16 + 1:03d}.jpg'),
                    cv2.vconcat(rows))


def main():
    if OUTPUT.exists():
        raise FileExistsError(f'independent test already has an output: {OUTPUT}')
    if not BEST.exists():
        raise FileNotFoundError(BEST)
    OUTPUT.mkdir(parents=True)
    model = YOLO(str(BEST))
    metrics = model.val(data=str(DATASET / 'dataset.yaml'), split='test',
                        imgsz=320, batch=16, device=0, workers=0,
                        plots=True, project=str(OUTPUT), name='ultralytics')
    paths = sorted((DATASET / 'images/test').glob('*.jpg'))
    results = model.predict(source=[str(path) for path in paths],
                            imgsz=320, conf=0.25, iou=0.5, device=0,
                            verbose=False)
    per_image = []
    mismatches = set()
    for path, result in zip(paths, results, strict=True):
        labels_path = DATASET / 'labels/test' / (path.stem + '.txt')
        truth = Counter(int(line.split()[0]) for line in labels_path.read_text().splitlines())
        predicted = Counter(int(value) for value in result.boxes.cls.cpu().tolist())
        if truth != predicted:
            mismatches.add(path.stem)
        per_image.append({'image': path.name, 'truth': dict(truth),
                          'predicted': dict(predicted),
                          'confidences': [round(float(value), 4)
                                          for value in result.boxes.conf.cpu().tolist()]})
    render_contact_sheets(results, mismatches)
    report = {
        'model_sha256': sha256(BEST),
        'split_manifest_sha256': sha256(DATASET / 'split_manifest.json'),
        'pytorch_version': torch.__version__,
        'ultralytics_version': ultralytics.__version__,
        'test_images': len(paths),
        'box': {'map50': float(metrics.box.map50), 'map50_95': float(metrics.box.map),
                'precision': float(metrics.box.mp), 'recall': float(metrics.box.mr)},
        'mask': {'map50': float(metrics.seg.map50), 'map50_95': float(metrics.seg.map),
                 'precision': float(metrics.seg.mp), 'recall': float(metrics.seg.mr)},
        'count_mismatch_images': len(mismatches),
        'per_image': per_image,
    }
    (OUTPUT / 'summary.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps({key: report[key] for key in
                      ('test_images', 'box', 'mask', 'count_mismatch_images')},
                     indent=2))
    print(f'Report: {OUTPUT / "summary.json"}')


if __name__ == '__main__':
    main()
