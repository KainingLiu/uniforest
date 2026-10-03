#!/usr/bin/env python3
"""Draw the reviewed six-point visible-outline model on new still images."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
from ultralytics import YOLO

from evaluate_visible_outline import predicted_entries, render, sha256
from propose_visible_outline import EDGES


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WEIGHTS = ROOT / 'models/cube_visible_outline_v2.pt'
IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png', '.bmp', '.webp'}


def image_paths(source: Path):
    if source.is_file() and source.suffix.lower() in IMAGE_SUFFIXES:
        return [source]
    if source.is_dir():
        return sorted(path for path in source.iterdir()
                      if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES)
    raise ValueError(f'no supported still images at {source}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--weights', type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument('--imgsz', type=int, default=320)
    parser.add_argument('--conf', type=float, default=.25)
    parser.add_argument('--keypoint-conf', type=float, default=.5)
    args = parser.parse_args()
    if not args.weights.is_file():
        parser.error(f'missing weights: {args.weights}')
    if not 0 < args.conf < 1 or not 0 < args.keypoint_conf < 1:
        parser.error('confidence thresholds must lie between 0 and 1')
    if args.imgsz < 160 or args.imgsz % 32:
        parser.error('--imgsz must be at least 160 and divisible by 32')
    paths = image_paths(args.source)
    if not paths:
        parser.error(f'no supported still images at {args.source}')
    if args.output.exists():
        parser.error(f'output already exists: {args.output}')
    args.output.mkdir(parents=True)
    model = YOLO(str(args.weights))
    results = model.predict(source=[str(path) for path in paths],
                            imgsz=args.imgsz, conf=args.conf, iou=.5,
                            verbose=False)
    records = []
    for path, result in zip(paths, results, strict=True):
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError(f'cannot decode {path}')
        predictions = predicted_entries(result)
        rendered = render(image, predictions, 'YOLO visible edges',
                          args.keypoint_conf)
        output_image = args.output / f'{path.stem}_outline.jpg'
        if not cv2.imwrite(str(output_image), rendered):
            raise OSError(f'could not save {output_image}')
        objects = []
        for entry in predictions:
            visible = entry['visibility_score'] >= args.keypoint_conf
            objects.append({
                'class_id': entry['class_id'],
                'score': entry['score'],
                'box_xyxy': entry['box'].tolist(),
                'keypoints_xy': entry['points'].tolist(),
                'keypoint_confidence': entry['visibility_score'].tolist(),
                'visible_edges': [[first, second] for first, second in EDGES
                                  if visible[first] and visible[second]],
            })
        records.append({'source': str(path), 'preview': str(output_image),
                        'objects': objects})
    manifest = {'model_sha256': sha256(args.weights),
                'imgsz': args.imgsz,
                'keypoint_conf_threshold': args.keypoint_conf,
                'detections': records}
    (args.output / 'predictions.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    print(f'Saved {len(records)} image previews to {args.output}')


if __name__ == '__main__':
    main()
