#!/usr/bin/env python3
"""Export reviewed six-point visible cube outlines to a YOLO pose dataset."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT / 'data/raw/collection_20260926'
SOURCE = ROOT / 'data/collection/collection_v1'
PROPOSALS = ROOT / 'data/labels/visible_outline_v2_review'
PLAN = ROOT / 'docs/VISIBLE_OUTLINE_V2_REVIEW.json'
OUTPUT = ROOT / 'data/collection/visible_outline_v2'
CLASSES = {'orange_cube': 0, 'purple_cube': 1}
KEYPOINTS = ('top_back_left', 'top_back_right', 'top_front_right',
             'top_front_left', 'front_bottom_left', 'front_bottom_right')


def read_json(path: Path):
    return json.loads(path.read_text(encoding='utf-8'))


def sha256(path: Path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def box_from_segmentation(label: str):
    values = label.split()
    coords = np.asarray([float(value) for value in values[1:]],
                        dtype=np.float64).reshape(-1, 2)
    low, high = coords.min(axis=0), coords.max(axis=0)
    return ((low + high) / 2, high - low)


def pose_line(class_name: str, segmentation: str, points, visibility,
              width: int, height: int):
    center, size = box_from_segmentation(segmentation)
    if np.any(size <= 0) or np.any(center < 0) or np.any(center > 1):
        raise ValueError('invalid source box')
    if len(points) != 6 or len(visibility) != 6:
        raise ValueError('visible outline requires six keypoints')
    values = [str(CLASSES[class_name]),
              *(f'{value:.6f}' for value in (*center, *size))]
    for point, flag in zip(points, visibility, strict=True):
        if flag == 0:
            values.extend(('0.000000', '0.000000', '0'))
            continue
        normalized = np.asarray(point, dtype=np.float64) / [width, height]
        if not np.isfinite(normalized).all() or np.any(normalized < 0) or np.any(normalized > 1):
            raise ValueError('visible keypoint is outside image')
        values.extend((f'{normalized[0]:.6f}', f'{normalized[1]:.6f}', '2'))
    return ' '.join(values)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', type=Path, default=RAW)
    parser.add_argument('--source', type=Path, default=SOURCE)
    parser.add_argument('--proposals', type=Path, default=PROPOSALS)
    parser.add_argument('--plan', type=Path, default=PLAN)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()

    source = [json.loads(line) for line in
              (args.source / 'reviewed.jsonl').read_text(encoding='utf-8').splitlines()]
    proposed = [json.loads(line) for line in
                (args.proposals / 'proposals.jsonl').read_text(encoding='utf-8').splitlines()]
    plan = read_json(args.plan)
    splits = read_json(args.source / 'split_manifest.json')
    if splits['source_archive_sha256'] != plan['source_archive_sha256']:
        raise ValueError('review plan refers to another source archive')
    if len(source) != len(proposed) or len(source) != 418:
        raise ValueError('source/reviewed image count mismatch')
    source_by_id = {item['id']: item for item in source}
    proposed_by_id = {item['id']: item for item in proposed}
    if len(source_by_id) != len(source) or set(source_by_id) != set(proposed_by_id):
        raise ValueError('source/proposal IDs differ')
    for sheet in range(1, 28):
        if not (args.proposals / 'contact_sheets' /
                f'sheet_{sheet:03d}.jpg').is_file():
            raise FileNotFoundError(f'missing visual review sheet {sheet}')
    selected = {split: set(ids) for split, ids in splits['splits'].items()}
    if set().union(*selected.values()) & set(splits['excluded_ids']):
        raise ValueError('excluded source image entered a data split')
    rejected = set(plan['rejected_object_keys'])
    overrides = plan['manual_point_overrides']
    rows = []
    counts = Counter()
    for image_id in sorted(source_by_id):
        item, proposal = source_by_id[image_id], proposed_by_id[image_id]
        if item['image'] != proposal['image'] or item['review_status'] != proposal['source_review_status']:
            raise ValueError(f'source and proposal mismatch at image {image_id}')
        if len(item['labels']) != len(proposal['objects']):
            raise ValueError(f'instance count mismatch at image {image_id}')
        split = next((name for name, ids in selected.items() if image_id in ids), None)
        output_objects = []
        for index, (segmentation, class_name, obj) in enumerate(
                zip(item['labels'], item['classes'], proposal['objects'], strict=True)):
            key = f'{image_id}:{index}'
            if key in rejected:
                continue
            if obj['class'] != class_name or 'error' in obj:
                raise ValueError(f'unreviewable proposal {key}')
            points = obj['points']
            visibility = obj['visibility']
            if key in overrides:
                points = overrides[key]['points']
                visibility = overrides[key]['visibility']
            label = pose_line(class_name, segmentation, points, visibility,
                              item['width'], item['height'])
            output_objects.append({'key': key, 'class': class_name,
                                   'points': points, 'visibility': visibility,
                                   'flags': obj['flags'], 'label': label})
            counts[(split or 'unused', class_name)] += 1
            counts[(split or 'unused', 'full' if all(visibility) else 'partial')] += 1
        rows.append({**item, 'split': split, 'visible_outline_status':
                     ('excluded' if item['review_status'] == 'excluded' else
                      'approved_empty' if not output_objects else 'approved'),
                     'objects': output_objects})
    if args.check_only:
        print(f'Validated {len(rows)} reviewed images, '
              f'{sum(len(row["objects"]) for row in rows)} instances')
        return
    if args.output.exists():
        raise FileExistsError(f'dataset already exists: {args.output}')
    args.output.mkdir(parents=True)
    with (args.output / 'reviewed_visible.jsonl').open('w', encoding='utf-8') as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False) + '\n')
    for row in rows:
        if row['split'] is None:
            continue
        name = f'{row["session_id"]}_{Path(row["image"]).stem}'
        image_dir = args.output / 'images' / row['split']
        label_dir = args.output / 'labels' / row['split']
        image_dir.mkdir(parents=True, exist_ok=True)
        label_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(args.raw / row['image'], image_dir / f'{name}.jpg')
        (label_dir / f'{name}.txt').write_text(
            '\n'.join(obj['label'] for obj in row['objects']) +
            ('\n' if row['objects'] else ''), encoding='utf-8')
    dataset = {'path': args.output.as_posix(),
               'train': 'images/train', 'val': 'images/val',
               'test': 'images/test', 'kpt_shape': [6, 3],
               'flip_idx': [1, 0, 3, 2, 5, 4],
               'names': {identifier: name for name, identifier in CLASSES.items()}}
    (args.output / 'dataset.yaml').write_text(
        yaml.safe_dump(dataset, sort_keys=False, allow_unicode=True),
        encoding='utf-8')
    manifest = {'source_archive_sha256': splits['source_archive_sha256'],
                'source_split_manifest_sha256': sha256(args.source / 'split_manifest.json'),
                'review_plan_sha256': sha256(args.plan),
                'keypoint_order': KEYPOINTS,
                'label_semantics': 'visible top and front edges only; no hidden or 3-D coordinate labels',
                'splits': splits['splits'], 'embargoed_ids': splits['embargoed_ids'],
                'excluded_ids': splits['excluded_ids'],
                'counts': {f'{split}/{name}': number
                           for (split, name), number in sorted(counts.items())}}
    (args.output / 'split_manifest.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    print(json.dumps({'images': {split: len(ids) for split, ids in selected.items()},
                      'instance_counts': manifest['counts'],
                      'output': str(args.output)}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
