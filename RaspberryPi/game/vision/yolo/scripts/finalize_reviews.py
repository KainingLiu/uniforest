#!/usr/bin/env python3
"""Export individually reviewed cube proposals to a leakage-checked YOLO set.

The review plan records human decisions for every numbered overlay. This
script never interprets a pending proposal as an approved training label.
"""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil

import cv2
import numpy as np
import yaml


YOLO_ROOT = Path(__file__).resolve().parents[1]
RAW = YOLO_ROOT / 'data/raw/collection_20260926'
REVIEW = YOLO_ROOT / 'data/labels/collection_20260926_review'
PLAN = YOLO_ROOT / 'docs/collection_20260926_review_plan.json'
OUTPUT = YOLO_ROOT / 'data/collection/collection_v1'
CLASSES = {'orange_cube': 0, 'purple_cube': 1}
VALIDATION_SESSION = '20260926T151703_396768Z_9f29ed07'
TEST_SESSION = '20260926T164458_468022Z_e6f4e016'
MAX_NEAR_DISTANCE = 5


def read_json(path: Path):
    return json.loads(path.read_text(encoding='utf-8'))


def expanded_exclusions(plan):
    excluded = {int(i): plan['excluded_ids_reason'] for i in plan['excluded_ids']}
    for entry in plan['excluded_ranges']:
        for image_id in range(entry['first'], entry['last'] + 1):
            if image_id in excluded:
                raise ValueError(f'duplicate exclusion: {image_id}')
            excluded[image_id] = entry['reason']
    return excluded


def polygon_label(proposal, width: int, height: int):
    class_id = CLASSES[proposal['class']]
    points = np.asarray(proposal['polygon'], dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3:
        raise ValueError('instance polygon needs at least three 2-D points')
    if not np.isfinite(points).all() or cv2.contourArea(points) <= 4:
        raise ValueError('invalid instance polygon area')
    if np.any(points < 0) or np.any(points[:, 0] >= width) or np.any(points[:, 1] >= height):
        raise ValueError('instance polygon exceeds its image')
    normalized = points / np.asarray([width, height], dtype=np.float32)
    return str(class_id) + ' ' + ' '.join(f'{v:.6f}' for v in normalized.ravel())


def phash(image_path: Path):
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f'cannot decode image: {image_path}')
    low = cv2.resize(image, (32, 32)).astype(np.float32)
    coefficients = cv2.dct(low)[:8, :8].ravel()
    median = np.median(coefficients[1:])
    bits = sum(int(value > median) << i for i, value in enumerate(coefficients))
    return bits, (image.shape[1], image.shape[0])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', type=Path, default=RAW)
    parser.add_argument('--review', type=Path, default=REVIEW)
    parser.add_argument('--plan', type=Path, default=PLAN)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()

    source = read_json(args.raw / 'import_manifest.json')
    plan = read_json(args.plan)
    proposal_source = read_json(args.review / 'source.json')
    if source['source_sha256'] != plan['source_sha256']:
        raise ValueError('review plan refers to a different source archive')
    if (proposal_source['source_sha256'] != source['source_sha256']
            or proposal_source['image_count'] != source['image_count']):
        raise ValueError('proposals refer to a different source archive')
    if source['image_count'] != len(source['images']):
        raise ValueError('source image count mismatch')
    proposals = {}
    with (args.review / 'proposals.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            entry = json.loads(line)
            if entry['id'] in proposals:
                raise ValueError(f'duplicate proposal ID: {entry["id"]}')
            proposals[entry['id']] = entry
    ids = {item['id'] for item in source['images']}
    if len(ids) != source['image_count'] or ids != set(proposals):
        raise ValueError('source and proposal IDs differ')

    excluded = expanded_exclusions(plan)
    approved_empty = set(plan['approved_empty_ids'])
    if set(excluded) & approved_empty or not (set(excluded) | approved_empty) <= ids:
        raise ValueError('review decisions overlap or reference unknown IDs')
    reviewed = []
    for item in source['images']:
        image_id = item['id']
        proposal = proposals[image_id]
        if proposal['image'] != item['image'] or proposal['session_id'] != item['session_id']:
            raise ValueError(f'proposal source mismatch at image {image_id}')
        image_path = args.raw / item['image']
        fingerprint, size = phash(image_path)
        if size != (proposal['width'], proposal['height']):
            raise ValueError(f'image size mismatch at image {image_id}')
        if image_id in excluded:
            status, selected, reason = 'excluded', [], excluded[image_id]
        elif image_id in approved_empty:
            status, selected, reason = ('approved_empty', [],
                                        plan['approved_empty_reason'])
        else:
            selected = proposal['proposals']
            status = 'approved' if selected else 'approved_empty'
            reason = plan['acceptance_rule']
        labels = [polygon_label(p, *size) for p in selected]
        evidence = f'contact_sheets/sheet_{(image_id - 1) // 16 + 1:03d}.jpg'
        if not (args.review / evidence).exists():
            raise FileNotFoundError(f'missing visual review evidence: {evidence}')
        reviewed.append({**item, 'width': size[0], 'height': size[1],
                         'review_status': status, 'review_reason': reason,
                         'reviewer': plan['reviewer'],
                         'review_evidence': evidence,
                         'labels': labels,
                         'classes': [p['class'] for p in selected],
                         'phash': f'{fingerprint:016x}'})

    usable = {item['id']: item for item in reviewed if item['review_status'] != 'excluded'}
    test = {image_id for image_id, item in usable.items()
            if item['session_id'] == TEST_SESSION}
    val_candidates = {image_id for image_id, item in usable.items()
                      if item['session_id'] == VALIDATION_SESSION}
    if not test or not val_candidates:
        raise ValueError('validation or test session has no reviewed images')
    fingerprints = {image_id: int(item['phash'], 16) for image_id, item in usable.items()}

    def close_to(image_id, heldout):
        return any((fingerprints[image_id] ^ fingerprints[other]).bit_count()
                   <= MAX_NEAR_DISTANCE for other in heldout)

    val = {image_id for image_id in val_candidates if not close_to(image_id, test)}
    heldout = test | val
    train_candidates = set(usable) - test - val_candidates
    train = {image_id for image_id in train_candidates if not close_to(image_id, heldout)}
    embargo = set(usable) - train - val - test
    if min(map(len, (train, val, test))) < 20:
        raise ValueError('too few images in one of the independent splits')
    splits = {'train': train, 'val': val, 'test': test}
    for left, right in (('train', 'val'), ('train', 'test'), ('val', 'test')):
        if any(close_to(image_id, splits[right]) for image_id in splits[left]):
            raise AssertionError(f'near-duplicate leak: {left}/{right}')

    if args.check_only:
        previous = args.output / 'split_manifest.json'
        if previous.exists():
            old = read_json(previous)
            if (old['splits'] != {name: sorted(values) for name, values in splits.items()}
                    or old['embargoed_ids'] != sorted(embargo)
                    or old['review_plan_sha256'] != hashlib.sha256(args.plan.read_bytes()).hexdigest()):
                raise ValueError('existing export differs from current review decisions')
        print(f'Validated {len(reviewed)} reviews: {len(train)} train, {len(val)} val, '
              f'{len(test)} test, {len(embargo)} embargoed, {len(excluded)} excluded')
        return

    if args.output.exists():
        raise FileExistsError(f'output already exists; choose a new directory: {args.output}')
    args.output.mkdir(parents=True)
    (args.output / 'reviewed.jsonl').write_text(
        ''.join(json.dumps(item, ensure_ascii=False) + '\n' for item in reviewed),
        encoding='utf-8')
    by_id = {item['id']: item for item in reviewed}
    for split, split_ids in splits.items():
        images_dir = args.output / 'images' / split
        labels_dir = args.output / 'labels' / split
        images_dir.mkdir(parents=True)
        labels_dir.mkdir(parents=True)
        for image_id in sorted(split_ids):
            item = by_id[image_id]
            name = f'{item["session_id"]}_{Path(item["image"]).stem}'
            shutil.copy2(args.raw / item['image'], images_dir / f'{name}.jpg')
            (labels_dir / f'{name}.txt').write_text(
                '\n'.join(item['labels']) + ('\n' if item['labels'] else ''),
                encoding='utf-8')
    dataset = {'path': args.output.as_posix(),
               'train': 'images/train', 'val': 'images/val',
               'test': 'images/test',
               'names': {class_id: name for name, class_id in CLASSES.items()}}
    (args.output / 'dataset.yaml').write_text(
        yaml.safe_dump(dataset, allow_unicode=True, sort_keys=False), encoding='utf-8')
    split_manifest = {
        'source_archive_sha256': source['source_sha256'],
        'review_plan_sha256': hashlib.sha256(
            args.plan.read_bytes()).hexdigest(),
        'reviewer': plan['reviewer'], 'review_date': plan['review_date'],
        'split_rule': 'whole sessions, then discard cross-split pHash near-duplicates',
        'validation_session': VALIDATION_SESSION, 'test_session': TEST_SESSION,
        'max_phash_hamming_distance': MAX_NEAR_DISTANCE,
        'splits': {name: sorted(values) for name, values in splits.items()},
        'embargoed_ids': sorted(embargo),
        'excluded_ids': sorted(excluded),
        'review_status_counts': dict(Counter(item['review_status'] for item in reviewed)),
        'instance_counts': {name: dict(Counter(
            class_name for image_id in values for class_name in by_id[image_id]['classes']))
            for name, values in splits.items()},
    }
    (args.output / 'split_manifest.json').write_text(
        json.dumps(split_manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({key: split_manifest[key] for key in
                      ('review_status_counts', 'instance_counts', 'embargoed_ids')},
                     ensure_ascii=False, indent=2))
    print(f'Exported {len(train)} train, {len(val)} val, {len(test)} test; '
          f'{len(embargo)} near-duplicate frames withheld; {len(excluded)} review exclusions.')
    print(f'Dataset: {args.output / "dataset.yaml"}')


if __name__ == '__main__':
    main()
