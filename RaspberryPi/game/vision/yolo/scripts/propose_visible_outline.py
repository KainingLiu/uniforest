#!/usr/bin/env python3
"""Propose six *visible* cube corners for per-image review.

The top/front seam comes from brightness inside an already reviewed instance
mask. No hidden corners, depth, or planar PnP labels are generated here.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT / 'data/raw/collection_20260926'
SOURCE = ROOT / 'data/collection/collection_v1/reviewed.jsonl'
OUTPUT = ROOT / 'data/labels/visible_outline_v2_review'
EDGES = ((0, 1), (1, 2), (2, 3), (3, 0), (3, 4), (2, 5), (4, 5))
COLORS = {'orange_cube': (0, 145, 255), 'purple_cube': (220, 80, 180)}


def instance_mask(label: str, width: int, height: int):
    values = label.split()
    points = np.asarray([[float(values[i]) * width,
                          float(values[i + 1]) * height]
                         for i in range(1, len(values), 2)], dtype=np.float32)
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [np.rint(points).astype(np.int32)], 255)
    return mask


def _top_component(frame, mask):
    value = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[:, :, 2]
    pixels = value[mask > 0]
    if not len(pixels):
        raise ValueError('empty instance mask')
    threshold, _ = cv2.threshold(pixels, 0, 255,
                                 cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    bright = np.uint8((value >= threshold) & (mask > 0)) * 255
    bright = cv2.morphologyEx(bright, cv2.MORPH_OPEN,
                              np.ones((3, 3), dtype=np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(bright)
    if count <= 1:
        raise ValueError('no bright top-face component')
    selected = max(range(1, count), key=lambda i: stats[i, cv2.CC_STAT_AREA])
    return np.uint8(labels == selected) * 255, float(threshold)


def _top_corners(top):
    contours, _ = cv2.findContours(top, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise ValueError('top component has no contour')
    hull = cv2.convexHull(max(contours, key=cv2.contourArea))
    perimeter = cv2.arcLength(hull, True)
    corners, method = None, 'min_area_rect'
    for epsilon_ratio in (0.01, 0.02, 0.03, 0.04, 0.05):
        candidate = cv2.approxPolyDP(hull, epsilon_ratio * perimeter, True)
        if len(candidate) == 4:
            corners = candidate.reshape(4, 2).astype(np.float32)
            method = 'top_brightness_quad'
            break
    if corners is None:
        corners = cv2.boxPoints(cv2.minAreaRect(hull))
    by_y = corners[np.argsort(corners[:, 1])]
    back = by_y[:2][np.argsort(by_y[:2, 0])]
    front = by_y[2:][np.argsort(by_y[2:, 0])]
    return np.stack((back[0], back[1], front[1], front[0])), method


def _bottom_corners(mask):
    ys, xs = np.nonzero(mask)
    height = float(ys.max() - ys.min() + 1)
    band = ys >= ys.max() - max(3, round(height * 0.04))
    if band.sum() < 8:
        raise ValueError('insufficient visible bottom pixels')
    bottom_y = float(np.percentile(ys[band], 96))
    return np.asarray([[np.percentile(xs[band], 2), bottom_y],
                       [np.percentile(xs[band], 98), bottom_y]],
                      dtype=np.float32)


def propose(frame, label, class_name):
    height, width = frame.shape[:2]
    mask = instance_mask(label, width, height)
    top, threshold = _top_component(frame, mask)
    quad, method = _top_corners(top)
    bottom = _bottom_corners(mask)
    points = np.concatenate((quad, bottom), axis=0)
    flags = []
    mask_area = int(np.count_nonzero(mask))
    top_fraction = float(np.count_nonzero(top) / mask_area)
    if method != 'top_brightness_quad':
        flags.append('top_quad_fallback')
    if not 0.15 <= top_fraction <= 0.85:
        flags.append('top_fraction_outlier')
    top_depth = float(quad[3, 1] + quad[2, 1] - quad[0, 1] - quad[1, 1]) / 2
    front_height = float(bottom[:, 1].mean() - quad[2:, 1].mean())
    if top_depth < 8:
        flags.append('shallow_top')
    if front_height < 8:
        flags.append('shallow_front')
    if bottom[1, 0] - bottom[0, 0] < 20:
        flags.append('narrow_bottom')
    visible = [2] * 6
    touches_left = bool(np.any(mask[:, 0] > 0))
    touches_right = bool(np.any(mask[:, width - 1] > 0))
    if touches_left:
        flags.append('left_image_crop')
        for i in (0, 3, 4):
            visible[i] = 0
    if touches_right:
        flags.append('right_image_crop')
        for i in (1, 2, 5):
            visible[i] = 0
    for i, point in enumerate(points):
        if not (0 <= point[0] < width and 0 <= point[1] < height):
            visible[i] = 0
    if len(flags) > 2:
        flags.append('manual_review_priority')
    return {'class': class_name, 'points': points.tolist(),
            'visibility': visible, 'flags': flags,
            'method': method, 'brightness_threshold': threshold,
            'top_fraction': top_fraction, 'top_depth_px': top_depth,
            'front_height_px': front_height, 'mask_area_px': mask_area}


def overlay(frame, proposals, image_id):
    view = frame.copy()
    cv2.rectangle(view, (0, 0), (250, 28), (20, 20, 20), -1)
    cv2.putText(view, f'{image_id:03d} visible outline', (5, 20),
                cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1,
                cv2.LINE_AA)
    for index, proposal in enumerate(proposals):
        if 'error' in proposal:
            continue
        color = COLORS[proposal['class']]
        points = np.rint(proposal['points']).astype(np.int32)
        visible = proposal['visibility']
        for first, second in EDGES:
            if visible[first] and visible[second]:
                cv2.line(view, tuple(points[first]), tuple(points[second]),
                         color, 2, cv2.LINE_AA)
        for keypoint, point in enumerate(points):
            if visible[keypoint]:
                cv2.circle(view, tuple(point), 4, (0, 255, 0), -1,
                           cv2.LINE_AA)
                cv2.putText(view, str(keypoint), tuple(point + [3, -4]),
                            cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 255),
                            1, cv2.LINE_AA)
        cv2.putText(view, f'{index}: {",".join(proposal["flags"][:2])}',
                    (5, 48 + index * 21), cv2.FONT_HERSHEY_SIMPLEX,
                    .42, color, 1, cv2.LINE_AA)
    return view


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=SOURCE)
    parser.add_argument('--raw', type=Path, default=RAW)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f'proposal output already exists: {args.output}')
    records = [json.loads(line) for line in
               args.source.read_text(encoding='utf-8').splitlines()]
    preview_dir = args.output / 'preview'
    preview_dir.mkdir(parents=True)
    proposals, flags = [], Counter()
    for item in records:
        frame = cv2.imread(str(args.raw / item['image']))
        if frame is None:
            raise ValueError(f'cannot decode image {item["id"]}')
        objects = []
        if item['review_status'] != 'excluded':
            for label, class_name in zip(item['labels'], item['classes'],
                                         strict=True):
                try:
                    result = propose(frame, label, class_name)
                    flags.update(result['flags'])
                except (ValueError, cv2.error) as error:
                    result = {'class': class_name, 'error': str(error),
                              'flags': ['proposal_error']}
                    flags['proposal_error'] += 1
                objects.append(result)
        proposals.append({'id': item['id'], 'image': item['image'],
                          'source_review_status': item['review_status'],
                          'objects': objects, 'review_status': 'pending'})
        cv2.imwrite(str(preview_dir / f'{item["id"]:03d}.jpg'),
                    overlay(frame, objects, item['id']),
                    [cv2.IMWRITE_JPEG_QUALITY, 93])
    with (args.output / 'proposals.jsonl').open('w', encoding='utf-8') as stream:
        for record in proposals:
            stream.write(json.dumps(record, ensure_ascii=False) + '\n')
    sheets = args.output / 'contact_sheets'
    sheets.mkdir()
    for first in range(0, len(proposals), 16):
        cells = []
        for item in proposals[first:first + 16]:
            cell = cv2.imread(str(preview_dir / f'{item["id"]:03d}.jpg'))
            cell = cv2.resize(cell, (384, 288),
                              interpolation=cv2.INTER_AREA)
            title = f'{item["id"]:03d} {item["source_review_status"]}'
            if any(obj.get('flags') for obj in item['objects']):
                title += ' !'
            cv2.rectangle(cell, (0, 0), (383, 26), (15, 15, 15), -1)
            cv2.putText(cell, title, (6, 18), cv2.FONT_HERSHEY_SIMPLEX,
                        .5, (255, 255, 255), 1, cv2.LINE_AA)
            cells.append(cell)
        while len(cells) < 16:
            cells.append(np.zeros((288, 384, 3), dtype=np.uint8))
        sheet = cv2.vconcat([cv2.hconcat(cells[row:row + 4])
                             for row in (0, 4, 8, 12)])
        cv2.imwrite(str(sheets / f'sheet_{first // 16 + 1:03d}.jpg'), sheet)
    print(json.dumps({'images': len(proposals),
                      'objects': sum(len(x['objects']) for x in proposals),
                      'flags': dict(flags), 'output': str(args.output)},
                     ensure_ascii=False))


if __name__ == '__main__':
    main()
