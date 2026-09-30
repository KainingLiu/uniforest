#!/usr/bin/env python3
"""Generate REVIEW-ONLY cube instance proposals from a collected image batch.

These colour/geometry masks are provisional. Never pass them directly to YOLO
training: review every full image and its proposal before dataset export.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path

import cv2
import numpy as np


YOLO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = YOLO_ROOT / 'data/raw/collection_20260926'
DEFAULT_OUTPUT = YOLO_ROOT / 'data/labels/collection_20260926_review'

COLOURS = (
    ('orange_cube', (0, 45, 45), (35, 255, 255), (0, 145, 255)),
    ('purple_cube', (100, 30, 25), (160, 255, 255), (220, 80, 180)),
)


def _brightness_seam(hsv, box, expected, spacing):
    """Find a dark vertical cube boundary close to an equal-width cut."""
    x, y, w, h = box
    strip = hsv[y + max(1, round(h * .10)):y + max(2, round(h * .90)),
                x:x + w, 2]
    if strip.size == 0:
        return expected
    values = cv2.GaussianBlur(strip, (5, 5), 0).mean(axis=0)
    search = max(12, min(round(spacing * .33), round(h * .28)))
    left = max(4, expected - x - search)
    right = min(w - 4, expected - x + search + 1)
    if right <= left:
        return expected
    candidates = np.arange(left, right)
    flank = min(20, max(8, round(spacing * .11)))
    contrasts = []
    for col in candidates:
        if col - flank < 0 or col + flank >= w:
            contrasts.append(-np.inf)
        else:
            contrasts.append((values[col - flank] + values[col + flank]) / 2
                             - values[col])
    best = int(np.argmax(contrasts))
    return x + int(candidates[best]) if contrasts[best] >= 3.0 else expected


def _split_count(w, h):
    ratio = w / max(h, 1)
    if ratio < 1.36:
        return 1
    if ratio < 2.31:
        return 2
    if ratio < 3.30:
        return 3
    return min(4, round(ratio))


def _polygon(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 60:
        return []
    contour = cv2.approxPolyDP(contour, .005 * cv2.arcLength(contour, True),
                               True).reshape(-1, 2)
    return contour.astype(int).tolist() if len(contour) >= 3 else []


def _component_candidates(frame, hsv, name, low, high, profile):
    height, width = frame.shape[:2]
    mask = cv2.inRange(hsv, low, high)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    proposals = []
    discarded = []
    for index in range(1, count):
        x, y, w, h, area = map(int, stats[index])
        if area < 95 or w < 12 or h < 12:
            continue
        box = [x, y, w, h]
        fill = area / (w * h)
        reason = None
        if name == 'purple_cube':
            if area < 500 or w < 28 or h < 32:
                reason = 'small_colour_region'
            elif y + h / 2 < 240:
                reason = 'upper_blue_distractor'
            elif w / h > 3 or h / w > 3.5 or fill < .50:
                reason = 'irregular_purple_region'
        else:
            if area < 850 or w < 30 or h < 32:
                reason = 'small_colour_region'
            elif profile == 'task2_purple' and y < 220 and area < 4000:
                reason = 'upper_orange_distractor_in_purple_view'
            elif fill < .44 or w / h < .43:
                reason = 'irregular_orange_region'
        if reason:
            discarded.append({'class': name, 'box': box, 'reason': reason})
            continue

        instances = _split_count(w, h) if name == 'orange_cube' else 1
        cuts = [x]
        for part in range(1, instances):
            expected = x + round(w * part / instances)
            cuts.append(_brightness_seam(hsv, box, expected, w / instances))
        cuts.append(x + w)
        if any(right <= left for left, right in zip(cuts, cuts[1:])):
            discarded.append({'class': name, 'box': box,
                              'reason': 'invalid_instance_split'})
            continue

        for part, (left, right) in enumerate(zip(cuts, cuts[1:]), 1):
            instance_mask = np.zeros((height, width), np.uint8)
            component = labels[y:y + h, left:right] == index
            instance_mask[y:y + h, left:right] = component.astype(np.uint8) * 255
            poly = _polygon(instance_mask)
            if not poly:
                discarded.append({'class': name,
                                  'box': [left, y, right - left, h],
                                  'reason': 'no_valid_polygon'})
                continue
            flags = []
            if name == 'orange_cube' and instances > 1:
                flags.append('auto_split_needs_boundary_review')
            if x == 0 or y == 0 or x + w == width or y + h == height:
                flags.append('image_edge')
            if fill < .62:
                flags.append('low_mask_fill')
            if area / (width * height) > .22:
                flags.append('large_colour_region')
            proposals.append({'class': name, 'polygon': poly,
                              'source_component': box, 'split_part': part,
                              'split_count': instances, 'flags': flags})
    return proposals, discarded


def propose(frame, profile='default'):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    proposals, discarded = [], []
    for name, low, high, _ in COLOURS:
        found, rejected = _component_candidates(frame, hsv, name, low, high,
                                                profile)
        proposals.extend(found)
        discarded.extend(rejected)
    return proposals, discarded


def overlay(frame, proposals, discarded):
    view = frame.copy()
    colours = {name: draw_colour for name, _, _, draw_colour in COLOURS}
    for proposal in proposals:
        points = np.array(proposal['polygon'], dtype=np.int32)
        colour = colours[proposal['class']]
        cv2.polylines(view, [points], True, colour, 2)
        x, y, _, _ = cv2.boundingRect(points)
        marker = 'O' if proposal['class'] == 'orange_cube' else 'P'
        if proposal['flags']:
            marker += '!'
        cv2.putText(view, marker, (x, max(16, y - 3)),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, colour, 2)
    for rejected in discarded:
        x, y, w, h = rejected['box']
        if w * h >= 2500:
            cv2.rectangle(view, (x, y), (x + w, y + h), (0, 0, 255), 1)
    return view


def run(source, output):
    source = source.resolve()
    manifest = json.loads((source / 'import_manifest.json').read_text(
        encoding='utf-8'))
    records = manifest['images']
    output.mkdir(parents=True, exist_ok=True)
    (output / 'preview').mkdir(exist_ok=True)
    results = []
    counts = Counter()
    for record in records:
        path = source / record['image']
        frame = cv2.imread(str(path))
        if frame is None:
            raise ValueError(f'cannot decode {path}')
        proposals, discarded = propose(frame, record['profile'])
        result = {**record, 'width': frame.shape[1], 'height': frame.shape[0],
                  'proposals': proposals, 'discarded': discarded,
                  'review_status': 'pending'}
        results.append(result)
        counts.update(p['class'] for p in proposals)
        cv2.imwrite(str(output / 'preview' / f"{record['id']:03d}.jpg"),
                    overlay(frame, proposals, discarded),
                    [cv2.IMWRITE_JPEG_QUALITY, 93])
    with (output / 'proposals.jsonl').open('w', encoding='utf-8') as stream:
        for record in results:
            stream.write(json.dumps(record, ensure_ascii=False) + '\n')
    (output / 'source.json').write_text(json.dumps({
        'source_archive': manifest['source_archive'],
        'source_sha256': manifest['source_sha256'],
        'image_count': len(results), 'proposal_counts': dict(counts),
        'annotation_method': 'HSV connected components and seam splitting',
        'review_status': 'pending',
    }, ensure_ascii=False, indent=2), encoding='utf-8')
    sheets = output / 'contact_sheets'
    sheets.mkdir(exist_ok=True)
    for first in range(0, len(results), 16):
        canvas = np.full((4 * 320, 4 * 384, 3), 25, np.uint8)
        for offset, result in enumerate(results[first:first + 16]):
            x, y = (offset % 4) * 384, (offset // 4) * 320
            preview = cv2.imread(str(output / 'preview' /
                                     f"{result['id']:03d}.jpg"))
            canvas[y + 30:y + 318, x:x + 384] = cv2.resize(
                preview, (384, 288), interpolation=cv2.INTER_AREA)
            tally = Counter(p['class'] for p in result['proposals'])
            count_text = f"O{tally['orange_cube']} P{tally['purple_cube']}"
            flagged = sum(bool(p['flags']) for p in result['proposals'])
            title = (f"{result['id']:03d} {result['phase']} "
                     f"{count_text} !{flagged}")
            cv2.putText(canvas, title, (x + 6, y + 19),
                        cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1)
        cv2.imwrite(str(sheets / f'sheet_{first // 16 + 1:03d}.jpg'),
                    canvas, [cv2.IMWRITE_JPEG_QUALITY, 91])
    print(json.dumps({'images': len(results), 'proposals': dict(counts),
                      'review_pending': len(results),
                      'output': str(output)}, ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=DEFAULT_SOURCE)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    run(args.source, args.output)


if __name__ == '__main__':
    main()
