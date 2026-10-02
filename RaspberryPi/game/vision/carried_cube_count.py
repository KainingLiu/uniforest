"""Fixed-pose storage observation shared by competition and standalone capture.

Stacked cubes occlude each other. Count is classified from orange width in the bottom 30% of the image
at the inspection pose. Purple in this region overrides the count to one.
"""

from __future__ import annotations

import cv2
import numpy as np

FEATURE_VERSION = 4


def roi_pixels(frame, roi):
    """Resolve a normalized ROI without shifting full-frame coordinates."""
    x, y, w, h = roi
    if not all(np.isfinite([x, y, w, h])) or not (
            0 <= x < 1 and 0 <= y < 1 and w > 0 and h > 0
            and x + w <= 1 and y + h <= 1):
        raise ValueError('roi must be normalized [x, y, width, height] inside image')
    height, width = frame.shape[:2]
    x0, y0 = round(x * width), round(y * height)
    x1, y1 = round((x + w) * width), round((y + h) * height)
    if x1 - x0 < 8 or y1 - y0 < 8:
        raise ValueError('storage ROI is too small')
    return x0, y0, x1, y1


def purple_evidence(frame, rule):
    """Require a substantial purple component inside the lower storage area."""
    if not rule or not rule.get('enabled'):
        return {'purple_detected': False, 'purple_fraction': 0.0}
    x0, y0, x1, y1 = roi_pixels(frame, rule['roi'])
    hsv = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(rule['hsv_low'], dtype=np.uint8),
                        np.array(rule['hsv_high'], dtype=np.uint8))
    _, _, components, _ = cv2.connectedComponentsWithStats(mask)
    if len(components) < 2:
        return {'purple_detected': False, 'purple_fraction': 0.0}
    component = max(components[1:], key=lambda item: item[cv2.CC_STAT_AREA])
    x, y, w, h, area = (int(value) for value in component)
    fraction = area / mask.size
    detected = (fraction >= rule['min_area_fraction']
                and w / mask.shape[1] >= rule['min_width_fraction']
                and h / mask.shape[0] >= rule['min_height_fraction'])
    return {'purple_detected': bool(detected), 'purple_fraction': float(fraction),
            'purple_bbox': [x0 + x, y0 + y, w, h] if detected else None}


def observe(frame, config):
    """Measure orange width using only the bottom 30%; never inspect the top edge."""
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError('expected a BGR camera image')
    if config.get('roi') != [0, .7, 1, .3]:
        raise ValueError('width calibration requires full-width bottom 30% ROI')
    height, width = frame.shape[:2]
    x0, y0, x1, y1 = roi_pixels(frame, config['roi'])
    crop = frame[y0:y1, x0:x1]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    rule = config['orange_width_rule']
    mask = cv2.inRange(hsv, np.array(rule['hsv_low'], dtype=np.uint8),
                           np.array(rule['hsv_high'], dtype=np.uint8))
    _, labels, components, _ = cv2.connectedComponentsWithStats(mask)
    clean = np.zeros_like(mask)
    if len(components) > 1:
        index = 1 + int(np.argmax(components[1:, cv2.CC_STAT_AREA]))
        if components[index, cv2.CC_STAT_AREA] >= mask.size * rule['min_area_fraction']:
            clean[labels == index] = 255
    # The median of per-row spans is translation tolerant and ignores isolated
    # holes/highlights. Missing rows count as zero; never infer width from the top.
    row_widths = []
    endpoints = []
    for row in clean:
        xs = np.flatnonzero(row)
        row_widths.append(int(xs[-1] - xs[0] + 1) if xs.size else 0)
        endpoints.append((int(xs[0]), int(xs[-1])) if xs.size else None)
    orange_width = float(np.median(row_widths))
    purple_rule = config.get('purple_rule') or {}
    if purple_rule.get('enabled'):
        _, py0, _, _ = roi_pixels(frame, purple_rule['roi'])
        if py0 < y0:
            raise ValueError('purple ROI must also stay in bottom 30%')
    purple = purple_evidence(frame, purple_rule)
    preview = frame.copy()
    tint = crop.copy()
    tint[clean > 0] = (0, 255, 0)
    preview[y0:y1, x0:x1] = cv2.addWeighted(crop, .7, tint, .3, 0)
    cv2.rectangle(preview, (x0, y0), (x1 - 1, y1 - 1), (255, 255, 0), 2)
    if orange_width:
        visible_rows = np.flatnonzero(np.array(row_widths) > 0)
        row_index = int(visible_rows[np.argmin(
            np.abs(np.array(row_widths)[visible_rows] - orange_width))])
        left, right = endpoints[row_index]
        cv2.line(preview, (left, y0 + row_index), (right, y0 + row_index), (0, 0, 255), 2)
    if purple['purple_detected']:
        px, py, pw, ph = purple['purple_bbox']
        cv2.rectangle(preview, (px, py), (px + pw - 1, py + ph - 1), (255, 0, 255), 2)
    cv2.putText(preview, f'orange width: {orange_width:.1f}px', (10, y0 + 22),
                cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 2)
    return {'feature_version': FEATURE_VERSION, **purple,
            'features': [orange_width / width],
            'orange_width_px': orange_width,
            'row_widths_px': row_widths,
            'colored_fraction': float(np.mean(clean > 0)),
            'brightness': float(np.median(hsv[:, :, 2])) / 255,
            'roi_pixels': [x0, y0, x1 - x0, y1 - y0],
            'image_size': [width, height]}, preview


def classify(observations, config):
    """Use explicitly supplied count prototypes, with distance/ambiguity rejection.

    Calibration must be generated using this exact config, camera and pose.
    No thresholds or count references are inferred from unlabelled images.
    """
    if not observations:
        raise ValueError('no observations')
    prototypes = config.get('count_prototypes', [])
    if config.get('roi') is None or not prototypes:
        return {'count': None, 'status': 'uncalibrated'}
    if config.get('feature_version') != FEATURE_VERSION:
        raise ValueError('unsupported calibration feature version')
    if any(o.get('feature_version') != FEATURE_VERSION for o in observations):
        raise ValueError('observation version differs; re-extract from raw images')
    if len({tuple(o['image_size']) for o in observations}) != 1:
        return {'count': None, 'status': 'unstable', 'reason': 'image_size_changed'}
    if min(o['brightness'] for o in observations) < config['orange_width_rule']['min_brightness']:
        return {'count': None, 'status': 'uncertain', 'reason': 'image_too_dark'}
    purple_frames = sum(bool(o.get('purple_detected')) for o in observations)
    rule = config.get('purple_rule', {})
    if rule.get('enabled') and purple_frames:
        required = int(np.ceil(len(observations) * rule['min_frame_fraction']))
        confirmed = purple_frames >= required
        return {'count': 1 if confirmed else None,
                'status': 'ok' if confirmed else 'unstable',
                'method': 'purple_rule',
                'reason': 'purple_in_storage' if confirmed else 'purple_not_consistent',
                'purple_frames': purple_frames, 'frame_count': len(observations)}
    radius = config.get('max_distance')
    margin = config.get('min_margin')
    if radius is None or margin is None or not (0 < radius <= 1 and 0 < margin <= 1):
        raise ValueError('calibration needs positive max_distance and min_margin')
    distances = []
    sample = np.median([item['features'] for item in observations], axis=0)
    expected_counts = set(range(config['max_count'] + 1))
    if {p['count'] for p in prototypes} != expected_counts or len(prototypes) != len(expected_counts):
        raise ValueError('calibrate every count including empty, exactly once')
    for prototype in prototypes:
        # Group real placement references by count before computing the margin;
        # two references of the same count are not competing count classes.
        candidates = [prototype['features'], *prototype.get('additional_features', [])]
        class_distances = []
        for candidate in candidates:
            reference = np.asarray(candidate, dtype=float)
            if reference.shape != sample.shape or not np.all(np.isfinite(reference)):
                raise ValueError('invalid count prototype')
            class_distances.append(float(np.sqrt(np.mean((sample - reference) ** 2))))
        distances.append((min(class_distances), prototype['count']))
    distances.sort()
    best, count = distances[0]
    gap = distances[1][0] - best if len(distances) > 1 else 0
    status = 'ok' if best <= radius and gap >= margin else 'uncertain'
    # A median must not conceal strongly inconsistent frames.
    spread = max(float(np.sqrt(np.mean((np.asarray(o['features']) - sample) ** 2)))
                 for o in observations)
    if spread > radius:
        status = 'unstable'
    reason = ('unstable_frames' if status == 'unstable' else
              'outside_calibration' if best > radius else
              'ambiguous_count' if gap < margin else 'matched')
    return {'count': count if status == 'ok' else None, 'status': status,
            'method': 'orange_width', 'reason': reason, 'nearest_count': count,
            'orange_width_px': float(np.median([o['orange_width_px'] for o in observations])),
            'orange_width_fraction': float(sample[0]),
            'distance': best, 'margin': gap, 'frame_spread': spread}
