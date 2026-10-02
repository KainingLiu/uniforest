"""Conservative row previews in the existing, fixed pickup plane.

Only observed outer edges or dark seams bound a position preview. A continuous
row is reported separately, so the strategy can use a nominal one-cube step
without presenting inferred individual positions as measurements. Thresholds
are software assumptions requiring field validation.
"""

import cv2
import numpy as np

try:
    from .orange_fixed_geometry import geometry_for, image_to_world, world_to_image
except ImportError:
    from orange_fixed_geometry import geometry_for, image_to_world, world_to_image


def _sample_top(frame, xs, profile, cube_cm):
    """Rectify only a narrow interior strip, retaining a visibility mask."""
    height, width = frame.shape[:2]
    ys = np.linspace(.2 * cube_cm, .8 * cube_cm, 17)
    xx, yy = np.meshgrid(xs, ys)
    points = np.stack((xx, yy, np.ones_like(xx)), axis=0).reshape(3, -1)
    projected = geometry_for(profile)[0] @ points
    if (projected[2] <= 0).any():
        raise ValueError('top strip is behind the calibrated plane')
    px = (projected[0] / projected[2] * width / 1280).reshape(xx.shape).astype(np.float32)
    py = (projected[1] / projected[2] * height / 720).reshape(xx.shape).astype(np.float32)
    visible = (px >= 3) & (px < width - 3) & (py >= 3) & (py < height - 3)
    sampled = cv2.remap(frame, px, py, cv2.INTER_LINEAR)
    gray = cv2.cvtColor(sampled, cv2.COLOR_BGR2GRAY).astype(float)
    hsv = cv2.cvtColor(sampled, cv2.COLOR_BGR2HSV)
    orange = cv2.inRange(hsv, (0, 45, 60), (40, 255, 255)) > 0
    return gray, orange, visible


def _row_limits(frame, corners, profile):
    height, width = frame.shape[:2]
    corners = np.asarray(corners, dtype=float)
    if corners.shape != (4, 2) or not np.isfinite(corners).all():
        return None
    left = float(image_to_world(corners[3], width, height, profile)[0])
    right = float(image_to_world(corners[2], width, height, profile)[0])
    return left, right


def continuous_row(frame, corners, *, profile, cube_cm=10.0):
    """Evidence of a row continuing beyond the first cube, without locating each.

    Require at least 1.8 nominal widths of visible orange support after the
    known left edge. Thin dark seams are allowed; a large hole/gap rejects the
    assumption. The strategy applies the user-selected 100 mm nominal pitch.
    """
    limits = _row_limits(frame, corners, profile)
    if limits is None or not 1.8*cube_cm <= limits[1]-limits[0] <= 100.0:
        return False
    xs = np.linspace(limits[0], limits[0]+1.8*cube_cm, round(18*cube_cm)+1)
    _, orange, visible = _sample_top(frame, xs, profile, cube_cm)
    if not visible.all() or np.mean(orange) < .8:
        return False
    missing = np.flatnonzero(np.mean(orange, axis=0) < .6)
    groups = np.split(missing, np.where(np.diff(missing) > 1)[0]+1) if len(missing) else ()
    return all(len(group) <= 4 for group in groups)


def row_previews(frame, corners, *, profile, cube_cm=10.0, clipped_right=False):
    """Return complete (center_world, quad_pixels) previews from one fitted row.

    Sample the interior of the calibrated top plane at 1 mm spacing. A seam
    must be dark relative to both sides across most of the sampled depth.
    Adjacent observed boundaries must enclose roughly one cube, with orange
    support in its interior. Cropped right edges never close a candidate.
    """
    height, width = frame.shape[:2]
    limits = _row_limits(frame, corners, profile)
    if limits is None:
        return []
    left, right = limits
    length = right-left
    if not .8*cube_cm <= length <= 100.0:
        return []
    xs = np.linspace(left, right, round(length*10)+1)
    try:
        gray, orange, visible = _sample_top(frame, xs, profile, cube_cm)
    except ValueError:
        return []
    dark = np.zeros(len(xs), bool)
    # At least 70% of depth samples must agree; a small printed mark is ignored.
    count = len(xs) - 12
    windows = np.lib.stride_tricks.sliding_window_view(gray, 4, axis=1)
    a = np.median(windows[:, :count], axis=2)
    b = np.median(windows[:, 9:9+count], axis=2)
    baseline = np.minimum(a, b)
    evidence = (baseline >= 45) & (gray[:, 6:-6] < .65 * baseline)
    dark[6:-6] = np.mean(evidence & visible[:, 6:-6], axis=0) >= .7
    boundaries = [0]
    indices = np.flatnonzero(dark)
    if len(indices):
        groups = np.split(indices, np.where(np.diff(indices) > 1)[0] + 1)
        boundaries.extend(int(np.median(group)) for group in groups
                          if len(group) <= 12)
    if not clipped_right:
        boundaries.append(len(xs) - 1)
    previews = []
    for start, end in zip(boundaries, boundaries[1:]):
        span = xs[end] - xs[start]
        if not .8 * cube_cm <= span <= 1.2 * cube_cm:
            continue
        inset = max(2, round((end - start) * .15))
        interior = slice(start + inset, end - inset + 1)
        if not visible[:, start:end+1].all() or np.mean(orange[:, interior]) < .8:
            continue
        quad = np.array([world_to_image(p, width, height, profile) for p in
                         ((xs[start], 0), (xs[end], 0),
                          (xs[end], cube_cm), (xs[start], cube_cm))], np.float32)
        if ((quad[:, 0] < 3) | (quad[:, 0] >= width-3)
                | (quad[:, 1] < 3) | (quad[:, 1] >= height-3)).any():
            continue
        previews.append((((xs[start] + xs[end]) / 2, cube_cm / 2), quad))
    return previews
