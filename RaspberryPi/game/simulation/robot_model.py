"""Extract a lightweight conservative model from the team's published GLB.

Uses accessor bounds and node transforms, so Draco decompression is unnecessary.
Boxes are a visual/collision approximation of CAD parts, not a solid mesh.
"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import struct

import numpy as np

SOURCE_URL = 'https://ustclty.github.io/2026RoboGame/models/assembly.glb'


def transform(node):
    if 'matrix' in node:
        return np.array(node['matrix'], dtype=float).reshape(4, 4).T
    x, y, z, w = node.get('rotation', [0, 0, 0, 1])
    r = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    result = np.eye(4)
    result[:3, :3] = r @ np.diag(node.get('scale', [1, 1, 1]))
    result[:3, 3] = node.get('translation', [0, 0, 0])
    return result


def extract(path):
    blob = Path(path).read_bytes()
    magic, version, total = struct.unpack_from('<III', blob)
    if magic != 0x46546C67 or version != 2 or total != len(blob):
        raise ValueError('expected a complete GLB 2.0 file')
    size, kind = struct.unpack_from('<II', blob, 12)
    if kind != 0x4E4F534A:
        raise ValueError('missing GLB JSON chunk')
    gltf = json.loads(blob[20:20+size])
    parts = []

    def visit(index, parent):
        node = gltf['nodes'][index]
        matrix = parent @ transform(node)
        if 'mesh' in node:
            for primitive in gltf['meshes'][node['mesh']]['primitives']:
                bounds = gltf['accessors'][primitive['attributes']['POSITION']]
                corners = np.array([(*p, 1) for p in itertools.product(
                    *zip(bounds['min'], bounds['max']))])
                world = (matrix @ corners.T).T[:, :3] * 1000
                low, high = world.min(0), world.max(0)
                parts.append({'node': index, 'name': node.get('name', ''),
                              'low': low, 'high': high})
        for child in node.get('children', []):
            visit(child, matrix)

    for index in gltf['scenes'][gltf.get('scene', 0)]['nodes']:
        visit(index, np.eye(4))
    low = np.min([p['low'] for p in parts], axis=0)
    high = np.max([p['high'] for p in parts], axis=0)
    wheels = [p for p in parts if p['name'].startswith('RM-')]
    wheel_center = np.mean([(p['low']+p['high'])/2 for p in wheels], axis=0)
    if not len(wheels):
        wheel_center = (low+high)/2
    # Team web viewer identifies -Z as front, +Y as vertical. Preserve metre
    # scale before the website's display-only normalization to size 3.
    boxes = []
    for p in parts:
        a, b = p['low'], p['high']
        center = (a+b)/2
        boxes.append({'x': round(wheel_center[2]-center[2], 2),
                      'y': round(center[0]-wheel_center[0], 2),
                      'z': round(center[1]-low[1], 2),
                      'length': round(b[2]-a[2], 2),
                      'width': round(b[0]-a[0], 2),
                      'height': round(b[1]-a[1], 2),
                      'kind': 'wheel' if p['name'].startswith('RM-') else 'part'})
    front, rear = wheel_center[2]-low[2], high[2]-wheel_center[2]
    left, right = wheel_center[0]-low[0], high[0]-wheel_center[0]
    return {
        'source_url': SOURCE_URL, 'source_sha256': hashlib.sha256(blob).hexdigest(),
        'units': 'mm', 'coordinate': 'x forward (-GLB Z), y right (+GLB X), z up (+GLB Y)',
        'reference': 'wheel group center, lowest accessor bound at z=0',
        'source_status': 'published_plan_cad_unconfirmed_current_hardware',
        'length_mm': round(high[2]-low[2], 2),
        'width_mm': round(high[0]-low[0], 2),
        'height_mm': round(high[1]-low[1], 2),
        'collision_length_mm': round(2*max(front, rear), 2),
        'collision_width_mm': round(2*max(left, right), 2),
        'front_extent_mm': round(front, 2), 'rear_extent_mm': round(rear, 2),
        'arm_extension_mm': 270,
        'arm_extension_source': 'current Grap2/Grap3 commanded horizontal stroke; not a measured swept envelope',
        'boxes': boxes,
        'notes': ['Published CAD shows the earlier claw assembly; current suction mechanism can differ.',
                  'Accessor boxes are conservative bounds, not precise meshes.',
                  '270 mm extension is an optional added envelope, not a validated arm posture model.'],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('glb', type=Path)
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('robot_model.json'))
    args = parser.parse_args()
    data = extract(args.glb)
    args.output.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in data.items() if k != 'boxes'}, ensure_ascii=False, indent=2))
    print(f"CAD proxy: {len(data['boxes'])} part bounds")


if __name__ == '__main__':
    main()
