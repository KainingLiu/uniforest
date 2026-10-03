"""Reproduce the compact field model from the repository's GLB and field map.

No hardware or competition modules are imported. Geometry extraction uses only
the Python standard library. The known CAD/rule conflicts remain explicit in
the generated JSON; this script does not modify the localization configuration.
"""

import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import struct


ROOT = Path(__file__).resolve().parents[2]
CAD_NAME = '场地图v_1.0.glb'
RULES_NAME = 'RoboGame2026 竞技组规则手册2_1.pdf'
PLAN_NAME = 'RoboGame 2026 Uniforest队计划书.pdf'
MAP_NAME = 'RaspberryPi/vision/opencv/field_map.json'


def read_glb(path):
    data = path.read_bytes()
    magic, version, size = struct.unpack_from('<4sII', data)
    if (magic, version, size) != (b'glTF', 2, len(data)):
        raise ValueError('expected a complete GLB 2 file')
    chunks, offset = {}, 12
    while offset < size:
        length, kind = struct.unpack_from('<II', data, offset)
        offset += 8
        chunks[kind] = data[offset:offset+length]
        offset += length
    document = json.loads(chunks[0x4e4f534a])
    binary = chunks[0x004e4942]
    # The present model has one identity mesh node and one independent camera.
    mesh_nodes = [n for n in document['nodes'] if 'mesh' in n]
    if len(mesh_nodes) != 1 or any(k in mesh_nodes[0] for k in ('matrix', 'translation', 'rotation', 'scale')):
        raise ValueError('CAD node transform changed; review the extraction frame')

    def accessor(index):
        a = document['accessors'][index]
        if 'sparse' in a:
            raise ValueError('sparse accessors require an explicit extractor update')
        view = document['bufferViews'][a['bufferView']]
        fmt = {5121: 'B', 5123: 'H', 5125: 'I', 5126: 'f'}[a['componentType']]
        count = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}[a['type']]
        packed = struct.Struct('<' + fmt*count)
        stride = view.get('byteStride', packed.size)
        start = view.get('byteOffset', 0) + a.get('byteOffset', 0)
        return [packed.unpack_from(binary, start+i*stride) for i in range(a['count'])]

    primitives = []
    mesh = document['meshes'][mesh_nodes[0]['mesh']]
    for index, p in enumerate(mesh['primitives']):
        if p.get('mode', 4) != 4:
            raise ValueError('expected triangle primitives')
        vertices = accessor(p['attributes']['POSITION'])
        indices = [v[0] for v in accessor(p['indices'])]
        triangles = [tuple(vertices[i] for i in indices[j:j+3]) for j in range(0, len(indices), 3)]
        primitives.append({'index': index, 'material': p['material'], 'triangles': triangles})
    return document, primitives


def bounds(triangles):
    vertices = [v for triangle in triangles for v in triangle]
    return tuple(min(v[d] for v in vertices) for d in range(3)), tuple(max(v[d] for v in vertices) for d in range(3))


def components(triangles):
    """Connect coplanar triangles by their CAD vertex coordinates."""
    groups = []
    for triangle in triangles:
        keys = {tuple(round(c, 5) for c in v) for v in triangle}
        touches = [i for i, (existing, _) in enumerate(groups) if keys & existing]
        merged = [triangle]
        for i in reversed(touches):
            existing, values = groups.pop(i)
            keys |= existing
            merged.extend(values)
        groups.append((keys, merged))
    return [values for _, values in groups]


def make_model(repo):
    document, primitives = read_glb(repo / CAD_NAME)
    all_triangles = [t for p in primitives for t in p['triangles']]
    outer_min, outer_max = bounds(all_triangles)
    width = round((outer_max[0]-outer_min[0])*1000)
    height = round((outer_max[1]-outer_min[1])*1000)
    if (width, height) != (4800, 7200):
        raise ValueError('CAD outer bounds differ from the reviewed rule dimensions')
    floor_m = .010  # CAD top of the base board; see horizontal gray surfaces.

    def point(x, y):
        return [round((x-outer_min[0])*1000, 3), round((outer_max[1]-y)*1000, 3)]

    def rectangle(x0, y0, x1, y1):
        return [point(x0, y1), point(x1, y1), point(x1, y0), point(x0, y0)]

    shapes = []

    def add(identifier, label, kind, box, h, collision, source, certainty='cad_geometry', **extra):
        shape = {'id': identifier, 'label': label, 'kind': kind,
                 'polygon': rectangle(*box), 'height_mm': round(h, 3),
                 'collision': collision, 'source': source, 'certainty': certainty, **extra}
        shapes.append(shape)
        return shape

    cad_source = f'{CAD_NAME}: mesh 0, identity node 1; converted m to mm'
    add('floor', '内场地面', 'floor', (-2, -3.2, 2, 3.2), 0, False, cad_source)
    for ident, box in (
        ('north', (-2.4, 3.2, 2.4, 3.6)), ('south', (-2.4, -3.6, 2.4, -3.2)),
        ('west', (-2.4, -3.2, -2, 3.2)), ('east', (2, -3.2, 2.4, 3.2))):
        add(f'boundary_{ident}', '外侧围挡', 'wall', box, 400, True,
            f'{cad_source}; {RULES_NAME} p7', 'cad_and_rules')

    regions = {}
    for primitive in primitives:
        material = document['materials'][primitive['material']]
        color = material['pbrMetallicRoughness'].get('baseColorFactor', [1, 1, 1, 1])
        team = 'red' if color[0] > .9 and color[1] < .1 else 'blue' if color[2] > .9 and color[0] < .1 else None
        if team is None:
            continue
        horizontal = defaultdict(list)
        inclined = []
        for t in primitive['triangles']:
            if max(v[2] for v in t)-min(v[2] for v in t) < 1e-6:
                horizontal[round(t[0][2], 3)].append(t)
            else:
                inclined.append(t)
        for z, triangles in horizontal.items():
            lo, hi = bounds(triangles)
            elevation = round((z-floor_m)*1000)
            kind = {200: 'platform', 100: 'building', 10: 'start'}[elevation]
            box = lo[0], lo[1], hi[0], hi[1]
            regions[f'{kind}_{team}'] = box
            rule_page = 8 if kind == 'start' else 9
            extra = {'team_color_in_cad': team}
            if kind == 'start':
                # Rules define a tape square, not the CAD's decorative 10 mm rise.
                extra.update(cad_display_rise_mm=10, nominal_size_mm=[600, 600])
                elevation = 0
            if kind == 'building':
                extra.update(rule_nominal_size_mm=[2400, 600], cad_size_mm=[2800, 600],
                             scoring_boundary_verified=False)
            add(f'{kind}_{team}', {'platform': '中央高台', 'building': '搭建台', 'start': '启动区'}[kind],
                kind, box, elevation, kind == 'building',
                f'{cad_source}, primitive {primitive["index"]}; {RULES_NAME} p{rule_page}',
                'cad_rule_size_conflict' if kind == 'building' else 'cad_and_rules', **extra)
        lo, hi = bounds(inclined)
        box = lo[0], lo[1], hi[0], hi[1]
        regions[f'ramp_{team}'] = box
        low_y = hi[1] if team == 'red' else lo[1]
        high_y = lo[1] if team == 'red' else hi[1]
        add(f'ramp_{team}', '可行驶坡道', 'ramp', box, 200, False,
            f'{cad_source}, inclined surface primitive {primitive["index"]}; {RULES_NAME} p9',
            'cad_and_rules', team_color_in_cad=team, slope_deg=round(math.degrees(math.atan2(200, 800)), 4),
            low_edge=[point(lo[0], low_y), point(hi[0], low_y)],
            high_edge=[point(lo[0], high_y), point(hi[0], high_y)],
            low_height_mm=0, high_height_mm=200)

    red, blue = regions['platform_red'], regions['platform_blue']
    central_min_y, central_max_y = regions['ramp_blue'][1], regions['ramp_red'][3]
    add('central_divider', '中央材料隔墙', 'material_wall',
        (red[2], central_min_y, blue[0], central_max_y), 400, True, cad_source,
        note='2800 mm structural span includes two 300 mm joining ends; roof resource region is 2200 mm.')
    add('orange_shelf_southwest', '西南橙块材料台', 'material_wall',
        (red[0], central_min_y, red[2], red[1]), 400, True,
        f'{cad_source}; {RULES_NAME} p10', 'cad_and_rules', resource='orange')
    add('orange_shelf_northeast', '东北橙块材料台', 'material_wall',
        (blue[0], blue[3], blue[2], central_max_y), 400, True,
        f'{cad_source}; {RULES_NAME} p10', 'cad_and_rules', resource='orange')
    add('roof_resource_region', '紫块材料区', 'resource_region',
        (red[2], red[1], blue[0], blue[3]), 400, False,
        f'{cad_source}; {RULES_NAME} p10', 'cad_and_rules', resource='purple')

    # Locate CAD recess floors, retain originals, and label current-rule overlays.
    groove_floor = [t for p in primitives for t in p['triangles']
                    if all(abs(v[2]-.360) < 1e-5 for v in t)]
    cad_grooves, orange_centers = [], []
    for group in components(groove_floor):
        lo, hi = bounds(group)
        item = {'native_min_m': list(lo), 'native_max_m': list(hi),
                'size_mm': [round((hi[i]-lo[i])*1000) for i in range(2)]}
        cad_grooves.append(item)
        if hi[0]-lo[0] > .5:
            orange_centers.append(((lo[0]+hi[0])/2, (lo[1]+hi[1])/2))
    for index, (x, y) in enumerate(sorted(orange_centers)):
        add(f'orange_slot_{index+1}', '橙块槽（规则尺寸）', 'slot',
            (x-.750, y-.055, x+.750, y+.055), 400, False,
            f'{RULES_NAME} p10; center inherited from CAD recess',
            'rule_dimensions_inferred_center', resource='orange', floor_height_mm=350,
            depth_mm=50, capacity=10, placement_verified=False)
    for index, y in enumerate((-.310, 0, .310)):
        add(f'purple_slot_{index+1}', '紫块槽（规则尺寸）', 'slot',
            (-.055, y-.055, .055, y+.055), 400, False,
            f'{RULES_NAME} p4,p10: 110 mm square, 200 mm clear spacing, centered group',
            'rule_dimensions_centered_layout', resource='purple', floor_height_mm=350,
            depth_mm=50, capacity=1, placement_verified=False)

    anchors = {}
    for team, heading in (('blue', 270), ('red', 90)):
        box = regions[f'start_{team}']
        p = point((box[0]+box[2])/2, (box[1]+box[3])/2)
        anchors[f'start_{team}'] = {'x_mm': p[0], 'y_mm': p[1], 'yaw_deg': heading,
            'source': cad_source, 'certainty': 'cad_center_heading_for_demo',
            'note': 'Center from CAD; initial facing chosen for the matching demo half, not measured robot pose.'}
    for name, x, y, facing in (
        ('orange_ground_face', -1.1, -1.4, 270),
        ('orange_highland_face', 1.1, 1.1, 270),
        ('purple_highland_face', .2, 0, 180),
        ('blue_building_face', .6, -2.6, 90)):
        p = point(x, y)
        anchors[name] = {'x_mm': p[0], 'y_mm': p[1], 'yaw_deg': facing,
                         'source': cad_source, 'certainty': 'surface_anchor',
                         'note': 'Contact surface point; subtract the robot front extent before placing its center.'}
    field_map = json.loads((repo / MAP_NAME).read_text(encoding='utf-8'))
    tags = []
    for identifier, tag in field_map['tags'].items():
        p = point(tag['x_m'], tag['y_m'])
        item = {'id': int(identifier), 'x_mm': p[0], 'y_mm': p[1],
                'normal_yaw_deg': (-tag['normal_deg']) % 360,
                'size_mm': field_map['tag_size_m']*1000,
                'center_height_mm': 325, 'configured_center_height_mm':
                    tag.get('center_height_m', field_map['tag_center_height_m'])*1000,
                'source': MAP_NAME, 'certainty': 'existing_configuration_not_surveyed'}
        if identifier == '1':
            item.update(rule_diagram_conflict=True,
                        note='Rule p12 Fig3.9 places Tag1 by the start-side left corridor; configured y=+2 m mirrors it vertically.')
        tags.append(item)
        anchors[f'tag_{identifier}'] = {k: item[k] for k in ('x_mm', 'y_mm', 'source', 'certainty')}

    sources = {name: {'sha256': hashlib.sha256((repo/name).read_bytes()).hexdigest()}
               for name in (CAD_NAME, RULES_NAME, PLAN_NAME, MAP_NAME)}
    return {
        'version': 1, 'width_mm': width, 'height_mm': height,
        'coordinate_system': {'origin': 'outer northwest corner', 'x': 'right', 'y': 'down',
            'yaw_zero': 'right', 'yaw_positive': 'clockwise', 'z': 'up from inner floor',
            'cad_to_sim': 'x=1000*X+2400; y=3600-1000*Y; z=1000*Z-10',
            'field_map_to_sim': 'x=1000*x_m+2400; y=3600-1000*y_m; yaw=-yaw_math_deg'},
        'robot': {'model_ref': 'robot_model.json', 'assumed': False,
                  'source': f'{PLAN_NAME} p8 web CAD; dimensions and model limitations are in robot_model.json'},
        'playable_bounds_mm': {'left': 400, 'top': 400, 'right': 4400, 'bottom': 6800},
        'shapes': shapes, 'anchors': anchors, 'tags': tags,
        'rules': {'cube_side_mm': 100, 'cube_tolerance_fraction': .05, 'orange_blocks_total': 20,
                  'purple_blocks_total': 3, 'tape_width_mm': 50, 'tape_placement_tolerance_fraction': .05,
                  'initial_robot_envelope_mm': [600, 600, 600],
                  'expanded_robot_envelope_mm': [800, 800, 1000],
                  'robot_dimensions_are_limits_not_measurements': True},
        'cad': {'file': CAD_NAME, 'generator': document['asset'].get('generator'),
                'node_count': len(document['nodes']), 'mesh_count': len(document['meshes']),
                'triangle_count': len(all_triangles), 'units': 'm; checked against rule outer dimensions',
                'native_min': list(outer_min), 'native_max': list(outer_max),
                'floor_top_m': floor_m, 'original_grooves': cad_grooves},
        'uncertainties': [
            'Solid geometry follows supplied CAD; no on-site survey or physics calibration.',
            'CAD building platforms are 2800x600 mm; rule p9 says approximately 2400x600 mm. Scoring boundary requires confirmation.',
            'CAD orange recesses are 1000x100 mm; current rules say 1500x110 mm. Display overlays preserve CAD centers with rule sizes.',
            'Rule p10 orange shelf length 1800, slot length 1500, and two end offsets of 400 mm cannot all hold; slot placement remains unverified.',
            'CAD purple recesses are 100 mm with 300 mm center spacing; current rules imply 110 mm and 310 mm center spacing.',
            'Tag1 configured y=+2 m conflicts with the start-side location in rule Fig3.9; current localization data is preserved.',
            'Configured Tag3/4 heights are 125 mm, while the rule wall-top geometry gives 325 mm globally; a 200 mm platform datum explains the difference but is not independently surveyed.',
            'Red/blue surfaces are CAD illustration colors. Rules state real surfaces use the boundary color.',
            'Start heading, cube placements and robot dynamics must be selected separately for simulation; they are not measured facts.',
        ],
        'sources': sources,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=ROOT)
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('field_model.json'))
    args = parser.parse_args()
    model = make_model(args.repo)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(model, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(f'{model["width_mm"]} x {model["height_mm"]} mm; '
          f'{len(model["shapes"])} shapes; {model["cad"]["triangle_count"]} source triangles')


if __name__ == '__main__':
    main()
