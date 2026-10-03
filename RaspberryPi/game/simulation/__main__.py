"""Build the local simulation report and self-contained viewer (no hardware)."""
from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
import json
import math
from pathlib import Path

from .catalog import run_catalog
from .geometry import assess_trace

HERE = Path(__file__).resolve().parent


def demo_start(case, field, robot):
    """Explicit, independent demonstration placements, never surveyed anchors.

    No post-hoc motion to hide collisions. All three candidates share one pose.
    Wall contact poses, visual corrections and global mission poses are unknown.
    """
    route, profile = case['route'], case['profile']
    reach = robot['front_extent_mm'] + 25.0
    offsets = {'ground-1': 100, 'ground-2': 400, 'ground-3': -500,
               'building-1': 100, 'building-2': 400, 'building-3': -500}
    lateral = case['start_assumptions']['search_lateral_mm']
    if route in ('depart_a', 'depart_b'):
        x, y, yaw = 700, 6500, 270
    elif route == 'ground_to_delivery':
        x, y, yaw = 600+lateral, 5000+reach+400, 270
    elif route == 'purple_to_orange':
        x, y, yaw = 2600+reach, 3600, 180
    elif route == 'orange_to_build':
        x, y, yaw = 3100+lateral, 2500+reach+100, 270
    elif route in ('build_offset', 'ground_tag_offset'):
        x, y, yaw = 3500, 6200-reach, 90
    elif route == 'build_return':
        x, y, yaw = 3500-offsets[profile], 6200-reach, 90
    elif route == 'ground_delivery_depart':
        x, y, yaw = 3500-offsets[profile], 6200-reach-300, 90
    elif route == 'return_orange':
        x, y, yaw = 3500, 6200-reach-300, 90
    elif route == 'staged_initial':
        x, y, yaw = 3500, 6200-reach-300, 90
    elif route == 'staged_to_build':
        x, y, yaw = 3750, 6200-reach, 90
    elif route in ('staged_return_first', 'staged_return_final'):
        x, y, yaw = 2810, 6200-reach-250, 90
    elif route == 'unload_depart':
        x, y, yaw = (3900 if profile == 'unload-1' else 3600), 6200-reach-300, 90
    else:
        raise ValueError(f'missing explicit demonstration placement: {route}')
    return dict(x_mm=round(x, 3), y_mm=round(y, 3), yaw_deg=yaw,
                certainty='assumed_independent_demo_pose',
                note='Nominal region with CAD nose clearance; not a surveyed mission anchor.')


def build_report(max_points=90):
    field = json.loads((HERE/'field_model.json').read_text(encoding='utf-8'))
    robot = json.loads((HERE/'robot_model.json').read_text(encoding='utf-8'))
    report = run_catalog(max_points=max_points)
    report['field'], report['robot'] = field, robot
    for case in report['cases']:
        case['start_pose'] = demo_start(case, field, robot)
        for layer in case['layers']:
            layer['assessment'] = assess_trace(layer['points'], case['start_pose'], field, robot)
            layer['collision_checked'] = True
            layer['collision_check_kind'] = 'postprocessed_2d_envelope_under_assumed_placement'
    all_layers = [l for c in report['cases'] for l in c['layers']]
    report['meta'] = dict(
        generated_at=datetime.now(timezone.utc).isoformat(),
        route_count=report['curve_family_count'], case_count=len(report['cases']),
        run_count=len(all_layers), controller_completed=report['completed'],
        envelope_interference_runs=sum(l['assessment']['collision_count'] > 0 for l in all_layers),
        no_interference_runs=sum(l['assessment']['collision_count'] == 0 for l in all_layers),
        status='offline_simulation_not_field_validation',
    )
    report['limitations'].extend([
        'Every world start pose is an independent demonstration assumption; routes are not a full-plan global replay.',
        'Collision is assessed after the motion simulation; no collision force or contact response is simulated.',
        'Planar CAD envelope can flag high parts over low obstacles; height clearance requires a separate 3D check.',
        'Published robot CAD is the earlier claw version. Current suction mechanism must be checked physically.',
    ])
    return report


def viewer_data(report):
    """Keep inline output below 1 MB without discarding any simulated case."""
    result = {k: copy.deepcopy(report[k]) for k in ('meta', 'field', 'robot', 'cases')}
    for shape in result['field']['shapes']:
        shape['show_label'] = shape['kind'] in ('platform', 'building', 'start', 'material_wall')
        # Repeated walls/resources need geometry, not repeated overlaid labels.
        if shape['id'] == 'central_divider':
            shape['show_label'] = False
    for case in result['cases']:
        for key in ('original_steps', 'prefix_waypoints', 'plan_steps', 'preserved_tail'):
            case.pop(key, None)
        for layer in case['layers']:
            layer.pop('reference_runs', None)
            a = layer['assessment']
            a['collisions'] = a['collisions'][:12]
            # Fine raw controller metrics and warnings remain in report.json.
            for hit in a['collisions']:
                for key in list(hit):
                    if key not in ('shape_id', 'label', 't', 'x_mm', 'y_mm', 'evidence'):
                        hit.pop(key)
            for p in layer['points']:
                for key in list(p):
                    if key not in ('t', 'x', 'y', 'yaw', 'vx', 'vy', 'wz'):
                        p.pop(key)
                    elif isinstance(p[key], float):
                        p[key] = round(p[key], 2)
    return result


def write_outputs(report, output, inline=None):
    output.mkdir(parents=True, exist_ok=True)
    (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    encoded = json.dumps(viewer_data(report), ensure_ascii=False, separators=(',', ':'), allow_nan=False).replace('</', '<\\/')
    template = (HERE/'viewer.template.html').read_text(encoding='utf-8')
    fragment = template.replace('__SIMULATION_DATA__', encoded)
    if len(fragment.encode('utf-8')) >= 1_000_000:
        raise ValueError('inline visual exceeded 1 MB; lower max_points')
    (output/'fragment.html').write_text(fragment, encoding='utf-8')
    shell = (HERE/'viewer.shell.html').read_text(encoding='utf-8')
    (output/'simulator.html').write_text(shell.replace('<!--SIMULATOR-->', fragment), encoding='utf-8')
    if inline:
        inline.parent.mkdir(parents=True, exist_ok=True)
        inline.write_text(fragment, encoding='utf-8')
    summary = report['meta']
    lines = ['# 离线曲线仿真结果', '',
             f"覆盖 {summary['route_count']} 类路线、{summary['case_count']} 个轮次/路线配对、{summary['run_count']} 次运行。",
             f"控制完成 {summary['controller_completed']} 次；演示摆位下 {summary['envelope_interference_runs']} 次存在平面包络干涉。",
             '', '耗时仅比较相同虚拟动力学下的移动段，墙接触和视觉阶段不计入。',
             '全部结果为 simulation_only，不能导入实机标定配置。', '',
             '| 场景 | 方式 | 控制结果 | 时间/s | 路长/mm | 末端误差/mm | 平面包络 |',
             '| --- | --- | --- | ---: | ---: | ---: | --- |']
    for case in report['cases']:
        for layer in case['layers']:
            m = layer['metrics']
            hit = '干涉' if layer['assessment']['collision_count'] else '未检出'
            lines.append(f"| {case['id']} | {layer['label']} | {layer['status']} | {m['elapsed_s']:.2f} | {m['path_length_mm']:.1f} | {m['final_position_error_mm']:.1f} | {hit} |")
    (output/'summary.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False))
    print(f"Viewer: {output/'simulator.html'}; inline bytes={len(fragment.encode('utf-8'))}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=HERE/'output')
    parser.add_argument('--inline', type=Path)
    parser.add_argument('--max-points', type=int, default=90)
    parser.add_argument('--routes-only', action='store_true',
                        help='Build independent route comparisons instead of full PlanA/PlanB replays')
    args = parser.parse_args()
    if not 20 <= args.max_points <= 150:
        parser.error('--max-points must be between 20 and 150')
    if args.routes_only:
        write_outputs(build_report(args.max_points), args.output.resolve(), args.inline)
    else:
        from .match_report import build_match_report, write_match_outputs
        write_match_outputs(build_match_report(), args.output.resolve(), args.inline)


if __name__ == '__main__':
    main()
