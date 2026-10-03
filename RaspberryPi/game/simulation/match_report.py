"""Generate full-match replays without touching robot hardware or field tuning."""
from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
import json
from pathlib import Path

from .geometry import assess_trace

HERE = Path(__file__).resolve().parent


def build_match_report():
    from .match import run_match
    from .core import SimulationSettings
    field = json.loads((HERE/'field_model.json').read_text(encoding='utf-8'))
    robot = json.loads((HERE/'robot_model.json').read_text(encoding='utf-8'))
    settings = SimulationSettings()
    matches = [run_match(name, max_points=1200, settings=settings, route_mode=mode)
               for name in ('PlanA', 'PlanB') for mode in ('stop_turn', 'continuous')]
    for match in matches:
        # Match points already use world coordinates. Identity avoids applying
        # the original start transform a second time.
        match['field_assessment'] = assess_trace(
            match['points'], {'x_mm': 0, 'y_mm': 0, 'yaw_deg': 0}, field, robot)
        match['field_validated'] = False
    return {'generated_at': datetime.now(timezone.utc).isoformat(),
            'simulation_only': True, 'field': field, 'robot': robot, 'matches': matches}


def compact_data(report):
    value = copy.deepcopy(report)
    for match in value['matches']:
        for key in list(match):
            if key not in ('plan', 'route_mode', 'status', 'points', 'events', 'state_frames',
                           'initial_cubes', 'metrics', 'field_assessment', 'curve_runs'):
                match.pop(key)
        for event in match['events']:
            for key in list(event):
                if key not in ('index', 'name', 'kind', 'profile', 'label', 't0', 't1', 'status'):
                    event.pop(key)
        for run in match.get('curve_runs', []):
            run['source_motion_leg_count'] = len(run.get('source_motion_legs', []))
            for key in list(run):
                if key not in ('action', 'name', 'route', 'profile', 't0', 't1', 'prefix_end_s',
                               'curve_enabled', 'multi_leg', 'source_motion_leg_count',
                               'prefix_controller_runs', 'internal_nodes'):
                    run.pop(key)
        for point in match['points']:
            for key in list(point):
                if key not in ('t', 'x', 'y', 'yaw', 'vx', 'vy', 'wz', 'action', 'stage'):
                    point.pop(key)
                elif isinstance(point[key], float):
                    point[key] = round(point[key], 2)
        # All actions and resource transfers remain present in the visual.
        assessment = match['field_assessment']
        assessment['collisions'] = assessment['collisions'][:24]
        for hit in assessment['collisions']:
            for key in list(hit):
                if key not in ('shape_id', 't', 'x_mm', 'y_mm'):
                    hit.pop(key)
    return value


def write_match_outputs(report, output, inline=None):
    from Strategy.plans import PLAN_A, PLAN_B
    from .catalog import TITLES
    source = {p.name: {s.name: s for s in p.steps} for p in (PLAN_A, PLAN_B)}
    route_titles = dict(TITLES, to_purple='前往紫块采集区',
                        ground_delivery_reverse='离开地面橙块墙', orange_depart_reverse='离开高地橙块墙',
                        unload_approach='接近混合方块暂存位置')
    for match in report['matches']:
        for event in match['events']:
            spec = source[match['plan']][event['name']]
            params, kind = spec.parameters, spec.kind
            region = '地面' if spec.profile.startswith('ground-') else '高地'
            color = '紫块' if '.purple.' in spec.name else '橙块'
            prefix = f"第{spec.name.split('.')[1]}轮 · " if spec.name.startswith('a.') and spec.name.split('.')[1].isdigit() else ''
            label = {'navigate': route_titles.get(params.get('route'), '移动'),
                     'begin_collection': f"开始{region}{'紫块' if params.get('color')=='purple' else '橙块'}采集",
                     'acquire_cube': f"搜索并对准{region}{color}",
                     'grab_cube': f"抓取{region}{color} · {params.get('method','')}",
                     'inspect_cargo': '查看舱内并检查携带数量',
                     'unload': '卸下橙块形成基座' if spec.profile.startswith('ground-') else '卸下混合方块暂存',
                     'load_staged': '从暂存位置装载方块', 'build': '逐块搭建并回收机构',
                     'align_building': '对准已有建筑基座', 'align_tag': '识别 Tag 并定位',
                     'anchor_wall': '靠墙定位', 'rebase_heading': '更新航向基准'}[kind]
            event['label'] = prefix + label
    output.mkdir(parents=True, exist_ok=True)
    (output/'match-report.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    data = json.dumps(compact_data(report), ensure_ascii=False,
                      separators=(',', ':'), allow_nan=False).replace('</', '<\\/')
    fragment = (HERE/'match.template.html').read_text(encoding='utf-8').replace('__MATCH_DATA__', data)
    if len(fragment.encode('utf-8')) >= 1_000_000:
        raise ValueError('full-match inline replay exceeded 1 MB')
    (output/'match-fragment.html').write_text(fragment, encoding='utf-8')
    shell = (HERE/'viewer.shell.html').read_text(encoding='utf-8').replace(
        '<title>Uniforest 曲线仿真</title>', '<title>Uniforest 整局比赛仿真</title>')
    html = shell.replace('<!--SIMULATOR-->', fragment)
    (output/'match.html').write_text(html, encoding='utf-8')
    # Preserve the old curve lab under a stable name while making the user's
    # already-open /simulator.html show the requested full-match replay.
    previous = output/'simulator.html'
    if previous.exists() and 'uniforest-motion-lab' in previous.read_text(encoding='utf-8'):
        (output/'curves.html').write_text(previous.read_text(encoding='utf-8'), encoding='utf-8')
    previous.write_text(html, encoding='utf-8')
    if inline:
        inline.parent.mkdir(parents=True, exist_ok=True)
        inline.write_text(fragment, encoding='utf-8')
    print(json.dumps({f"{m['plan']}/{m.get('route_mode','stop_turn')}": {
        'status': m['status'], 'actions': len(m['events']), 'duration_s': m['points'][-1]['t'],
        'builds': m['state_frames'][-1]['build_count'],
        'remaining': len(m['state_frames'][-1]['remaining']),
        'interference_segments': m['field_assessment']['collision_count'],
        'curve_runs': m['metrics'].get('curve_enabled_runs', 0),
        'multi_leg_curves': m['metrics'].get('curve_multi_leg_runs', 0),
        'source_motion_s': m['metrics'].get('source_motion_elapsed_s'),
    } for m in report['matches']}, ensure_ascii=False))
    print(f"Full match viewer: {output/'simulator.html'}; inline bytes={len(fragment.encode('utf-8'))}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=HERE/'output')
    parser.add_argument('--inline', type=Path)
    parser.add_argument('--render-report', type=Path,
                        help='Re-render an existing report without re-running the simulation')
    args = parser.parse_args()
    report = (json.loads(args.render_report.read_text(encoding='utf-8'))
              if args.render_report else build_match_report())
    write_match_outputs(report, args.output.resolve(), args.inline)


if __name__ == '__main__':
    main()
