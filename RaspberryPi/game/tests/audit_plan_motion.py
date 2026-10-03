"""Current local-route coverage with isolated, non-guided dispatch probes.

Outputs describe source coverage and isolated dispatch probes. They deliberately
do not label simulation, mocked execution, or deployment notes as field evidence.
"""
from collections import Counter
from dataclasses import asdict
from datetime import datetime,timezone,timedelta
import csv
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch

import main
from Strategy.cli import load_execution_config
from Strategy.plans import PLANS
from Strategy.flows.factory import ROUTE_PROFILES
from Strategy.flows.curves import CURVE_ROUTES
from Strategy.optimizations.local_routes import LOCAL_ROUTES
from Strategy.transition_switches import transition_sites
from simulation.catalog import _assumptions
from tests.test_local_routes import environment

ROOT=Path(__file__).resolve().parents[1]
FLAGS=['--trial-optimizations','--enable-transition','next-cube',
       '--enable-transition','last-departure','--enable-transition','purple-departure',
       '--enable-transition','inspect-departure','--enable-transition','build-return',
       '--enable-motion-planning','--enable-fast-alignment']

COMPONENTS={
    'navigate':('原路线局部平滑','单直线复用原控制器，连续移动转向按原路线合并', 'Strategy/optimizations/local_routes.py'),
    'anchor_wall':('专用贴墙控制','直接轮速与堵转反馈，不经过 MotionPlanning','Strategy/controllers.py:_drive_until_wall'),
    'acquire_cube':('专用搜索＋快速视觉对准','搜索/边缘恢复直接轮速；当前命令开启 fast_alignment；条件槽可能跳过','Strategy/flows/operations.py:acquire_cube'),
    'grab_cube':('机构＋专用顶墙＋可选衔接','顶墙直接轮速；下一块盲移/接管为独立控制；紫块后续路线经 run_route','Strategy/flows/transitions.py:start_grab'),
    'inspect_cargo':('机构检查＋可选路线＋补抓恢复','exit_route 经 run_route；补抓返回 _return_for_refill 直接经典前进/贴墙；可能循环','Strategy/flows/transitions.py:inspect'),
    'align_tag':('行进 Tag6 或原对准','相邻运输/Tag6/偏置匹配时融合；未接管则保留原对准','Strategy/flows/factory.py:_align_tag'),
    'align_building':('独立建筑视觉控制','直接轮速；没有走 navigation.tracking.PositionTracker','Strategy/building_alignment.py:_align_building'),
    'unload':('经典定距后退','_unload_cubes 内部直接 _checked_move，绕过 MotionPlanning','Strategy/controllers.py:_unload_cubes'),
    'load_staged':('机构＋专用贴墙','开舱、向前贴墙、关舱；贴墙直接控制','Strategy/flows/operations.py:load_staged'),
    'begin_collection':('无底盘位移','切检测配置、记录编码器原点','Strategy/flows/operations.py:begin_collection'),
    'rebase_heading':('无底盘位移','仅重设坐标零点；不等于实际旋转','Strategy/flows/factory.py:_ordinary'),
    'build':('机构动作','底盘返回位移在后续 navigate；可在 CHASSIS_READY 后重叠','Strategy/flows/factory.py:_start_build'),
}


def probe(profile,route,config):
    base=next(r for r in sorted(CURVE_ROUTES) if profile in ROUTE_PROFILES[r])
    a=_assumptions(route,profile)
    env,c,_,_=environment(base,profile,heading=a['heading_cw_deg'],lateral=a['search_lateral_mm'],reverse=a['reverse_already_done'])
    env.transition_config=config
    events=[]
    env.robot.diagnostics=SimpleNamespace(write=lambda event,**values:events.append(dict(event=event,**values)))
    c._align_delivery_tag_or_continue=Mock(return_value=True)
    c._turn_to_heading=Mock()
    # A field-planner/PID entry here would disprove the claimed local-only path.
    with (patch('Strategy.navigation.planner.FieldPlanner.__init__',side_effect=AssertionError('field planner entered')),
          patch('Strategy.navigation.tracking.PositionTracker.__init__',side_effect=AssertionError('field PID entered'))):
        env.run_route(route,profile)
    return dict(profile=profile,route=route,
                backend=next(e['backend'] for e in events if e['event']=='motion_backend_selected'),
                trajectory_calls=env.robot.chassis.follow_trajectory.call_count,
                classic_move_calls=c._checked_move.call_count,
                classic_turn_calls=c._turn_to_heading.call_count,
                wall_calls=c._drive_until_wall.call_count,
                tag_calls=c._align_delivery_tag_or_continue.call_count,
                geometry=next((e for e in events if e['event']=='planned_local_route'),None),
                evidence='isolated dispatch with fake hardware; not deployment or field execution')


def audit():
    rows=[]; plans={}; probes={}
    for name in ('PlanA','PlanB'):
        plan=PLANS[name]
        config=load_execution_config(main.parse_args(['--strategy',name,*FLAGS,'--show-plan']),plan)
        sites={}
        for kind,source in transition_sites(plan.steps):sites.setdefault(source.name,[]).append(kind)
        refs=[]
        for index,step in enumerate(plan.steps,1):
            category,detail,source=COMPONENTS[step.kind]
            references=[]
            for key in ('route','exit_route','followup_route'):
                if key in step.parameters:
                    route=step.parameters[key]; ref=f'{step.profile}/{route}'
                    references.append(f'{key}={ref}'); refs.append(ref)
                    if ref not in probes:probes[ref]=probe(step.profile,route,config)
            rows.append(dict(plan=name,sequence=index,action=step.name,kind=step.kind,profile=step.profile,
                movement_controller=category,route_references='; '.join(references),
                transition_candidates='; '.join(sites.get(step.name,[])),
                details=detail,source=source,field_planner_active=False,
                field_position_pid_active=False,remote_verified=False))
        plans[name]=dict(declared_actions=len(plan.steps),kinds=dict(Counter(s.kind for s in plan.steps)),
                        route_reference_slots=len(refs),distinct_route_profile_keys=len(set(refs)),
                        motion_enabled=config.motion_planning_enabled,navigation_config_present=config.navigation is not None,
                        explicit_calibrated_curves=list(config.curves),flags=['--strategy',name,*FLAGS])
        env,_,_,_=environment('depart_a','depart-a');env.transition_config=config
        compiled=env.compile(plan)
        plans[name]['action_boundaries']=len(compiled.edges)
        plans[name]['boundaries_without_transition_candidate']=sum(not edge for edge in compiled.edges)
    sources=[p for p in (ROOT/'Strategy').rglob('*.py')]+[ROOT/'main.py',ROOT/'control/chassis.py',ROOT/'control/trajectory.py']
    hashes={str(p.relative_to(ROOT)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(sources)}
    result=dict(generated_at=datetime.now(timezone(timedelta(hours=8))).isoformat(),
        scope='current local source plus isolated entry probes; no robot commands, no deployment',
        remote_check='No deployment or hardware checks performed by this audit.',
        plans=plans,route_probes=probes,actions=rows,source_sha256=hashes,
        findings=[
            dict(id='F01',finding='用户命令仅启用 LocalRoutes，未选择全场 FieldPlanner',source='Strategy/cli.py; Strategy/optimizations/motion_planning.py'),
            dict(id='F02',finding='LocalRoutes 使用 control.trajectory 的比例反馈，新增 PositionTracker 的 I/D 未接入该链路',source='control/trajectory.py:follow_trajectory'),
            dict(id='F03',finding='相邻运输/Tag6/偏置可融合；本工具单路线探测未配置计划，移动视觉另由 test_moving_tag_approach 验证',source='Strategy/optimizations/local_routes.py; tests/test_moving_tag_approach.py'),
            dict(id='F04',finding='PlanB depart_b 的 180 度终点朝向保留，未联合后续 to_purple 消除中间朝向要求',source='Strategy/settings.py:Departure2Config; Strategy/plans/plan_b.py'),
            dict(id='F05',finding='局部轨迹终点要求零速；无匹配衔接的动作边界再次 stop，尚无整局连续轨迹编译',source='control/trajectory.py; Strategy/execution/runtime.py'),
            dict(id='F06',finding='卸货内部后退、补抓返回直接经典控制；搜索/贴墙/视觉/盲移使用各自控制器',source='Strategy/controllers.py:_unload_cubes; Strategy/flows/transitions.py:_return_for_refill'),
            dict(id='F07',finding='LocalRoutes 速度从旧命令生成，未使用 CompetitionMotion 的 1.10 倍巡航',source='Strategy/optimizations/local_routes.py:route_profile'),
            dict(id='F08',finding='单动作、净空退离及接触/视觉边界保持原控制语义；比赛无全场配置入口',source='Strategy/optimizations/local_routes.py:LocalRoutes.prepare'),
            dict(id='F09',finding='开启优化时最终建筑视觉对准必须成功，异常由执行器停止且不进入Build；Tag6完成独立于建筑确认',source='Strategy/flows/factory.py:_ordinary; tests/test_local_planning_entry.py'),
        ],
        limitations=['declarative slots are not actual movement counts',
                     'conditional third orange slot, already-completed callback routes, retries and stops change runtime execution',
                     'non-route wall/vision/search controls intentionally own their feedback loops',
                     'bypass inside unload and refill recovery is separate from named-route coverage'])
    output=ROOT/'simulation/output';output.mkdir(exist_ok=True)
    (output/'plan-motion-audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    with (output/'plan-motion-audit.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    print(json.dumps(dict(plans=plans,probe_count=len(probes),backends=dict(Counter(p['backend'] for p in probes.values()))),ensure_ascii=False,indent=2))
    return result


if __name__=='__main__':audit()
