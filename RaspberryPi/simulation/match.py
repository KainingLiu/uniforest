"""Continuous whole-plan replay with explicit, hypothetical perception/mechanisms.

One virtual plant carries its pose through every source ActionSpec. Source route
orchestration is replayed, but visual results, contact goals and mechanism timing
are nominal scenario inputs. This is not a full production execution validation.
"""
from __future__ import annotations

from collections import Counter
from copy import deepcopy
import json
import math
from pathlib import Path

from control.chassis import COUNTS_PER_CM, LATERAL_DISTANCE_SCALE
from control.trajectory import CubicRoute, Waypoint
from Strategy.common import wrap_angle
from Strategy.flows.routes import ROUTES
from Strategy.flows.curves import CURVE_ROUTES, run_curve
from Strategy.plans import PLAN_A, PLAN_B
from Strategy.settings import PROFILES
from .catalog import RouteHarness, _calibration
from .core import MecanumPlant, SimulationSettings, as_point


HERE = Path(__file__).resolve().parent
LABELS = {'navigate': '移动', 'anchor_wall': '靠墙定位', 'rebase_heading': '更新航向基准',
          'begin_collection': '开始采集', 'acquire_cube': '搜索并对准方块', 'grab_cube': '抓取',
          'inspect_cargo': '检查携带数量', 'align_tag': 'Tag 对准',
          'align_building': '建筑对准', 'unload': '卸货', 'load_staged': '装载暂存方块',
          'build': '搭建'}


def load_scenario(path=None):
    """Read nominal material locations and transfer assumptions, never hardware config."""
    return json.loads(Path(path or HERE/'match_scenarios.json').read_text(encoding='utf-8'))


class _MatchPlant(MecanumPlant):
    def __init__(self, settings):
        self.action_index, self.stage = -1, 'start'
        super().__init__(settings)

    def _record(self):
        super()._record()
        self.trace[-1].update(action=self.action_index, stage=self.stage)


class _MatchRoutes(RouteHarness):
    """Reuse ROUTES and its movement adapter with persistent match references."""
    def __init__(self, owner):
        self.owner = owner
        case = dict(profile='depart-a', start_assumptions=dict(
            heading_cw_deg=0, search_lateral_mm=0, reverse_already_done=False))
        super().__init__(case, owner.plant)
        self.data = owner.data
        self.robot.reset_field_localization_filter = lambda: None

    def control(self, profile):
        self.case['profile'], self.config = profile, PROFILES[profile]
        self._sync(Waypoint(0, 0, 0))
        return self

    def phase(self, control, name):
        self.owner.stage(name.lower())

    def _heading_error(self, target):
        # Production helper returns CCW error; source commands consume CW.
        return wrap_angle(self.owner.world_pose()['yaw_deg']-
                          (self.owner.heading_zero_world+target))

    def _measure_lateral_displacement_mm(self, origin):
        deltas = [round(c)-old for c, old in zip(self.plant.counts, origin)]
        tr, tl, bl, br = deltas
        return (tr+tl-bl-br)*10.0/(4*COUNTS_PER_CM*LATERAL_DISTANCE_SCALE)

    def _drive_until_wall(self, *, direction, **kwargs):
        self.owner.wall(direction)
        self._sync(Waypoint(0, 0, 0))

    def _recalibrate_heading_zero(self, reference_cw_deg=0):
        self.owner.heading_zero_world = self.owner.world_pose()['yaw_deg']-reference_cw_deg
        self.owner.milestone('heading_reference', reference_cw_deg=reference_cw_deg,
                             evidence='virtual IMU reference only; physical pose unchanged')

    def _align_delivery_tag_or_continue(self, **kwargs):
        self.owner.align_tag(purpose='purple')
        return True


class _Match:
    def __init__(self, plan, settings, scenario, route_mode):
        self.plan, self.settings, self.scenario = plan, settings, deepcopy(scenario)
        self.field = json.loads((HERE/'field_model.json').read_text(encoding='utf-8'))
        self.robot_shape = json.loads((HERE/'robot_model.json').read_text(encoding='utf-8'))
        self.start_pose = dict(self.scenario.get('start_pose', self.field['anchors']['start_blue']))
        self.plant = _MatchPlant(settings)
        self.heading_zero_world = self.start_pose['yaw_deg']
        self.front = self.robot_shape['front_extent_mm']+25.0
        self.side = self.robot_shape['width_mm']/2+25.0
        self.initial = deepcopy(self.scenario['resources'])
        self.lookup = {cube['id']: cube for cube in self.initial}
        if len(self.lookup) != len(self.initial):
            raise ValueError('scenario cube identifiers must be unique')
        self.remaining, self.cargo, self.lost = list(self.lookup), [], []
        self.placements = [dict(p, cubes=[]) for p in self.scenario['placements']]
        self.buildings = []
        self.build_count = 0
        self.frames, self.events, self.milestones = [], [], []
        self.required_points = {0}
        self.data = {}
        self.adapter = _MatchRoutes(self)
        self.current = None
        self.current_index = -1
        self.route_mode, self.curve_runs = route_mode, []
        self.nominal_correction_elapsed_s = self.nominal_wait_elapsed_s = 0.0
        self.event_assumptions = []
        self.warnings = [
            'Whole-plan nominal replay: visual observations, mechanism success and global anchor mapping are assumed.',
            'Original route legs use the production trajectory follower with simulated dynamics; this is not legacy PID timing.',
            'Nominal contact/vision corrections physically move the virtual robot and may cross obstacles; no collision response or avoidance is modeled.',
            'Mechanism stages run sequentially in this replay; no measured competition speedup or full transition-executor validation is claimed.',
        ]
        self.snapshot()

    def world_pose(self):
        angle = math.radians(self.start_pose['yaw_deg'])
        return dict(x_mm=self.start_pose['x_mm']+math.cos(angle)*self.plant.x-math.sin(angle)*self.plant.y,
                    y_mm=self.start_pose['y_mm']+math.sin(angle)*self.plant.x+math.cos(angle)*self.plant.y,
                    yaw_deg=self.start_pose['yaw_deg']+self.plant.yaw)

    def snapshot(self):
        frame = dict(t=round(self.plant.now, 6), cargo=list(self.cargo),
                     remaining=list(self.remaining), lost=list(self.lost),
                     placements=deepcopy(self.placements), buildings=deepcopy(self.buildings),
                     build_count=self.build_count)
        all_ids = frame['cargo']+frame['remaining']+frame['lost']
        all_ids += [c for p in frame['placements'] for c in p['cubes']]
        all_ids += [c for b in frame['buildings'] for c in b['layers']]
        if Counter(all_ids) != Counter(self.lookup.keys()) or len(self.cargo) > 3:
            raise RuntimeError('nominal material ownership or carrying-capacity invariant failed')
        self.frames.append(frame)

    def stage(self, name):
        if name != self.plant.stage:
            self.required_points.add(len(self.plant.trace)-1)
            self.plant.stage = name
            self.plant._record()  # Same t/pose, new display metadata only.
            self.required_points.add(len(self.plant.trace)-1)

    def milestone(self, stage, **details):
        self.milestones.append(dict(t=round(self.plant.now, 6), action=self.current_index,
                                    stage=stage, simulation_only=True, **details))
        self.required_points.add(len(self.plant.trace)-1)

    def assume(self, description):
        if description not in self.event_assumptions:
            self.event_assumptions.append(description)

    def wait(self, duration, stage):
        if not math.isfinite(duration) or duration < 0:
            raise ValueError('nominal stage duration must be finite and nonnegative')
        self.stage(stage)
        self.milestone(stage, estimated_duration_s=duration, evidence='scenario assumption, not measured')
        remaining, started = duration, self.plant.now
        try:
            while remaining > 1e-8:
                self.plant.guard()
                dt = min(self.settings.control_period_s, remaining)
                self.plant.sleep(dt)
                remaining -= dt
        finally:
            self.nominal_wait_elapsed_s += self.plant.now-started

    def duration(self, name):
        return float(self.scenario['durations_s'][name])

    def move_world(self, x, y, yaw, reason):
        """Drive to an assumed global target; never reassign plant pose."""
        pose = self.world_pose()
        angle = math.radians(pose['yaw_deg'])
        dx, dy = x-pose['x_mm'], y-pose['y_mm']
        local = Waypoint(math.cos(angle)*dx+math.sin(angle)*dy,
                         -math.sin(angle)*dx+math.cos(angle)*dy,
                         -wrap_angle(pose['yaw_deg']-yaw))
        distance = math.hypot(dx, dy)
        self.assume(f'{reason}: global target is a scenario assumption, reached by continuous simulated motion')
        self.milestone('assumed_anchor_correction', reason=reason,
                       target=dict(x_mm=x, y_mm=y, yaw_deg=yaw), distance_mm=round(distance, 3))
        if distance > .01 or abs(local.yaw_deg) > .01:
            self.stage(reason)
            started = self.plant.now
            try:
                self.plant.follow_local((Waypoint(0, 0, 0), local))
                self.adapter._sync(Waypoint(0, 0, 0))
            finally:
                self.nominal_correction_elapsed_s += self.plant.now-started

    def wall(self, direction):
        """Nominal ray-to-CAD contact target with footprint clearance, no contact forces."""
        self.assume('wall detection is a nominal CAD ray/contact target; no torque or collision-force simulation')
        pose = self.world_pose()
        relative = {'forward': 0, 'backward': 180, 'left': -90, 'right': 90}[direction]
        angle = math.radians(pose['yaw_deg']+relative)
        ux, uy = math.cos(angle), math.sin(angle)
        intersections = []
        for shape in self.field['shapes']:
            if not shape.get('collision', False):
                continue
            poly = shape['polygon']
            for a, b in zip(poly, poly[1:]+poly[:1]):
                ex, ey = b[0]-a[0], b[1]-a[1]
                determinant = ux*ey-uy*ex
                if abs(determinant) < 1e-9:
                    continue
                rx, ry = a[0]-pose['x_mm'], a[1]-pose['y_mm']
                travel = (rx*ey-ry*ex)/determinant
                fraction = (rx*uy-ry*ux)/determinant
                if travel > 0 and 0 <= fraction <= 1:
                    intersections.append((travel, shape['id']))
        if not intersections:
            self.warnings.append(f'{self.current.name}: nominal {direction} wall ray found no obstacle; no pose correction')
            self.wait(.2, 'wall_unknown')
            return
        distance, shape_id = min(intersections)
        reach = self.front if direction == 'forward' else (
            self.robot_shape['rear_extent_mm']+25 if direction == 'backward' else self.side)
        correction = distance-reach
        self.move_world(pose['x_mm']+ux*correction, pose['y_mm']+uy*correction,
                        pose['yaw_deg'], f'nominal_wall_{direction}')
        self.milestone('wall_target_reached', shape_id=shape_id, direction=direction,
                       evidence='assumed contact; movement simulated, contact forces omitted')
        self.wait(.15, 'wall_confirm')

    def placement(self, identity):
        return next(p for p in self.placements if p['id'] == identity)

    def transfer(self, spec):
        return self.scenario['transfers'][self.plan.name][spec.name]

    def at_placement(self, placement, reason):
        self.move_world(placement['x'], 6200.0-self.front, 90.0, reason)

    def run_route(self, route, profile):
        self.adapter.control(profile)
        record = dict(route=route, profile=profile, action=self.current_index,
                      name=self.current.name, route_mode=self.route_mode,
                      t0=round(self.plant.now, 6), start_pose=self.world_pose(),
                      curve_enabled=False, multi_leg=False, source_motion_legs=[],
                      waypoints=[], internal_nodes=[], prefix_controller_runs=0,
                      controller_runs_before=self.plant.controller_runs,
                      status='running', simulation_only=True)
        self.curve_runs.append(record)
        try:
            prefix, source_legs = self.route_prefix(route, profile)
            record['source_motion_legs'] = source_legs
            enabled = self.route_mode == 'continuous' and len(prefix) >= 2
            if enabled:
                self.assume('continuous candidate is generated from live source-route corners; no field calibration or collision avoidance')
                self.stage(f'curve_{route}')
                initial = self.plant.measured_velocity()
                reference = CubicRoute(prefix, self.plant.profile, initial)
                started = self.plant.now
                trace_start = len(self.plant.trace)-1
                record.update(curve_enabled=True, multi_leg=len(source_legs)>1,
                              waypoints=[as_point(p) for p in prefix])
                # This is the production route adapter and production follower,
                # with actual virtual wheel output. run_curve retains wall,
                # heading-rebase and post-wall legs from its route contract.
                result = run_curve(self.adapter, route, profile,
                                   _calibration(prefix, self.settings))
                record['prefix_controller_runs'] = 1
                record['prefix_elapsed_s'] = round(result.elapsed_s, 6)
                record['prefix_end_s'] = round(started+result.elapsed_s, 6)
                prefix_trace = [p for p in self.plant.trace[trace_start:]
                                if p['t'] <= started+result.elapsed_s+1e-8]
                for index, knot in enumerate(reference.knots[1:-1], 1):
                    sample = min(prefix_trace, key=lambda p: abs(p['t']-(started+knot)))
                    self.required_points.add(self.plant.trace.index(sample, trace_start))
                    speed = math.hypot(sample['vx'], sample['vy'])
                    record['internal_nodes'].append(dict(
                        index=index, t=round(sample['t'], 6),
                        reference=as_point(prefix[index]),
                        measured_speed_mm_s=round(speed, 3),
                        measured_yaw_speed_deg_s=round(sample['wz'], 3),
                        moving=(speed > self.plant.profile.settle_speed_mm_s or
                                abs(sample['wz']) > self.plant.profile.settle_yaw_speed_deg_s)))
            else:
                record['fallback_reason'] = ('baseline_requested' if self.route_mode == 'stop_turn'
                                              else 'no_curve_contract_or_already_completed')
                ROUTES[route](self.adapter, profile)
            record['status'] = 'completed'
        except Exception as exc:
            record.update(status='failed', error=f'{type(exc).__name__}: {exc}')
            raise
        finally:
            record.update(t1=round(self.plant.now, 6), end_pose=self.world_pose(),
                          controller_runs_after=self.plant.controller_runs,
                          controller_runs=self.plant.controller_runs-record['controller_runs_before'])
            if record['curve_enabled']:
                record['prefix_controller_runs'] = min(1, record['controller_runs'])
            self.milestone('route_executed', route=route, profile=profile,
                           curve_enabled=record['curve_enabled'], status=record['status'],
                           controller_runs=record['controller_runs'])

    def route_prefix(self, route, profile):
        """Capture live source geometry on a separate non-moving recorder.

        Only this geometry recorder starts at local zero. The match plant and
        its world pose are never reassigned. Encoder origins, frozen search
        corrections and early-reverse flags are copied from the current match.
        """
        if route not in CURVE_ROUTES:
            return (), []
        case = dict(profile=profile, start_assumptions=dict(
            heading_cw_deg=self.world_pose()['yaw_deg']-self.heading_zero_world,
            search_lateral_mm=0.0, reverse_already_done=False))
        recorder = RouteHarness(case)
        recorder.data = deepcopy(self.data)
        recorder._measure_lateral_displacement_mm = self.adapter._measure_lateral_displacement_mm
        ROUTES[route](recorder, profile)
        prefix, legs = [Waypoint(0, 0, 0)], []
        for event in recorder.events:
            if event['kind'] == 'wall':
                break
            if event['kind'] in ('move', 'turn'):
                end = event['end']
                point = Waypoint(end['x_mm'], end['y_mm'], end['yaw_deg'])
                if point != prefix[-1]:
                    prefix.append(point)
                    legs.append(event)
        return tuple(prefix), legs

    def selected(self, spec):
        if not spec.parameters.get('conditional_on_purple'):
            return True
        cfg = PROFILES[spec.profile]
        maximum = (cfg.orange_target_count if self.data.get('purple_grabbed')
                   else cfg.orange_target_count_without_purple)
        return spec.parameters.get('index', 1) <= maximum

    def begin_collection(self, spec):
        color = spec.parameters['color']
        highland = spec.profile.startswith('highland-')
        region = 'purple' if color == 'purple' else 'highland' if highland else 'ground'
        origin = tuple(round(c) for c in self.plant.counts)
        self.data['collection'] = dict(color=color, region=region, origin=origin, acquired=None)
        key = 'purple' if color == 'purple' else 'orange' if highland else 'ground'
        self.data.update({f'{key}_origin': origin, f'{key}_lateral_mm': None,
                          f'{key}_reverse_done': False})
        if color == 'purple':
            self.data.update(purple_grabbed=False, purple_route_done=False)
        self.milestone('collection_started', color=color, region=region)
        return 'completed'

    def acquire(self, spec):
        collection = self.data['collection']
        collection['acquired'] = None
        if not self.selected(spec):
            return 'skipped_conditional'
        self.assume('nominal perfect cube observation and alignment; production camera/tracker is not executed')
        self.wait(self.duration('search'), 'search_cube')
        candidates = [self.lookup[i] for i in self.remaining
                      if self.lookup[i]['color'] == collection['color']
                      and self.lookup[i]['region'] == collection['region']]
        if not candidates:
            return 'skipped_no_resource'
        # An explicit nominal observation chooses a remaining physical cube.
        pose = self.world_pose()
        cube = min(candidates, key=lambda c: math.hypot(c['x']-pose['x_mm'], c['y']-pose['y_mm']))
        if collection['region'] == 'purple':
            target = (2600+self.front, cube['y'], 180)
        else:
            face_y = 5000 if collection['region'] == 'ground' else 2500
            target = (cube['x'], face_y+self.front, 270)
        self.move_world(*target, reason='nominal_cube_alignment')
        self.wait(self.duration('align_cube'), 'cube_aligned')
        collection['acquired'] = cube['id']
        self.milestone('cube_observed', cube_id=cube['id'], evidence='nominal scenario observation')
        return 'completed'

    def grab(self, spec):
        if not self.selected(spec):
            return 'skipped_conditional'
        collection = self.data['collection']
        cube = collection.get('acquired')
        if cube is None or cube not in self.remaining:
            return 'skipped_no_acquired_cube'
        if len(self.cargo) >= 3:
            return 'skipped_capacity'
        self.assume('successful grasp and arm return are nominal timed milestones with no camera or suction feedback')
        method = spec.parameters['method']
        clearance = self.scenario['grab_clearance_s'][method]
        self.wait(clearance, f'{method}_lift')
        self.remaining.remove(cube)
        self.cargo.append(cube)
        self.snapshot()
        self.milestone('cube_fully_lifted', cube_id=cube, method=method)
        self.wait(self.duration(method)-clearance, f'{method}_restore')
        collection['acquired'] = None
        if collection['color'] == 'purple':
            self.data['purple_grabbed'] = True
        else:
            self.adapter._recalibrate_heading_zero()
        self.milestone('arm_home', evidence='nominal timed mechanism completion')
        # The subsequent source navigate performs purple transit once. No
        # artificial duplicate route is run inside this sequential replay.
        return 'completed'

    def align_tag(self, purpose):
        self.assume('Tag observation is a nominal anchor correction; no production detector output is claimed')
        target = ((2600+self.front, 3600, 180) if purpose == 'purple'
                  else (3500, 6200-self.front, 90))
        self.move_world(*target, reason='nominal_tag_alignment')
        self.wait(self.duration('align_tag'), 'tag_aligned')

    def unload(self, spec):
        transfer = self.transfer(spec)
        placement = self.placement(transfer['placement'])
        self.assume('deposit position and material order are scenario assumptions; hatch timing is estimated')
        self.at_placement(placement, 'nominal_deposit_alignment')
        self.wait(self.duration('unload'), 'hatch_open')
        placement['cubes'].extend(self.cargo)
        deposited = list(self.cargo)
        self.cargo.clear()
        self.snapshot()
        self.milestone('material_deposited', placement=placement['id'], cubes=deposited)
        cfg = PROFILES[spec.profile]
        self.adapter._checked_move('backward', cfg.unload_reverse_mm, cfg.unload_reverse_speed_mm_s)
        return 'completed' if deposited else 'skipped_empty_cargo'

    def load(self, spec):
        transfer = self.transfer(spec)
        source = self.placement(transfer['placement'])
        self.assume('load uses existing staged cube identities; reload order and success are nominal assumptions')
        self.at_placement(source, 'nominal_load_alignment')
        self.wait(self.duration('load'), 'load_staged')
        count = min(transfer.get('count', 3), 3-len(self.cargo), len(source['cubes']))
        loaded = source['cubes'][:count]
        del source['cubes'][:count]
        self.cargo.extend(loaded)
        self.snapshot()
        self.milestone('material_loaded', placement=source['id'], cubes=loaded)
        return 'completed' if count == transfer.get('count', 3) else 'partial_load' if count else 'skipped_empty_stage'

    def build(self, spec):
        transfer = self.transfer(spec)
        base = self.placement(transfer['base'])
        if not self.cargo:
            return 'skipped_empty_cargo'
        self.assume('base position/order, top-first cargo releases and successful stacking are nominal scenario assumptions')
        self.at_placement(base, 'nominal_build_alignment')
        building = dict(id=transfer['target'], x=base['x'], y=base['y'],
                        layers=list(base['cubes']), assumed=True)
        base_count = len(base['cubes'])
        base['cubes'].clear()
        self.buildings.append(building)
        self.snapshot()
        last, released = 0.0, 0
        for index, at in enumerate(self.scenario['build_release_s'], 1):
            self.wait(at-last, f'build_release_{index}')
            if self.cargo:
                cube = self.cargo.pop()
                building['layers'].append(cube)
                released += 1
                self.snapshot()
                self.milestone('build_cube_released', building=building['id'], cube_id=cube,
                               layer=len(building['layers']))
            last = at
        self.wait(self.duration('build')-last, 'build_restore')
        self.build_count += 1
        self.snapshot()
        return 'completed' if base_count == 3 and released == 3 else 'partial_build'

    def execute(self, spec):
        self.adapter.control(spec.profile)
        kind = spec.kind
        if kind == 'navigate':
            self.run_route(spec.parameters['route'], spec.profile)
        elif kind == 'rebase_heading':
            self.adapter._recalibrate_heading_zero(spec.parameters['reference_cw_deg'])
        elif kind == 'anchor_wall':
            self.wall(spec.parameters.get('direction', 'forward'))
            if spec.parameters.get('recalibrate'):
                self.adapter._recalibrate_heading_zero(spec.parameters.get('reference_cw_deg', 0))
        elif kind == 'begin_collection':
            return self.begin_collection(spec)
        elif kind == 'acquire_cube':
            return self.acquire(spec)
        elif kind == 'grab_cube':
            return self.grab(spec)
        elif kind == 'inspect_cargo':
            self.assume('cargo count is the simulation material ledger; camera confirmation and timing are nominal')
            self.wait(self.duration('inspect'), 'inspect_cargo')
            if spec.parameters.get('exit_route'):
                self.run_route(spec.parameters['exit_route'], spec.profile)
        elif kind == 'align_tag':
            self.align_tag(spec.parameters.get('purpose', 'delivery'))
        elif kind == 'align_building':
            next_build = next(s for s in self.plan.steps[self.current_index+1:] if s.kind == 'build')
            base = self.placement(self.transfer(next_build)['base'])
            self.assume('building detection is a nominal target pose; no production building detector is replayed')
            self.at_placement(base, 'nominal_building_alignment')
            self.wait(self.duration('align_building'), 'building_aligned')
        elif kind == 'unload':
            return self.unload(spec)
        elif kind == 'load_staged':
            return self.load(spec)
        elif kind == 'build':
            return self.build(spec)
        else:
            raise ValueError(f'whole-match simulator has no handler for {kind}')
        return 'completed'

    def run(self, max_points):
        failed = None
        for index, spec in enumerate(self.plan.steps):
            self.current, self.current_index = spec, index
            self.plant.action_index = index
            self.event_assumptions = []
            self.stage('action_start')
            t0, start = self.plant.now, self.world_pose()
            before = len(self.frames)-1
            try:
                status = self.execute(spec) if failed is None else 'skipped_after_failure'
            except Exception as exc:
                status, failed = 'failed', f'{type(exc).__name__}: {exc}'
                self.plant.emergency_stop()
                self.warnings.append(f'{spec.name}: {failed}; later source actions retained as skipped')
            self.required_points.add(len(self.plant.trace)-1)
            after = len(self.frames)-1
            self.events.append(dict(index=index, name=spec.name, kind=spec.kind, profile=spec.profile,
                                    label=LABELS[spec.kind], t0=round(t0, 6), t1=round(self.plant.now, 6),
                                    start_pose={k: round(v, 6) for k,v in start.items()},
                                    end_pose={k: round(v, 6) for k,v in self.world_pose().items()},
                                    status=status, cargo_before=list(self.frames[before]['cargo']),
                                    cargo_after=list(self.frames[after]['cargo']), state_before=before,
                                    state_after=after, assumptions=list(self.event_assumptions)))
        points = self.export_points(max_points)
        metrics = self.plant.metrics()
        metrics.update(source_action_count=len(self.plan.steps),
                       completed_actions=sum(e['status']=='completed' for e in self.events),
                       skipped_actions=sum(e['status'].startswith('skipped') for e in self.events),
                       build_count=self.build_count, initial_cube_count=len(self.initial),
                       built_cube_count=sum(len(b['layers']) for b in self.buildings),
                       nominal_corrections=sum(m['stage']=='assumed_anchor_correction' for m in self.milestones),
                       curve_enabled_runs=sum(r['curve_enabled'] for r in self.curve_runs),
                       curve_multi_leg_runs=sum(r['curve_enabled'] and r['multi_leg'] for r in self.curve_runs),
                       curve_moving_internal_nodes=sum(n['moving'] for r in self.curve_runs for n in r['internal_nodes']),
                       nominal_correction_elapsed_s=round(self.nominal_correction_elapsed_s, 6),
                       nominal_wait_elapsed_s=round(self.nominal_wait_elapsed_s, 6),
                       source_motion_elapsed_s=round(self.plant.now-self.nominal_correction_elapsed_s-
                                                     self.nominal_wait_elapsed_s, 6))
        return dict(plan=self.plan.name, status='failed' if failed else 'nominal_completed',
                    route_mode=self.route_mode, curve_runs=self.curve_runs,
                    simulation_only=True, field_validated=False, start_pose=self.start_pose,
                    events=self.events, points=points, state_frames=self.frames,
                    initial_cubes=self.initial, milestones=self.milestones, metrics=metrics,
                    assumptions=self.scenario.get('assumptions', [])+[
                        'One virtual plant and one initial world placement persist through the whole source plan.',
                        'Motion follows source route legs; all added global corrections are labelled and physically integrated.',
                        'Material success, colour ordering and durations are nominal scenario assumptions.',
                        ('Registered route prefixes use real continuous control through live source corners; unregistered routes retain separate legs.'
                         if self.route_mode == 'continuous' else
                         'Stop-turn baseline executes source motion legs separately under the same simulated limits.'),
                    ], warnings=self.warnings, collision_checked=False,
                    coordinate_frame='world mm, +x right, +y down, unwrapped clockwise degrees')

    def export_points(self, maximum):
        if type(maximum) is not int or maximum < 2:
            raise ValueError('max_points must be at least two')
        count = len(self.plant.trace)
        required = sorted(self.required_points | {count-1})
        limit = max(maximum, len(required))
        selected = set(required)
        remaining = limit-len(selected)
        if remaining:
            candidates = [i for i in range(count) if i not in selected]
            n = min(remaining, len(candidates))
            if n:
                selected.update(candidates[round(i*(len(candidates)-1)/max(1,n-1))] for i in range(n))
        angle = math.radians(self.start_pose['yaw_deg'])
        points = []
        for index in sorted(selected):
            p = self.plant.trace[index]
            point = dict(p)
            point.update(x=self.start_pose['x_mm']+math.cos(angle)*p['x']-math.sin(angle)*p['y'],
                         y=self.start_pose['y_mm']+math.sin(angle)*p['x']+math.cos(angle)*p['y'],
                         yaw=self.start_pose['yaw_deg']+p['yaw'])
            points.append({k: round(v, 3) if isinstance(v, float) else v for k,v in point.items()})
        return points


def run_match(plan_name='PlanA', *, settings=None, scenario=None, max_points=1200,
              route_mode='stop_turn'):
    """Run every ActionSpec, keeping conditionally skipped steps and failure tails."""
    plans = {PLAN_A.name: PLAN_A, PLAN_B.name: PLAN_B}
    if plan_name not in plans:
        raise ValueError('whole-match simulation currently supports PlanA and PlanB')
    if route_mode not in ('stop_turn', 'continuous'):
        raise ValueError('route_mode must be stop_turn or continuous')
    return _Match(plans[plan_name], settings or SimulationSettings(),
                  load_scenario() if scenario is None else scenario, route_mode).run(max_points)
