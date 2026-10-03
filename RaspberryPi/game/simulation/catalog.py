"""Enumerate and run every registered route/profile pair in an isolated plant.

The stop-turn reference executes the production ROUTES orchestration against a
recording/simulated motion adapter. Each original geometric leg uses the same
trajectory follower and dynamics as the candidates; legacy PID timing is not
replayed. Wall contact travel/time is unknown and omitted in every mode.
"""
from dataclasses import asdict
import math
from types import SimpleNamespace

from control.trajectory import BodyVelocity, Waypoint
from Strategy.common import wrap_angle
from Strategy.flows.curves import CURVE_ROUTES, run_curve
from Strategy.flows.factory import ROUTE_PROFILES
from Strategy.flows.routes import ROUTES
from Strategy.plans import PLAN_A, PLAN_B
from Strategy.settings import PROFILES
from Strategy.transition_config import CurveCalibration
from .core import MecanumPlant, SimulationSettings, as_point


TITLES = {
    'depart_a': 'PlanA 出发', 'depart_b': 'PlanB 出发',
    'return_orange': '返回橙块区', 'ground_to_delivery': '地面橙块区到投放区',
    'purple_to_orange': '紫块区到橙块区', 'orange_to_build': '高地橙块区到搭建区',
    'build_return': '搭建后返程', 'staged_initial': '接近暂存材料',
    'staged_to_build': '暂存装载后到搭建区', 'staged_return_first': '首次暂存搭建后返程',
    'staged_return_final': '最后一次暂存搭建后离场',
    'ground_tag_offset': '投放 Tag 对准后横移', 'ground_delivery_depart': '地面卸货后离场',
    'build_offset': '搭建 Tag 对准后横移', 'unload_depart': '暂存卸货后离场',
}
MODE_LABELS = {
    'stop_turn': '原路线逐段停车（几何停走估计）',
    'rounded_corners': '经过原拐点的连续曲线候选',
    'endpoint_shortcut': '仅连接终点的捷径候选',
}


def _waypoint(data):
    return Waypoint(data['x_mm'], data['y_mm'], data['yaw_deg'])


def _calibration(points, settings):
    def entry(point):
        return dict(x_mm=point.x_mm, y_mm=point.y_mm, yaw_deg=point.yaw_deg,
                    dx_scale=0.0, dy_scale=0.0, dyaw_scale=0.0)
    values = [entry(p) for p in points]
    values[-1] = dict(x_mm=0.0, y_mm=0.0, yaw_deg=0.0,
                      dx_scale=1.0, dy_scale=1.0, dyaw_scale=1.0)
    return CurveCalibration(tuple(values), settings.controller_profile())


class RouteHarness:
    """Minimal production-route API, with no hardware construction whatsoever."""
    def __init__(self, case, plant=None):
        self.case, self.plant = case, plant
        self.config = PROFILES[case['profile']]
        self.pose = Waypoint(0.0, 0.0, 0.0)
        self.events = []
        self.curve_points = ()
        self.context = SimpleNamespace(check_active=self._check_active)
        assumptions = case['start_assumptions']
        self.heading = assumptions['heading_cw_deg']
        self.lateral = assumptions['search_lateral_mm']
        reverse = assumptions['reverse_already_done']
        self.data = dict(ground_origin='ground', purple_origin='purple', orange_origin='orange',
                         ground_lateral_mm=self.lateral if reverse else None,
                         orange_lateral_mm=self.lateral if reverse else None,
                         ground_reverse_done=reverse, orange_reverse_done=reverse,
                         purple_route_done=False)
        self.robot = SimpleNamespace(
            telem=SimpleNamespace(yaw_deg=wrap_angle(-self.heading)),
            chassis=SimpleNamespace(turn=self.turn, follow_trajectory=self.follow_curve,
                                    measured_body_velocity=self.measured_velocity),
            move_chassis=self._checked_move)

    def control(self, profile):
        if profile != self.case['profile']:
            raise ValueError('simulation harness profile mismatch')
        return self

    def phase(self, *args):
        self._check_active()

    def _check_active(self, **kwargs):
        if self.plant:
            self.plant.guard()

    def _measure_lateral_displacement_mm(self, origin):
        return self.lateral

    _wrap_angle = staticmethod(wrap_angle)

    def _heading_error(self, target):
        return wrap_angle(-target-self.robot.telem.yaw_deg)

    def _sync(self, nominal):
        self.pose = (Waypoint(self.plant.ox, self.plant.oy, self.plant.oyaw)
                     if self.plant else nominal)
        self.robot.telem.yaw_deg = wrap_angle(-self.heading-self.pose.yaw_deg)

    def _checked_move(self, direction, distance_mm, speed_mm_s, **kwargs):
        self._check_active()
        axes = {'forward': (1, 0), 'backward': (-1, 0), 'right': (0, 1), 'left': (0, -1)}
        dx, dy = [v*distance_mm for v in axes[direction]]
        angle = math.radians(self.pose.yaw_deg)
        target = Waypoint(self.pose.x_mm+math.cos(angle)*dx-math.sin(angle)*dy,
                          self.pose.y_mm+math.sin(angle)*dx+math.cos(angle)*dy,
                          self.pose.yaw_deg)
        start = as_point(self.pose)
        if self.plant and distance_mm:
            self.plant.follow_local((Waypoint(0, 0, 0), Waypoint(dx, dy, 0)))
        self._sync(target)
        self.events.append(dict(kind='move', direction=direction, distance_mm=distance_mm,
                                configured_speed_mm_s=speed_mm_s, start=start, end=as_point(self.pose)))
        return SimpleNamespace(timed_out=False, cancelled=False)

    def turn(self, target_deg, speed_deg_s, **kwargs):
        self._check_active()
        target = Waypoint(self.pose.x_mm, self.pose.y_mm, self.pose.yaw_deg+target_deg)
        start = as_point(self.pose)
        if self.plant and target_deg:
            self.plant.follow_local((Waypoint(0, 0, 0), Waypoint(0, 0, target_deg)))
        self._sync(target)
        self.events.append(dict(kind='turn', cw_deg=target_deg,
                                configured_speed_deg_s=speed_deg_s, start=start, end=as_point(self.pose)))

    def _turn_to_heading(self, target_cw_deg, **kwargs):
        self.turn(-self._heading_error(target_cw_deg), self.config.delivery_turn_speed_deg_s)

    def _drive_until_wall(self, *, direction, **kwargs):
        self._check_active()
        self.events.append(dict(kind='wall', direction=direction, at=as_point(self.pose),
                                omitted_contact_travel_and_time=True))
        if self.plant:
            self.plant.barrier('wall', direction=direction,
                               assumption='contact travel and dwell omitted')

    def _recalibrate_heading_zero(self, *args):
        self.events.append(dict(kind='heading_rebase', at=as_point(self.pose)))
        if self.plant:
            self.plant.barrier('heading_rebase', assumption='ideal IMU; no physical correction')

    def measured_velocity(self):
        return self.plant.measured_velocity() if self.plant else BodyVelocity()

    def follow_curve(self, points, profile, *, check, initial_velocity):
        check()
        self.curve_points = tuple(points)
        result = (self.plant.follow_local(points, profile=profile, initial_velocity=initial_velocity)
                  if self.plant else None)
        self._sync(points[-1])
        self.events.append(dict(kind='curve', end=as_point(self.pose)))
        return result

    def record_transition(self, *args):
        self._check_active()


def _assumptions(route, profile, overrides=None):
    cfg = PROFILES[profile]
    if route in ('depart_a', 'depart_b', 'ground_to_delivery', 'orange_to_build'):
        heading = 0.0
    elif route == 'purple_to_orange':
        heading = cfg.delivery_heading_target_cw_deg
    else:
        heading = 180.0
    lateral = -100.0 if route == 'purple_to_orange' else 300.0 if route in (
        'ground_to_delivery', 'orange_to_build') else 0.0
    result = dict(heading_cw_deg=heading, search_lateral_mm=lateral,
                  reverse_already_done=route in ('ground_to_delivery', 'orange_to_build'),
                  initial_velocity_mm_s=0.0,
                  source='nominal scenario assumptions, not measurements or field calibration',
                  wall_contact='unknown contact distance and dwell omitted in all modes')
    if overrides:
        result.update(overrides)
    return result


def _describe(route, profile, *, assumptions=None):
    uses = {plan.name: [s.name for s in plan.steps if s.kind == 'navigate'
                       and s.profile == profile and s.parameters.get('route') == route]
            for plan in (PLAN_A, PLAN_B)}
    case = dict(id=f'{profile}/{route}', route=route, profile=profile,
                title=f'{TITLES[route]} · {profile}', simulation_only=True,
                plans=[name for name, steps in uses.items() if steps], plan_steps=uses,
                start_assumptions=_assumptions(route, profile, assumptions))
    legacy = RouteHarness(case)
    ROUTES[route](legacy, profile)
    prefix = [Waypoint(0, 0, 0)]
    for event in legacy.events:
        if event['kind'] == 'wall':
            break
        if event['kind'] in ('move', 'turn'):
            p = _waypoint(event['end'])
            if p != prefix[-1]:
                prefix.append(p)
    # Obtain the candidate endpoint and mandatory tails from actual run_curve.
    curve = RouteHarness(case)
    run_curve(curve, route, profile,
              _calibration((Waypoint(0, 0, 0), prefix[-1]), SimulationSettings()))
    endpoint = curve.curve_points[-1]
    mismatch = max(abs(a-b) for a, b in zip(asdict(endpoint).values(), asdict(prefix[-1]).values()))
    if mismatch > 1e-5:
        raise AssertionError(f'{case["id"]}: curve and legacy prefix destinations differ')
    prefix[-1] = endpoint
    case.update(original_steps=legacy.events, prefix_waypoints=[as_point(p) for p in prefix],
                nominal_endpoint=as_point(endpoint), nominal_final_pose=as_point(legacy.pose),
                preserved_tail=[e for e in curve.events if e['kind'] != 'curve'],
                comparison_basis='same simulated follower/dynamics, original geometric stop-turn legs; legacy PID timing not replayed')
    return case


def build_catalog(*, assumption_overrides=None):
    """All legal CURVE_ROUTES/ROUTE_PROFILES pairs, including plan occurrence data."""
    overrides = assumption_overrides or {}
    return [_describe(route, profile, assumptions=overrides.get(f'{profile}/{route}'))
            for route in sorted(CURVE_ROUTES) for profile in sorted(ROUTE_PROFILES[route])]


def _run_layer(case, mode, settings, max_points):
    plant = MecanumPlant(settings)
    harness = RouteHarness(case, plant)
    status, error = 'completed', None
    points = tuple(_waypoint(p) for p in case['prefix_waypoints'])
    if mode == 'endpoint_shortcut':
        points = (points[0], points[-1])
    try:
        if mode == 'stop_turn':
            ROUTES[case['route']](harness, case['profile'])
        else:
            run_curve(harness, case['route'], case['profile'], _calibration(points, settings))
    except Exception as exc:
        status, error = 'failed', f'{type(exc).__name__}: {exc}'
        if not plant.stopped:
            plant.emergency_stop()
    metrics = plant.metrics()
    goal = case['nominal_final_pose']
    metrics['final_position_error_mm'] = round(math.hypot(plant.x-goal['x_mm'], plant.y-goal['y_mm']), 3)
    metrics['final_yaw_error_deg'] = round(abs(plant.yaw-goal['yaw_deg']), 3)
    return dict(id=mode, label=MODE_LABELS[mode], simulation_only=True, status=status,
                error=error, metrics=metrics, points=plant.export_trace(max_points),
                waypoints=[as_point(p) for p in points], barriers=plant.barriers,
                reference_runs=plant.references,
                collision_checked=False, field_validated=False)


def run_case(case, *, settings=None, max_points=100):
    """Run all three modes independently. Failures retain their partial trace."""
    settings = settings or SimulationSettings()
    return dict(case, layers=[_run_layer(case, mode, settings, max_points) for mode in MODE_LABELS])


def run_catalog(*, settings=None, max_points=100, assumption_overrides=None):
    settings = settings or SimulationSettings()
    cases = [run_case(case, settings=settings, max_points=max_points)
             for case in build_catalog(assumption_overrides=assumption_overrides)]
    return dict(simulation_only=True, field_validated=False, coordinate_frame='local x forward, y right, unwrapped CW degrees',
                settings=asdict(settings), curve_family_count=len(CURVE_ROUTES),
                case_count=len(cases), layer_count=sum(len(c['layers']) for c in cases),
                completed=sum(l['status'] == 'completed' for c in cases for l in c['layers']),
                cases=cases,
                limitations=[
                    'No Robot/Transport or real hardware; no model-based field safety claim.',
                    'Stop-turn reference reproduces source route geometry with common follower limits, not legacy controller timing.',
                    'Wall contact travel/dwell and any resulting anchor displacement are unknown and omitted.',
                    'No obstacles, contact dynamics, payload, pitch, static friction, or perception latency in this local model.',
                    'Configured profiles inside the plant are synthetic; output must never be used as a hardware calibration file.',
                ])
