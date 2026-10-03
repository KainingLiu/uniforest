"""Competition recipes -> measured field pose -> planner -> real wheel adapter.

The geometry files are measured inputs, never inferred from simulation defaults.
Loading an unverified document is allowed for previews; live execution rejects it.
"""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

from .planner import FieldPlanner, Pose, NavigationError, wrap
from .robot_adapter import NavigationCalibration, RobotNavigation


@dataclass(frozen=True)
class CompetitionNavigationConfig:
    field: dict
    robot: dict
    calibration: NavigationCalibration

    @classmethod
    def load(cls, path):
        path = Path(path)
        doc = json.loads(path.read_text(encoding='utf-8'))
        keys = {'version', 'verified', 'record', 'capture_delay_s', 'field_file', 'robot_file'}
        if not isinstance(doc, dict) or set(doc) != keys or doc['version'] != 1:
            raise ValueError('navigation config requires version 1 and measured geometry/calibration fields')
        if type(doc['verified']) is not bool or not isinstance(doc['record'], str):
            raise ValueError('navigation verified must be boolean and record must be text')
        delay = doc['capture_delay_s']
        if isinstance(delay, bool) or not isinstance(delay, (int, float)) or not math.isfinite(delay) or not 0 <= delay <= .2:
            raise ValueError('navigation capture_delay_s must be finite and within 0..0.2')
        geometry = []
        for key in ('field_file', 'robot_file'):
            if not isinstance(doc[key], str) or not doc[key].strip():
                raise ValueError(f'navigation {key} must name a geometry JSON file')
            geometry.append(json.loads((path.parent / doc[key]).read_text(encoding='utf-8')))
        result = cls(*geometry, NavigationCalibration(doc['verified'], doc['record'], delay))
        result.planner()  # Reject unsupported map topology before connecting hardware.
        if doc['verified']:
            result.calibration.validate()
        return result

    def planner(self):
        return FieldPlanner(self.field, self.robot)


def measured_pose(robot, calibration, *, now=None):
    """Use fresh unsmoothed surveyed Tag solutions, in the planner's frame."""
    field = robot.field_pose
    if field is None or not field.valid or field.calibrated is not True:
        raise NavigationError('competition planning requires calibrated field localization')
    captured = field.captured_monotonic - calibration.capture_delay_s
    # A camera thread can publish after an earlier clock read. Snapshot its
    # acquisition timestamp first, then evaluate age against the current time.
    now = time.monotonic() if now is None else now
    if not math.isfinite(captured) or not 0 <= now - captured <= .3:
        raise NavigationError('competition planning field pose is stale or future-dated')
    candidates = [tag for tag in field.tag_solutions
                  if all(isinstance(getattr(tag, name, None), (int, float))
                         and not isinstance(getattr(tag, name), bool)
                         and math.isfinite(getattr(tag, name))
                         for name in ('x_m', 'y_m', 'yaw_deg', 'area_px', 'reprojection_error_px'))
                  and tag.area_px >= 200 and 0 <= tag.reprojection_error_px <= 4]
    if not candidates:
        raise NavigationError('competition planning has no usable raw Tag pose')
    best = min(candidates, key=lambda tag: tag.reprojection_error_px)
    pose = Pose(2400 + 1000 * best.x_m, 3600 - 1000 * best.y_m, -best.yaw_deg)
    for tag in candidates:
        if math.hypot(tag.x_m-best.x_m, tag.y_m-best.y_m) > .15 or abs(wrap(tag.yaw_deg-best.yaw_deg)) > 8:
            raise NavigationError('competition planning Tag poses disagree; relocalize')
    return pose


class CompetitionRoutes:
    """Generate paths to current recipe endpoints, retaining contact boundaries."""
    def __init__(self):
        self._config = self._planner = None

    @staticmethod
    def _configuration(env):
        config = env.transition_config.navigation
        if config is None:
            raise ValueError('Tag route planning requires --navigation-config with measured calibration')
        config.calibration.validate()
        return config

    def start(self, env, plan):
        config = self._configuration(env)
        env.context.check_active()
        start = measured_pose(env.robot, config.calibration)
        env.robot.diagnostics.write('route_planner_started', backend='field_navigation:competition_recipe',
            start=vars(start), source='calibrated_tag_pose', planner='FieldPlanner',
            timing='CompetitionMotion.route_timing', tracking='PositionTracker')

    def close(self):
        self._config = self._planner = None

    def prepare(self, env, route, profile):
        from ..flows.curves import CURVE_ROUTES, prepare_route_geometry
        from ..flows.routes import ROUTES
        from ..optimizations.motion_planning import PreparedMotion
        config = self._configuration(env)
        env.context.check_active()
        if route not in CURVE_ROUTES:
            return PreparedMotion('classic:contact_or_visual_barrier', lambda: ROUTES[route](env, profile))
        # These owners intentionally move before their mechanism completes.
        # Keep their measured overlap route; the stationary planner may not
        # take control of a chassis already owned by an action session.
        if (env._mechanisms or any(env.transitions.grabs.values()) or env.transitions.inspections):
            return PreparedMotion('classic:mechanism_overlap', lambda: ROUTES[route](env, profile))
        env.context.check_active()
        start = measured_pose(env.robot, config.calibration)
        geometry = prepare_route_geometry(env, route, profile)
        if geometry is None:
            return PreparedMotion('field_navigation:already_completed', lambda: None)
        end = geometry.endpoint
        angle = math.radians(start.yaw)
        goal = Pose(start.x + math.cos(angle)*end.x_mm - math.sin(angle)*end.y_mm,
                    start.y + math.sin(angle)*end.x_mm + math.cos(angle)*end.y_mm,
                    start.yaw + end.yaw_deg)
        if self._config is not config:
            self._planner, self._config = config.planner(), config
        adapter = RobotNavigation(env.robot, env.context, self._planner, config.calibration)
        planned = adapter.prepare_between(start, goal)

        def execute():
            current = measured_pose(env.robot, config.calibration)
            if math.hypot(current.x-start.x, current.y-start.y) > 10 or abs(wrap(current.yaw-start.yaw)) > 2:
                raise NavigationError('competition planning start changed during preparation')
            env.robot.diagnostics.write('planned_competition_route', route=route, profile=profile,
                start=vars(start), goal=vars(goal), length_mm=planned.nominal_length_mm,
                candidates=planned.candidate_count)
            result = adapter.execute(planned)
            geometry.complete()
            env.record_transition('field_navigation', route, 'complete')
            return result

        return PreparedMotion('field_navigation:competition_recipe', execute)
