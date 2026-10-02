"""Existing route endpoints -> the shared free-space curve and speed planner.

Recording obtains endpoint geometry and live displacement compensation only.
Old intermediate move/turn points are never passed to the path search. Contact,
visual correction, and the existing clearance-retreat boundaries remain explicit.
"""
import json
import math
from pathlib import Path

from control.trajectory import Waypoint
from .planner import FieldPlanner, Pose, NavigationError
from .robot_adapter import RobotNavigation


def default_planner():
    # Reuse the existing CAD inputs. Their provenance stays nominal/unconfirmed;
    # loading this model does not assert that a camera or geometry was calibrated.
    root = Path(__file__).resolve().parents[2] / 'simulation'
    return FieldPlanner(json.loads((root/'field_model.json').read_text(encoding='utf-8')),
                        json.loads((root/'robot_model.json').read_text(encoding='utf-8')))


def endpoint_from_recipe(start, origin, end):
    angle = math.radians(start.yaw-origin.yaw_deg)
    dx,dy = end.x_mm-origin.x_mm, end.y_mm-origin.y_mm
    return Pose(start.x+math.cos(angle)*dx-math.sin(angle)*dy,
                start.y+math.sin(angle)*dx+math.cos(angle)*dy,
                start.yaw+end.yaw_deg-origin.yaw_deg)


class RecipeRoutes:
    def __init__(self):
        self.planner = None
        self.odometry = None

    def start(self, env, plan):
        if env.transition_config.navigation is not None:
            return
        self.planner = default_planner()
        if plan.entry_anchor == 'start':
            anchor = self.planner.field['anchors']['start_blue']
            start = Pose(anchor['x_mm'], anchor['y_mm'], anchor['yaw_deg']+plan.entry_heading_deg)
        else:
            # Standalone mid-match entries have no implied startup placement.
            raise NavigationError('standalone field planning needs --navigation-config; PlanA/PlanB use the existing start placement')
        self.odometry = env.robot.begin_route_odometry(start)
        env.robot.diagnostics.write('route_planner_started', backend='field_curve:recipe_endpoints',
            start=vars(start), source='existing_map_start_and_encoder_imu',
            geometry_field_validated=False, planner='FieldPlanner',
            timing='CompetitionMotion.route_timing', tracking='PositionTracker')

    def close(self, env):
        if self.odometry is not None:
            env.robot.end_route_odometry(self.odometry)
            self.odometry = None

    def prepare(self, env, route, profile):
        from ..optimizations.motion_planning import PreparedMotion
        from ..flows.route_source import RouteRecorder, CLEARANCE_RETREATS
        from ..flows.routes import ROUTES
        if self.planner is None or self.odometry is None:
            raise NavigationError('recipe curve planner was not initialized at task entry')
        recorder = RouteRecorder(env, profile)
        ROUTES[route](recorder, profile)
        actions, group = [], []
        origin = Waypoint(0.,0.,0.)

        def flush():
            nonlocal origin, group
            if group:
                end = group[-1][1]
                if end != origin:
                    actions.append(('route', (origin, end)))
                origin, group = end, []

        for index, event in enumerate(recorder.events):
            if event[0] in ('move','turn'):
                group.append(event)
                if index == 0 and route in CLEARANCE_RETREATS and event[0] == 'move' and event[2][0] == 'backward':
                    flush()
            else:
                flush()
                actions.append(('boundary',event))
                origin = event[1]
        flush()
        changed = {key:recorder.data[key] for key in ('ground_lateral_mm','orange_lateral_mm',
            'purple_lateral_mm','ground_reverse_done','orange_reverse_done','purple_route_done')
            if key in recorder.data}
        adapter = RobotNavigation(env.robot, env.context, self.planner, None, odometry=self.odometry)

        def execute():
            result = None
            c = env.control(profile)
            for index, (kind, item) in enumerate(actions):
                env.context.check_active()
                if kind == 'route':
                    start = self.odometry.snapshot().pose
                    goal = endpoint_from_recipe(start, *item)
                    planned = adapter.prepare_between(start, goal)
                    env.robot.diagnostics.write('planned_recipe_curve', route=route, profile=profile,
                        section=index, start=vars(start), goal=vars(goal),
                        length_mm=planned.nominal_length_mm, candidates=planned.candidate_count,
                        geometry_field_validated=False, source='existing_route_endpoint')
                    current = self.odometry.snapshot().pose
                    if math.hypot(current.x-start.x,current.y-start.y)>10 or abs(current.yaw-start.yaw)>2:
                        raise NavigationError('recipe curve start moved during planning')
                    result = adapter.execute(planned)
                else:
                    boundary,_,args,kwargs = item
                    if boundary == 'wall': c._drive_until_wall(**kwargs)
                    elif boundary == 'rebase': c._recalibrate_heading_zero(*args)
                    elif boundary == 'tag_reset': env.robot.reset_field_localization_filter()
                    elif boundary == 'tag_align': c._align_delivery_tag_or_continue(**kwargs)
                    else: raise NavigationError(f'unsupported recipe boundary: {boundary}')
            env.context.check_active()
            env.data.update(changed)
            env.record_transition('field_recipe_curve', route, 'complete')
            return result
        return PreparedMotion('field_curve:recipe_endpoints', execute)
