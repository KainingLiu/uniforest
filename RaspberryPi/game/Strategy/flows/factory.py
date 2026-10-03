"""Lower semantic action specifications to guarded executable phases."""

import math
from contextlib import suppress

from protocol.commands import ACTION_BUILD
from ..controllers import RobotController, Phase
from ..execution import Action, ActionFlow, Transition, TransitionRegistry, compile_flow
from ..settings import PROFILES
from .operations import OPERATIONS
from .routes import ROUTES
from .curves import CURVE_ROUTES, run_curve
from .transitions import ActionTransitions
from ..transition_config import TransitionConfig


PARAMETERS = {
    'navigate': {'route', 'after_build'},
    'rebase_heading': {'reference_cw_deg'},
    'anchor_wall': {'direction', 'recalibrate', 'reference_cw_deg', 'settle_s',
                    'timeout_s', 'speed_mm_s'},
    'begin_collection': {'color', 'detector_profile'},
    'acquire_cube': {'index', 'max_count', 'conditional_on_purple'},
    'grab_cube': {'index', 'max_count', 'conditional_on_purple', 'method', 'followup_route'},
    'inspect_cargo': {'method', 'exit_route', 'cross_region_refill'},
    'unload': set(), 'load_staged': set(),
    'align_tag': {'purpose'}, 'align_building': set(), 'build': set(),
}

ROUTE_PROFILES = {
    **{name: {'ground-1','ground-2','ground-3'} for name in (
        'ground_delivery_reverse','ground_to_delivery','ground_tag_offset','ground_delivery_depart')},
    **{name: {'highland-1','highland-2'} for name in (
        'to_purple','purple_to_orange','orange_depart_reverse','orange_to_build')},
    **{name: {'building-1','building-2','building-3'} for name in ('build_offset','build_return')},
    **{name: {'unload-1','unload-2'} for name in ('unload_approach','unload_depart')},
    **{name: {'staged-building'} for name in (
        'staged_initial','staged_to_build','staged_return_first','staged_return_final')},
    'depart_a': {'depart-a'}, 'depart_b': {'depart-b'}, 'return_orange': {'return-orange'},
}


def _validate_route(spec, route):
    if route not in ROUTES or spec.profile not in ROUTE_PROFILES[route]:
        raise ValueError(f'{spec.name}: route/profile combination has no calibration')


def validate_spec(spec):
    if spec.kind not in PARAMETERS:
        raise ValueError(f'unknown functional action: {spec.kind}')
    if spec.profile not in PROFILES:
        raise ValueError(f'unknown calibration profile: {spec.profile}')
    if set(spec.parameters) - PARAMETERS[spec.kind]:
        raise ValueError(f'{spec.name}: unknown action parameters')
    for key, value in spec.parameters.items():
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError(f'{spec.name}: {key} must be finite')
    params = spec.parameters
    if spec.kind == 'navigate' and params.get('route') not in ROUTES:
        raise ValueError(f'{spec.name}: unknown route')
    for key in ('exit_route', 'followup_route'):
        if key in params and params[key] not in ROUTES:
            raise ValueError(f'{spec.name}: unknown {key}')
    for key in ('route','exit_route','followup_route'):
        if key in params:
            _validate_route(spec, params[key])
    for key in ('after_build','recalibrate','conditional_on_purple','cross_region_refill'):
        if key in params and type(params[key]) is not bool:
            raise ValueError(f'{spec.name}: {key} must be boolean')
    for key in ('reference_cw_deg','settle_s','timeout_s','speed_mm_s'):
        if key in params:
            value=params[key]
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
                raise ValueError(f'{spec.name}: invalid {key}')
            if key == 'settle_s' and value < 0 or key in ('timeout_s','speed_mm_s') and value <= 0:
                raise ValueError(f'{spec.name}: invalid {key}')
    if spec.kind == 'rebase_heading' and 'reference_cw_deg' not in params:
        raise ValueError(f'{spec.name}: reference_cw_deg is required')
    config=PROFILES[spec.profile]
    required = {
        'anchor_wall':'far_wall_speed_mm_s', 'begin_collection':'search_speed_mm_s',
        'acquire_cube':'search_speed_mm_s', 'grab_cube':'near_wall_speed_mm_s',
        'inspect_cargo':'search_speed_mm_s', 'unload':'unload_reverse_mm',
        'load_staged':'hatch_close_settle_ms', 'align_building':'building_target_z_mm',
        'build':'building_target_z_mm',
        'align_tag':'build_tag_id' if params.get('purpose') == 'build' else 'delivery_tag_id',
    }.get(spec.kind)
    if required and not hasattr(config,required):
        raise ValueError(f'{spec.name}: profile does not support this action')
    if (params.get('conditional_on_purple') or params.get('color') == 'purple') and not hasattr(config,'purple_min_confidence'):
        raise ValueError(f'{spec.name}: profile lacks purple collection calibration')
    if params.get('after_build') and params.get('route') not in ('build_return','staged_return_first','staged_return_final'):
        raise ValueError(f'{spec.name}: route has no verified build overlap contract')
    if spec.kind in ('grab_cube', 'inspect_cargo') and params.get('method') not in ('grap1','grap2','grap3'):
        raise ValueError(f'{spec.name}: unknown mechanism')
    if params.get('cross_region_refill'):
        ground = spec.profile.startswith('ground-')
        if ((not ground and not spec.profile.startswith('highland-'))
                or params.get('method') != ('grap3' if ground else 'grap1')
                or params.get('exit_route') != ('ground_delivery_reverse' if ground else 'orange_depart_reverse')):
            raise ValueError(f'{spec.name}: cross-region refill requires an orange collection route')
    for key in ('index', 'max_count'):
        if key in params and (type(params[key]) is not int or not 1 <= params[key] <= 3):
            raise ValueError(f'{spec.name}: {key} must be an integer in 1..3')
    if spec.kind == 'begin_collection' and params.get('color') not in ('orange','purple'):
        raise ValueError(f'{spec.name}: invalid cube color')
    if 'detector_profile' in params and params['detector_profile'] not in ('default','task2_orange','task2_purple'):
        raise ValueError(f'{spec.name}: unknown pickup detection profile')
    if spec.kind == 'anchor_wall' and params.get('direction','forward') not in ('forward','backward','left','right'):
        raise ValueError(f'{spec.name}: invalid wall direction')
    if spec.kind == 'align_tag' and params.get('purpose','delivery') not in ('delivery','build'):
        raise ValueError(f'{spec.name}: invalid tag purpose')


class ActionEnvironment:
    """One action flow's hardware bindings and transient collection measurements."""

    def __init__(self, robot, context, *, transition_config=None):
        self.robot, self.context = robot, context
        self.data = {}
        self._controllers = {}
        self._mechanisms = {}
        self.transition_config = transition_config or TransitionConfig()
        from ..optimizations.motion_planning import MotionPlanning
        self.motion_planning = MotionPlanning(
            enabled=self.transition_config.motion_planning_enabled,
            moving_tag6_enabled=self.transition_config.moving_tag6_enabled)
        self.transitions = ActionTransitions(self)
        for key in self.transition_config.curves:
            profile, route = key.split('/')
            if route not in CURVE_ROUTES or profile not in ROUTE_PROFILES[route]:
                raise ValueError(f'{key}: no continuous route calibration contract')

    def record_transition(self, kind, source, result):
        diagnostics=getattr(self.robot,'diagnostics',None)
        if diagnostics is not None:
            diagnostics.write('transition_result',kind=kind,source=source,result=result)

    def control(self, profile):
        if profile not in self._controllers:
            self._controllers[profile] = RobotController(
                self.robot, PROFILES[profile], context=self.context,
                operation_name=self.context.current_action or profile)
        return self._controllers[profile]

    def phase(self, controller, name):
        self.context.check_active()
        controller.operation_name = self.context.current_action or controller.operation_name
        controller.state = Phase[name]

    def run_route(self, route, profile):
        self.context.check_active()
        if 'collection' in self.data:
            self.data['collection']['acquired'] = False
        if self.transition_config.motion_planning_enabled:
            result = self.motion_planning.run(self,route,profile,classic_routes=ROUTES)
        else:
            from ..optimizations.motion_planning import MotionPlanning
            result = MotionPlanning(enabled=False).run(self,route,profile,classic_routes=ROUTES)
        self.context.check_active()
        return result

    def stop(self):
        if not self.robot.chassis.set_speeds([0,0,0,0]):
            raise RuntimeError('failed to stop chassis at action boundary')

    def abort(self):
        self.transitions.abort()
        for session, _ in self._mechanisms.values():
            with suppress(BaseException):
                session.abort()
        self._mechanisms.clear()
        # Restore the detector selection even if collection failed before its
        # inspection phase. This does not reopen an invalid camera-pose gate.
        setter = getattr(self.robot, 'set_cube_detection_profile', None)
        if setter is not None and getattr(self.robot, 'has_vision', False):
            with suppress(BaseException):
                setter('default')

    def _enter(self, spec):
        self.context.check_active()
        if spec.kind != 'grab_cube' and 'collection' in self.data:
            self.data['collection']['acquired'] = False
        if spec.requires_anchor is not None and self.context.anchor != spec.requires_anchor:
            raise RuntimeError(f'{spec.name}: route anchor changed before execution')
        self.context.current_action = spec.name
        self.control(spec.profile).operation_name = spec.name

    def _complete(self, spec, result=None):
        self.context.check_active()
        if spec.ends_at is not None:
            self.context.anchor = spec.ends_at
        return result

    def _align_tag(self, spec):
        if self.transition_config.motion_planning_enabled and self.motion_planning.consume_completed(spec.name):
            return True
        c = self.control(spec.profile)
        cfg = c.config
        self.phase(c, 'TAG_ALIGN')
        self.robot.reset_field_localization_filter()
        if spec.parameters.get('purpose') == 'build':
            return c._align_delivery_tag_or_continue(
                tag_id=cfg.build_tag_id, target_distance_mm=cfg.build_tag_distance_mm,
                heading_target_cw_deg=cfg.build_tag_heading_target_cw_deg,
                distance_tolerance_mm=cfg.build_tag_distance_tolerance_mm,
                lateral_tolerance_mm=cfg.build_tag_lateral_tolerance_mm,
                heading_tolerance_deg=cfg.build_tag_heading_tolerance_deg,
                fine_gain_scale=cfg.build_tag_fine_gain_scale,
                vision_stale_s=cfg.build_tag_vision_stale_s,
                lost_timeout_s=cfg.build_tag_lost_timeout_s,
                fine_align_enabled=False, stop_axes_in_tolerance=True, independent_heading=True)
        return c._align_delivery_tag_or_continue(
            fine_align_enabled=False, stop_axes_in_tolerance=True, independent_heading=True)

    def _ordinary(self, spec):
        c = self.control(spec.profile)
        if spec.kind == 'acquire_cube' and self.transitions.acquisitions.pop(spec.name,False):
            self.data['collection']['acquired']=True
            result=True
        elif spec.kind in OPERATIONS:
            result = OPERATIONS[spec.kind](self, spec)
        elif spec.kind == 'rebase_heading':
            c._heading_zero_deg = c._wrap_angle(
                self.robot.telem.yaw_deg + spec.parameters['reference_cw_deg'])
            result = c._heading_zero_deg
        elif spec.kind == 'align_tag':
            result = self._align_tag(spec)
        elif spec.kind == 'align_building':
            self.phase(c, 'BUILDING_ALIGN')
            self.robot.reset_vision_filter()
            if self.transition_config.motion_planning_enabled:
                # A smoothed Tag approach proves its own target, not the actual
                # building position. Missing terminal vision must block Build.
                c._align_building()
                result = True
            else:
                result = c._align_building_or_continue()
        else:
            raise ValueError(f'no functional operation for {spec.kind}')
        return self._complete(spec, result)

    def _start_build(self, spec):
        self._enter(spec)
        # Both highland recipes (two orange + purple, or three orange) need
        # the existing cargo confirmation before starting the same Build.
        if self.data.get('purple_grabbed') is not None and self.data.get('carried_count') != 3:
            raise RuntimeError('highland cargo count must be confirmed before building')
        self.phase(self.control(spec.profile), 'BUILD')
        begin_pose = getattr(self.robot, 'begin_cube_camera_pose_change', None)
        pose = begin_pose('build') if begin_pose else None
        session = self.robot.actions.begin(ACTION_BUILD)
        self._mechanisms[spec.name] = (session, pose)

    def _finish_build(self, spec):
        session, pose = self._mechanisms[spec.name]
        session.wait_done()
        session.close()
        end_pose = getattr(self.robot, 'end_cube_camera_pose_change', None)
        if end_pose and end_pose(pose, settle_s=PROFILES[spec.profile].post_grab_settle_s) is False:
            raise RuntimeError('camera pose restoration was superseded or invalidated')
        del self._mechanisms[spec.name]
        self._complete(spec)

    def bind(self, spec):
        validate_spec(spec)
        metadata = {'profile': spec.profile, **spec.parameters}
        if spec.kind == 'grab_cube':
            def enter_grab():
                self._enter(spec)
                self.transitions.start_grab(spec)
            return Action('grab_cube',spec.name,enter=enter_grab,
                          body=lambda:self.transitions.wait_grab_clear(spec),
                          exit=lambda:self.transitions.finish_grab(spec),context=metadata)
        if spec.kind == 'inspect_cargo' and hasattr(self.robot,'begin_carried_cube_inspection'):
            return Action('inspect_cargo',spec.name,enter=lambda:self._enter(spec),
                          body=lambda:self._complete(spec,self.transitions.inspect(spec)),
                          exit=lambda:self.transitions.finish_inspection(spec),context=metadata)
        if spec.kind == 'build':
            return Action('build', spec.name,
                          enter=lambda: self._start_build(spec),
                          body=lambda: self._mechanisms[spec.name][0].wait_chassis_ready(),
                          exit=lambda: self._finish_build(spec), context=metadata)
        if spec.kind == 'navigate':
            def enter():
                self._enter(spec)
                if (self.transition_config.motion_planning_enabled
                        and self.motion_planning.consume_completed(spec.name)):
                    return
                self.run_route(spec.parameters['route'], spec.profile)
            return Action('navigate', spec.name, enter=enter,
                          body=lambda: self._complete(spec), context=metadata)
        return Action(spec.kind, spec.name, enter=lambda: self._enter(spec),
                      body=lambda: self._ordinary(spec), context=metadata)

    def compile(self, plan):
        from ..transition_switches import transition_enabled, validate_transition_selection
        validate_transition_selection(self.transition_config, plan)
        if self.transition_config.motion_planning_enabled:
            self.motion_planning.start(self, plan)
        specs = {spec.name: spec for spec in plan.steps}

        def overlap(previous, following, context):
            session, _ = self._mechanisms[previous.name]
            spec = specs[following.name]
            self._enter(spec)
            with self.robot.chassis.monitor_action(session.check):
                self.run_route(spec.parameters['route'], spec.profile)
            self._finish_build(specs[previous.name])

        def matches(previous, following, context):
            target = specs[following.name]
            return (transition_enabled(self.transition_config, 'build-return', specs[previous.name]) and
                    following.context.get('after_build') is True and
                    (target.requires_anchor is None or target.requires_anchor == self.context.anchor))

        registry = TransitionRegistry((*self.transitions.registry(plan.steps),Transition(
            'build_release_to_route', 'build', 'navigate', overlap,
            matches=matches),))
        actions=[]
        for spec in plan.steps:
            validate_spec(spec)
        groups=(self.motion_planning.group_steps(plan.steps) if self.transition_config.motion_planning_enabled
                else ((spec,) for spec in plan.steps))
        for group in groups:
            if len(group)==1:
                actions.append(self.bind(group[0]))
                continue
            def enter_chain(group=group):
                self._enter(group[0])
                self.context.current_action=group[-1].name
                self.motion_planning.run_chain(self,group)
            actions.append(Action('navigate',group[-1].name,enter=enter_chain,
                body=lambda last=group[-1]:self._complete(last),
                context={'profile':group[-1].profile,'route':'plan_b_first',
                         'source_actions':[s.name for s in group]}))
        return compile_flow(ActionFlow(plan.name, tuple(actions)), registry)
