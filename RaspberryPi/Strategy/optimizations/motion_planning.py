"""Pluggable route optimization with a classic, pre-motion fallback.

prepare() must not command hardware. Once an optimized operation starts, any
exception propagates to the existing emergency-stop path; never replay classic
movement from a partially completed optimized route.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class PreparedMotion:
    backend: str
    execute: object

    def __post_init__(self):
        if not self.backend or not callable(self.execute):
            raise ValueError('prepared motion requires a backend and callable')


class CalibratedCurves:
    """Existing curve implementation remains one interchangeable optimizer."""
    def prepare(self, env, route, profile):
        calibration=env.transition_config.curves.get(f'{profile}/{route}')
        if calibration is None:
            return None
        from ..flows.curves import run_curve
        return PreparedMotion('calibrated_curve',lambda:run_curve(env,route,profile,calibration))


class FieldNavigation:
    """Use the goal navigator for explicitly mapped competition routes.

    bindings maps profile/route to a read-only request callback. It returns
    (source Location, target Location, measured start Pose), or None when its
    localization/operation contract is unavailable. The adapter factory binds
    a calibrated RobotNavigation to this execution context. No nominal start
    is silently substituted for a missing measured pose.
    """
    def __init__(self, bindings, adapter_factory):
        self.bindings=dict(bindings)
        if not callable(adapter_factory) or any(not callable(v) for v in self.bindings.values()):
            raise ValueError('field navigation requires request callbacks and an adapter factory')
        self.adapter_factory=adapter_factory

    def prepare(self, env, route, profile):
        request=self.bindings.get(f'{profile}/{route}')
        if request is None: return None
        values=request(env)
        if values is None: return None
        source,target,pose=values
        from ..navigation import Location, Pose
        if not isinstance(source,Location) or not isinstance(target,Location) or not isinstance(pose,Pose):
            raise ValueError('field route needs operation locations and measured pose')
        adapter=self.adapter_factory(env)
        planned=adapter.prepare(source,target,actual_start=pose)
        return PreparedMotion('field_navigation',lambda:adapter.execute(planned))


class MotionPlanning:
    def __init__(self, *, enabled=True, optimizers=None):
        from ..navigation.competition import CompetitionRoutes
        from ..navigation.recipe import RecipeRoutes
        if type(enabled) is not bool: raise ValueError('motion planning switch must be boolean')
        self.enabled=enabled
        self.optimizers=tuple(optimizers) if optimizers is not None else (CompetitionRoutes(), RecipeRoutes())
        if any(not callable(getattr(item,'prepare',None)) for item in self.optimizers):
            raise TypeError('optimizer must implement prepare')

    def start(self, env, plan):
        if self.enabled:
            for optimizer in self.optimizers:
                start=getattr(optimizer,'start',None)
                if start is not None:
                    start(env,plan)

    def close(self, env):
        for optimizer in self.optimizers:
            close=getattr(optimizer,'close',None)
            if close is not None:
                close(env)

    def run(self, env, route, profile, *, classic_routes=None):
        if classic_routes is None:
            from ..flows.routes import ROUTES
            classic_routes=ROUTES
        def record(backend):
            diagnostics=getattr(env.robot,'diagnostics',None)
            if diagnostics is not None:
                diagnostics.write('motion_backend_selected',route=route,profile=profile,backend=backend)
        env.context.check_active()
        selected=None
        if self.enabled:
            for optimizer in self.optimizers:
                selected=optimizer.prepare(env,route,profile)
                if selected is not None:
                    if not isinstance(selected,PreparedMotion): raise TypeError('invalid prepared motion')
                    break
        if selected is None:
            reason='disabled' if not self.enabled else 'no_applicable_calibration_or_binding'
            record(f'classic:{reason}')
            return classic_routes[route](env,profile)
        record(selected.backend)
        env.context.check_active()
        return selected.execute()
