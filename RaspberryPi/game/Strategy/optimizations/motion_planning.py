"""One competition optimizer: original local commands plus moving Tag6 guidance.

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


class MotionPlanning:
    def __init__(self, *, enabled=True, moving_tag6_enabled=False):
        from .local_routes import LocalRoutes
        if type(enabled) is not bool: raise ValueError('motion planning switch must be boolean')
        if type(moving_tag6_enabled) is not bool: raise ValueError('moving Tag6 switch must be boolean')
        if moving_tag6_enabled and not enabled: raise ValueError('moving Tag6 requires motion planning')
        self.enabled=enabled
        self.moving_tag6_enabled=moving_tag6_enabled
        self._routes=LocalRoutes()

    def start(self, env, plan):
        if self.enabled:
            self._routes.configure(plan, moving_tag6_enabled=self.moving_tag6_enabled)

    def close(self, env):
        self._routes.completed_actions.clear()

    def consume_completed(self, name):
        return self.enabled and self._routes.consume_completed(name)

    def group_steps(self, steps):
        from .route_chain import grouped_steps
        return grouped_steps(steps,enabled=self.enabled)

    def run_chain(self, env, steps):
        from .route_chain import prepare_chain
        if not self.enabled:
            raise RuntimeError('route fusion requires motion planning')
        env.context.check_active()
        selected=prepare_chain(self._routes,env,steps)
        diagnostics=getattr(env.robot,'diagnostics',None)
        if diagnostics is not None:
            diagnostics.write('motion_backend_selected',route='plan_b_first',
                profile=steps[-1].profile,backend=selected.backend)
        return selected.execute()

    def run_refill(self, env, source, destination, **profiles):
        from .refill_transfer import run_transfer
        if not self.enabled:
            raise RuntimeError('refill smoothing requires motion planning')
        return run_transfer(self._routes, env, source, destination, **profiles)

    def run(self, env, route, profile, *, classic_routes=None):
        if classic_routes is None:
            from ..flows.routes import ROUTES
            classic_routes=ROUTES
        def record(backend):
            diagnostics=getattr(env.robot,'diagnostics',None)
            if diagnostics is not None:
                diagnostics.write('motion_backend_selected',route=route,profile=profile,backend=backend)
        env.context.check_active()
        if not self.enabled:
            record('classic:disabled')
            return classic_routes[route](env,profile)
        selected=self._routes.prepare(env,route,profile)
        if not isinstance(selected,PreparedMotion): raise TypeError('local optimizer did not prepare a route')
        record(selected.backend)
        env.context.check_active()
        return selected.execute()
