"""Concrete pickup/inspection phase replacements with one chassis owner."""

from contextlib import suppress
from dataclasses import dataclass, replace
import math
import time

from protocol.commands import ACTION_GRAP1, ACTION_GRAP2, ACTION_GRAP3
from ..execution import Transition
from ..execution.blind import BlindSample, run_blind_transition
from ..execution.pickup_motion import acquire_after_blind
from ..transition_switches import transition_enabled
from ..optimizations.adaptive_blind import expected_blind_distance
from . import operations


GRABS = {'grap1': ACTION_GRAP1, 'grap2': ACTION_GRAP2, 'grap3': ACTION_GRAP3}


@dataclass
class PendingGrab:
    spec: object
    session: object
    pose: object
    press: object
    restore_s: float
    reset_at: float | None = None
    restored: bool = False
    next_observation: object = None


class ActionTransitions:
    def __init__(self, env):
        self.env = env
        self.grabs = {}
        self.inspections = {}
        self.acquisitions = {}
        self.early_departure = None
        self.specs = ()

    def policy(self, spec):
        return self.env.transition_config.pickup(spec.profile, spec.parameters.get('method'))

    def start_grab(self, spec):
        env = self.env
        c = env.control(spec.profile)
        collection = operations._collection(env, spec)
        self.grabs[spec.name] = None
        if not collection['acquired'] or not operations._selected_pickup(env, spec, c):
            return
        collection['acquired'] = False
        env.phase(c, 'GRAB')
        # Consume before begin_pose_change invalidates the arm-camera geometry.
        observation = None
        observer = collection.pop('neighbor_observer', None)
        if observer is not None:
            frame = env.robot.vision_result
            _, _, _, link_epoch = env.robot.inspection_link_snapshot()
            observation = observer.consume(now=time.monotonic(),
                pose_epoch=getattr(frame, 'pose_epoch', None), link_epoch=link_epoch,
                stop_generation=env.robot.transport.emergency_stop_generation)
        begin = getattr(env.robot, 'begin_cube_camera_pose_change', None)
        pose = begin('grab') if begin else None
        press = c._grab_press_step(recalibrate_heading_zero=collection['color'] == 'orange')
        session = env.robot.actions.begin(GRABS[spec.parameters['method']])
        policy = self.policy(spec)
        restore = policy.arm_restore_s if policy else c.config.post_grab_settle_s
        self.grabs[spec.name] = PendingGrab(spec, session, pose, press, restore,
                                           next_observation=observation)

    def check_grab(self, pending):
        self.env.context.check_active()
        pending.session.check()
        if pending.session.done and pending.reset_at is None:
            pending.reset_at = time.monotonic() + pending.restore_s
            end = getattr(self.env.robot, 'end_cube_camera_pose_change', None)
            if end and end(pending.pose, settle_s=pending.restore_s) is False:
                raise RuntimeError('grab camera pose generation was invalidated')
        pending.restored = pending.reset_at is not None and time.monotonic() >= pending.reset_at

    def wait_grab_clear(self, spec):
        pending = self.grabs.get(spec.name)
        if pending is None:
            return False
        pending.session.wait_chassis_ready(pending.press)
        self.check_grab(pending)
        return True

    def finish_grab(self, spec):
        pending = self.grabs.get(spec.name)
        if pending is None:
            return
        pending.session.wait_done()
        self.check_grab(pending)
        while not pending.restored:
            time.sleep(min(.01, max(0, pending.reset_at-time.monotonic())))
            self.check_grab(pending)
        pending.session.close()
        collection = operations._collection(self.env, spec)
        collection['pickups'] += 1
        collection['acquired'] = False
        if collection['color'] == 'purple':
            self.env.data['purple_grabbed'] = True
        # Pose restoration has already invalidated old/in-flight results. Do
        # not reset geometry here: a successful handoff is using the new frame.
        del self.grabs[spec.name]
        self.env._complete(spec)

    def _next_effective(self, source):
        index = next(i for i,s in enumerate(self.specs) if s.name == source.name)
        for spec in self.specs[index+1:]:
            if spec.kind in ('acquire_cube','grab_cube') and not operations._selected_pickup(
                    self.env,spec,self.env.control(spec.profile)):
                continue
            return spec
        return None

    def _enabled(self, source):
        return (self.grabs.get(source.name) is not None and self.policy(source) is not None
                and (self.env.robot.actions.pickup_full_lift_validated or
                     self.env.transition_config.trial_run and getattr(self.env.robot.actions,'pickup_trial_enabled',False)))

    def next_matches(self, source, target):
        if not self._enabled(source) or not transition_enabled(self.env.transition_config, 'next-cube', source):
            return False
        policy = self.policy(source)
        following = self._next_effective(source)
        return (policy.next_blind is not None and source.profile == target.profile
                and following is not None and following.name == target.name
                and target.kind == 'acquire_cube')

    def next_cube(self, source, target):
        env, policy = self.env, self.policy(source)
        pending = self.grabs[source.name]
        env._enter(target)
        c = env.control(source.profile)
        origin = c._capture_lateral_origin()
        phase_origin = env.data['collection']['origin']
        yaw = env.robot.telem.yaw_deg
        if not math.isfinite(yaw):
            raise RuntimeError('pickup heading telemetry is invalid')
        acquired = False
        limit = min(c.config.search_max_distance_mm,policy.next_acquire.max_distance_mm)
        budget_position = c._measure_lateral_displacement_mm(phase_origin)
        if not math.isfinite(budget_position):
            raise RuntimeError('pickup encoder displacement is invalid')
        remaining = min(limit-budget_position,limit-c._search_position_mm)
        budget_at = time.monotonic()
        last_command = 0.0
        blind_owns_motion = True
        last_feedback = None
        minimum_progress = minimum_velocity = 0.0

        def consume_blind_budget():
            nonlocal budget_position, budget_at
            if not blind_owns_motion:
                return
            now = time.monotonic()
            position = c._measure_lateral_displacement_mm(phase_origin)
            if not math.isfinite(position):
                raise RuntimeError('pickup encoder displacement is invalid')
            c._search_position_mm += max(0.0,position-budget_position)
            if last_command > 0:
                c._orange_recovery.search_elapsed_s += max(0.0,now-budget_at)
            budget_position,budget_at = position,now
        time_left = (c.config.search_max_distance_mm/c.config.search_speed_mm_s
                     + c.config.orange_edge_timeout_s
                     + c.config.target_cube_count*c.config.vision_observe_s
                     - c._orange_recovery.search_elapsed_s)
        if (remaining <= policy.next_blind.braking_margin_mm or
                time_left <= policy.next_blind.tick_s+policy.next_blind.max_command_delay_s):
            self.finish_grab(source)
            env.stop()
            env.record_transition('grab_to_next_cube',source.name,'search_boundary_fallback')
            return
        expected_distance = policy.next_cube_distance_mm
        adaptive = (policy.next_adaptive if
                    env.transition_config.alignment(source.profile, 'orange') is not None else None)
        if adaptive is not None:
            _, _, _, link_epoch = env.robot.inspection_link_snapshot()
            expected_distance, reason = expected_blind_distance(adaptive, pending.next_observation,
                position_mm=budget_position, yaw_deg=yaw, now=time.monotonic(),
                link_epoch=link_epoch, stop_generation=env.robot.transport.emergency_stop_generation,
                expected_pose_epoch=pending.pose, legacy_distance_mm=policy.next_cube_distance_mm)
            with suppress(Exception):
                env.record_transition('adaptive_blind', source.name, f'{reason}:{expected_distance:.1f}mm')
            if expected_distance <= 0:
                self.finish_grab(source)
                env.stop()
                return
            if pending.next_observation is not None:
                time_left = min(time_left, adaptive.max_prediction_age_s
                                - (time.monotonic()-pending.next_observation.captured_s))
                if time_left <= policy.next_blind.tick_s + policy.next_blind.max_command_delay_s:
                    self.finish_grab(source)
                    env.stop()
                    return
        blind_profile = replace(policy.next_blind,
                                # The blind controller reserves braking_margin
                                # inside its envelope; keep the desired travel
                                # separate from that already calibrated margin.
                                max_distance_mm=min(policy.next_blind.max_distance_mm,remaining,
                                    expected_distance+policy.next_blind.braking_margin_mm),
                                max_duration_s=min(policy.next_blind.max_duration_s,time_left))

        def check():
            self.check_grab(pending)
            if adaptive is not None and pending.next_observation is not None and blind_owns_motion:
                current_yaw = env.robot.telem.yaw_deg
                if (not math.isfinite(current_yaw) or abs(c._wrap_angle(
                        current_yaw-pending.next_observation.yaw_deg)) > adaptive.max_yaw_change_deg):
                    raise RuntimeError('adaptive blind heading exceeded the observation envelope')

        def command(speed):
            nonlocal last_command
            consume_blind_budget()
            check()
            current_yaw=env.robot.telem.yaw_deg
            if not math.isfinite(current_yaw):
                raise RuntimeError('pickup heading telemetry is invalid')
            heading_error = c._wrap_angle(yaw-current_yaw)
            yaw_speed = max(-c.config.delivery_heading_max_yaw_deg_s,
                            min(c.config.delivery_heading_max_yaw_deg_s,
                                heading_error*c.config.tag6_heading_kp))
            lateral_scale = env.robot.chassis.lateral_distance_scale
            sent = env.robot.chassis.set_speeds(
                env.robot.chassis.mecanum_rpm(0, speed*lateral_scale/10, yaw_speed))
            last_command = speed
            return sent

        def sample():
            nonlocal last_feedback, minimum_progress, minimum_velocity
            check()
            consume_blind_budget()
            _, received_at, _, _ = env.robot.inspection_link_snapshot()
            frame = env.robot.vision_result
            last_feedback = BlindSample(
                pending.session.token, pending.pose, received_at,
                c._measure_lateral_displacement_mm(origin),
                env.robot.chassis.measured_body_velocity().vy_mm_s,
                pending.session.chassis_ready, pending.restored, pending.reset_at,
                getattr(frame,'captured_monotonic',None), getattr(frame,'pose_epoch',None),
                target_visible=any(getattr(b,'color_name','').casefold() == 'orange'
                    and getattr(b,'confidence',0) >= c.config.orange_min_confidence
                    for b in getattr(frame,'all_blocks',())))
            minimum_progress = min(minimum_progress, last_feedback.displacement_mm * blind_profile.direction)
            minimum_velocity = min(minimum_velocity, last_feedback.velocity_mm_s * blind_profile.direction)
            return last_feedback

        def handoff(speed):
            nonlocal acquired, blind_owns_motion
            consume_blind_budget()
            blind_owns_motion = False
            acquired = acquire_after_blind(
                c, initial_speed_mm_s=speed, guard=check, pose_epoch=pending.pose,
                arm_reset_at_s=pending.reset_at, phase_origin=phase_origin,
                profile=policy.next_acquire, heading_yaw_deg=yaw,
                command_speed=command, stop=env.stop,
                clock=time.monotonic, sleep=time.sleep,
                alignment=env.transition_config.alignment(source.profile,'orange'),
                neighbor_observer=operations._neighbor_observer(env, 'orange'))
            return True  # A bounded search failure still accepted ownership.

        result = None
        try:
            with env.robot.chassis.monitor_action(check):
                result = run_blind_transition(
                    blind_profile,token=pending.session.token,epoch=pending.pose,
                    read_sample=sample,command_speed=command,stop=env.stop,
                    guard=check,handoff=handoff,clock=time.monotonic,sleep=time.sleep)
        finally:
            diagnostics = getattr(env.robot, 'diagnostics', None)
            if diagnostics is not None and last_feedback is not None:
                # One event per blind leg, including the raw signed feedback
                # on failure. Diagnostic I/O must not interrupt arm restoration.
                with suppress(Exception):
                    diagnostics.write('blind_feedback', source=source.name,
                        outcome=result.status.value if result is not None else 'fault',
                        reason=result.reason if result is not None else 'exception',
                        requested_distance_mm=expected_distance,
                        duration_s=result.duration_s if result is not None else None,
                        displacement_mm=last_feedback.displacement_mm,
                        velocity_mm_s=last_feedback.velocity_mm_s,
                        minimum_progress_mm=minimum_progress, minimum_velocity_mm_s=minimum_velocity,
                        last_command_mm_s=last_command, lifted=last_feedback.lifted,
                        arm_reset=last_feedback.arm_reset, token=last_feedback.token,
                        epoch=last_feedback.epoch,
                        feedback_position_tolerance_mm=blind_profile.feedback_position_tolerance_mm,
                        feedback_speed_tolerance_mm_s=blind_profile.feedback_speed_tolerance_mm_s)
        consume_blind_budget()
        self.finish_grab(source)
        if acquired:
            env.data['collection']['acquired'] = True
            self.acquisitions[target.name] = True
        env.record_transition('grab_to_next_cube', source.name, result.status.value)

    def route_matches(self, source, target):
        return (self._enabled(source) and transition_enabled(self.env.transition_config, 'purple-departure', source)
                and target.parameters.get('route') == source.parameters.get('followup_route')
                and target.profile == source.profile)

    def depart(self, source, target):
        pending = self.grabs[source.name]
        self.env._enter(target)
        with self.env.robot.chassis.monitor_action(lambda:self.check_grab(pending)):
            self.env.run_route(target.parameters['route'],target.profile)
        self.finish_grab(source)
        self.env.record_transition('grab_to_route',source.name,'joined')

    def last_matches(self, source, target):
        if not self._enabled(source) or not transition_enabled(self.env.transition_config, 'last-departure', source):
            return False
        following = self._next_effective(source)
        # One retreat has one owner: when inspection overlap is requested,
        # reserve it for lowering/counting instead of consuming it beforehand.
        if following is not None and transition_enabled(self.env.transition_config, 'inspect-departure', following):
            return False
        return (following is not None and following.kind == 'inspect_cargo'
                and following.profile == source.profile
                and following.parameters.get('exit_route') in
                    ('ground_delivery_reverse','orange_depart_reverse'))

    def last_depart(self, source, target):
        following = self._next_effective(source)
        pending = self.grabs[source.name]
        chassis = self.env.robot.chassis
        origin = chassis.capture_motor_positions()
        with chassis.monitor_action(lambda:self.check_grab(pending)):
            self.env.run_route(following.parameters['exit_route'],source.profile)
        distance = -chassis.forward_displacement_mm(origin)
        if not math.isfinite(distance) or distance < 0:
            raise RuntimeError('early departure moved in the wrong direction')
        self.finish_grab(source)
        self.early_departure = (source.profile, distance, following.parameters['exit_route'])
        self.env._enter(target)
        self.env.record_transition('last_grab_departure',source.name,'joined')

    def _return_for_refill(self):
        if self.early_departure is None:
            return
        profile, distance, route = self.early_departure
        c = self.env.control(profile)
        speed = (c.config.delivery_reverse_speed_mm_s if route == 'ground_delivery_reverse'
                 else c.config.post_orange_reverse_speed_mm_s)
        if distance > 0:
            c._checked_move('forward',distance,speed)
        c._drive_until_wall(context='Return for verified cargo refill')
        c._recalibrate_heading_zero()
        self.env.robot.reset_vision_filter()
        self.env.data['ground_reverse_done' if route == 'ground_delivery_reverse'
                      else 'orange_reverse_done'] = False
        self.early_departure = None

    def inspect(self, spec):
        env = self.env
        collection = operations._collection(env,spec)
        c = env.control(spec.profile)
        try:
            while not collection['exhausted']:
                env.phase(c,'COUNT_CHECK')
                session = env.robot.begin_carried_cube_inspection(
                    allow_visual_failure=True, allow_idle=collection.get('pickups') == 0)
                self.inspections[spec.name] = session
                route = spec.parameters.get('exit_route')
                overlap = (transition_enabled(env.transition_config,'inspect-departure',spec)
                           and route in ('ground_delivery_reverse','orange_depart_reverse')
                           and self.early_departure is None)

                def retreat():
                    chassis = env.robot.chassis
                    origin = chassis.capture_motor_positions()
                    started = time.monotonic()
                    env.run_route(route,spec.profile)
                    distance = -chassis.forward_displacement_mm(origin)
                    if not math.isfinite(distance) or distance < 0:
                        raise RuntimeError('inspection retreat moved in the wrong direction')
                    self.early_departure = (spec.profile,distance,route)
                    env.robot.diagnostics.write('inspection_retreat',source=spec.name,
                        started_s=started,finished_s=time.monotonic(),distance_mm=distance)

                count = session.inspect(chassis_followup=retreat) if overlap else session.inspect()
                if count is not None and (type(count) is not int or not 0 <= count <= 3):
                    raise RuntimeError(f'invalid carried cube count: {count}')
                env.data['carried_count'] = count
                if spec.parameters.get('cross_region_refill'):
                    from ..refill_policy import checked_count
                    checked_count(count)
                if count is None or count == 3:
                    if count is None and env.data.get('purple_grabbed') is True:
                        raise RuntimeError('mixed cargo count unconfirmed; building is prohibited')
                    self.early_departure = None
                    return count
                self.finish_inspection(spec)
                self._return_for_refill()
                for index in range(1,4-count):
                    refill = replace(spec,kind='acquire_cube',name=f'{spec.name}.refill.{index}',
                                     parameters={'index':index,'method':spec.parameters['method']})
                    if not operations.acquire_cube(env,refill):
                        if collection['exhausted']:
                            from .refill import on_exhausted
                            return on_exhausted(env, spec)
                        return None
                    refill = replace(refill,kind='grab_cube')
                    self.start_grab(refill)
                    self.wait_grab_clear(refill)
                    self.finish_grab(refill)
            from .refill import on_exhausted
            return on_exhausted(env, spec)
        finally:
            c._set_cube_profile('default')

    def finish_inspection(self, spec):
        session = self.inspections.get(spec.name)
        if session is not None:
            session.finish_restore()
            session.close()
            del self.inspections[spec.name]

    def inspect_matches(self, source, target):
        session = self.inspections.get(source.name)
        return (transition_enabled(self.env.transition_config, 'inspect-departure', source)
                and session is not None and session.count in (None,3)
                and source.profile == target.profile
                and source.parameters.get('exit_route') in ('ground_delivery_reverse','orange_depart_reverse')
                and target.parameters.get('route') in ('ground_to_delivery','orange_to_build'))

    def inspect_depart(self, source, target):
        session = self.inspections[source.name]
        self.env._enter(target)
        with self.env.robot.chassis.monitor_action(session.check_restore):
            # Only the calibrated straight retreat overlaps the inspection
            # pose. Turning/curves begin after restoration releases ownership.
            self.env.run_route(source.parameters['exit_route'],source.profile)
        self.finish_inspection(source)
        self.env.run_route(target.parameters['route'],target.profile)
        self.env.record_transition('inspect_to_route',source.name,'joined')

    def registry(self, specs):
        self.specs=tuple(specs)
        by_name={s.name:s for s in specs}
        def item(name,left,right,match,execute,priority=0):
            return Transition(name,left,right,
                lambda a,b,ctx:execute(by_name[a.name],by_name[b.name]),
                matches=lambda a,b,ctx:match(by_name[a.name],by_name[b.name]),priority=priority)
        return (
            item('grab_to_next_cube','grab_cube','acquire_cube',self.next_matches,self.next_cube),
            item('grab_to_route','grab_cube','navigate',self.route_matches,self.depart),
            item('last_grab_to_inspect','grab_cube','inspect_cargo',self.last_matches,self.last_depart,10),
            item('last_grab_skips_unused_slot','grab_cube','acquire_cube',self.last_matches,self.last_depart,10),
            item('inspect_to_route','inspect_cargo','navigate',self.inspect_matches,self.inspect_depart),
        )

    def abort(self):
        for pending in self.grabs.values():
            if pending is not None:
                with suppress(BaseException): pending.session.abort()
        for session in self.inspections.values():
            with suppress(BaseException): session.abort()
        self.grabs.clear()
        self.inspections.clear()
