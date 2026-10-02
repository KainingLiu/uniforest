"""Inspect storage using the running Robot camera/link, then restore the arm."""

import json
from pathlib import Path
import time

from protocol.commands import ACTION_DONE, ACTION_IDLE
from vision.carried_cube_count import observe, classify

CONFIG_PATH = Path(__file__).resolve().parents[1] / 'tools/carried_cube_count_config.json'


class InspectionVisionUnavailable(RuntimeError):
    """No current images are available; link and mechanism failures differ."""


class CarriedInspectionSession:
    """Own inspection and arm restoration across a chassis handoff.

    ``inspect`` returns the count after initiating the existing return sequence.
    The caller keeps this session alive while a route calls ``check_restore``.
    ``finish_restore`` then ``close`` are required before another mechanism
    command. Restoration means commands/timing completed; no angle sensor is
    available to confirm the physical arm or camera pose.
    """

    def __init__(self, robot, config, *, allow_visual_failure=False, allow_idle=False):
        if type(allow_idle) is not bool:
            raise ValueError('allow_idle must be an explicit boolean')
        self.robot, self.config = robot, config
        self.transport, self.actions = robot.transport, robot.actions
        self.allow_visual_failure = allow_visual_failure
        # Only a caller with zero completed pickups may opt in. IDLE still
        # needs the same three fresh, stationary telemetry samples as DONE.
        self.allow_idle = allow_idle
        self._generation = self.transport.emergency_stop_generation
        self._link_generation = robot.inspection_link_snapshot()[3]
        self._owns_lock = False
        self.closed = self.inspected = self.restored = False
        self.count = None
        self.result = None
        self.observations = []
        self._restore_at = None
        self._abort_succeeded = True
        self.cleanup_error = None

    def _check(self):
        if self.closed:
            raise RuntimeError('carried-count inspection session is closed')
        self.actions._check_cancelled()
        telem, received_at, pong_at, epoch = self.robot.inspection_link_snapshot()
        now = time.monotonic()
        if (not self.transport.connected or not self.robot._running
                or self.transport.emergency_stop_generation != self._generation
                or epoch != self._link_generation or telem is None
                or received_at is None or pong_at is None
                or now - received_at > .15 or now - pong_at > .15):
            raise RuntimeError('carried-count inspection communication/cancellation fault')
        return telem

    def _wait(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._check()
            time.sleep(min(.005, max(0, deadline - time.monotonic())))
        self._check()

    def _servo(self, servo_id, angle):
        self.transport.set_servo_angle_checked(servo_id, angle, self._check)

    def inspect(self, *, chassis_followup=None):
        """Count from fresh raw frames and begin restoration without joining it."""
        try:
            self._check()
            if self.inspected:
                raise RuntimeError('carried-count inspection already performed')
            return self._inspect(chassis_followup=chassis_followup)
        except BaseException:
            self.abort()
            raise

    def _inspect(self, *, chassis_followup=None):
        robot, transport, config = self.robot, self.transport, self.config
        check, wait, servo = self._check, self._wait, self._servo
        allow_visual_failure = self.allow_visual_failure
        check()
        probe_at = time.monotonic()
        if not transport.query_action_status():
            raise RuntimeError('inspection action status query failed')
        while True:
            check()
            status = transport.get_action_status()
            if status is not None and status[1] >= probe_at:
                if status[0].state != ACTION_DONE and not (self.allow_idle and status[0].state == ACTION_IDLE):
                    raise RuntimeError(f'A-board action not complete before inspection: {status[0].state}')
                break
            if time.monotonic() - probe_at > .15:
                raise RuntimeError('inspection action status unavailable')
            wait(.005)
        # ACTION_DONE and full telemetry are separate messages. Do not reject
        # the last pre-completion telemetry packet as an active mechanism.
        stopped_at = time.monotonic()
        if not robot.chassis.set_speeds([0, 0, 0, 0]):
            raise RuntimeError('inspection chassis zero-speed command failed')
        last_uptime = None
        confirmed = 0
        initial = check()
        print('[Count] Waiting for stationary telemetry: '
              f'stepper_busy={initial.stepper_busy}, '
              f'rpm={[m.speed_rpm for m in initial.motors]}', flush=True)
        while True:
            check()
            telem, received_at, _, _ = robot.inspection_link_snapshot()
            rpm = [m.speed_rpm for m in telem.motors]
            after_action = ((telem.uptime_ms - status[0].uptime_ms) & 0xffffffff) < 0x80000000
            if received_at >= stopped_at and after_action and telem.uptime_ms != last_uptime:
                last_uptime = telem.uptime_ms
                quiet = not telem.stepper_busy and all(abs(speed) <= 10 for speed in rpm)
                confirmed = confirmed + 1 if quiet else 0
                if confirmed >= 3:
                    break
            if time.monotonic() - stopped_at >= 1.0:
                robot.diagnostics.write('carried_count_stationary_timeout',
                                        stepper_busy=telem.stepper_busy, rpm=rpm,
                                        fresh_after_action=after_action,
                                        confirmed_frames=confirmed)
                raise RuntimeError(
                    'carried-count inspection stationary timeout (1s): '
                    f'stepper_busy={telem.stepper_busy}, rpm={rpm}, '
                    f'fresh_after_action={after_action}, confirmed={confirmed}/3')
            wait(.005)
        print(f'[Count] Stationary confirmed in {time.monotonic() - stopped_at:.3f}s',
              flush=True)
        pose_started = False
        observations = []
        try:
            # Validate configuration and a live frame before any pose command.
            sample = robot.cube_raw_frame
            if sample is None or not 0 <= time.monotonic() - sample[1] <= .5:
                raise InspectionVisionUnavailable('inspection camera frame unavailable')
            classify([observe(sample[0], config)[0]], config)
            servo(0, 37.2)
            pose_started = True
            lowering_started = time.monotonic()
            lower_at = lowering_started + .200
            lower_sent = False
            ready_at = None
            first_after = last_timestamp = None
            skipped = 0
            moving_frames = 0

            def advance_lowering_and_count():
                nonlocal lower_sent, ready_at, first_after, last_timestamp, skipped, moving_frames
                check()
                now = time.monotonic()
                if not lower_sent and now >= lower_at:
                    lower_sent = True
                    servo(1, 120)
                    ready_at = time.monotonic() + .300
                    robot.diagnostics.write('inspection_head_down',at_s=time.monotonic())
                if ready_at is None or now < ready_at or len(observations) >= 8:
                    return
                if first_after is None:
                    first_after = last_timestamp = now
                if now - first_after > 4:
                    raise InspectionVisionUnavailable('inspection fresh camera frames timed out')
                sample = robot.cube_raw_frame
                if sample is not None and sample[1] > last_timestamp:
                    frame, last_timestamp = sample
                    if skipped < 3:
                        skipped += 1
                    else:
                        observations.append(observe(frame, config)[0])
                        telem = check()
                        moving_frames += any(abs(m.speed_rpm) > 10 for m in telem.motors)

            if chassis_followup is not None:
                robot.diagnostics.write('inspection_overlap_start',at_s=lowering_started)
                # The route owns chassis motion. Its existing cooperative
                # monitor advances servo deadlines and frame collection in
                # the same thread, without a competing motor writer.
                try:
                    with robot.chassis.monitor_action(advance_lowering_and_count):
                        chassis_followup()
                except BaseException:
                    # A visual fault inside a moving route must never be
                    # swallowed as a count fallback while a setpoint is live.
                    self.abort()
                    raise
                robot.diagnostics.write('inspection_overlap_retreat_done',at_s=time.monotonic())
            while len(observations) < 8:
                advance_lowering_and_count()
                wait(.005)
            robot.diagnostics.write('inspection_sampling',started_s=first_after,
                finished_s=time.monotonic(),frames=len(observations),moving_frames=moving_frames,
                overlapping_retreat=chassis_followup is not None)
            result = classify(observations, config)
        except InspectionVisionUnavailable as exc:
            if not allow_visual_failure:
                raise
            check()
            result = {'count': None, 'reason': str(exc), 'visual_fallback': True}
            robot.diagnostics.write('visual_fallback', stage='carried count', reason=str(exc))
            print(f'[Count] {exc}; count unknown, continue mission', flush=True)
        check()
        count = result['count']
        if count is not None and (type(count) is not int or not 0 <= count <= 3):
            raise RuntimeError(f'invalid carried cube count: {count}')
        if pose_started:
            servo(1, 90)
        self._restore_at = time.monotonic() + .200
        self.restored = not pose_started
        self.count, self.result, self.observations = count, result, observations
        self.inspected = True
        return count

    def check_restore(self):
        """Guard the route and advance timed restoration without blocking."""
        try:
            self._check()
            if not self.inspected:
                raise RuntimeError('inspect must complete before arm restoration')
            if not self.restored and time.monotonic() >= self._restore_at:
                self._servo(0, 97.2)
                self._check()
                self.robot.reset_vision_filter(after_inspection=True)
                self.restored = True
            return self.restored
        except BaseException:
            self.abort()
            raise

    def finish_restore(self):
        """Wait for restoration; retain mechanism ownership until close()."""
        try:
            while not self.check_restore():
                self._wait(.005)
            self._check()
        except BaseException:
            self.abort()
            raise

    def close(self):
        """Release completed inspection; interrupt an unfinished session."""
        if self.closed:
            return
        if not self.inspected or not self.restored:
            self.abort()
            return
        try:
            self._check()
            self.robot.diagnostics.write(
                'carried_cube_count', result=self.result,
                observations=self.observations,
                feature_version=self.config['feature_version'])
            print(f'[Count] {self.result}', flush=True)
        except BaseException:
            self.abort()
            raise
        self.closed = True
        self._release()

    def _release(self):
        if self._owns_lock:
            self._owns_lock = False
            self.actions._action_lock.release()

    def abort(self):
        """Stop and release once, with no subsequent servo commands."""
        if self.closed:
            return self._abort_succeeded
        self.closed = True
        try:
            self._abort_succeeded = self.transport.emergency_stop() is not False
        except BaseException as exc:
            self.cleanup_error = exc
            self._abort_succeeded = False
        finally:
            self._release()
        return self._abort_succeeded

    def __enter__(self):
        try:
            self._check()
        except BaseException:
            self.abort()
            raise
        return self

    def __exit__(self, exc_type, exc, traceback):
        if exc_type is not None:
            self.abort()
        else:
            self.close()
        return False


def begin_carried_inspection(robot, *, allow_visual_failure=False, allow_idle=False):
    """Acquire mechanism ownership for a phased inspection; no pose move yet."""
    config = json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
    session = CarriedInspectionSession(robot, config,
                                       allow_visual_failure=allow_visual_failure, allow_idle=allow_idle)
    if not robot.actions._action_lock.acquire(blocking=False):
        raise RuntimeError('another mechanical action is running')
    session._owns_lock = True
    try:
        session._check()
    except BaseException:
        session.abort()
        raise
    return session


def inspect_carried_cubes(robot, *, chassis_followup=None, allow_visual_failure=False, allow_idle=False):
    """Legacy blocking API using the same phased session and guarded restore.

    Full/unknown counts may overlap restoration with a chassis-only callback.
    Partial counts finish restoration before returning for another pickup.
    """
    session = begin_carried_inspection(robot, allow_visual_failure=allow_visual_failure, allow_idle=allow_idle)
    try:
        count = session.inspect()
        if chassis_followup is not None and count in (None, 3):
            with robot.chassis.monitor_action(session.check_restore):
                chassis_followup()
        session.finish_restore()
        session.close()
        return count
    except BaseException:
        session.abort()
        raise
