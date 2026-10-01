"""Mechanical action client; Grap1/2/3 and Build execute on the A-board."""

import secrets
import threading
import time

from protocol.commands import (
    ACTION_GRAP1, ACTION_GRAP2, ACTION_GRAP3, ACTION_BUILD,
    ACTION_RUNNING, ACTION_DONE, ACTION_CANCELLED, ACTION_CHASSIS_READY,
)
from .servo import (
    ANGLE_HATCH_A_OPEN, ANGLE_HATCH_B_OPEN,
    ANGLE_HATCH_A_CLOSED, ANGLE_HATCH_B_CLOSED,
)

ACTION_POLL_MS = 50
ACTION_START_TIMEOUT_S = 1.0
ACTION_STALE_S = 0.5
ACTION_TIMEOUT_S = 125.0


class ActionCancelled(RuntimeError):
    """Raised when an operator or the A-board cancels a mechanical action."""


class ActionSession:
    """Cooperatively supervised firmware action, created by :meth:`Actions.begin`.

    The Actions lock remains owned until close. Call check from every companion
    movement loop, including after the mechanism reports DONE. No background
    thread issues motion or resumes a failed session.
    """

    def __init__(self, actions, action_id, test_mode=False):
        self._actions = actions
        self._t = actions._t
        self.action_id = action_id
        self.test_mode = test_mode
        self.token = None
        # Capture before probing/sending: an emergency stop during start_action
        # must invalidate this session even when the send itself succeeds.
        self._stop_generation = self._t.emergency_stop_generation
        self._started = 0.0
        self._last_progress = 0.0
        self._last_uptime = None
        self._last_query = 0.0
        self._accepted = False
        self._done = False
        self._chassis_ready = False
        self._closed = False
        self.cleanup_error = None
        self._abort_succeeded = True
        self._pickup_full_lift_validated = actions.pickup_full_lift_validated

    @property
    def chassis_ready(self):
        return self._chassis_ready and not self._closed

    @property
    def done(self):
        return self._done

    @property
    def closed(self):
        return self._closed

    def _check_live(self):
        self._actions._check_cancelled()
        if self._t.emergency_stop_generation != self._stop_generation:
            raise ActionCancelled('emergency stop during mechanical action/route')
        if not getattr(self._t, 'connected', True):
            raise RuntimeError('A-board disconnected during mechanical action/route')

    def _start(self):
        self._check_live()
        probe_at = time.monotonic()
        self._actions._query()
        while True:
            self._check_live()
            sample = self._t.get_action_status()
            if sample is not None and sample[1] >= probe_at:
                if sample[0].state in (ACTION_RUNNING, ACTION_CHASSIS_READY):
                    raise RuntimeError('A-board is already executing an action')
                break
            if time.monotonic() - probe_at >= ACTION_START_TIMEOUT_S:
                raise RuntimeError('A-board action interface unavailable; flash new firmware')
            self._actions._wait(ACTION_POLL_MS)
            self._check_live()
            self._actions._query()

        token = secrets.randbelow(0xFFFFFFFF) + 1
        while token == sample[0].token:
            token = secrets.randbelow(0xFFFFFFFF) + 1
        self.token = token
        self._last_uptime = sample[0].uptime_ms
        self._check_live()
        self._started = self._last_progress = time.monotonic()
        if not self._t.start_action(token, self.action_id, self.test_mode):
            raise RuntimeError('failed to send A-board action command')
        self._check_live()

    def check(self):
        """Poll status and reject cancellation, link loss, restart or stale data.

        Any failure immediately stops and closes this session. Reusing a closed
        session raises; callers must explicitly start a new execution afterward.
        """
        if self._closed:
            raise RuntimeError('mechanical action session is closed')
        try:
            self._check_live()
            now = time.monotonic()
            if now - self._last_query >= ACTION_POLL_MS / 1000.0:
                self._actions._query()
                self._last_query = now
            self._check_live()
            sample = self._t.get_action_status()
            if sample is not None:
                status, received_at = sample
                if (received_at >= self._started and status.token == self.token
                        and status.action_id == self.action_id):
                    if now - received_at > ACTION_STALE_S:
                        raise RuntimeError('A-board action telemetry stale')
                    if self._last_uptime is not None:
                        delta = (status.uptime_ms - self._last_uptime) & 0xFFFFFFFF
                        if delta >= 0x80000000:
                            raise RuntimeError('A-board restarted during action')
                    if status.uptime_ms != self._last_uptime:
                        self._last_uptime = status.uptime_ms
                        self._last_progress = now
                    if now - self._last_progress > ACTION_STALE_S:
                        raise RuntimeError('A-board action clock stopped')
                    self._accepted = True
                    if status.state == ACTION_CANCELLED:
                        raise ActionCancelled('A-board cancelled mechanical action')
                    if status.state not in (ACTION_RUNNING, ACTION_CHASSIS_READY, ACTION_DONE):
                        raise RuntimeError(
                            f'A-board action failed: state={status.state}, stage={status.stage}')
                    if self._done and status.state != ACTION_DONE:
                        raise RuntimeError('A-board action state regressed after completion')
                    self._done = status.state == ACTION_DONE
                    # Older flashed Grap2 firmware used state 6 after only a
                    # partial lift. Wire compatibility alone cannot establish
                    # full block clearance for any pickup action.
                    pickup = self.action_id in (ACTION_GRAP1, ACTION_GRAP2, ACTION_GRAP3)
                    released = (status.state == ACTION_DONE or
                                (status.state == ACTION_CHASSIS_READY and
                                 (not pickup or self._pickup_full_lift_validated)))
                    self._chassis_ready = self._chassis_ready or released
                elif self._accepted and received_at >= self._started:
                    raise RuntimeError('A-board action token or identity changed')
            if not self._accepted and now - self._started >= ACTION_START_TIMEOUT_S:
                raise RuntimeError('A-board did not accept mechanical action')
            if self._accepted and now - self._last_progress > ACTION_STALE_S:
                raise RuntimeError('A-board action telemetry lost')
            if not self._done and now - self._started >= ACTION_TIMEOUT_S:
                raise RuntimeError('A-board mechanical action timed out')
        except BaseException:
            self._abort_and_close()
            raise

    def _wait_until(self, milestone, parallel_step=None):
        parallel_done = parallel_step is None
        try:
            while True:
                self.check()
                if not parallel_done:
                    parallel_done = bool(parallel_step())
                    self.check()
                if milestone() and parallel_done:
                    return
                self._actions._wait(ACTION_POLL_MS)
        except BaseException:
            self._abort_and_close()
            raise

    def wait_chassis_ready(self, parallel_step=None):
        """Wait for the enabled clearance milestone and companion work.

        Firmware reports sequence/pulse progress, not a physical clearance
        sensor; pickup overlap requires the explicitly validated capability.
        """
        self._wait_until(lambda: self._chassis_ready, parallel_step)

    def wait_done(self, parallel_step=None):
        """Wait for mechanism completion and optional nonblocking companion."""
        self._wait_until(lambda: self._done, parallel_step)

    def run_followup(self, route_check_callback):
        """Run callback(check) after chassis clearance, retaining action ownership."""
        try:
            self.check()
            if not self._chassis_ready:
                raise RuntimeError('mechanical action has not released the chassis')
            result = route_check_callback(self.check)
            self.check()
            return result
        except BaseException:
            self._abort_and_close()
            raise

    def abort(self):
        """Best-effort stop and release; repeated cleanup never masks a fault.

        Returns whether the stop request succeeded. A transport exception is
        retained in cleanup_error for diagnostics. This reports a send outcome,
        with no claim that physical stopping has been confirmed.
        """
        if self._closed:
            return self._abort_succeeded
        self._closed = True
        try:
            self._abort_succeeded = self._t.emergency_stop() is not False
        except BaseException as exc:
            self.cleanup_error = exc
            self._abort_succeeded = False
        finally:
            self._actions._action_lock.release()
        return self._abort_succeeded

    def _abort_and_close(self):
        return self.abort()

    def close(self):
        """Release a finished action; emergency-stop any unfinished action."""
        if self._closed:
            return
        if not self._done:
            self._abort_and_close()
            return
        self.check()
        self._closed = True
        self._actions._action_lock.release()

    def __enter__(self):
        self.check()
        return self

    def __exit__(self, exc_type, exc, traceback):
        if exc_type is not None:
            self._abort_and_close()
        else:
            self.close()
        return False


class Actions:
    def __init__(self, servo, stepper, telem_getter=None, transport=None, *,
                 pickup_full_lift_validated=False):
        if not isinstance(pickup_full_lift_validated, bool):
            raise TypeError('pickup_full_lift_validated must be an explicit bool')
        self.servo = servo
        self.stepper = stepper
        self._t = transport if transport is not None else stepper._t
        self._cancel_event = None
        self._action_lock = threading.Lock()
        # Enable only after matching firmware and physical full-lift clearance
        # have been validated. Query/action status does not negotiate this.
        self._pickup_full_lift_validated = pickup_full_lift_validated

    @property
    def pickup_full_lift_validated(self):
        return self._pickup_full_lift_validated

    def set_cancel_event(self, event):
        self._cancel_event = event

    def _check_cancelled(self):
        if self._cancel_event is not None and self._cancel_event.is_set():
            raise ActionCancelled('mechanical action cancelled')

    def _wait(self, ms):
        if self._cancel_event is None:
            time.sleep(ms / 1000.0)
        elif self._cancel_event.wait(ms / 1000.0):
            raise ActionCancelled('mechanical action cancelled')

    def _query(self):
        if not self._t.query_action_status():
            raise RuntimeError('failed to query A-board action status')

    def begin(self, action_id, test_mode=False):
        """Start one supervised action; caller must close the returned session."""
        if not self._action_lock.acquire(blocking=False):
            raise RuntimeError('another mechanical action is running')
        try:
            session = ActionSession(self, action_id, test_mode)
        except BaseException:
            self._action_lock.release()
            raise
        try:
            session._start()
        except BaseException:
            session._abort_and_close()
            raise
        return session

    def _run_action(self, action_id, test_mode=False, *, parallel_step=None,
                    chassis_followup=None):
        """Compatibility wrapper using the same supervisor as phased execution.

        parallel_step must be nonblocking. chassis_followup(check) must call
        check in its movement loop. Ownership lasts through all companion work.
        """
        with self.begin(action_id, test_mode) as session:
            if chassis_followup is not None:
                session.wait_chassis_ready(parallel_step)
                session.run_followup(chassis_followup)
                session.wait_done()
            else:
                session.wait_done(parallel_step)

    def grap1(self, test_mode=False, *, parallel_step=None, chassis_followup=None):
        self._run_action(ACTION_GRAP1, test_mode, parallel_step=parallel_step,
                         chassis_followup=chassis_followup)

    def grap2(self, test_mode=False, *, parallel_step=None, chassis_followup=None):
        self._run_action(ACTION_GRAP2, test_mode, parallel_step=parallel_step,
                         chassis_followup=chassis_followup)

    def grap3(self, test_mode=False, *, parallel_step=None, chassis_followup=None):
        self._run_action(ACTION_GRAP3, test_mode, parallel_step=parallel_step,
                         chassis_followup=chassis_followup)

    def build(self, *, chassis_followup=None):
        self._run_action(ACTION_BUILD, chassis_followup=chassis_followup)

    def servo_home(self, settle_ms=300):
        self._check_cancelled()
        self.servo.home_all()
        if settle_ms > 0:
            self._wait(settle_ms)

    def hatch_open(self, settle_ms=500):
        self._check_cancelled()
        self.servo.set_angle(2, ANGLE_HATCH_A_OPEN)
        self.servo.set_angle(3, ANGLE_HATCH_B_OPEN)
        if settle_ms > 0:
            self._wait(settle_ms)

    def hatch_close(self, settle_ms=500):
        self._check_cancelled()
        self.servo.set_angle(2, ANGLE_HATCH_A_CLOSED)
        self.servo.set_angle(3, ANGLE_HATCH_B_CLOSED)
        if settle_ms > 0:
            self._wait(settle_ms)
