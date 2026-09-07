"""Mechanical action client; Grap1/2/3 and Build execute on the A-board."""

import secrets
import threading
import time

from protocol.commands import (
    ACTION_GRAP1, ACTION_GRAP2, ACTION_GRAP3, ACTION_BUILD,
    ACTION_RUNNING, ACTION_DONE, ACTION_CANCELLED,
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


class Actions:
    def __init__(self, servo, stepper, telem_getter=None, transport=None):
        self.servo = servo
        self.stepper = stepper
        self._t = transport if transport is not None else stepper._t
        self._cancel_event = None
        self._action_lock = threading.Lock()

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

    def _run_action(self, action_id, test_mode=False):
        if not self._action_lock.acquire(blocking=False):
            raise RuntimeError('another mechanical action is running')
        try:
            self._check_cancelled()
            # Require a fresh capability response before sending any motion.
            probe_at = time.monotonic()
            self._query()
            while True:
                self._check_cancelled()
                sample = self._t.get_action_status()
                if sample is not None and sample[1] >= probe_at:
                    if sample[0].state == ACTION_RUNNING:
                        raise RuntimeError('A-board is already executing an action')
                    break
                if time.monotonic() - probe_at >= ACTION_START_TIMEOUT_S:
                    raise RuntimeError('A-board action interface unavailable; flash new firmware')
                self._wait(ACTION_POLL_MS)
                self._query()

            token = secrets.randbelow(0xFFFFFFFF) + 1
            while token == sample[0].token:
                token = secrets.randbelow(0xFFFFFFFF) + 1
            self._check_cancelled()
            started = time.monotonic()
            if not self._t.start_action(token, action_id, test_mode):
                raise RuntimeError('failed to send A-board action command')
            last_progress = started
            last_uptime = None
            accepted = False
            while True:
                self._check_cancelled()
                now = time.monotonic()
                sample = self._t.get_action_status()
                if sample is not None:
                    status, received_at = sample
                    if (received_at >= started and status.token == token and
                            status.action_id == action_id):
                        if now - received_at > ACTION_STALE_S:
                            raise RuntimeError('A-board action telemetry stale')
                        if last_uptime is not None:
                            delta = (status.uptime_ms - last_uptime) & 0xFFFFFFFF
                            if delta >= 0x80000000:
                                raise RuntimeError('A-board restarted during action')
                        if status.uptime_ms != last_uptime:
                            last_uptime = status.uptime_ms
                            last_progress = now
                        if now - last_progress > ACTION_STALE_S:
                            raise RuntimeError('A-board action clock stopped')
                        accepted = True
                        if status.state == ACTION_DONE:
                            return
                        if status.state == ACTION_CANCELLED:
                            raise ActionCancelled('A-board cancelled mechanical action')
                        if status.state != ACTION_RUNNING:
                            raise RuntimeError(
                                f'A-board action failed: state={status.state}, stage={status.stage}')
                if not accepted and now - started >= ACTION_START_TIMEOUT_S:
                    raise RuntimeError('A-board did not accept mechanical action')
                if accepted and now - last_progress > ACTION_STALE_S:
                    raise RuntimeError('A-board action telemetry lost')
                if now - started >= ACTION_TIMEOUT_S:
                    raise RuntimeError('A-board mechanical action timed out')
                self._wait(ACTION_POLL_MS)
                self._query()  # Also keeps the 200 ms communication watchdog alive.
        except BaseException:
            self._t.emergency_stop()
            raise
        finally:
            self._action_lock.release()

    def grap1(self, test_mode=False):
        self._run_action(ACTION_GRAP1, test_mode)

    def grap2(self, test_mode=False):
        self._run_action(ACTION_GRAP2, test_mode)

    def grap3(self, test_mode=False):
        self._run_action(ACTION_GRAP3, test_mode)

    def build(self):
        self._run_action(ACTION_BUILD)

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

    def hatch_close(self):
        self._check_cancelled()
        self.servo.set_angle(2, ANGLE_HATCH_A_CLOSED)
        self.servo.set_angle(3, ANGLE_HATCH_B_CLOSED)
        self._wait(500)
