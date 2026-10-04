"""One execution's hardware guard and explicit Task2 -> Task3/Task4 handoff."""

from dataclasses import dataclass, field
import math
import threading
import time
from typing import Optional


@dataclass(frozen=True)
class BuildApproach:
    """Tag6 approach at 180 degrees after Task2 or the Task1-0 refill exit."""
    heading_zero_deg: float
    source_task: str
    # None preserves the normal three-cube build; a refill exit may supply 0..3.
    carried_cube_count: Optional[int] = None

    def __post_init__(self):
        if not math.isfinite(self.heading_zero_deg):
            raise ValueError('heading zero must be finite')
        if (self.carried_cube_count is not None and
                (type(self.carried_cube_count) is not int or self.carried_cube_count not in range(4))):
            raise ValueError('handoff carried cube count must be None or an integer from 0 to 3')


@dataclass
class TaskContext:
    robot: object
    cancel_event: threading.Event = field(default_factory=threading.Event)
    build_approach: Optional[BuildApproach] = None
    current_task: str = ''
    closed: bool = False
    _last_uptime: Optional[int] = field(default=None, init=False)
    _link_generation: Optional[int] = field(default=None, init=False)
    _stop_generation: int = field(init=False)

    def __post_init__(self):
        self._stop_generation = self.robot.transport.emergency_stop_generation

    def check_active(self, *, require_telemetry=True):
        """A stopped, disconnected or stale run cannot start another step."""
        transport = self.robot.transport
        if self.closed or self.cancel_event.is_set():
            raise RuntimeError('strategy cancelled or already closed')
        if transport.emergency_stop_generation != self._stop_generation:
            raise RuntimeError('strategy interrupted by emergency stop')
        if not transport.connected:
            raise RuntimeError('strategy communication lost')
        telem, received_at, _, link_generation = self.robot.inspection_link_snapshot()
        if telem is None or received_at is None:
            if require_telemetry:
                raise RuntimeError('strategy telemetry unavailable')
            return
        if time.monotonic() - received_at > 0.5:
            raise RuntimeError('strategy telemetry stale')
        if self._link_generation is not None and link_generation != self._link_generation:
            raise RuntimeError('strategy link continuity lost; do not resume after reconnect')
        self._link_generation = link_generation
        if (self._last_uptime is not None
                and ((telem.uptime_ms - self._last_uptime) & 0xffffffff) >= 0x80000000):
            raise RuntimeError('A-board restarted during strategy')
        self._last_uptime = telem.uptime_ms

    def publish_build_approach(self, heading_zero_deg, *, carried_cube_count=None):
        self.check_active()
        self.build_approach = BuildApproach(heading_zero_deg, self.current_task,
                                           carried_cube_count)

    def take_build_approach(self):
        self.check_active()
        if self.build_approach is None:
            raise RuntimeError('Task3/Task4 needs the Task2 exit heading handoff')
        handoff, self.build_approach = self.build_approach, None
        return handoff

    def close(self):
        self.build_approach = None
        self.closed = True
