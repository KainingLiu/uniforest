"""Ephemeral execution state; persistent world state belongs to strategy."""
from dataclasses import dataclass, field
import math
import threading
import time
from typing import Optional


@dataclass
class ExecutionContext:
    robot: object
    cancel_event: threading.Event = field(default_factory=threading.Event)
    heading_zero_deg: Optional[float] = None
    anchor: str = 'start'
    current_action: str = ''
    closed: bool = False
    _last_uptime: Optional[int] = field(default=None, init=False)
    _link_generation: Optional[int] = field(default=None, init=False)
    _stop_generation: int = field(init=False)

    def __post_init__(self):
        if self.heading_zero_deg is not None and not math.isfinite(self.heading_zero_deg):
            raise ValueError('heading zero must be finite')
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

    def close(self):
        self.closed = True
