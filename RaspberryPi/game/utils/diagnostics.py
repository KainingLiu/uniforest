"""Small dependency-free diagnostics helpers for competition runs."""

from __future__ import annotations

import json
import os
import time
import threading
from typing import Any, Optional

from .run_logs import current_archive


def classify_failure(exc: BaseException) -> str:
    """Map common runtime failures to an actionable category."""
    message = str(exc).casefold()
    hardware_terms = ('serial', 'telemetry', 'communication', 'emergency stop',
                      'pong', 'uart', 'crc')
    perception_terms = ('visual', 'vision', 'tag ', 'tag_', 'cube',
                        'building', 'target lost', 'not visible')
    degraded_terms = ('timeout accepted', 'progress 90%', 'timed out after')
    if any(term in message for term in hardware_terms):
        return 'hardware_fault'
    if any(term in message for term in perception_terms):
        return 'perception_fault'
    if any(term in message for term in degraded_terms):
        return 'motion_degraded'
    return 'strategy_fault'


class JsonlDiagnostics:
    """Write to this run's archive and, when requested, an additional path."""

    def __init__(self, path: Optional[str] = None):
        self.path = path
        self.archive = current_archive()
        self.lock = threading.Lock()

    def write(self, event: str, **fields: Any) -> None:
        if not self.path and self.archive is None:
            return
        record = {'ts': time.time(), 'event': event, **fields}
        line = json.dumps(record, ensure_ascii=True, separators=(',', ':')) + '\n'
        with self.lock:
            if self.archive is not None:
                self.archive.diagnostic(line)
            if self.path and (self.archive is None or os.path.abspath(self.path) != str(self.archive.diagnostics_path)):
                parent = os.path.dirname(os.path.abspath(self.path))
                os.makedirs(parent, exist_ok=True)
                with open(self.path, 'a', encoding='utf-8') as stream:
                    stream.write(line)


__all__ = ['JsonlDiagnostics', 'classify_failure']
