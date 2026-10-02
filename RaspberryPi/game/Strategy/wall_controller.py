"""State helpers for wall-contact controllers."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class StallConfirmation:
    """Count contact time, pausing across bounded feedback fluctuations."""

    started_at: float | None = None
    _last_stalled_at: float | None = None
    _dropout_at: float | None = None
    _confirmed_s: float = 0.0

    def reset(self):
        self.started_at = None
        self._last_stalled_at = None
        self._dropout_at = None
        self._confirmed_s = 0.0

    def update(self, stalled: bool, now: float, confirm_s: float, *,
               dropout_s: float = 0.0) -> bool:
        if not stalled:
            if dropout_s <= 0.0:
                self.reset()
            elif self.started_at is not None:
                if self._dropout_at is None:
                    self._dropout_at = now
                elif now - self._dropout_at > dropout_s:
                    self.reset()
            return False

        if self._dropout_at is not None:
            if now - self._dropout_at > dropout_s:
                self.reset()
            # Time across a failed sample does not count as wall contact.
        elif self._last_stalled_at is not None:
            self._confirmed_s += now - self._last_stalled_at
        if self.started_at is None:
            self.started_at = now
        self._last_stalled_at = now
        self._dropout_at = None
        return self._confirmed_s >= confirm_s


__all__ = ['StallConfirmation']
