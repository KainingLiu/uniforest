"""State helpers for wall-contact controllers."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class StallConfirmation:
    """Tracks a continuously stalled interval without hardware knowledge."""

    started_at: float | None = None

    def reset(self):
        self.started_at = None

    def update(self, stalled: bool, now: float, confirm_s: float) -> bool:
        if not stalled:
            self.started_at = None
            return False
        if self.started_at is None:
            self.started_at = now
        return now - self.started_at >= confirm_s


__all__ = ['StallConfirmation']
