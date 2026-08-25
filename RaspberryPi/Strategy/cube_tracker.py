"""Target association for multi-cube vision results.

The detector publishes independent per-frame candidates.  This module owns
the target lifecycle used by motion control: acquire, lock, reject jumps,
smooth the locked position, and release only after repeated misses.
"""

from __future__ import annotations

import copy
import time
from collections import deque
from typing import Optional


class CubeTargetTracker:
    """Track one colored cube across frames without switching candidates."""

    def __init__(self, *, max_x_jump_mm: float, max_z_jump_mm: float,
                 confirm_frames: int = 2, lost_frames: int = 3,
                 smoothing_frames: int = 3, ambiguity_margin_mm: float = 0.0):
        self.max_x_jump_mm = max_x_jump_mm
        self.max_z_jump_mm = max_z_jump_mm
        self.confirm_frames = max(1, confirm_frames)
        self.lost_frames = max(1, lost_frames)
        self.ambiguity_margin_mm = max(0.0, ambiguity_margin_mm)
        self._history = deque(maxlen=max(1, smoothing_frames))
        self.reset()

    def reset(self):
        self._locked_x = None
        self._locked_z = None
        self._stable_x = None
        self._stable_z = None
        self._pending = 0
        self._misses = 0
        self._last_timestamp = None

    @property
    def locked(self) -> bool:
        return self._locked_x is not None

    def _copy_with_stable_position(self, block):
        if self._stable_x is None:
            return block
        # Do not mutate detector-owned snapshots.  The copy keeps metadata and
        # quad coordinates while exposing the filtered control position.
        filtered = copy.copy(block)
        filtered.x = self._stable_x
        if self._stable_z is not None:
            filtered.z = self._stable_z
        return filtered

    def update(self, result, *, color_name: str, min_confidence: float,
               max_age_s: float, reference_x: Optional[float] = None,
               reference_z: Optional[float] = None):
        """Return the stable target for a new result, or ``None``.

        A result timestamp is consumed once.  During a short visual dropout
        no fake position is returned to the controller, but the target lock is
        retained so a new nearest candidate cannot replace it immediately.
        """
        if result is None or time.time() - result.timestamp > max_age_s:
            self._misses += 1
            if self._misses >= self.lost_frames:
                self.reset()
            return None
        if result.timestamp == self._last_timestamp:
            return None
        self._last_timestamp = result.timestamp

        candidates = [
            block for block in result.all_blocks
            if block.color_name.casefold() == color_name.casefold()
            and block.confidence >= min_confidence
        ]
        if self._locked_x is not None:
            candidates = [
                block for block in candidates
                if abs(block.x - self._locked_x) <= self.max_x_jump_mm
                and (self._locked_z is None or
                     abs(block.z - self._locked_z) <= self.max_z_jump_mm)
            ]
        elif reference_x is not None:
            candidates = [
                block for block in candidates
                if abs(block.x - reference_x) <= self.max_x_jump_mm
                and (reference_z is None or
                     abs(block.z - reference_z) <= self.max_z_jump_mm)
            ]
        if not candidates:
            self._misses += 1
            if self._misses >= self.lost_frames:
                self.reset()
            return None

        anchor_x = self._locked_x if self._locked_x is not None else reference_x
        anchor_z = self._locked_z if self._locked_z is not None else reference_z
        def score(block):
            if anchor_x is None and anchor_z is None:
                return block.x ** 2 + block.y ** 2 + block.z ** 2
            dx = abs(block.x - anchor_x) if anchor_x is not None else 0.0
            dz = (abs(block.z - anchor_z) if anchor_z is not None else 0.0)
            return dx + 0.5 * dz

        ranked = sorted(candidates, key=score)
        if (len(ranked) > 1 and self.ambiguity_margin_mm > 0.0
                and score(ranked[1]) - score(ranked[0])
                < self.ambiguity_margin_mm):
            self._misses += 1
            return None

        block = ranked[0]
        self._misses = 0
        if self._locked_x is None:
            self._locked_x = block.x
            self._locked_z = block.z
            self._stable_x = block.x
            self._stable_z = block.z
            self._history.clear()
            self._history.append((block.x, block.z))
            self._pending += 1
            if self._pending < self.confirm_frames:
                return None
            return block

        self._pending = self.confirm_frames
        self._locked_x = block.x
        self._locked_z = block.z
        self._history.append((block.x, block.z))
        self._stable_x = sum(x for x, _ in self._history) / len(self._history)
        self._stable_z = sum(z for _, z in self._history) / len(self._history)
        return self._copy_with_stable_position(block)


def select_tracked_block(result, color_name: str, reference_x: float,
                         min_confidence: float, cfg,
                         reference_z: Optional[float] = None,
                         ambiguity_margin_mm: Optional[float] = None):
    """Select the same cube, refusing stale, jumping, or ambiguous candidates."""
    if result is None or time.time() - result.timestamp > cfg.vision_stale_s:
        return None
    candidates = [
        block for block in result.all_blocks
        if block.color_name.casefold() == color_name.casefold()
        and block.confidence >= min_confidence
        and abs(block.x - reference_x) <= cfg.align_track_max_x_jump_mm
        and (reference_z is None
             or abs(block.z - reference_z) <= cfg.align_track_max_z_jump_mm)
    ]
    if not candidates:
        return None
    margin = (cfg.align_track_ambiguity_margin_mm
              if ambiguity_margin_mm is None else ambiguity_margin_mm)
    scored = sorted(
        ((abs(block.x - reference_x)
          + (0.0 if reference_z is None
             else 0.5 * abs(block.z - reference_z)), block)
         for block in candidates),
        key=lambda item: item[0])
    if len(scored) > 1 and margin > 0.0:
        if scored[1][0] - scored[0][0] < margin:
            return None
    return scored[0][1]


__all__ = ['CubeTargetTracker', 'select_tracked_block']
