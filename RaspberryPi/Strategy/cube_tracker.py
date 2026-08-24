"""Target association for multi-cube vision results."""

from __future__ import annotations

import time
from typing import Optional


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


__all__ = ['select_tracked_block']
