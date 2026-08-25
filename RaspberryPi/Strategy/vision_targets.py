"""Quick calibration entry point for cube visual alignment targets.

Edit this file after a camera calibration run. Values are camera-relative
millimetres and the ranges are absolute X windows, not offsets.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class CubeVisionTarget:
    target_x_mm: float
    align_min_x_mm: float
    align_max_x_mm: float
    fine_min_x_mm: float
    fine_max_x_mm: float


TASK1_ORANGE = CubeVisionTarget(0.0, -20.0, 5.0, -3.0, 3.0)
TASK2_PURPLE = CubeVisionTarget(0.0, -5.0, 5.0, -3.0, 3.0)
TASK2_ORANGE = CubeVisionTarget(0.0, -20.0, 5.0, -3.0, 3.0)
