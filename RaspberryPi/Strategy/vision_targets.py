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


TASK1_ORANGE = CubeVisionTarget(-1.0, -21.0, 2.0, -3.5, 1.5)
TASK2_PURPLE = CubeVisionTarget(-0.5, -5.0, 5.0, -3.5, 1.5)
TASK2_ORANGE = CubeVisionTarget(-0.1, -20.1, 4.9, -3.1, 2.9)
