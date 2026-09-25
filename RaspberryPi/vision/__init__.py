"""Vision compatibility package.

The active classical implementation lives in :mod:`Vision.opencv`.
This package keeps the historical imports working while the YOLO backend is
developed separately in :mod:`Vision.yolo`.
"""

from .opencv.cube_detector import CubeDetector, VisionResult
from .opencv.camera_devices import default_camera_selector
from .opencv.field_localizer import FieldLocalizer, FieldPose
from .camera_devices import default_camera_selector, resolve_camera_source
from .field_localizer import FieldLocalizer, FieldPose

__all__ = [
    "CubeDetector",
    "VisionResult",
    "default_camera_selector",
    "resolve_camera_source",
    "FieldLocalizer",
    "FieldPose",
]
