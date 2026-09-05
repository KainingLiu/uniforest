"""Orange pickup geometry adapted from desktop demo task1/task2.
Values are demo baselines, pending validation on the competition robot.
Camera capture/exposure and other color profiles are not changed here.
"""
from types import SimpleNamespace

TASK1_X_OFFSET_MM = 5.0
TASK2_X_OFFSET_MM = 5.0


def config_for(profile, width):
    if profile not in ("default", "task2_orange"):
        raise ValueError(profile)
    # Demo operates at 1280 px; scale pixel gates to the existing capture mode.
    scale = min(width, 1280) / 1280.0
    return SimpleNamespace(
        CUBE_SIZE_CM=10.0, CAMERA_TILT_DEG=45.0,
        NOMINAL_FOCAL_RATIO=0.8, AUTO_CALIBRATE_F=False,
        CALIB_CONFIRM_FRAMES=3, MIN_ASPECT_FOR_AUTOCAL=1.8,
        PROCESS_WIDTH=1280, HSV_ORANGE_LOW=(0, 45, 60),
        HSV_ORANGE_HIGH=(40, 255, 255),
        MORPH_KERNEL=max(3, int(round(5 * scale)) | 1),
        MIN_CLUSTER_AREA=4000 * scale * scale,
        MIN_RELATIVE_CLUSTER_AREA=0.0, MIN_CLUSTER_ASPECT=0.90,
        MIN_MASK_FRAC=0.001, FRAME_EDGE_MARGIN_PX=max(1, round(3 * scale)),
        GEOMETRY_PROFILE=profile,
        X_OFFSET_MM=(TASK2_X_OFFSET_MM if profile == "task2_orange"
                     else TASK1_X_OFFSET_MM),
    )
