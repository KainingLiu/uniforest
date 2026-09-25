"""Fixed Task1 orange pickup geometry.

The reference plane is generated once from the operator-provided complete
cube image. Runtime frames only project detected edges onto that plane.
"""
import numpy as np


REFERENCE_SIZE = (1280, 720)
REFERENCE_FX = 663.8648510814408
REFERENCE_FY = 414.2488458332086
REFERENCE_CX = 640.0
REFERENCE_CY = 360.0
REFERENCE_QUAD = np.asarray(
    [[493.3937, 47.4998], [791.5239, 47.2170],
     [853.9650, 225.0187], [436.4120, 218.0691]], np.float64)
CUBE_CM = 10.0


TASK1_H = np.array([
    [30.5230902, -18.2767911, 486.0],
    [-0.3057294, 10.3785768, 62.0],
    [-0.00009711, -0.02915138, 1.0]], np.float64)
TASK1_H_INV = np.array([
    [0.0328200062, 0.0110672114, -16.6366901],
    [0.0008071944, 0.0823338282, -5.4969938],
    [0.0000267179, 0.0024012195, 0.838139477]], np.float64)
TASK1_R = np.array([[0.99989715, 0.01242605, -0.01127999],
                    [-0.01418610, 1.09357413, -0.63258604],
                    [-0.00210756, -0.63267730, -1.09363793]], np.float64)
TASK1_T = np.array([-5.03459066, -15.61270223, 21.70316739], np.float64)
TASK2_H = np.array([[23.4500442, -12.5070961, 554.0],
                    [2.9274465, 6.0877013, 454.0],
                    [0.00598124, -0.01891701, 1.0]], np.float64)
TASK2_H_INV = np.array([[0.04958912, 0.00684933, -30.5819682],
                        [-0.00071620, 0.06803943, -30.4931249],
                        [-0.00031015, 0.00124613, 0.60607954]], np.float64)
TASK2_R = np.array([[0.97825629, -0.01995257, 0.24272390],
                    [0.06185565, 1.03048834, -0.60853172],
                    [0.19796087, -0.62609518, -1.00931588]], np.float64)
TASK2_T = np.array([-4.28752574, 7.51025202, 33.09694928], np.float64)


def geometry_for(profile):
    if profile == "task2_orange":
        return TASK2_H, TASK2_H_INV, TASK2_R, TASK2_T
    return TASK1_H, TASK1_H_INV, TASK1_R, TASK1_T


def image_to_world(point, width, height, profile="default"):
    """Map a processed-frame pixel to the fixed top-plane coordinates (cm)."""
    sx = REFERENCE_SIZE[0] / float(width)
    sy = REFERENCE_SIZE[1] / float(height)
    p = np.array([float(point[0]) * sx, float(point[1]) * sy, 1.0])
    q = geometry_for(profile)[1] @ p
    return q[:2] / max(q[2], 1e-12)


def world_to_camera(point, profile="default"):
    p = np.array([float(point[0]), float(point[1]), 0.0])
    r, t = geometry_for(profile)[2:]
    return r @ p + t


def world_to_image(point, width, height, profile="default"):
    q = geometry_for(profile)[0] @ np.array([float(point[0]), float(point[1]), 1.0])
    p = q[:2] / max(q[2], 1e-12)
    return np.array([p[0] * width / REFERENCE_SIZE[0],
                     p[1] * height / REFERENCE_SIZE[1]])
