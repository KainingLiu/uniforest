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
REFERENCE_QUAD = np.array(
    [[457.6807861328125, 38.07045936584473],
     [852.0655517578125, 42.832706451416016],
     [940.5615234375, 209.23072814941406],
     [386.069091796875, 204.38351440429688]], np.float64)
CUBE_CM = 10.0


# 2026-10-02: 100 mm top face, 12-frame fit / 12-frame static holdout.
# Retain the existing plane decomposition and pixel scaling conventions.
TASK1_H = np.array(
    [[40.11480745297361, -18.357681491003476, 457.6807861328125],
     [0.5102235543836848, 10.703914457299751, 38.070457458496094],
     [0.0007937545514627479, -0.029001316850561613, 1.0]], np.float64)
TASK1_H_INV = np.array(
    [[0.02502615009505426, 0.010775852905296071, -11.864229698992279],
     [-0.0010173330004934454, 0.08425024350244599, -2.741831543703114],
     [-4.9368617233382155e-05, 0.002434814624261934, 0.9299005609733761]], np.float64)
TASK1_R = np.array(
    [[0.9998702745045439, 0.005128788866346773, 0.01579353325739453],
     [0.009081429869342999, 0.855435660060786, -0.48604431699020156],
     [0.013302698738831747, -0.48603914192551484, -0.8552781115095494]], np.float64)
TASK1_T = np.array(
    [-4.602632414653895, -13.02425986029168, 16.759209398317463], np.float64)
# 2026-10-02: Task2 bottom-half ROI, 12-frame fit / 12-frame static holdout.
TASK2_H = np.array(
    [[25.371331003145805, -11.475100975116804, 514.3944702148438],
     [0.13490610953813462, 6.898882233413883, 427.201171875],
     [0.00020150590643823693, -0.018111777888383807, 1.0]], np.float64)
TASK2_H_INV = np.array(
    [[0.039564978856620935, 0.00583490170429371, -22.844683183862728],
     [-0.00013197796627680095, 0.06830402497950938, -29.111670778982692],
     [-1.0362932539079108e-05, 0.0012359315621545915, 0.4773392234835708]], np.float64)
TASK2_R = np.array(
    [[0.9999781200450538, 0.004612654779424892, 0.006400598373787662],
     [0.003959247317346815, 0.8519280058444009, -0.4763366091908951],
     [0.00529941429214417, -0.47632258673897, -0.8518911030569543]], np.float64)
TASK2_T = np.array(
    [-4.97587173310958, 4.266341656344897, 26.29905190282092], np.float64)


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
