#!/usr/bin/env python3
"""Interactive UVC camera tuning for Windows and Linux OpenCV backends."""

import argparse
import json
import os
import sys

import cv2
import numpy as np

from cube_detector import CONFIG, detect_all_blocks
from camera_devices import default_camera_selector, resolve_camera_source


SETTINGS_PATH = os.path.join(os.path.dirname(__file__), "camera_settings.json")
WINDOW = "Camera Exposure Tuner"
PLATFORM_KEY = "windows" if sys.platform == "win32" else "linux"


def _load_settings():
    try:
        with open(SETTINGS_PATH, "r", encoding="utf-8") as stream:
            return json.load(stream)
    except (OSError, json.JSONDecodeError):
        return {}


def _save_settings(exposure, gain, white_balance):
    data = _load_settings()
    data[PLATFORM_KEY] = {
        "exposure": exposure,
        "gain": gain,
        "white_balance": white_balance,
    }
    with open(SETTINGS_PATH, "w", encoding="utf-8") as stream:
        json.dump(data, stream, indent=2)
    print(f"Saved {PLATFORM_KEY} camera settings to {SETTINGS_PATH}")


def _noop(_value):
    pass


def _set_manual_controls(cap, exposure, gain, white_balance):
    manual_mode = 0.25 if sys.platform == "win32" else 1.0
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, manual_mode)
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    cap.set(cv2.CAP_PROP_GAIN, gain)
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)
    cap.set(cv2.CAP_PROP_WB_TEMPERATURE, white_balance)


def main():
    parser = argparse.ArgumentParser(description="Interactive camera exposure tuner")
    parser.add_argument("--camera", default=default_camera_selector(),
                        help="camera role (cube/tag), stable path, or diagnostic index")
    parser.add_argument("--resolution", default="640x480")
    args = parser.parse_args()
    width, height = map(int, args.resolution.lower().split("x"))

    # CAP_ANY may select a broken GStreamer pipeline on Raspberry Pi.  Match
    # the detector and force V4L2 for UVC cameras on Linux.
    backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2
    source = resolve_camera_source(args.camera)
    cap = cv2.VideoCapture(source, backend)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open camera {args.camera} ({source})")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)

    saved = _load_settings().get(PLATFORM_KEY, {})
    initial_exposure = int(saved.get("exposure", -6 if sys.platform == "win32" else 100))
    initial_gain = int(saved.get("gain", 0))
    initial_wb = int(saved.get("white_balance", 4500))

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    # Windows UVC exposure is normally -13..-1. Store it as slider+13.
    cv2.createTrackbar("Exposure", WINDOW,
                       initial_exposure + 13 if sys.platform == "win32" else initial_exposure,
                       12 if sys.platform == "win32" else 1000, _noop)
    cv2.createTrackbar("Gain", WINDOW, max(0, initial_gain), 255, _noop)
    cv2.createTrackbar("WB x100K", WINDOW,
                       max(20, min(80, initial_wb // 100)), 80, _noop)

    last_values = None
    try:
        while True:
            exposure_slider = cv2.getTrackbarPos("Exposure", WINDOW)
            exposure = exposure_slider - 13 if sys.platform == "win32" else exposure_slider
            gain = cv2.getTrackbarPos("Gain", WINDOW)
            white_balance = max(2000, cv2.getTrackbarPos("WB x100K", WINDOW) * 100)
            values = (exposure, gain, white_balance)
            if values != last_values:
                _set_manual_controls(cap, *values)
                last_values = values

            ok, frame = cap.read()
            if not ok:
                continue

            h, w = frame.shape[:2]
            value = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[:, :, 2]
            saturated = value >= 250
            global_clip = float(np.mean(saturated) * 100.0)
            x1, x2 = int(w * 0.25), int(w * 0.75)
            y1, y2 = int(h * 0.35), int(h * 0.85)
            center_clip = float(np.mean(saturated[y1:y2, x1:x2]) * 100.0)

            state = {"fx": 331.9324, "fy": 207.1244,
                     "cx": w / 2.0, "cy": h / 2.0, "ema_pos": None}
            blocks = detect_all_blocks(frame, state)
            status = "ORANGE OK" if any(b.color_name == "Orange" for b in blocks) else "NO TARGET"
            status_color = (0, 220, 0) if status == "ORANGE OK" else (0, 0, 255)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 180, 0), 1)
            cv2.putText(frame,
                        f"EXP {exposure}  GAIN {gain}  WB {white_balance}K",
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
            cv2.putText(frame,
                        f"CLIP all {global_clip:.1f}%  center {center_clip:.1f}%  {status}",
                        (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)
            cv2.imshow(WINDOW, frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("s"):
                _save_settings(*values)
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
