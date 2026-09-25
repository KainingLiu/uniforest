"""Stable camera-role mapping for the robot's USB cameras."""

from __future__ import annotations

import json
import os
import sys
from typing import Mapping, Optional, Union


CameraSource = Union[int, str]
CAMERA_DEVICES_FILE = os.path.join(os.path.dirname(__file__),
                                   "camera_devices.json")
DEFAULT_CAMERA_ROLE = "cube"


def platform_key() -> str:
    return "windows" if sys.platform == "win32" else "linux"


def default_camera_selector() -> CameraSource:
    """Use the stable cube-camera role on Linux and legacy index on Windows."""
    return 1 if sys.platform == "win32" else DEFAULT_CAMERA_ROLE


def load_camera_roles(key: Optional[str] = None) -> Mapping[str, CameraSource]:
    key = platform_key() if key is None else key
    try:
        with open(CAMERA_DEVICES_FILE, "r", encoding="utf-8") as stream:
            data = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot load camera role config: {exc}") from exc
    roles = data.get(key, {})
    if not isinstance(roles, dict):
        raise RuntimeError(f"camera role config for {key!r} must be an object")
    return roles


def resolve_camera_source(
        selector: Optional[CameraSource] = None,
        *,
        key: Optional[str] = None,
        roles: Optional[Mapping[str, CameraSource]] = None,
        require_exists: bool = True) -> CameraSource:
    """Resolve a role, explicit device path, or diagnostic numeric index."""
    key = platform_key() if key is None else key
    if selector is None:
        selector = 1 if key == "windows" else DEFAULT_CAMERA_ROLE
    if isinstance(selector, int):
        return selector

    selector = str(selector).strip()
    if selector.lstrip("+-").isdigit():
        return int(selector)

    configured_roles = load_camera_roles(key) if roles is None else roles
    if selector in configured_roles:
        source = configured_roles[selector]
    elif os.path.isabs(selector):
        source = selector
    else:
        known = ", ".join(sorted(configured_roles)) or "none"
        raise ValueError(f"unknown camera role {selector!r}; configured roles: {known}")

    if key == "linux" and require_exists and not os.path.exists(str(source)):
        raise FileNotFoundError(
            f"camera {selector!r} is unavailable at stable path {source!r}")
    return source
