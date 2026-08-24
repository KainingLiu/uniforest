"""Protocol contract loading and validation shared by tests and diagnostics."""

from __future__ import annotations

import json
import os
from typing import Any, Mapping

from . import commands

SCHEMA_FILE = os.path.join(os.path.dirname(__file__), "schema.json")


def load_schema(path: str = SCHEMA_FILE) -> Mapping[str, Any]:
    with open(path, encoding="utf-8") as stream:
        return json.load(stream)


def validate_python_constants(schema: Mapping[str, Any] | None = None) -> None:
    """Fail fast when Python protocol constants drift from the checked schema."""
    schema = load_schema() if schema is None else schema
    frame = schema["frame"]
    if frame["sync"] != commands.PROTO_SYNC:
        raise AssertionError("protocol sync byte differs from schema")
    if frame["max_data_length"] != commands.PROTO_MAX_DATA_LEN:
        raise AssertionError("protocol max payload differs from schema")

    for name, definition in schema["commands"].items():
        if getattr(commands, name) != definition["id"]:
            raise AssertionError(f"{name} differs from schema")

    telemetry = schema["telemetry"]
    for name, definition in telemetry.items():
        if name == "full_fields":
            continue
        if getattr(commands, name) != definition["id"]:
            raise AssertionError(f"{name} differs from schema")

    if commands.TelemBatch.PAYLOAD_SIZE != telemetry["TELEM_FULL"]["data_length"]:
        raise AssertionError("telemetry payload size differs from schema")


def command_data_length(name: str) -> int:
    """Return the exact payload length declared for a command."""
    return int(load_schema()["commands"][name]["data_length"])


__all__ = ["SCHEMA_FILE", "command_data_length", "load_schema",
           "validate_python_constants"]
