"""Configuration loading for the Uniforest natural-language Agent."""

from __future__ import annotations

import os
import re
from pathlib import Path


DEFAULT_CONFIG_PATHS = (
    Path(__file__).resolve().parents[2] / "Docs" / "API.md",
    Path.home() / ".config" / "uniforest" / "API.md",
)


def _read_profiles(path: Path) -> dict[str, dict[str, str]]:
    profiles = {}
    current = None
    if not path.is_file():
        return profiles
    for line in path.read_text(encoding="utf-8").splitlines():
        heading = re.match(r"^\s*##\s*(.*?)\s*$", line)
        if heading:
            current = heading.group(1).strip()
            profiles[current] = {}
            continue
        match = re.match(r"^\s*(model|base\s*url|key)\s*:\s*(.*?)\s*$",
                         line, flags=re.IGNORECASE)
        if match and current:
            key = re.sub(r"\s+", "_", match.group(1).lower())
            profiles[current][key] = match.group(2)
    return profiles


def load_api_config(path: str | os.PathLike | None = None,
                    profile: str | None = None) -> dict[str, str]:
    """Load one API.md profile, with environment variables taking priority."""
    candidates = (Path(path),) if path else DEFAULT_CONFIG_PATHS
    profiles = {}
    for candidate in candidates:
        profiles = _read_profiles(candidate)
        if profiles:
            break
    requested = profile or os.environ.get("UNIFOREST_AGENT_PROFILE") or "APIFUN gpt"
    values = profiles.get(requested)
    if values is None:
        lowered = requested.lower()
        values = next((data for name, data in profiles.items()
                       if name.lower() == lowered or lowered in name.lower()), {})
    if not values and profiles:
        values = next(iter(profiles.values()))
    base_url = os.environ.get("OPENAI_BASE_URL", values.get("base_url", ""))
    selected_name = next((name for name, data in profiles.items() if data is values), requested)
    if ("right" in selected_name.lower()
            and base_url.rstrip("/") == "https://www.rightapi.ai"):
        base_url = base_url.rstrip("/") + "/codex/v1"
    return {
        "model": os.environ.get("UNIFOREST_AGENT_MODEL", values.get("model", "gpt-6-astra")),
        "base_url": base_url,
        "api_key": os.environ.get("OPENAI_API_KEY", values.get("key", "")),
        "profile": selected_name,
    }
