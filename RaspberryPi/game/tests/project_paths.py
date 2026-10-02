"""Locate this runtime's firmware in standalone and embedded checkouts."""
from pathlib import Path

RUNTIME = Path(__file__).resolve().parents[1]
PROJECT = RUNTIME.parent.parent if RUNTIME.name == 'game' else RUNTIME.parent
FIRMWARE = PROJECT / 'Uniforest_A'
if RUNTIME.name == 'game':
    FIRMWARE /= 'game'
