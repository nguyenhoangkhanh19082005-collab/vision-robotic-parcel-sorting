from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class AppConfig:
    raw: dict[str, Any]
    path: Path

    def section(self, name: str) -> dict[str, Any]:
        value = self.raw.get(name, {})
        if not isinstance(value, dict):
            raise ValueError(f"Configuration section '{name}' must be a mapping")
        return value


def _merge(base: dict[str, Any], overlay: dict[str, Any]) -> dict[str, Any]:
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = copy.deepcopy(value)
    return base


def resolve_label_format(raw: dict[str, Any]) -> dict[str, Any]:
    """Return a copy of ``raw`` with the selected ``label_formats`` entry merged in.

    ``label_format: "A7"`` overlays ``label_formats.A7`` (e.g. its own logo template
    and calibration file) onto the ``label`` / ``logo`` / ... sections, so switching
    the shipping-label size is one line and each size keeps its own calibration.
    Idempotent: the result records the applied format and is returned unchanged
    when resolved again.
    """
    selected = raw.get("label_format")
    if not selected or raw.get("_label_format_applied") == selected:
        return raw
    formats = raw.get("label_formats") or {}
    if selected not in formats:
        raise ValueError(
            f"label_format '{selected}' không có trong label_formats ({', '.join(formats) or 'trống'})"
        )
    resolved = _merge(copy.deepcopy(raw), formats[selected] or {})
    resolved["_label_format_applied"] = selected
    return resolved


def load_config(path: str | Path) -> AppConfig:
    config_path = Path(path).resolve()
    with config_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}
    if not isinstance(raw, dict):
        raise ValueError("Configuration root must be a mapping")
    return AppConfig(raw=resolve_label_format(raw), path=config_path)
