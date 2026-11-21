"""Đọc cấu hình trạm (config/station.yaml) và nạp đường dẫn hai dự án gốc vào sys.path.

Hai dự án gốc được giữ nguyên vị trí:
- MachineVision (package `machine_vision`, thư mục src/)
- robotarm-6-axis (module phẳng: main.py, communication.py, donghoc.py, robot_simulation.py, uis/)
"""

from __future__ import annotations

import copy
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

STATION_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = STATION_ROOT / "config" / "station.yaml"


@dataclass(frozen=True)
class StationPaths:
    station_root: Path
    machine_vision_root: Path
    machine_vision_config: Path
    robot_root: Path


def load_station_config(path: str | Path | None = None) -> dict[str, Any]:
    config_path = Path(path or DEFAULT_CONFIG).resolve()
    with config_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}
    if not isinstance(raw, dict):
        raise ValueError("station.yaml: gốc cấu hình phải là mapping")
    raw = copy.deepcopy(raw)
    raw["_config_path"] = str(config_path)
    return raw


def resolve_paths(config: dict[str, Any]) -> StationPaths:
    base = Path(config.get("_config_path", DEFAULT_CONFIG)).resolve().parent.parent
    section = config.get("paths", {})
    mv_root = (base / section.get("machine_vision_root", "../Webcamandconveyor/too/MachineVision")).resolve()
    mv_config = (mv_root / section.get("machine_vision_config", "config/default.yaml")).resolve()
    robot_root = (base / section.get("robot_root", "../robotarm-6-axis")).resolve()
    for label, folder in (("MachineVision", mv_root), ("robotarm-6-axis", robot_root)):
        if not folder.is_dir():
            raise FileNotFoundError(f"Không tìm thấy thư mục {label}: {folder}")
    if not mv_config.is_file():
        raise FileNotFoundError(f"Không tìm thấy cấu hình MachineVision: {mv_config}")
    return StationPaths(base, mv_root, mv_config, robot_root)


def install_import_paths(paths: StationPaths) -> None:
    """Cho phép import `machine_vision` và các module robot (donghoc, communication...)."""

    for folder in (paths.machine_vision_root / "src", paths.robot_root):
        text = str(folder)
        if text not in sys.path:
            sys.path.insert(0, text)
