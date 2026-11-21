import os
import sys
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sorting_station.station_config import install_import_paths, load_station_config, resolve_paths  # noqa: E402

STATION_CONFIG = load_station_config()
PATHS = resolve_paths(STATION_CONFIG)
install_import_paths(PATHS)


@pytest.fixture(scope="session")
def qt_app():
    from PyQt6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


@pytest.fixture(scope="session")
def station_config():
    return STATION_CONFIG


@pytest.fixture(scope="session")
def paths():
    return PATHS


@pytest.fixture(scope="session")
def model(paths, station_config):
    from sorting_station.robot_model import build_model_from_robot_project

    tool = float(station_config["robot"]["tool_length_mm"]) / 1000.0
    scale = float(station_config["robot"].get("scale", 1.0))
    return build_model_from_robot_project(paths.robot_root, tool_length_m=tool, scale=scale)
