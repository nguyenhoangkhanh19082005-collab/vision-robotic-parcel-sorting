"""Real station recordings (dataset/station_runs/) replayed through the current detector.

The recordings come from automatic runs on the real belt (ui/run_recorder.py);
dataset/station_runs/manifest.json says what each parcel folder must give:
"NO_STOP" (empty belt: the detector must never fire) or a decision for a parcel
(the detector fires exactly once with the parcel fully inside; the inspection burst,
when saved, must give that decision).
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from machine_vision.config import load_config
from machine_vision.services.arrival import ArrivalSettings
from machine_vision.services.run_replay import (
    parcel_folders,
    parcel_key,
    parse_expectation,
    replay_arrival,
    replay_inspection,
)

PROJECT = Path(__file__).resolve().parents[1]
RUNS = PROJECT / "dataset" / "station_runs"
MANIFEST = json.loads((RUNS / "manifest.json").read_text(encoding="utf-8")) if (RUNS / "manifest.json").exists() else {}
CASES = [(parcel_key(f), f) for f in parcel_folders([RUNS])] if RUNS.exists() else []
CONFIG = load_config(PROJECT / "config" / "default.yaml").raw
SETTINGS = ArrivalSettings.from_config(CONFIG["station_cycle"]["arrival"])


def test_every_recorded_parcel_has_an_expectation():
    missing = [key for key, _folder in CASES if key not in MANIFEST]
    assert not missing, f"Thêm kết quả mong đợi vào dataset/station_runs/manifest.json: {missing}"


@pytest.mark.parametrize("key,folder", CASES, ids=[key for key, _ in CASES])
def test_recorded_arrival(key, folder):
    expected, _decision = parse_expectation(MANIFEST[key])
    fires = replay_arrival(folder, SETTINGS)
    if expected == "NO_STOP":
        assert fires == [], f"băng trống mà vẫn dừng: {fires}"
        return
    assert len(fires) == 1, f"cần dừng đúng 1 lần, được {fires}"
    _frame, start, end = fires[0]
    if expected == "OVERLONG":     # longer than the view: trailing end cut, leading end inside
        assert start <= 0.01 and end <= 1 - SETTINGS.edge_margin
        return
    assert SETTINGS.edge_margin <= start and end <= 1 - SETTINGS.edge_margin


_PIPELINE = None


def _pipeline():
    global _PIPELINE
    if _PIPELINE is None:
        from machine_vision.app import build_pipeline
        _PIPELINE, _ = build_pipeline(CONFIG, PROJECT, PROJECT)
    return _PIPELINE


INSPECTED = [(k, f) for k, f in CASES
             if parse_expectation(MANIFEST.get(k))[1] is not None and (f / "burst").exists()]


@pytest.mark.parametrize("key,folder", INSPECTED, ids=[key for key, _ in INSPECTED])
def test_recorded_inspection(key, folder):
    result = replay_inspection(_pipeline().inspect, folder)
    assert result.classification.decision.value == parse_expectation(MANIFEST[key])[1],         result.classification.reason
