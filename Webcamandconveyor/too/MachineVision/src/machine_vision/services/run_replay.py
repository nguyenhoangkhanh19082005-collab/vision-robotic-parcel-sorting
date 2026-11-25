"""Replay recorded station runs (ui/run_recorder.py) through ArrivalDetector.

Shared by scripts/replay_station_runs.py and tests/test_station_runs.py.
"""

from __future__ import annotations

import json
from pathlib import Path

import cv2

from machine_vision.services.arrival import ArrivalDetector, ArrivalSettings


def parcel_folders(paths: list[Path]) -> list[Path]:
    found: list[Path] = []
    for path in paths:
        found += sorted(p.parent for p in Path(path).rglob("meta.json"))
    return found


def parcel_key(folder: Path) -> str:
    return f"{folder.parent.name}/{folder.name}"


def replay_arrival(folder: Path, settings: ArrivalSettings) -> list[tuple[int, float, float]]:
    """Frames of one parcel folder through a fresh detector: [(frame counter, start, end)] fired."""
    stamps_file = folder / "arrival" / "stamps.json"
    background = cv2.imread(str(folder.parent / "background.png"))
    if background is None or not stamps_file.exists():
        return []
    stamps = json.loads(stamps_file.read_text(encoding="utf-8"))
    names = sorted(stamps)
    if not names:
        return []
    first = cv2.imread(str(folder / "arrival" / names[0]))
    # Frames were saved downscaled: bring the reference to the same size.
    background = cv2.resize(background, (first.shape[1], first.shape[0]), interpolation=cv2.INTER_AREA)
    detector = ArrivalDetector(background, settings)
    fires = []
    for name in names:
        frame = cv2.imread(str(folder / "arrival" / name))
        if detector.update(frame, stamps[name]):
            blob = detector.state.blob
            fires.append((int(name[1:8]), round(blob.start, 3), round(blob.end, 3)))
    meta = json.loads((folder / "meta.json").read_text(encoding="utf-8"))
    st = detector.state
    if (not fires and "tail_frames" not in meta and meta.get("fired_frame") == int(names[-1][1:8])
            and st.armed and st.inside and st.moved and abs(st.velocity) >= settings.moving_speed):
        # Recordings made before 2026-11-25 end exactly at the live trigger frame. The
        # replay (JPEG, 1280 px) can read a slightly lower speed and fire one frame
        # later: fully inside, moving and armed on the last frame counts as that fire.
        fires.append((int(names[-1][1:8]) + 1, round(st.blob.start, 3), round(st.blob.end, 3)))
    return fires


def replay_inspection(process, folder: Path):
    """Saved bursts (one list per attempt, meta.json "bursts") through ``process`` like the cycle."""
    from machine_vision.services.station_cycle import inspection_failure

    meta = json.loads((folder / "meta.json").read_text(encoding="utf-8"))
    groups = meta.get("bursts") or []
    # dataset/station_runs keeps the burst of only some parcels: nothing to replay then.
    if not groups or not all((folder / "burst" / name).exists() for names in groups for name in names):
        return None
    last_error, frame = "không có ảnh", None
    for names in groups:
        frames = [cv2.imread(str(folder / "burst" / name)) for name in names]
        frame = frames[-1] if frames else frame
        try:
            return process(frames)
        except Exception as exc:  # noqa: BLE001 - same handling as CameraStationCycle._inspect
            last_error = str(exc)
    return inspection_failure(last_error, frame)


DECISIONS = ("PASS", "REJECT", "MANUAL_REVIEW")


def parse_expectation(value: str | None) -> tuple[str | None, str | None]:
    """Manifest value -> (stop kind, decision).

    stop kind: "NO_STOP" (must never fire), "OVERLONG" (fires once, trailing end cut),
    "INSIDE" (fires once, fully inside) or None (no expectation). Examples: "PASS",
    "REJECT", "NO_STOP", "OVERLONG", "OVERLONG PASS".
    """
    if value is None:
        return None, None
    tokens = value.split()
    decision = next((tok for tok in tokens if tok in DECISIONS), None)
    if "NO_STOP" in tokens:
        return "NO_STOP", None
    return ("OVERLONG" if "OVERLONG" in tokens else "INSIDE"), decision
