"""Replay recorded automatic runs (logs/runs/, ui/run_recorder.py) through the current code.

Usage:
    python scripts/replay_station_runs.py logs/runs/20261125_101500 [more runs ...]
    python scripts/replay_station_runs.py dataset/station_runs --expect dataset/station_runs/manifest.json

For every parcel folder:
* arrival: the saved frames (with their capture times) go through ArrivalDetector with the
  *current* config/default.yaml settings and the run's own background.png. Reports how
  many times it fires, where (blob ends) and whether the parcel is fully inside.
* inspection: the saved burst goes through the current inspection pipeline; the decision
  is compared with the live one and with the expected one from the manifest
  (``{"<run>/<parcel>": "PASS" | "REJECT" | "MANUAL_REVIEW" | "NO_STOP"}``).

Exit code 1 if any expectation fails.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

import cv2

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))

from machine_vision.config import load_config  # noqa: E402
from machine_vision.services.arrival import ArrivalSettings  # noqa: E402
from machine_vision.services.run_replay import (  # noqa: E402
    parcel_folders,
    parcel_key,
    parse_expectation,
    replay_arrival,
    replay_inspection,
)


@dataclass
class ParcelReplay:
    key: str
    live_fired: int | None
    fires: list[tuple[int, float, float]] = field(default_factory=list)  # frame, start, end
    live_decision: str | None = None
    decision: str | None = None
    reason: str = ""
    qr_text: str | None = None
    qr_ratio: float | None = None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="+", type=Path)
    parser.add_argument("--expect", type=Path)
    parser.add_argument("--no-inspect", action="store_true")
    args = parser.parse_args()

    config = load_config(PROJECT / "config" / "default.yaml").raw
    settings = ArrivalSettings.from_config(config["station_cycle"]["arrival"])
    expect = json.loads(args.expect.read_text(encoding="utf-8")) if args.expect else {}
    pipeline = None
    if not args.no_inspect:
        from machine_vision.app import build_pipeline
        pipeline, _ = build_pipeline(config, PROJECT, PROJECT)

    failures = 0
    for folder in parcel_folders(args.paths):
        meta = json.loads((folder / "meta.json").read_text(encoding="utf-8"))
        key = parcel_key(folder)
        rep = ParcelReplay(key, meta.get("fired_frame"))
        if (folder / "arrival" / "stamps.json").exists():
            rep.fires = replay_arrival(folder, settings)
        results = meta.get("results") or []
        rep.live_decision = results[-1]["decision"] if results else None
        result = replay_inspection(pipeline.inspect, folder) if pipeline is not None else None
        if result is not None:
            c = result.classification
            rep.decision, rep.reason = c.decision.value, c.reason
            rep.qr_text, rep.qr_ratio = c.parcel.qr_text, c.parcel.qr_read_ratio

        problems = []
        wanted, wanted_decision = parse_expectation(expect.get(key))
        if wanted == "NO_STOP":
            if rep.fires:
                problems.append(f"dừng nhầm {len(rep.fires)} lần")
        elif wanted is not None or rep.live_fired is not None:
            if len(rep.fires) != 1:
                problems.append(f"bộ dò kích hoạt {len(rep.fires)} lần (cần 1)")
            elif wanted == "OVERLONG":
                if not (rep.fires[0][1] <= 0.01 and rep.fires[0][2] <= 1 - settings.edge_margin):
                    problems.append("kiện dài: đầu kiện đã ra khỏi khung lúc dừng")
            elif not (settings.edge_margin <= rep.fires[0][1] and rep.fires[0][2] <= 1 - settings.edge_margin):
                problems.append("lúc dừng kiện chưa nằm trọn khung")
        if wanted_decision is not None and rep.decision is not None and rep.decision != wanted_decision:
            problems.append(f"kết quả {rep.decision}, cần {wanted_decision}")
        failures += bool(problems)
        mark = "OK " if not problems else "LỖI"
        qr = f"{rep.qr_text or '-'} ({rep.qr_ratio:.0%})" if rep.qr_ratio is not None else (rep.qr_text or "-")
        print(f"[{mark}] {key:24s} dừng live={rep.live_fired} replay={rep.fires} | "
              f"live {rep.live_decision} -> replay {rep.decision} | QR {qr}")
        if rep.reason and rep.decision != "PASS":
            print(f"        {rep.reason}")
        for problem in problems:
            print(f"        -> {problem}")
    print(f"\n{failures} kiện lỗi")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
