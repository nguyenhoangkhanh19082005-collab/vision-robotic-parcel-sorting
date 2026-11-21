"""Rehearse the automatic station cycle end to end without hardware.

Simulated: the ESP32 (controllers/fake_esp32.py, real UDP on localhost), the
camera (recorded station images) and Supabase (in-memory orders + a list that
collects the sorting_logs rows). Real: StationCycle, the full vision pipeline
(QR, logo, damage, classifier) and the sorting_logs row builder.

    .venv\\Scripts\\python.exe scripts\\rehearse_auto_cycle.py

Writes a JSON summary to tmp/rehearsal/auto_cycle_summary.json.
"""

from __future__ import annotations

import copy
import glob
import json
import sys
import threading
import time
from pathlib import Path

import cv2
import yaml

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))

from machine_vision.app import build_pipeline
from machine_vision.controllers.conveyor_udp import ConveyorLink
from machine_vision.controllers.fake_esp32 import FakeConveyorEsp32
from machine_vision.repositories.shipments import InMemoryShipmentRepository
from machine_vision.repositories.sorting_logs import SortingLogReporter
from machine_vision.services.station_cycle import (
    CyclePhase,
    CycleSettings,
    StationCycle,
)

GOOD_CODE = "SPXVN54506627407"


def _frames(pattern: str, limit: int = 6) -> list:
    paths = sorted(glob.glob(str(PROJECT / pattern)))[:limit]
    return [cv2.imread(path) for path in paths]


def scenarios() -> list[dict]:
    good = _frames("dataset/station/good_parcel_verify_*/frame_*.png")
    # One saved frame per GUI inspection: repeat it as a still burst.
    torn = [cv2.imread(p) for p in glob.glob(str(PROJECT / "captures/*/*T110146*/frame.jpg"))] * 6
    empty = _frames("dataset/station/empty_belt_*/frame_*.png")
    blurred = [cv2.GaussianBlur(frame, (0, 0), 9) for frame in good]
    return [
        {"name": "kiện tốt, có trong đơn hàng", "frames": good, "in_db": True,
         "expect": ("PASS", 2)},
        {"name": "kiện tốt, KHÔNG có trong đơn hàng", "frames": good, "in_db": False,
         "expect": ("REJECT", 0)},
        {"name": "nhãn rách (ảnh trạm 2026-11-04)", "frames": torn, "in_db": True,
         "expect": ("REJECT", 0)},
        {"name": "ảnh mờ (mất nét / rung)", "frames": blurred, "in_db": True,
         "expect": ("MANUAL_REVIEW", 0)},
        {"name": "vật không có nhãn (khung trống)", "frames": empty, "in_db": True,
         "expect": ("MANUAL_REVIEW", 0)},
    ]


class SwitchableOrders(InMemoryShipmentRepository):
    """Orders table; ``visible=False`` simulates a code missing from the database."""

    visible = True

    def find_by_waybill(self, code: str):
        return super().find_by_waybill(code) if self.visible else None


def main() -> int:
    config = yaml.safe_load((PROJECT / "config" / "default.yaml").read_text(encoding="utf-8"))
    config = copy.deepcopy(config)
    config["classification"]["local_test_mode"] = False  # real database path
    pipeline, warning = build_pipeline(config, PROJECT, PROJECT)
    if warning:
        print("warning:", warning)
    orders = SwitchableOrders(
        [{"id": 101, "tracking_code": GOOD_CODE, "destination_chute_id": 2, "status": "PENDING"}],
        key="tracking_code",
    )
    pipeline.classifier.repository = orders
    uploaded: list[dict] = []
    reporter = SortingLogReporter(
        config["supabase"], uploaded.append, PROJECT / "tmp" / "rehearsal" / "outbox.jsonl"
    )

    cases = scenarios()
    esp = FakeConveyorEsp32(event_port=0, belt_speed=0.6, parcel_length=0.5)
    link = ConveyorLink("127.0.0.1", esp.address[1], event_port=0, timeout_s=0.3)
    esp.event_port = link.event_port
    esp.start()

    state = {"index": 0}
    log: list[dict] = []
    phases: list[str] = []
    stop = threading.Event()

    def capture():
        case = cases[state["index"]]
        orders.visible = case["in_db"]
        return list(case["frames"])

    def inspect(frames):
        started = time.perf_counter()
        result = pipeline.inspect(frames)
        state["inspect_s"] = time.perf_counter() - started
        return result

    def on_result(result):
        case = cases[state["index"]]
        reporter.report(result)
        row = uploaded[-1]
        decision = result.classification.decision.value
        log.append({
            "case": case["name"],
            "belt_at_inspection": esp.state,
            "sensor_at_inspection": esp.sensor,
            "decision": decision,
            "chute_id": row["chute_id"],
            "status": row["status"],
            "reason": result.classification.reason[:160],
            "qr": result.classification.parcel.qr_text,
            "inspect_s": round(state.pop("inspect_s", 0.0), 2),
            "ok": (decision, row["chute_id"]) == case["expect"],
        })
        state["index"] += 1

    def on_phase(phase, text):
        phases.append(f"{phase.value}: {text}")
        if phase == CyclePhase.RUNNING and state["index"] >= len(cases):
            stop.set()

    settings = CycleSettings(
        speed=150, auto_stop_delay_ms=100, stop_grace_s=0.3, settle_s=0.05,
        quality_retries=1, release_timeout_s=4.0, poll_s=0.05,
    )
    cycle = StationCycle(link, capture, inspect, settings, on_phase, on_result)
    esp.feed(len(cases), gap=0.9)
    worker = threading.Thread(target=cycle.run, args=(stop,))
    worker.start()
    worker.join(240)
    stop.set()
    worker.join(10)
    link.close()
    esp.close()

    for item in log:
        mark = "OK " if item["ok"] else "SAI"
        print(f"[{mark}] {item['case']:38s} -> {item['decision']:13s} sọt {item['chute_id']} "
              f"({item['status']}), băng tải {item['belt_at_inspection']}, {item['inspect_s']} s")
        print(f"       {item['reason']}")
    summary = {
        "parcels": len(log),
        "correct": sum(item["ok"] for item in log),
        "fwd_commands": esp.commands.count("FWD:150"),
        "autostop_events": sum(e.startswith("EVT:AUTOSTOP") for e in esp.events),
        "final_belt_state": esp.state,
        "final_auto": esp.auto,
        "last_phase": phases[-1] if phases else None,
        "rows": uploaded,
        "results": log,
    }
    out = PROJECT / "tmp" / "rehearsal" / "auto_cycle_summary.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
    print(f"\n{summary['correct']}/{summary['parcels']} đúng · FWD {summary['fwd_commands']} lần · "
          f"AUTOSTOP {summary['autostop_events']} · băng tải cuối {esp.state}, AUTO={int(esp.auto)}")
    print(summary["last_phase"])
    return 0 if summary["correct"] == len(cases) == summary["parcels"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
