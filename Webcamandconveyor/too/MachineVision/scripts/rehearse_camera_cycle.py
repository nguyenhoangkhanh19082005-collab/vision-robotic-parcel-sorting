"""Rehearse the two-stop cycle (camera stop + pick-sensor stop) without hardware.

Simulated: ESP32 (fake firmware v2 over localhost UDP), the moving belt as seen by
the webcam (the real parcel cut from a station photo, pasted on the real empty-belt
frames at the position the fake belt reports, 30 fps), the robot (takes the parcel
after 1 s) and the Supabase orders table.
Real: ArrivalDetector, CameraStationCycle, the full vision pipeline, sorting_logs rows.

    .venv\\Scripts\\python.exe scripts\\rehearse_camera_cycle.py

Belt units: the camera sees CAMERA +- HALF_VIEW; the pick sensor is PICK.
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
import numpy as np
import yaml

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))

from machine_vision.app import build_pipeline
from machine_vision.controllers.conveyor_udp import ConveyorLink
from machine_vision.controllers.fake_esp32 import FakeConveyorEsp32
from machine_vision.repositories.shipments import InMemoryShipmentRepository
from machine_vision.repositories.sorting_logs import SortingLogReporter
from machine_vision.services.arrival import ArrivalDetector, ArrivalSettings
from machine_vision.services.station_cycle import (
    CameraStationCycle,
    CyclePhase,
    CycleSettings,
)
from machine_vision.ui.auto_cycle import camera_arrival, fresh_burst
from machine_vision.ui.camera_worker import FrameBuffer

CAMERA, HALF_VIEW = 1.0, 0.3     # view covers 0.7 .. 1.3 belt units (top -> bottom of the image)
PICK = 2.2                       # pick sensor, outside the view
BELT_SPEED = 0.3                 # units / s  (1 unit = 2400 px of the 1440 px tall view)
GOOD_CODE = "SPXVN06275784488"   # tem QR 28 mm, ảnh trạm 23/11/2026


def cut_parcel(frame: np.ndarray, background: np.ndarray) -> tuple[np.ndarray, np.ndarray, int]:
    """Parcel pixels + mask (station photo minus empty belt), and its centre row."""
    a = cv2.cvtColor(cv2.GaussianBlur(frame, (9, 9), 0), cv2.COLOR_BGR2LAB).astype(np.int16)
    b = cv2.cvtColor(cv2.GaussianBlur(background, (9, 9), 0), cv2.COLOR_BGR2LAB).astype(np.int16)
    d = np.abs(a - b)
    mask = np.uint8(np.maximum(d[:, :, 0], 2 * np.maximum(d[:, :, 1], d[:, :, 2])) > 25) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((11, 11), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    part = np.uint8(labels == index) * 255
    contours, _ = cv2.findContours(part, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    filled = np.zeros_like(part)
    cv2.drawContours(filled, contours, -1, 255, cv2.FILLED)
    x, y, w, h = cv2.boundingRect(filled)
    return frame[y:y + h, :].copy(), filled[y:y + h, :].copy(), y + h // 2


class BeltCamera(threading.Thread):
    """Renders what the webcam sees for the fake belt, into a FrameBuffer at ~30 fps."""

    def __init__(self, esp, frames, empties, patch, patch_mask, px_per_unit):
        super().__init__(daemon=True)
        self.esp, self.frames, self.empties = esp, frames, empties
        self.patch, self.patch_mask, self.px = patch, patch_mask, px_per_unit
        self.stop_event = threading.Event()

    def render(self, index: int) -> np.ndarray:
        frame = self.empties[index % len(self.empties)].copy()
        height = frame.shape[0]
        for centre in self.esp.positions():
            row = int((centre - (CAMERA - HALF_VIEW)) * self.px) - self.patch.shape[0] // 2
            top, bottom = max(0, row), min(height, row + self.patch.shape[0])
            if bottom <= top:
                continue
            src = slice(top - row, bottom - row)
            region = self.patch_mask[src] > 0
            frame[top:bottom][region] = self.patch[src][region]
        return frame

    def run(self) -> None:
        index = 0
        while not self.stop_event.is_set():
            started = time.monotonic()
            self.frames.append(self.render(index))
            index += 1
            time.sleep(max(0.0, 1 / 30 - (time.monotonic() - started)))


class SwitchableOrders(InMemoryShipmentRepository):
    visible = True

    def find_by_waybill(self, code: str):
        return super().find_by_waybill(code) if self.visible else None


def main() -> int:
    config = copy.deepcopy(yaml.safe_load((PROJECT / "config/default.yaml").read_text(encoding="utf-8")))
    config["classification"]["local_test_mode"] = False
    pipeline, _ = build_pipeline(config, PROJECT, PROJECT)
    orders = SwitchableOrders(
        [{"id": 101, "tracking_code": GOOD_CODE, "destination_chute_id": 2}], key="tracking_code"
    )
    pipeline.classifier.repository = orders
    rows: list[dict] = []
    reporter = SortingLogReporter(config["supabase"], rows.append, PROJECT / "tmp/rehearsal/outbox.jsonl")

    empties = [cv2.imread(p) for p in sorted(glob.glob(str(PROJECT / "dataset/station/empty_belt_sensor_moved_*/frame_*.png")))]
    # Reference from the same session as the frames (23/11). config/calibration/empty_belt.png
    # was re-shot on 24/11 after the camera moved and no longer matches these images.
    background = np.median(np.stack(empties), axis=0).astype(np.uint8)
    good = cv2.imread(sorted(glob.glob(str(PROJECT / "dataset/station/good_parcel_qr28_size2_full_*/frame_*.png")))[0])
    patch, patch_mask, _centre_row = cut_parcel(good, background)
    px_per_unit = background.shape[0] / (2 * HALF_VIEW)
    length = patch.shape[0] / px_per_unit

    # Parcels: two well spaced, the third placed too close to the second.
    plan = [("có trong đơn hàng", True, "PASS", 2), ("không có trong đơn hàng", False, "REJECT", 0),
            ("đặt quá sát kiện trước", True, "MANUAL_REVIEW", 0)]
    esp = FakeConveyorEsp32(event_port=0, belt_speed=BELT_SPEED, parcel_length=length, sensor_position=PICK)
    spacing = (PICK - CAMERA) + length + 0.4
    with esp._lock:
        first = CAMERA - HALF_VIEW - length          # just above the view
        esp.parcels = [first + length / 2 + length / 2, first - spacing + length,
                       first - spacing - 0.75 * (PICK - CAMERA) + length]
    link = ConveyorLink("127.0.0.1", esp.address[1], event_port=0, timeout_s=0.3)
    esp.event_port = link.event_port
    esp.start()
    frames = FrameBuffer(15)
    camera = BeltCamera(esp, frames, empties, patch, patch_mask, px_per_unit)
    camera.start()

    arrival_cfg = dict(config["station_cycle"]["arrival"], latency_s=0.06)
    detector = ArrivalDetector(background, ArrivalSettings.from_config(arrival_cfg))
    detector.reset(armed=True)
    settings = CycleSettings(speed=40, trigger="camera", pick_stop_delay_ms=0, stop_grace_s=0.3,
                             settle_s=0.3, quality_retries=1, transfer_timeout_s=12, pick_timeout_s=10,
                             poll_s=0.05)
    log: list[dict] = []
    state = {"robot_due": None}
    stop = threading.Event()

    def inspect(burst):
        centres = [round(c, 3) for c in esp.positions()]
        nearest = min(centres, key=lambda c: abs(c - CAMERA))
        state["stop_offset_px"] = round((nearest - CAMERA) * px_per_unit)
        orders.visible = plan[len(log)][1] if len(log) < len(plan) else True
        return pipeline.inspect(burst)

    def on_result(result):
        reporter.report(result)
        name, _in_db, expect, chute = plan[len(log)]
        decision = result.classification.decision.value
        log.append({"case": name, "decision": decision, "chute_id": rows[-1]["chute_id"],
                    "stop_offset_px": state.pop("stop_offset_px", None),
                    "reason": result.classification.reason[:120],
                    "ok": (decision, rows[-1]["chute_id"]) == (expect, chute)})

    def on_pick_ready(_result):
        log[-1]["at_pick"] = (esp.state, esp.sensor)
        state["robot_due"] = time.monotonic() + 1.0

    def pick_done():
        if state["robot_due"] and time.monotonic() > state["robot_due"]:
            esp.take_at_sensor()  # suction lifts the parcel off the pick sensor
            state["robot_due"] = None
            state["lifted"] = True
        if state.get("lifted") and not esp.sensor:  # robot back home: done
            state["lifted"] = False
            return True
        return False

    def on_phase(phase, text):
        if phase == CyclePhase.RUNNING and len(log) >= len(plan):
            stop.set()
        if phase == CyclePhase.FAULT:
            print("FAULT:", text)

    cycle = CameraStationCycle(
        link, lambda: fresh_burst(frames, 6), inspect, camera_arrival(frames, detector), settings,
        on_phase, on_result, on_pick_ready, pick_done, rearm=detector.rearm,
    )
    worker = threading.Thread(target=cycle.run, args=(stop,))
    worker.start()
    worker.join(150)
    stop.set()
    worker.join(10)
    camera.stop_event.set()
    link.close()
    esp.close()

    for item in log:
        mark = "OK " if item["ok"] else "SAI"
        offset = item["stop_offset_px"]
        where = f"lệch tâm {offset:+d} px" if offset is not None else "không dừng ở camera"
        print(f"[{mark}] {item['case']:26s} -> {item['decision']:13s} sọt {item['chute_id']} | "
              f"{where} | tại điểm gắp: {item.get('at_pick')}")
        print(f"       {item['reason']}")
    summary = {"correct": sum(i["ok"] for i in log), "parcels": len(log), "results": log,
               "autostop_at_pick": sum(e.startswith("EVT:AUTOSTOP") for e in esp.events),
               "final_belt": esp.state, "final_auto": esp.auto}
    out = PROJECT / "tmp/rehearsal/camera_cycle_summary.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n{summary['correct']}/{len(plan)} đúng · dừng tại cảm biến điểm gắp {summary['autostop_at_pick']} lần"
          f" · băng tải cuối {esp.state}, AUTO={int(esp.auto)}")
    return 0 if summary["correct"] == len(plan) else 1


if __name__ == "__main__":
    raise SystemExit(main())
