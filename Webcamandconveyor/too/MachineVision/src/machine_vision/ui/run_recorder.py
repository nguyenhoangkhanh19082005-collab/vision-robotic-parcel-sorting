"""Record every automatic-cycle parcel for replay tests (2026-11-25).

One folder per run, one sub-folder per parcel::

    logs/runs/<YYYYmmdd_HHMMSS>/p01/
        arrival/f<counter>.jpg   frames fed to the arrival detector (last ``keep`` frames
                                 before the trigger, 1280 px wide; the counter matches
                                 the ``frame`` column of logs/arrival/*.csv)
        burst/b<k>.jpg           inspection burst, full resolution (JPEG q95)
        arrival/stamps.json      capture time of every saved arrival frame
        meta.json                trigger frame, detector state, decision, QR, reasons
    logs/runs/<run>/background.png   empty-belt reference the detector used
    logs/runs/<run>/settings.json    arrival + cycle settings of the run

The replay tests (tests/test_station_runs.py) feed these back through ArrivalDetector
and the inspection pipeline. Writing happens on a background thread so the trigger
timing is not affected; frames are dropped (and counted) if the writer falls behind.
Recording stops by itself when the disk has less than ``min_free_mb`` left.
"""

from __future__ import annotations

import json
import queue
import shutil
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

import cv2
import numpy as np

ARRIVAL_WIDTH = 1280


class RunRecorder:
    def __init__(self, root: Path, keep: int = 150, min_free_mb: int = 400):
        self.root = Path(root) / time.strftime("%Y%m%d_%H%M%S")
        self.keep = keep
        self.min_free_mb = min_free_mb
        self.enabled = True
        self.dropped = 0
        self._parcel = 0
        self._ring: deque[tuple[int, float, bytes, bool]] = deque(maxlen=keep)
        self._meta: dict[str, Any] = {}
        self._bursts = 0
        self._jobs: queue.Queue = queue.Queue(maxsize=90)
        self._thread = threading.Thread(target=self._work, name="run-recorder", daemon=True)
        self._thread.start()
        self._new_parcel()

    # -- called from the cycle thread (cheap: only queue work) -----------------------
    def frame(self, counter: int, stamp: float, frame: np.ndarray, present: bool) -> None:
        if self.enabled:
            self._put(lambda: self._encode_arrival(counter, stamp, frame, present), drop=True)

    def fired(self, counter: int, stamp: float, state: dict[str, Any], latest=None,
              tail_s: float = 0.4) -> None:
        """Trigger frame; with ``latest`` (FrameBuffer.latest) also keep ``tail_s`` of frames
        after it (belt braking), so a replay that fires one frame later can be checked.
        Runs on its own short thread: the STOP command is not delayed."""
        self._put(lambda: self._meta.update(fired_frame=counter, fired_time=stamp, at_fire=state))
        if latest is None or not self.enabled:
            return

        def tail() -> None:
            seen, count, end = counter, 0, time.monotonic() + tail_s
            while time.monotonic() < end:
                item = latest()
                if item is not None and item[0] > seen:
                    seen = item[0]
                    count += 1
                    self.frame(item[0], item[1], item[2], True)
                time.sleep(0.005)
            self._put(lambda: self._meta.update(tail_frames=count))

        threading.Thread(target=tail, name="run-recorder-tail", daemon=True).start()

    def run_info(self, background: np.ndarray, settings: dict[str, Any]) -> None:
        def write() -> None:
            if self._disk_ok():
                cv2.imwrite(str(self.root / "background.png"), background)
                (self.root / "settings.json").write_text(
                    json.dumps(settings, ensure_ascii=False, indent=1, default=str), encoding="utf-8"
                )
        self._put(write)

    def phase(self, name: str, text: str) -> None:
        stamp = time.strftime("%H:%M:%S") + f".{int(time.time() * 1000) % 1000:03d}"
        self._put(lambda: self._meta.setdefault("phases", []).append([stamp, name, text]))

    def note(self, **values: Any) -> None:
        self._put(lambda: self._meta.update(values))

    def burst(self, frames: list[np.ndarray]) -> None:
        if self.enabled:
            frames = list(frames)
            self._put(lambda: self._write_burst(frames))

    def result(self, summary: dict[str, Any]) -> None:
        self._put(lambda: self._meta.setdefault("results", []).append(summary))

    def next_parcel(self) -> None:
        self._put(self._finish_parcel)
        self._put(self._new_parcel)

    def close(self, timeout_s: float = 10.0) -> None:
        self._put(self._finish_parcel)
        self._put(None)
        self._thread.join(timeout_s)

    # -- writer thread ------------------------------------------------------------------
    def _put(self, job, drop: bool = False) -> None:
        try:
            self._jobs.put(job, block=not drop, timeout=None if not drop else 0)
        except queue.Full:
            self.dropped += 1

    def _work(self) -> None:
        while True:
            job = self._jobs.get()
            if job is None:
                return
            try:
                job()
            except Exception as error:  # noqa: BLE001 - recording must never stop the cycle
                self._meta.setdefault("recorder_errors", []).append(repr(error))

    def _folder(self) -> Path:
        return self.root / f"p{self._parcel:02d}"

    def _disk_ok(self) -> bool:
        try:
            self.root.mkdir(parents=True, exist_ok=True)
            free_mb = shutil.disk_usage(self.root).free / 1e6
        except OSError:
            free_mb = 0.0
        if free_mb < self.min_free_mb:
            self.enabled = False
        return self.enabled

    def _new_parcel(self) -> None:
        # Reuse the number when the previous parcel left nothing on disk (empty wait).
        if self._parcel == 0 or self._folder().exists():
            self._parcel += 1
        self._ring.clear()
        self._bursts = 0
        self._meta = {"parcel": self._parcel, "started": time.strftime("%Y-%m-%d %H:%M:%S")}

    def _encode_arrival(self, counter: int, stamp: float, frame: np.ndarray, present: bool) -> None:
        scale = ARRIVAL_WIDTH / frame.shape[1]
        small = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        ok, data = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if ok:
            self._ring.append((counter, stamp, data.tobytes(), present))

    def _write_burst(self, frames: list[np.ndarray]) -> None:
        if not self._disk_ok():
            return
        folder = self._folder() / "burst"
        folder.mkdir(parents=True, exist_ok=True)
        names = []
        for frame in frames:
            name = f"b{self._bursts:02d}.jpg"
            cv2.imwrite(str(folder / name), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            names.append(name)
            self._bursts += 1
        self._meta.setdefault("bursts", []).append(names)

    def _finish_parcel(self) -> None:
        """Write the parcel folder if anything happened (a trigger, a result, a blob seen)."""
        interesting = (
            "fired_frame" in self._meta or "results" in self._meta
            or any(present for _c, _s, _d, present in self._ring)
        )
        if not interesting or not self._disk_ok():
            return
        folder = self._folder() / "arrival"
        folder.mkdir(parents=True, exist_ok=True)
        stamps = {}
        for counter, stamp, data, _present in self._ring:
            (folder / f"f{counter:07d}.jpg").write_bytes(data)
            stamps[f"f{counter:07d}.jpg"] = round(stamp, 4)
        (folder / "stamps.json").write_text(json.dumps(stamps, indent=0), encoding="utf-8")
        self._meta["arrival_frames"] = len(self._ring)
        self._meta["dropped_frames"] = self.dropped
        (self._folder() / "meta.json").write_text(
            json.dumps(self._meta, ensure_ascii=False, indent=1, default=str), encoding="utf-8"
        )
        self._ring.clear()


def result_summary(result) -> dict[str, Any]:
    """JSON-friendly digest of an InspectionResult for meta.json."""
    c = result.classification
    p = c.parcel
    damage = p.damage
    logo = p.logo
    return {
        "decision": c.decision.value,
        "bin": c.destination_bin,
        "reason": c.reason,
        "qr_text": p.qr_text,
        "waybill": p.waybill_code,
        "qr_read_ratio": p.qr_read_ratio,
        "damaged": None if damage is None else damage.damaged,
        "damage_types": [] if damage is None else list(damage.damage_types),
        "damage_reason": None if damage is None else damage.reason,
        "logo_present": None if logo is None else logo.present,
        "focus_laplacian": round(float(result.focus.laplacian), 3),
        "motion": round(float(result.focus.motion), 3),
    }
