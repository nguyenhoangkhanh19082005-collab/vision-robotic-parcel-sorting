"""Qt thread around services.station_cycle.StationCycle for the main window."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import TextIO

import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal

from machine_vision.controllers.conveyor_udp import ConveyorLink
from machine_vision.models import InspectionResult
from machine_vision.services.arrival import ArrivalDetector, ArrivalSettings
from machine_vision.services.station_cycle import (
    CameraStationCycle,
    CyclePhase,
    CycleSettings,
    StationCycle,
)
from machine_vision.ui.camera_worker import FrameBuffer
from machine_vision.ui.run_recorder import RunRecorder, result_summary


def fresh_burst(frames: FrameBuffer, count: int, timeout_s: float = 4.0) -> list[np.ndarray]:
    """Drop buffered frames (taken while the belt moved) and wait for ``count`` new ones."""
    frames.clear()
    deadline = time.monotonic() + timeout_s
    while len(frames) < count:
        if time.monotonic() > deadline:
            raise RuntimeError(
                f"Camera chỉ trả {len(frames)}/{count} khung hình trong {timeout_s:.0f} s"
            )
        time.sleep(0.02)
    return frames.snapshot()


ARRIVAL_LOG_HEADER = (
    "time,frame,present,inside,start,end,velocity,frame_interval,armed,fired,"
    "moved,gain,static_ratio,absorbed,blob_l,blob_tex,glare_skipped\n"
)


def _log_row(log: TextIO | None, version: int, stamp: float, detector: ArrivalDetector, fired: bool) -> None:
    """One CSV line per analysed frame (logs/arrival/): tuning data for latency_s / speed."""
    if log is None:
        return
    st, blob = detector.state, detector.state.blob
    log.write(
        f"{stamp:.3f},{version},{int(st.present)},{int(st.inside)},"
        f"{blob.start if blob else -1:.3f},{blob.end if blob else -1:.3f},"
        f"{st.velocity:.3f},{st.frame_interval:.3f},{int(st.armed)},{int(fired)},"
        f"{int(st.moved)},{detector.last_gain:.2f},{st.static_ratio:.2f},{st.absorbed},"
        f"{st.blob_lightness if blob else 0:.0f},{st.blob_texture if blob else 0:.1f},{st.glare_skipped}\n"
    )


def _detector_snapshot(detector: ArrivalDetector) -> dict:
    st, blob = detector.state, detector.state.blob
    return {
        "start": None if blob is None else round(blob.start, 4),
        "end": None if blob is None else round(blob.end, 4),
        "centre": None if blob is None else round(blob.centre, 4),
        "velocity": round(st.velocity, 4),
        "frame_interval": round(st.frame_interval, 4),
        "gain": round(detector.last_gain, 3),
        "static_ratio": round(st.static_ratio, 3),
        "blob_lightness": round(st.blob_lightness, 1),
        "blob_texture": round(st.blob_texture, 1),
        "overlong": st.overlong,
    }


def camera_arrival(
    frames: FrameBuffer, detector: ArrivalDetector, log: TextIO | None = None,
    recorder: RunRecorder | None = None,
) -> Callable[[float], bool]:
    """Blocking ``arrival(timeout)`` for CameraStationCycle: runs the detector on new frames."""
    seen = {"version": -1}

    def wait(timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            latest = frames.latest()
            if latest is not None and latest[0] != seen["version"]:
                seen["version"] = latest[0]
                fired = detector.update(latest[2], latest[1])
                _log_row(log, latest[0], latest[1], detector, fired)
                if recorder is not None:
                    recorder.frame(latest[0], latest[1], latest[2], detector.state.present)
                    if fired:
                        recorder.fired(latest[0], latest[1], _detector_snapshot(detector), frames.latest)
                if fired:
                    return True
                if detector.light_fault(latest[1]):
                    raise RuntimeError(
                        f"Ánh sáng khác hẳn lúc chụp ảnh nền (độ sáng x{detector.last_gain:.2f}): "
                        "đèn bị tắt hoặc bị che. Bật lại đèn rồi chạy lại tự động; nếu đã đổi "
                        "đèn hẳn thì lấy hết kiện khỏi camera và bấm CHỤP NỀN TRỐNG."
                    )
                # Static mismatches are learnt within static_absorb_s; this only fires
                # for a changing full-view difference (flicker, belt texture after a move).
                if detector.background_mismatch(latest[1], hold_s=6.0):
                    # Raised into CameraStationCycle.run: belt STOP + FAULT with this text.
                    raise RuntimeError(
                        "Camera thấy cả khung hình khác ảnh nền băng tải trống (camera bị xê dịch "
                        "hoặc đổi đèn), nên không phát hiện được kiện. Lấy hết kiện khỏi vùng "
                        "camera, bấm CHỤP NỀN TRỐNG rồi chạy lại tự động."
                    )
            else:
                time.sleep(0.003)
        return False

    return wait


def camera_view_clear(frames: FrameBuffer, detector: ArrivalDetector) -> Callable[[], bool]:
    """True when no parcel-like blob is under the camera now (newest frame)."""

    def clear() -> bool:
        latest = frames.latest()
        return latest is not None and detector.measure(latest[2]) is None

    return clear


def camera_overshoot(
    frames: FrameBuffer, detector: ArrivalDetector, timeout_s: float = 1.0
) -> Callable[[], float | None]:
    """After the camera STOP: how far the leading end sticks out of the view (fresh frame)."""

    def measure() -> float | None:
        latest = frames.latest()
        start = latest[0] if latest is not None else -1
        deadline = time.monotonic() + timeout_s
        # Two new frames: the first may have been exposed while the belt still moved.
        while time.monotonic() < deadline:
            latest = frames.latest()
            if latest is not None and latest[0] >= start + 2:
                return detector.overshoot(detector.measure(latest[2]), detector.state.heading)
            time.sleep(0.005)
        return None

    return measure


class AutoCycleThread(QThread):
    phase_changed = pyqtSignal(str, str)
    result_ready = pyqtSignal(object)   # (InspectionResult, storage_error, upload_error)
    parcel_at_pick = pyqtSignal(object)  # InspectionResult of the parcel now at the pick point

    def __init__(
        self,
        host: str,
        port: int,
        config: dict,
        frames: FrameBuffer,
        burst: int,
        inspect: Callable[[list[np.ndarray]], InspectionResult],
        record: Callable[[InspectionResult], tuple[str | None, str | None]],
        background: np.ndarray | None = None,
        external_pick: Callable[[], bool] | None = None,
        log_dir: Path | None = None,
        parent=None,
    ):
        super().__init__(parent)
        self.host, self.port = host, port
        self.config = config
        self.frames, self.burst = frames, burst
        self.inspect = inspect
        self.record = record
        # Camera trigger needs the empty-belt reference (damage background).
        self.background = background
        # True when a robot takes parcels at the pick point (wait for notify_pick_done);
        # False: wait until the pick sensor clears (operator lifts the parcel).
        self.external_pick = external_pick or (lambda: False)
        self.log_dir = log_dir
        self._stop = threading.Event()
        self._picked = threading.Event()
        self._manual_picked = threading.Event()

    def request_stop(self) -> None:
        self._stop.set()

    def notify_pick_done(self) -> None:
        """Robot finished with the parcel at the pick point (thread-safe)."""
        self._picked.set()

    def notify_manual_pick(self) -> None:
        """Operator lifted the parcel at the pick point (no robot; GUI button, thread-safe)."""
        self._manual_picked.set()

    def _pick_done(self) -> bool | None:
        if not self.external_pick():
            # No robot: True once the operator confirmed, else None (sensor / timer rules).
            return True if self._manual_picked.is_set() else None
        return self._picked.is_set()

    def _on_pick_ready(self, result: InspectionResult) -> None:
        self._picked.clear()
        self._manual_picked.clear()
        self.parcel_at_pick.emit(result)

    def run(self) -> None:
        settings = CycleSettings.from_config(self.config)
        try:
            link = ConveyorLink(
                self.host, self.port, int(self.config.get("event_port", 8121))
            )
        except (OSError, ValueError) as exc:
            self.phase_changed.emit(CyclePhase.FAULT.value, f"Không mở được cổng sự kiện: {exc}")
            return

        def on_result(result: InspectionResult) -> None:
            # Every parcel is recorded, including MANUAL_REVIEW after a failed
            # inspection, before the belt releases it.
            if recorder is not None:
                recorder.result(result_summary(result))
            storage_error, upload_error = self.record(result)
            self.result_ready.emit((result, storage_error, upload_error))

        def on_phase(phase: CyclePhase, text: str) -> None:
            if recorder is not None:
                recorder.phase(phase.value, text)
                if phase in (CyclePhase.SETTLING, CyclePhase.FAULT):
                    recorder.note(**{f"phase_{phase.value.lower()}": text})
            self.phase_changed.emit(phase.value, text)

        def capture() -> list[np.ndarray]:
            burst = fresh_burst(self.frames, self.burst)
            if recorder is not None:
                recorder.burst(burst)
            return burst

        log: TextIO | None = None
        # 2026-11-25: every parcel of an automatic run is kept for replay tests
        # (logs/runs/, services in ui/run_recorder.py); station_cycle.record_runs: false
        # turns it off.
        recorder: RunRecorder | None = None
        if settings.trigger == "camera" and self.log_dir is not None and self.config.get("record_runs", True):
            recorder = RunRecorder(Path(self.log_dir) / "runs")
        if settings.trigger == "camera":
            if self.background is None:
                link.close()
                if recorder is not None:
                    recorder.close()
                self.phase_changed.emit(
                    CyclePhase.FAULT.value, "Chưa có ảnh nền băng tải trống cho camera phát hiện kiện"
                )
                return
            detector = ArrivalDetector(
                self.background, ArrivalSettings.from_config(self.config.get("arrival"))
            )

            def rearm() -> None:
                # Called before every parcel: the previous one has been carried to the
                # pick point / off the belt. Do not wait for an empty view (2026-11-24:
                # it never emptied, so only the first parcel of a run was stopped).
                detector.rearm()
                if recorder is not None:
                    recorder.next_parcel()

            detector.reset(armed=True)
            if recorder is not None:
                recorder.run_info(self.background, {
                    "arrival": dict(self.config.get("arrival") or {}),
                    "speed": settings.speed, "latency_s": detector.settings.latency_s,
                    "pick_stop_delay_ms": settings.pick_stop_delay_ms,
                })
            log = self._open_arrival_log()
            cycle = CameraStationCycle(
                link, capture, self.inspect, camera_arrival(self.frames, detector, log, recorder),
                settings=settings, on_phase=on_phase, on_result=on_result,
                on_pick_ready=self._on_pick_ready, pick_done=self._pick_done, rearm=rearm,
                parcel_passed=lambda: detector.state.passed_downstream,
                view_clear=camera_view_clear(self.frames, detector),
                overshoot=camera_overshoot(self.frames, detector),
            )
        else:
            cycle = StationCycle(
                link, capture=capture, process=self.inspect, settings=settings,
                on_phase=on_phase, on_result=on_result,
            )
        try:
            cycle.run(self._stop)
        finally:
            link.close()
            if log is not None:
                log.close()
            if recorder is not None:
                recorder.close()

    def _open_arrival_log(self) -> TextIO | None:
        if self.log_dir is None:
            return None
        try:
            folder = Path(self.log_dir) / "arrival"
            folder.mkdir(parents=True, exist_ok=True)
            log = (folder / time.strftime("arrival_%Y%m%d_%H%M%S.csv")).open(
                "w", encoding="utf-8", buffering=1
            )
        except OSError:
            return None
        log.write(ARRIVAL_LOG_HEADER)
        return log
