"""Qt thread around services.station_cycle.StationCycle for the main window."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable

import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal

from machine_vision.controllers.conveyor_udp import ConveyorLink
from machine_vision.models import InspectionResult
from machine_vision.services.station_cycle import CyclePhase, CycleSettings, StationCycle
from machine_vision.ui.camera_worker import FrameBuffer


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


class AutoCycleThread(QThread):
    phase_changed = pyqtSignal(str, str)
    result_ready = pyqtSignal(object)   # (InspectionResult, storage_error, upload_error)

    def __init__(
        self,
        host: str,
        port: int,
        config: dict,
        frames: FrameBuffer,
        burst: int,
        inspect: Callable[[list[np.ndarray]], InspectionResult],
        record: Callable[[InspectionResult], tuple[str | None, str | None]],
        parent=None,
    ):
        super().__init__(parent)
        self.host, self.port = host, port
        self.config = config
        self.frames, self.burst = frames, burst
        self.inspect = inspect
        self.record = record
        self._stop = threading.Event()

    def request_stop(self) -> None:
        self._stop.set()

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
            storage_error, upload_error = self.record(result)
            self.result_ready.emit((result, storage_error, upload_error))

        cycle = StationCycle(
            link,
            capture=lambda: fresh_burst(self.frames, self.burst),
            process=self.inspect,
            settings=settings,
            on_phase=lambda phase, text: self.phase_changed.emit(phase.value, text),
            on_result=on_result,
        )
        try:
            cycle.run(self._stop)
        finally:
            link.close()
