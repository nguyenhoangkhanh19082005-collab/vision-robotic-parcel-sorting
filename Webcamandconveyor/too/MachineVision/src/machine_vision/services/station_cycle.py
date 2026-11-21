"""Automatic station cycle: sensor -> stop -> inspect -> record -> release.

One parcel at a time, as required by the station:

1. The belt runs forward. The photo sensor sees a parcel and the ESP32 stops the
   belt by itself (firmware ``AUTO`` mode, no network latency). If that event is
   lost, the PC stops the belt once the sensor has been active for longer than the
   configured delay plus a grace period.
2. After a short settle time (belt vibration), a *fresh* burst is captured and
   inspected: QR, logo, surface damage, database lookup.
3. The result is stored (evidence + Supabase ``sorting_logs``) by ``process``.
4. The belt is released forward toward the pick point. A PASS parcel goes to the
   chute of its order, anything else to the reject chute; the robot reads that
   from the database row. The cycle waits for the sensor to clear before arming
   for the next parcel.

The class has no Qt dependency and is driven by ``run(stop_event)`` in a worker
thread; GUI integration lives in ``ui/auto_cycle.py``. ``STATUS`` is polled every
``poll_s`` and is the source of truth (UDP events may be lost); polling also keeps
the firmware watchdog (3 s) fed while the motor runs.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from typing import Any, Protocol

import numpy as np

from machine_vision.models import (
    ClassificationResult,
    Decision,
    FocusMetrics,
    InspectionResult,
    ParcelData,
)


class CyclePhase(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"          # belt forward, waiting for a parcel
    SETTLING = "SETTLING"        # stopped at the camera, waiting for vibration to fade
    INSPECTING = "INSPECTING"
    RELEASING = "RELEASING"      # forward again until the sensor clears
    FAULT = "FAULT"


class CycleFault(RuntimeError):
    pass


class Link(Protocol):
    def ping(self) -> None: ...
    def status(self) -> Any: ...
    def set_auto(self, enabled: bool, delay_ms: int) -> None: ...
    def drive(self, action: str, speed: int = 0) -> Any: ...
    def next_event(self, timeout_s: float) -> Any: ...
    def drain_events(self) -> None: ...


@dataclass(frozen=True)
class CycleSettings:
    speed: int = 150
    auto_stop_delay_ms: int = 300     # sensor edge -> belt stop (parcel under camera)
    stop_grace_s: float = 0.8         # PC fallback stop if the AUTOSTOP was missed
    settle_s: float = 0.4
    quality_retries: int = 2          # fresh bursts tried after a quality hold
    release_timeout_s: float = 6.0    # parcel must leave the sensor within this time
    poll_s: float = 0.25

    @classmethod
    def from_config(cls, config: dict | None) -> CycleSettings:
        config = config or {}
        known = {name: config[name] for name in cls.__dataclass_fields__ if name in config}
        return cls(**known)


def failure_result(reason: str, frame: np.ndarray | None) -> InspectionResult:
    """MANUAL_REVIEW result when no inspection could be completed (camera/quality).

    The parcel is still released and logged, so it ends in the reject chute
    instead of blocking the line.
    """
    image = frame if frame is not None else np.zeros((8, 8, 3), dtype=np.uint8)
    return InspectionResult(
        classification=ClassificationResult(
            Decision.MANUAL_REVIEW, "MANUAL_REVIEW", reason, ParcelData()
        ),
        focus=FocusMetrics(0.0, 0.0),
        original_frame=image,
        label_image=image,
    )


class StationCycle:
    def __init__(
        self,
        link: Link,
        capture: Callable[[], list[np.ndarray]],
        process: Callable[[list[np.ndarray]], InspectionResult],
        settings: CycleSettings | None = None,
        on_phase: Callable[[CyclePhase, str], None] | None = None,
        on_result: Callable[[InspectionResult], None] | None = None,
        sleep: Callable[[float], None] = time.sleep,
        clock: Callable[[], float] = time.monotonic,
    ):
        self.link = link
        self.capture = capture
        self.process = process
        self.settings = settings or CycleSettings()
        self.on_phase = on_phase or (lambda _phase, _text: None)
        self.on_result = on_result or (lambda _result: None)
        self.sleep = sleep
        self.clock = clock
        self.phase = CyclePhase.IDLE
        self.parcels = 0

    # -- helpers ------------------------------------------------------------
    def _set(self, phase: CyclePhase, text: str) -> None:
        self.phase = phase
        self.on_phase(phase, text)

    def _check_events(self, timeout_s: float) -> None:
        event = self.link.next_event(timeout_s)
        if event is not None and event.is_fault:
            raise CycleFault(f"ESP32 báo dừng an toàn: {event.raw}")

    # -- phases ---------------------------------------------------------------
    def _wait_for_parcel(self, stop: threading.Event) -> bool:
        """Return True with the belt stopped and a parcel at the sensor."""
        s = self.settings
        self._set(CyclePhase.RUNNING, "Băng tải chạy, chờ cảm biến thấy kiện")
        seen_at: float | None = None
        while not stop.is_set():
            self._check_events(s.poll_s)
            status = self.link.status()
            if status.state == "REV":
                raise CycleFault("Băng tải đang chạy lùi ngoài chu trình")
            if status.state == "STOP":
                if status.sensor:
                    return True  # firmware AUTO stop (event may have been lost)
                raise CycleFault("Băng tải dừng ngoài chu trình (STOP tay / watchdog)")
            if status.sensor:
                seen_at = seen_at if seen_at is not None else self.clock()
                if self.clock() - seen_at > s.auto_stop_delay_ms / 1000 + s.stop_grace_s:
                    self.link.drive("STOP")
                    return True
            else:
                seen_at = None
        return False

    def _inspect(self, stop: threading.Event) -> InspectionResult:
        s = self.settings
        self._set(CyclePhase.SETTLING, "Kiện đã dừng dưới camera, chờ ổn định")
        self.sleep(s.settle_s)
        last_error = "không có ảnh"
        frame = None
        for attempt in range(s.quality_retries + 1):
            if stop.is_set():
                break
            self._set(CyclePhase.INSPECTING, f"Đang kiểm tra (lần {attempt + 1})")
            try:
                frames = self.capture()
                frame = frames[-1] if frames else frame
                return self.process(frames)
            except Exception as exc:  # noqa: BLE001 - quality hold, camera error: retry with a new burst
                last_error = str(exc)
        return failure_result(f"Không kiểm tra được kiện: {last_error}", frame)

    def _release(self, stop: threading.Event) -> None:
        s = self.settings
        self._set(CyclePhase.RELEASING, "Cho kiện chạy tới điểm gắp")
        self.link.drive("FWD", s.speed)
        deadline = self.clock() + s.release_timeout_s
        while not stop.is_set():
            self._check_events(s.poll_s)
            status = self.link.status()
            if not status.sensor:
                return
            if status.state != "FWD":
                raise CycleFault("Băng tải dừng khi kiện chưa rời cảm biến")
            if self.clock() > deadline:
                self.link.drive("STOP")
                raise CycleFault("Kiện không rời cảm biến (kẹt?), đã dừng băng tải")

    def _handle_parcel(self, stop: threading.Event) -> None:
        result = self._inspect(stop)
        self.parcels += 1
        self.on_result(result)
        if not stop.is_set():
            self._release(stop)

    # -- main loop --------------------------------------------------------------
    def run(self, stop: threading.Event) -> None:
        s = self.settings
        try:
            self.link.ping()
            self.link.drive("STOP")
            self.link.set_auto(True, s.auto_stop_delay_ms)
            self.link.drain_events()
            if self.link.status().sensor:
                # A parcel already waits at the camera: inspect it first.
                self._handle_parcel(stop)
            else:
                self.link.drive("FWD", s.speed)
            while not stop.is_set():
                if not self._wait_for_parcel(stop):
                    break
                self._handle_parcel(stop)
        except Exception as exc:  # noqa: BLE001 - any failure must stop the belt
            self._shutdown()
            self._set(CyclePhase.FAULT, str(exc))
            return
        self._shutdown()
        self._set(CyclePhase.IDLE, f"Đã dừng chu trình tự động ({self.parcels} kiện)")

    def _shutdown(self) -> None:
        for action in (lambda: self.link.drive("STOP"), lambda: self.link.set_auto(False, 0)):
            try:
                action()
            except Exception:  # noqa: BLE001, S110 - best effort, link may be down
                pass
