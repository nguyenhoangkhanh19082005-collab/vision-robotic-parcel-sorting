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
from dataclasses import dataclass, replace
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
    PICKING = "PICKING"          # stopped at the pick sensor, robot taking the parcel
    EJECTING = "EJECTING"        # rejected parcel runs past the sensor and off the belt end
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
    # "camera": the webcam stops the parcel for inspection and the photo sensor sits
    # at the robot pick point (CameraStationCycle). "sensor": one sensor at the camera.
    trigger: str = "sensor"
    pick_stop_delay_ms: int = 0       # sensor edge -> stop at the pick point (firmware AUTO delay)
    # No robot and the delayed stop left the parcel past the sensor: the sensor cannot
    # see it being taken away, so the operator gets this long before the belt runs on.
    manual_pick_s: float = 8.0
    transfer_timeout_s: float = 10.0  # camera -> pick sensor
    # A parcel at the pick point that the camera never inspected (carried past it while
    # the previous parcel was transferred, lần 4) is run back (REV) under the camera for
    # at most this long; 0 disables (old behaviour: MANUAL_REVIEW, off the belt end).
    backup_max_s: float = 12.0
    pick_timeout_s: float = 120.0     # robot (or operator) must take the parcel within this
    # Belt PWM allowed in the automatic cycle (2026-11-23: the camera needs a slow
    # belt; the GUI slider value is clamped into this range).
    speed_min: int = 30
    speed_max: int = 50
    # Camera stop overshoot: back the belt up in short REV pulses until the leading
    # end is inside the view again (0 pulses = off).
    recenter_pulses: int = 3
    recenter_speed: int = 30
    recenter_pulse_s: float = 0.15

    @classmethod
    def from_config(cls, config: dict | None) -> CycleSettings:
        config = dict(config or {})
        if "speed_range" in config:
            config["speed_min"], config["speed_max"] = (int(v) for v in config["speed_range"])
        known = {name: config[name] for name in cls.__dataclass_fields__ if name in config}
        settings = cls(**known)
        speed = min(max(int(settings.speed), settings.speed_min), settings.speed_max)
        if speed != settings.speed:
            settings = replace(settings, speed=speed)
        return settings


def inspection_failure(last_error: str, frame: np.ndarray | None) -> InspectionResult:
    """Result after every burst failed (also used by the replay of recorded runs)."""
    if frame is not None and ("QR" in last_error or "chất lượng" in last_error):
        # The camera works but the code cannot be read on any burst: blurred,
        # torn or covered QR -> reject bin (operator rule 2026-11-23).
        return failure_result(
            f"Không đọc được mã QR (tem mờ, rách hoặc bị che): {last_error}",
            frame,
            Decision.REJECT,
        )
    return failure_result(f"Không kiểm tra được kiện: {last_error}", frame)


def failure_result(
    reason: str, frame: np.ndarray | None, decision: Decision = Decision.MANUAL_REVIEW
) -> InspectionResult:
    """Result when no inspection could be completed (camera/quality/unreadable QR).

    The parcel is still released and logged, so it ends in the reject bin
    instead of blocking the line.
    """
    image = frame if frame is not None else np.zeros((8, 8, 3), dtype=np.uint8)
    return InspectionResult(
        classification=ClassificationResult(decision, decision.value, reason, ParcelData()),
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
        return inspection_failure(last_error, frame)

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


class CameraStationCycle(StationCycle):
    """Two stops per parcel: webcam at the inspection point, photo sensor at the pick point.

    1. Belt forward; ``arrival(timeout)`` (services/arrival.py on live frames) returns
       True when the parcel reaches the middle of the view -> PC sends STOP.
    2. Inspect + record (same as the sensor cycle).
    3. Belt forward; the ESP32 stops it by itself when the pick sensor sees the parcel
       (firmware AUTO mode, ``pick_stop_delay_ms``): repeatable to a few mm.
    4. ``on_pick_ready(result)`` hands the parcel to the robot; the belt stays stopped
       until ``pick_done()`` is True (robot finished) or, without a robot, until the
       sensor clears (parcel taken away).

    Only PASS parcels stop at the pick sensor. REJECT / MANUAL_REVIEW parcels run past
    it with AUTO disabled and fall off the belt end (2026-11-22). A parcel that reaches
    the pick sensor without a camera stop (missed trigger) is logged as MANUAL_REVIEW
    and ejected the same way.
    """

    def __init__(
        self,
        link: Link,
        capture: Callable[[], list[np.ndarray]],
        process: Callable[[list[np.ndarray]], InspectionResult],
        arrival: Callable[[float], bool],
        settings: CycleSettings | None = None,
        on_phase: Callable[[CyclePhase, str], None] | None = None,
        on_result: Callable[[InspectionResult], None] | None = None,
        on_pick_ready: Callable[[InspectionResult], None] | None = None,
        pick_done: Callable[[], bool | None] | None = None,
        rearm: Callable[[], None] | None = None,
        parcel_passed: Callable[[], bool] | None = None,
        view_clear: Callable[[], bool] | None = None,
        overshoot: Callable[[], float | None] | None = None,
        sleep: Callable[[float], None] = time.sleep,
        clock: Callable[[], float] = time.monotonic,
    ):
        super().__init__(link, capture, process, settings, on_phase, on_result, sleep, clock)
        self.arrival = arrival
        # Fresh-frame measurement after the camera stop: how far (view fraction) the
        # leading end sticks out of the view; <= 0 inside, None unknown.
        self.overshoot = overshoot or (lambda: None)
        self.on_pick_ready = on_pick_ready or (lambda _result: None)
        # None from pick_done() means "no robot": fall back to the sensor clearing.
        self.pick_done = pick_done or (lambda: None)
        self.rearm = rearm or (lambda: None)
        # True when the camera saw a parcel leave its view towards the pick point since
        # the last rearm; None (no camera info): every sensor stop counts as a parcel.
        self.parcel_passed = parcel_passed or (lambda: True)
        # True when nothing is under the camera: backing up must not push a parcel just
        # placed at the belt head off the upstream end. None: unknown -> no back-up.
        self.view_clear = view_clear or (lambda: False)

    def _status_checked(self):
        status = self.link.status()
        if status.state == "REV":
            raise CycleFault("Băng tải đang chạy lùi ngoài chu trình")
        return status

    def _wait_for_camera(self, stop: threading.Event) -> str | None:
        """Belt forward until the camera fires ("camera") or a parcel slips to the pick sensor."""
        s = self.settings
        self._set(
            CyclePhase.RUNNING,
            f"Băng tải chạy (PWM {s.speed}), camera chờ kiện nằm trọn khung hình",
        )
        while not stop.is_set():
            if self.arrival(s.poll_s):
                self.link.drive("STOP")
                return "camera"
            self._check_events(0.0)
            status = self._status_checked()
            if status.state == "STOP":
                if (status.sensor or status.autostopped) and not self.parcel_passed():
                    # Nothing left the camera view: a hand / lifted parcel crossed the pick
                    # sensor (2026-11-25 lần 3 recorded a phantom MANUAL_REVIEW). Go on.
                    self._set(
                        CyclePhase.RUNNING,
                        "Cảm biến điểm gắp bị che nhưng camera không thấy kiện nào đi qua: "
                        "bỏ qua, băng tải chạy tiếp",
                    )
                    self.link.drive("FWD", s.speed)
                    continue
                if status.sensor:
                    return "missed"
                if status.autostopped:
                    # Firmware v2.1 delayed pick stop: the missed parcel is already past
                    # the sensor (2026-11-25: this raised "STOP ngoài chu trình").
                    return "missed_past"
                raise CycleFault("Băng tải dừng ngoài chu trình (STOP tay / watchdog)")
        return None

    def _recenter(self, stop: threading.Event, direction: str = "REV") -> None:
        """The belt coasted past the view after the camera STOP: back up in short pulses.

        REV moves the parcel away from the pick sensor (upstream), so the firmware
        AUTO stop is not involved. Each pulse is followed by a STOP, a settle time and
        a new measurement; after ``recenter_pulses`` the inspection runs anyway (the
        label is normally still in view). ``direction`` "FWD" after a stop made while
        backing up (the parcel coasted upstream).
        """
        s = self.settings
        for pulse in range(s.recenter_pulses):
            if stop.is_set():
                return
            self.sleep(s.settle_s)
            over = self.overshoot()
            if over is None or over <= 0:
                return
            self._set(
                CyclePhase.SETTLING,
                f"Kiện dừng lố khung {over:.0%}: chỉnh lại băng tải (lần {pulse + 1})",
            )
            self.link.drive(direction, s.recenter_speed)
            self.sleep(s.recenter_pulse_s)
            self.link.drive("STOP")

    def _back_up(self, stop: threading.Event) -> bool:
        """Run a parcel that reached the pick point uninspected back under the camera.

        True when the camera stopped it (belt STOP sent); False when backing up is off,
        the view is not clear, or the camera did not catch it in ``backup_max_s``.
        """
        s = self.settings
        if s.backup_max_s <= 0 or stop.is_set() or not self.view_clear():
            return False
        self._set(
            CyclePhase.RUNNING,
            "Kiện tới điểm gắp mà chưa được kiểm tra: chạy lùi đưa kiện về dưới camera",
        )
        self.rearm()
        self.link.drive("REV", s.speed)
        deadline = self.clock() + s.backup_max_s
        while not stop.is_set():
            if self.arrival(s.poll_s):
                self.link.drive("STOP")
                return True
            self._check_events(0.0)
            if self.link.status().state != "REV":
                raise CycleFault("Băng tải dừng khi đang lùi kiện về camera")
            if self.clock() > deadline:
                break
        self.link.drive("STOP")
        return False

    def _transfer_to_pick(self, stop: threading.Event) -> bool:
        s = self.settings
        self._set(CyclePhase.RELEASING, "Chuyển kiện tới điểm gắp (cảm biến)")
        self.link.drive("FWD", s.speed)
        deadline = self.clock() + s.transfer_timeout_s
        seen_at: float | None = None
        while not stop.is_set():
            self._check_events(s.poll_s)
            status = self._status_checked()
            if status.state == "STOP":
                # With a delayed pick stop the parcel may already be past the sensor:
                # the firmware (v2.1) reports that the stop was its AUTO stop.
                if status.sensor or status.autostopped or seen_at is not None:
                    return True
                raise CycleFault("Băng tải dừng trước khi kiện tới điểm gắp")
            if status.sensor and seen_at is None:
                seen_at = self.clock()
            # Firmware did not stop (lost AUTO state): stop from the PC after the same
            # delay, even if the parcel has meanwhile cleared the sensor.
            if seen_at is not None and self.clock() - seen_at > s.pick_stop_delay_ms / 1000 + s.stop_grace_s:
                self.link.drive("STOP")
                return True
            if self.clock() > deadline:
                self.link.drive("STOP")
                raise CycleFault(
                    f"Kiện không tới cảm biến điểm gắp sau {s.transfer_timeout_s:.0f} s"
                )
        return False

    def _wait_pick(self, stop: threading.Event, result: InspectionResult) -> bool:
        s = self.settings
        self._set(CyclePhase.PICKING, "Kiện ở điểm gắp, chờ robot gắp")
        self.on_pick_ready(result)
        deadline = self.clock() + s.pick_timeout_s
        # Delayed pick stop: the parcel may rest past the sensor (not visible to it).
        on_sensor = self.link.status().sensor
        manual_until = self.clock() + s.manual_pick_s
        while not stop.is_set():
            self._check_events(s.poll_s)
            done = self.pick_done()
            if done is None:  # no robot: wait for the parcel to be taken away
                if on_sensor:
                    done = not self.link.status().sensor
                else:
                    # Past the sensor nothing sees the parcel go. manual_pick_s <= 0:
                    # wait for the operator's confirmation (pick_done() -> True); the
                    # old 8 s timer ran the belt with the parcel still on it (2026-11-25).
                    done = s.manual_pick_s > 0 and self.clock() > manual_until
            elif done and self.link.status().sensor:
                # Robot/operator reports success but the parcel is still on the pick
                # sensor (suction slipped): running the belt would carry it away unsorted.
                raise CycleFault("Báo gắp xong nhưng kiện vẫn ở cảm biến điểm gắp")
            if done:
                return True
            if self.clock() > deadline:
                raise CycleFault(f"Kiện chưa được gắp sau {s.pick_timeout_s:.0f} s")
        return False

    def _eject(self, stop: threading.Event, past_sensor: bool = False) -> bool:
        """Rejected parcel: AUTO off, run it past the pick sensor and off the belt end.

        ``past_sensor``: the parcel already stopped beyond the sensor (missed by the
        camera, delayed AUTO stop); the running belt drops it, nothing to wait for.
        """
        s = self.settings
        self._set(CyclePhase.EJECTING, "Kiện không đạt chạy qua cảm biến, rơi khỏi băng tải")
        if past_sensor:
            self.link.drive("FWD", s.speed)
            return not stop.is_set()
        self.link.set_auto(False, 0)
        self.link.drive("FWD", s.speed)
        seen = False
        deadline = self.clock() + s.transfer_timeout_s
        while not stop.is_set():
            self._check_events(s.poll_s)
            status = self._status_checked()
            if status.state != "FWD":
                raise CycleFault("Băng tải dừng khi đang đẩy kiện lỗi ra")
            if status.sensor:
                seen = True
            elif seen:
                break  # the parcel has passed the sensor; the running belt drops it
            if self.clock() > deadline:
                self.link.drive("STOP")
                raise CycleFault(
                    f"Kiện lỗi không đi qua cảm biến sau {s.transfer_timeout_s:.0f} s"
                )
        # Re-arm the pick stop for the next parcel (the belt keeps running).
        self.link.set_auto(True, s.pick_stop_delay_ms)
        return not stop.is_set()

    def run(self, stop: threading.Event) -> None:
        s = self.settings
        try:
            self.link.ping()
            self.link.drive("STOP")
            self.link.set_auto(True, s.pick_stop_delay_ms)
            self.link.drain_events()
            if self.link.status().sensor:
                raise CycleFault("Có kiện đang nằm ở điểm gắp: lấy kiện ra rồi chạy lại")
            self.rearm()
            self.link.drive("FWD", s.speed)
            while not stop.is_set():
                outcome = self._wait_for_camera(stop)
                if outcome is None:
                    break
                if outcome in ("missed", "missed_past") and self._back_up(stop):
                    outcome = "camera_backed_up"
                if outcome in ("camera", "camera_backed_up"):
                    self._recenter(stop, "FWD" if outcome == "camera_backed_up" else "REV")
                    result = self._inspect(stop)
                else:
                    result = failure_result(
                        "Kiện tới điểm gắp mà camera không dừng được để kiểm tra", None
                    )
                self.parcels += 1
                self.on_result(result)
                if stop.is_set():
                    break
                if result.classification.decision != Decision.PASS:
                    # Only PASS parcels are picked; everything else falls off the belt end.
                    if not self._eject(stop, past_sensor=outcome == "missed_past"):
                        break
                    self.rearm()
                    continue
                if outcome in ("camera", "camera_backed_up") and not self._transfer_to_pick(stop):
                    break
                if not self._wait_pick(stop, result):
                    break
                self.rearm()
                self.link.drive("FWD", s.speed)
        except Exception as exc:  # noqa: BLE001 - any failure must stop the belt
            self._shutdown()
            self._set(CyclePhase.FAULT, str(exc))
            return
        self._shutdown()
        self._set(CyclePhase.IDLE, f"Đã dừng chu trình tự động ({self.parcels} kiện)")
