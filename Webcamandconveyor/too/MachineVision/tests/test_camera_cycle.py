"""Two-stop cycle: webcam stops the parcel for inspection, sensor stops it at the pick point."""

from __future__ import annotations

import threading
from dataclasses import replace
import time

import numpy as np
import pytest

from machine_vision.controllers.conveyor_udp import ConveyorLink
from machine_vision.controllers.fake_esp32 import FakeConveyorEsp32
from machine_vision.models import Decision
from machine_vision.services.station_cycle import (
    CameraStationCycle,
    CyclePhase,
    CycleSettings,
    failure_result,
)

CAMERA = 0.5          # camera view centre along the belt
PICK = 1.2            # pick sensor (FakeConveyorEsp32.sensor_position)
SETTINGS = CycleSettings(
    speed=150, trigger="camera", pick_stop_delay_ms=0, stop_grace_s=0.3, settle_s=0.02,
    quality_retries=0, transfer_timeout_s=4.0, pick_timeout_s=3.0, poll_s=0.03,
)


@pytest.fixture
def rig():
    esp = FakeConveyorEsp32(event_port=0, belt_speed=0.6, parcel_length=0.2,
                            sensor_position=PICK)
    link = ConveyorLink("127.0.0.1", esp.address[1], event_port=0, timeout_s=0.3)
    esp.event_port = link.event_port
    esp.start()
    yield esp, link
    link.close()
    esp.close()


class SimCamera:
    """Fires once per parcel when its centre reaches CAMERA (what ArrivalDetector does)."""

    def __init__(self, esp, enabled=True):
        self.esp, self.enabled, self.fired = esp, enabled, set()

    def __call__(self, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.esp._lock:
                travel = self.esp.travel
                centres = [p - self.esp.parcel_length / 2 for p in self.esp.parcels]
            for centre in centres:
                parcel_id = round(centre - travel, 2)  # constant while the parcel rides the belt
                if self.enabled and CAMERA <= centre <= CAMERA + 0.1 and parcel_id not in self.fired:
                    self.fired.add(parcel_id)
                    return True
            time.sleep(0.005)
        return False


def _result(decision: str = "PASS"):
    result = failure_result("ok", None)
    result.classification.decision = Decision[decision]
    return result


def _run(esp, link, parcels, camera_enabled=True, robot=True, pick_delay=0.2, gap=1.0,
         decisions=None, settings=SETTINGS):
    log = {"inspected_at": [], "picked_at": [], "results": [], "phases": []}
    stop = threading.Event()
    pending = {}
    decisions = list(decisions or ["PASS"] * parcels)

    def process(frames):
        log["inspected_at"].append((esp.state, [round(c, 2) for c in esp.positions()]))
        return _result(decisions[len(log["inspected_at"]) - 1])

    def on_pick_ready(result):
        log["picked_at"].append((esp.state, esp.sensor))
        pending["t"] = time.monotonic() + pick_delay

    def pick_done():
        if robot == "manual":   # no robot, operator presses ĐÃ LẤY KIỆN after pick_delay
            if "t" in pending and time.monotonic() > pending["t"]:
                esp.take_at_sensor()
                pending.clear()
                return True
            if "t" in pending:
                log.setdefault("belt_while_waiting", set()).add(esp.state)
            return None
        if not robot:
            if "t" in pending and time.monotonic() > pending["t"]:
                esp.take_at_sensor()  # operator removes the parcel
            return None
        if "t" in pending and time.monotonic() > pending["t"]:
            esp.take_at_sensor()  # robot lifted it ...
            pending.clear()
            pending["lifted"] = True
        # ... and reports done once the parcel is off the sensor (drop + home take seconds).
        if pending.get("lifted") and not esp.sensor:
            pending.clear()
            return True
        return False

    def on_phase(phase, text):
        log["phases"].append((phase, text))
        if phase == CyclePhase.RUNNING and len(log["results"]) >= parcels:
            stop.set()

    cycle = CameraStationCycle(
        link, capture=lambda: [np.zeros((4, 4, 3), np.uint8)], process=process,
        arrival=SimCamera(esp, camera_enabled), settings=settings, on_phase=on_phase,
        on_result=log["results"].append, on_pick_ready=on_pick_ready, pick_done=pick_done,
    )
    esp.feed(parcels, gap=gap, start=0.1)
    thread = threading.Thread(target=cycle.run, args=(stop,))
    thread.start()
    thread.join(25)
    stop.set()
    thread.join(5)
    return log


def test_each_parcel_stops_at_camera_then_at_pick_sensor(rig):
    esp, link = rig
    log = _run(esp, link, 3)
    assert len(log["results"]) == 3
    assert len(log["inspected_at"]) == 3  # every parcel was stopped by the camera
    for state, centres in log["inspected_at"]:
        assert state == "STOP"
        # A parcel sits under the camera, none has reached the pick point yet.
        assert any(abs(c - CAMERA) < 0.15 for c in centres)
    assert log["picked_at"] == [("STOP", True)] * 3
    assert esp.events.count("EVT:AUTOSTOP:1") == 1  # stop at the pick sensor by the ESP32
    assert esp.state == "STOP" and not esp.auto
    assert log["phases"][-1][0] == CyclePhase.IDLE


def test_missed_camera_trigger_runs_past_the_sensor_and_off_the_belt(rig):
    esp, link = rig
    log = _run(esp, link, 1, camera_enabled=False)
    assert log["inspected_at"] == []
    result = log["results"][0]
    assert result.classification.decision == Decision.MANUAL_REVIEW
    assert "camera" in result.classification.reason
    assert log["picked_at"] == []                     # never handed to the robot
    assert "AUTO:0:0" in esp.commands                 # pick stop disabled to let it pass
    assert esp.commands[-1].startswith("AUTO:0") or "AUTO:1:0" in esp.commands
    assert any(e.startswith("EVT:SENSOR:0") for e in esp.events)  # it left the sensor


def test_rejected_parcel_falls_off_and_next_pass_parcel_is_picked(rig):
    esp, link = rig
    log = _run(esp, link, 2, decisions=["REJECT", "PASS"])
    assert len(log["inspected_at"]) == 2
    assert [r.classification.decision.value for r in log["results"]] == ["REJECT", "PASS"]
    assert log["picked_at"] == [("STOP", True)]        # only the PASS parcel stopped there
    # AUTO was switched off for the reject and back on before the PASS parcel arrived.
    off = esp.commands.index("AUTO:0:0")
    assert "AUTO:1:0" in esp.commands[off:]


def test_without_robot_waits_for_the_parcel_to_be_removed(rig):
    esp, link = rig
    log = _run(esp, link, 2, robot=False)
    assert len(log["results"]) == 2
    assert len(log["inspected_at"]) == 2
    assert log["picked_at"] == [("STOP", True)] * 2


def test_parcel_not_picked_in_time_faults_and_stops(rig):
    esp, link = rig
    log = _run(esp, link, 1, pick_delay=99)
    assert log["phases"][-1][0] == CyclePhase.FAULT
    assert "chưa được gắp" in log["phases"][-1][1]
    assert esp.state == "STOP"


def test_parcels_closer_than_camera_to_pick_are_never_mislabelled(rig):
    """Second parcel is carried past the camera while the first goes to the pick point.

    It must reach the pick point as MANUAL_REVIEW and run off the belt, never
    inherit the first parcel's result.
    """
    esp, link = rig
    log = _run(esp, link, 2, gap=0.35)
    reasons = [r.classification.reason for r in log["results"]]
    assert len(log["inspected_at"]) == 1
    assert log["results"][0].classification.decision == Decision.PASS
    assert "camera" in reasons[1]
    assert log["picked_at"] == [("STOP", True)]  # only the inspected PASS parcel


def test_robot_done_but_parcel_still_on_sensor_faults(rig):
    esp, link = rig
    stop = threading.Event()
    phases = []
    cycle = CameraStationCycle(
        link, capture=list, process=lambda f: _result("PASS"),
        arrival=SimCamera(esp), settings=SETTINGS,
        on_phase=lambda p, t: phases.append((p, t)),
        pick_done=lambda: True,  # robot claims success without lifting the parcel
    )
    esp.feed(1, start=0.1)
    thread = threading.Thread(target=cycle.run, args=(stop,))
    thread.start()
    thread.join(15)
    stop.set()
    assert phases[-1][0] == CyclePhase.FAULT
    assert "vẫn ở cảm biến" in phases[-1][1]
    assert esp.state == "STOP"


class _StubLink:
    def __init__(self):
        self.commands = []

    def drive(self, action, speed=0):
        self.commands.append(f"{action}:{speed}")


def test_overshoot_after_camera_stop_is_backed_up_in_short_rev_pulses():
    link = _StubLink()
    readings = iter([0.08, 0.03, -0.01])
    cycle = CameraStationCycle(
        link, capture=lambda: [], process=lambda f: _result(), arrival=lambda t: False,
        settings=CycleSettings(settle_s=0.0, recenter_pulses=3, recenter_speed=30,
                               recenter_pulse_s=0.0),
        overshoot=lambda: next(readings), sleep=lambda _s: None,
    )
    cycle._recenter(threading.Event())
    assert link.commands == ["REV:30", "STOP:0", "REV:30", "STOP:0"]


def test_no_backing_up_when_inside_unknown_or_disabled():
    for overshoot, pulses in ((lambda: -0.02, 3), (lambda: None, 3), (lambda: 0.1, 0)):
        link = _StubLink()
        cycle = CameraStationCycle(
            link, capture=lambda: [], process=lambda f: _result(), arrival=lambda t: False,
            settings=CycleSettings(settle_s=0.0, recenter_pulses=pulses), overshoot=overshoot,
            sleep=lambda _s: None,
        )
        cycle._recenter(threading.Event())
        assert link.commands == []


def test_auto_cycle_speed_is_clamped_to_the_camera_range():
    assert CycleSettings.from_config({"speed": 150}).speed == 50
    assert CycleSettings.from_config({"speed": 10}).speed == 30
    assert CycleSettings.from_config({"speed": 40, "speed_range": [30, 50]}).speed == 40
    assert CycleSettings.from_config({"speed": 80, "speed_range": [20, 60]}).speed == 60


# Firmware v2.1 (2026-11-24): the pick sensor sits before the pick point; the belt
# runs on ~1.2 s after the sensor sees the parcel. Here: parcel 0.2 long at 0.6/s
# covers the sensor for 0.33 s, the stop comes 0.6 s after the edge -> it rests PAST
# the sensor (sensor clear) when the robot takes it.
DELAYED = replace(SETTINGS, pick_stop_delay_ms=600, manual_pick_s=0.3)


def test_delayed_pick_stop_parcel_rests_past_the_sensor(rig):
    esp, link = rig
    log = _run(esp, link, 2, settings=DELAYED)
    assert len(log["results"]) == 2
    # Stopped by the ESP32 (AUTO delay) with the sensor already clear: no FAULT.
    assert log["picked_at"] == [("STOP", False)] * 2
    assert "AUTO:1:600" in esp.commands
    assert esp.events.count("EVT:AUTOSTOP:1") == 1 and esp.events.count("EVT:AUTOSTOP:2") == 1
    assert not any(phase == CyclePhase.FAULT for phase, _ in log["phases"])
    assert esp.positions() == []                       # both lifted by the robot


def test_delayed_pick_stop_without_robot_waits_for_the_operator(rig):
    esp, link = rig
    log = _run(esp, link, 1, robot=False, pick_delay=10.0, settings=DELAYED)
    assert len(log["results"]) == 1
    assert log["picked_at"] == [("STOP", False)]
    # Sensor cannot see the parcel being taken: the belt waits manual_pick_s, then runs.
    assert not any(phase == CyclePhase.FAULT for phase, _ in log["phases"])


def test_missed_parcel_stopped_past_the_sensor_is_ejected_not_a_fault(rig):
    # 2026-11-25: firmware v2.1 stops a parcel the camera missed 600 ms after the sensor,
    # already past it; the cycle raised "Băng tải dừng ngoài chu trình".
    esp, link = rig
    log = _run(esp, link, 1, camera_enabled=False, settings=DELAYED)
    assert [r.classification.decision for r in log["results"]] == [Decision.MANUAL_REVIEW]
    assert not any(phase == CyclePhase.FAULT for phase, _ in log["phases"])


def test_without_robot_the_belt_waits_for_the_operator_confirmation(rig):
    # 2026-11-25: the fixed 8 s wait ran the belt with the PASS parcel still at the pick
    # point and dropped it in the reject bin. manual_pick_s 0: wait for ĐÃ LẤY KIỆN.
    esp, link = rig
    settings = replace(DELAYED, manual_pick_s=0.0)
    log = _run(esp, link, 2, robot="manual", pick_delay=1.5, settings=settings)
    assert len(log["results"]) == 2
    assert log["picked_at"] == [("STOP", False)] * 2
    assert log["belt_while_waiting"] == {"STOP"}       # never moved before the confirmation
    assert not any(phase == CyclePhase.FAULT for phase, _ in log["phases"])


def test_hand_at_the_pick_sensor_without_a_parcel_from_the_camera_is_ignored(rig):
    # 2026-11-25 lần 3: something crossed the pick sensor while the real parcel was still
    # under the camera; the cycle recorded a phantom MANUAL_REVIEW.
    esp, link = rig
    stop = threading.Event()
    results, phases = [], []
    cycle = CameraStationCycle(
        link, capture=lambda: [np.zeros((4, 4, 3), np.uint8)], process=lambda f: _result(),
        arrival=SimCamera(esp, False), settings=DELAYED, on_phase=lambda p, t: phases.append((p, t)),
        on_result=results.append, parcel_passed=lambda: False,
    )
    esp.feed(1, gap=1.0, start=0.1)            # reaches the sensor, the camera "saw" nothing
    thread = threading.Thread(target=cycle.run, args=(stop,))
    thread.start()
    time.sleep(6.0)
    stop.set()
    thread.join(5)
    assert results == []
    assert any("không thấy kiện nào đi qua" in text for _p, text in phases)
    assert not any(p == CyclePhase.FAULT for p, _ in phases)


def test_parcel_carried_past_the_camera_is_backed_up_and_inspected(rig):
    # 2026-11-25 lần 4: the second parcel, placed close behind, rode past the camera while
    # the first one was transferred to the pick point and was dropped as MANUAL_REVIEW.
    # Now it is run back (REV) under the camera and inspected like any other parcel.
    esp, link = rig
    stop = threading.Event()
    results, phases, inspected = [], [], []

    def process(frames):
        inspected.append([round(c, 2) for c in esp.positions()])
        return _result("PASS")

    def pick_done():
        esp.take_at_sensor()     # robot lifts the parcel, reports done once the sensor is clear
        return not esp.sensor

    def on_phase(phase, text):
        phases.append((phase, text))
        if phase == CyclePhase.RUNNING and len(results) >= 2:
            stop.set()

    cycle = CameraStationCycle(
        link, capture=lambda: [np.zeros((4, 4, 3), np.uint8)], process=process,
        arrival=SimCamera(esp), settings=replace(SETTINGS, backup_max_s=5.0),
        on_phase=on_phase, on_result=results.append, pick_done=pick_done,
        view_clear=lambda: True,
    )
    esp.feed(2, gap=0.3, start=0.1)          # second one 0.3 behind: closer than camera->pick
    thread = threading.Thread(target=cycle.run, args=(stop,))
    thread.start()
    thread.join(25)
    stop.set()
    thread.join(5)
    assert [r.classification.decision for r in results] == [Decision.PASS, Decision.PASS]
    assert any("chạy lùi" in text for _p, text in phases)
    assert "REV:150" in esp.commands
    assert not any(p == CyclePhase.FAULT for p, _ in phases)
    # Both inspected with the parcel at the camera (centre 0.5..0.6).
    assert all(any(0.45 <= c <= 0.65 for c in snapshot) for snapshot in inspected)
