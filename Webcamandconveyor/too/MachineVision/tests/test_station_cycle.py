"""Automatic cycle against the simulated firmware (real UDP on localhost)."""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest

from machine_vision.controllers.conveyor_udp import (
    ConveyorLink,
    parse_event,
    parse_status,
)
from machine_vision.controllers.fake_esp32 import FakeConveyorEsp32
from machine_vision.models import Decision
from machine_vision.services.station_cycle import (
    CyclePhase,
    CycleSettings,
    StationCycle,
    failure_result,
)

FAST = CycleSettings(
    speed=150, auto_stop_delay_ms=50, stop_grace_s=0.3, settle_s=0.02,
    quality_retries=1, release_timeout_s=3.0, poll_s=0.03,
)


def test_parse_status_and_events():
    status = parse_status(
        "STATUS:FWD:150:SENSOR=1:COUNT=4:AUTO=1:DELAY=300:ISR=12:ISL=0:UP=99"
    )
    assert (status.state, status.speed, status.sensor, status.count, status.auto) == (
        "FWD", 150, True, 4, True,
    )
    assert not status.autostopped                       # firmware v2.0: no ASTOP field
    v21 = parse_status(
        "STATUS:STOP:0:SENSOR=0:COUNT=4:AUTO=1:DELAY=1200:ISR=12:ISL=0:UP=99:ASTOP=1"
    )
    assert (v21.state, v21.sensor, v21.delay_ms, v21.autostopped) == ("STOP", False, 1200, True)
    assert parse_event("EVT:SENSOR:1:3:1200").values == (1, 3, 1200)
    assert parse_event("EVT:WATCHDOG_STOP").is_fault
    assert not parse_event("EVT:AUTOSTOP:3").is_fault
    for bad in ("STATUS:FWD", "EVT:SENSOR:2:1:1", "PONG", "EVT:sensor:1"):
        with pytest.raises(ValueError):
            (parse_status if bad.startswith("STATUS") else parse_event)(bad)


@pytest.fixture
def rig():
    esp = FakeConveyorEsp32(event_port=0)
    link = ConveyorLink("127.0.0.1", esp.address[1], event_port=0, timeout_s=0.3)
    esp.event_port = link.event_port
    esp.start()
    yield esp, link
    link.close()
    esp.close()


def _run_cycle(esp, link, parcels, process, settings=FAST, timeout=15.0):
    results, phases = [], []
    stop = threading.Event()

    def on_result(result):
        results.append((result, esp.state, esp.sensor))

    def on_phase(phase, text):
        phases.append((phase, text))
        # Stop once the last parcel has been released and the belt waits again.
        if phase == CyclePhase.RUNNING and len(results) == parcels:
            stop.set()

    cycle = StationCycle(
        link,
        capture=lambda: [np.zeros((4, 4, 3), np.uint8)],
        process=process,
        settings=settings,
        on_phase=on_phase,
        on_result=on_result,
    )
    thread = threading.Thread(target=cycle.run, args=(stop,))
    esp.feed(parcels)
    thread.start()
    thread.join(timeout)
    stop.set()
    thread.join(3)
    return cycle, results, phases


def test_cycle_stops_inspects_and_releases_each_parcel(rig):
    esp, link = rig
    seen = []

    def process(frames):
        seen.append(esp.state)
        return failure_result("ok", frames[-1])

    cycle, results, phases = _run_cycle(esp, link, 3, process)
    assert len(results) == 3
    # Every inspection ran with the belt stopped and the parcel on the sensor.
    assert seen == ["STOP", "STOP", "STOP"]
    assert all(state == "STOP" and sensor for _r, state, sensor in results)
    # Released forward after each inspection, then the cycle stopped the belt.
    assert esp.commands.count("FWD:150") >= 3
    assert esp.state == "STOP" and not esp.auto
    assert phases[-1][0] == CyclePhase.IDLE
    assert esp.events.count("EVT:AUTOSTOP:1") == 1


def test_quality_failure_is_logged_as_manual_review_and_released(rig):
    esp, link = rig
    calls = []

    def process(frames):
        calls.append(1)
        raise RuntimeError("Quality gate: blur")

    _cycle, results, _phases = _run_cycle(esp, link, 1, process)
    assert len(calls) == FAST.quality_retries + 1
    result = results[0][0]
    assert result.classification.decision == Decision.MANUAL_REVIEW
    assert "blur" in result.classification.reason
    assert esp.commands.count("FWD:150") >= 2  # released after the failed inspection


def test_lost_autostop_falls_back_to_pc_stop(rig):
    esp, link = rig
    esp.parcel_length = 0.6  # transit 1.2 s > delay + grace
    original = esp._reply

    def no_auto(command):  # firmware ignores AUTO -> PC must stop the belt itself
        reply = original(command)
        esp.auto = False
        return reply

    esp._reply = no_auto
    states = []

    def process(frames):
        states.append(esp.state)
        return failure_result("ok", None)

    _cycle, results, _ = _run_cycle(esp, link, 1, process)
    assert states == ["STOP"]
    assert "STOP:0" in esp.commands


def test_watchdog_event_faults_the_cycle(rig):
    esp, link = rig
    phases = []
    stop = threading.Event()
    cycle = StationCycle(
        link, capture=list, process=lambda f: failure_result("x", None),
        settings=FAST, on_phase=lambda p, t: phases.append((p, t)),
    )
    thread = threading.Thread(target=cycle.run, args=(stop,))
    thread.start()
    time.sleep(0.4)
    with esp._lock:
        esp._send_event("EVT:OVERCURRENT_STOP:2500")
        esp.state = "STOP"
    thread.join(5)
    assert phases[-1][0] == CyclePhase.FAULT
    assert "OVERCURRENT" in phases[-1][1] or "ngoài chu trình" in phases[-1][1]
    assert esp.state == "STOP"
