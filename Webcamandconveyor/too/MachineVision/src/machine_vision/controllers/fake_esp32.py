"""Software stand-in for the conveyor firmware v2 (firmware/conveyor_sensor).

Speaks the same UDP protocol (commands on one port, ``EVT:*`` pushed to port
``event_port`` of the last commander) and simulates parcels moving past the photo
sensor, the AUTO stop, and the 3 s watchdog. Used by tests and by
``scripts/fake_conveyor_esp32.py`` to rehearse the automatic cycle without hardware.
"""

from __future__ import annotations

import socket
import threading
import time


class FakeConveyorEsp32:
    WATCHDOG_S = 3.0

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 0,
        event_port: int = 8121,
        belt_speed: float = 0.5,       # belt length units per second at FWD
        parcel_length: float = 0.15,
        sensor_position: float = 1.0,
        tick_s: float = 0.01,
    ):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.settimeout(tick_s)
        self.address = self.sock.getsockname()
        self.event_port = event_port
        self.belt_speed = belt_speed
        self.parcel_length = parcel_length
        self.sensor_position = sensor_position
        self.tick_s = tick_s
        self.state = "STOP"
        self.speed = 0
        self.auto = False
        self.delay_ms = 0
        self.count = 0
        self.sensor = False
        self.parcels: list[float] = []   # front positions
        self.travel = 0.0                # total belt displacement (parcel id = centre - travel)
        self.commands: list[str] = []
        self.events: list[str] = []
        self._peer_ip: str | None = None
        self._autostop_at: float | None = None
        self.autostopped = False         # firmware v2.1 STATUS ASTOP flag
        self._last_packet = time.monotonic()
        self._started = time.monotonic()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    # -- test controls -------------------------------------------------------
    def start(self) -> FakeConveyorEsp32:
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        self._thread.join(2)
        self.sock.close()

    def feed(self, count: int = 1, gap: float = 0.4, start: float = 0.6) -> None:
        """Place ``count`` parcels upstream of the sensor, ``gap`` apart."""
        with self._lock:
            tail = min(self.parcels, default=self.sensor_position) - gap
            first = min(start, tail)
            for index in range(count):
                self.parcels.append(first - index * gap)

    def take_at_sensor(self) -> bool:
        """Remove the parcel at the pick point (the robot lifted it).

        That is the parcel covering the sensor or, with a delayed pick stop (firmware
        v2.1), the one that has just passed it (the rearmost front past the sensor).
        """
        with self._lock:
            past = [p for p in self.parcels if p >= self.sensor_position]
            if not past:
                return False
            self.parcels.remove(min(past))
            return True

    def positions(self) -> list[float]:
        """Parcel centres along the belt (same units as ``sensor_position``)."""
        with self._lock:
            return [p - self.parcel_length / 2 for p in self.parcels]

    # -- simulation ------------------------------------------------------------
    def _send_event(self, text: str) -> None:
        self.events.append(text)
        if self._peer_ip is not None:
            self.sock.sendto(text.encode("ascii"), (self._peer_ip, self.event_port))

    def _step(self, dt: float) -> None:
        now = time.monotonic()
        if self.state == "REV":
            self.travel -= self.belt_speed * dt
            self.parcels = [p - self.belt_speed * dt for p in self.parcels]
        if self.state == "FWD":
            self.travel += self.belt_speed * dt
            self.parcels = [p + self.belt_speed * dt for p in self.parcels]
            self.parcels = [
                p for p in self.parcels if p - self.parcel_length < self.sensor_position + 1.0
            ]
        active = any(
            p >= self.sensor_position > p - self.parcel_length for p in self.parcels
        )
        if active != self.sensor:
            self.sensor = active
            if active:
                self.count += 1
                # Only the first parcel starts the countdown (firmware v2.1).
                if self.auto and self.state == "FWD" and self._autostop_at is None:
                    self._autostop_at = now + self.delay_ms / 1000
            self._send_event(f"EVT:SENSOR:{int(active)}:{self.count}:{int((now - self._started) * 1000)}")
        if self._autostop_at is not None and now >= self._autostop_at:
            self._autostop_at = None
            if self.state == "FWD":
                self.state, self.speed = "STOP", 0
                self.autostopped = True
                self._send_event(f"EVT:AUTOSTOP:{self.count}")
        if self.state != "STOP" and now - self._last_packet > self.WATCHDOG_S:
            self.state, self.speed = "STOP", 0
            self._autostop_at = None
            self._send_event("EVT:WATCHDOG_STOP")

    def _reply(self, command: str) -> str:
        if command == "PING":
            return "PONG"
        if command == "STATUS":
            return (
                f"STATUS:{self.state}:{self.speed}:SENSOR={int(self.sensor)}:COUNT={self.count}"
                f":AUTO={int(self.auto)}:DELAY={self.delay_ms}:ISR=0:ISL=0"
                f":UP={int(time.monotonic() - self._started)}:ASTOP={int(self.autostopped)}"
            )
        if command == "RESETCOUNT":
            self.count = 0
            return "ACK:RESETCOUNT"
        action, _, value = command.partition(":")
        if action == "AUTO":
            enabled, _, delay = value.partition(":")
            if enabled not in {"0", "1"} or not delay.isdigit() or int(delay) > 10000:
                return "ERR:AUTO_FORMAT_AUTO:<0|1>:<0..10000ms>"
            self.auto, self.delay_ms = enabled == "1", int(delay)
            self._autostop_at = None
            return f"ACK:AUTO:{enabled}:{self.delay_ms}"
        if action in {"FWD", "REV", "STOP"} and value.isdigit():
            speed = int(value)
            if action != "STOP" and not 1 <= speed <= 255:
                return "ERR:PWM_RANGE"
            if action != "STOP":
                self.autostopped = False
            if action != "FWD":
                self._autostop_at = None
            self.state, self.speed = action, (0 if action == "STOP" else speed)
            return f"ACK:{action}:{value} (State: {action} Speed: {self.speed})"
        return "ERR:UNKNOWN_COMMAND"

    def _loop(self) -> None:
        last = time.monotonic()
        while not self._stop.is_set():
            try:
                payload, peer = self.sock.recvfrom(512)
            except TimeoutError:
                payload = None
            except OSError:
                break
            with self._lock:
                if payload is not None:
                    command = payload.decode("ascii", errors="replace").strip()
                    self.commands.append(command)
                    reply = self._reply(command)
                    if not reply.startswith("ERR"):
                        self._peer_ip = peer[0]
                        self._last_packet = time.monotonic()
                    self.sock.sendto(reply.encode("ascii"), peer)
                now = time.monotonic()
                self._step(now - last)
                last = now
