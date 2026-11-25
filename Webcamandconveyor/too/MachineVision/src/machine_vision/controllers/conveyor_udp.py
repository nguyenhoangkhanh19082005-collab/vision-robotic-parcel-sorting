"""UDP adapter for the ESP32/W5500 conveyor firmware.

The protocol has no transaction ID. Every request uses a fresh UDP socket, checks
the peer address, and accepts only the reply matching that request.
"""

from __future__ import annotations

import ipaddress
import re
import socket
from dataclasses import dataclass

_ACK = re.compile(r"^ACK:(FWD|REV|STOP):(\d{1,3}) \(State: (FWD|REV|STOP) Speed: (\d{1,3})\)$")


@dataclass(frozen=True)
class ConveyorReply:
    command: str
    state: str | None
    speed: int | None
    raw: str


def validate_endpoint(host: str, port: int) -> tuple[str, int]:
    host = str(ipaddress.IPv4Address(host.strip()))
    if not 1 <= port <= 65535:
        raise ValueError("UDP port phải trong khoảng 1–65535")
    return host, port


def make_command(action: str, speed: int = 0) -> str:
    if action == "PING":
        return "PING"
    if action == "STOP":
        return "STOP:0"
    if action not in {"FWD", "REV"}:
        raise ValueError("Lệnh băng tải không hợp lệ")
    if not 1 <= speed <= 255:
        raise ValueError("Tốc độ PWM phải trong khoảng 1–255")
    return f"{action}:{speed}"


def parse_reply(command: str, payload: bytes) -> ConveyorReply:
    raw = payload.decode("ascii", errors="strict").strip()
    if command == "PING":
        if raw != "PONG":
            raise ValueError(f"Phản hồi PING không hợp lệ: {raw!r}")
        return ConveyorReply(command, None, None, raw)
    match = _ACK.fullmatch(raw)
    if match is None:
        raise ValueError(f"ACK không hợp lệ: {raw!r}")
    action, requested_speed, state, actual_speed = match.groups()
    expected_action, expected_speed = command.split(":", 1)
    speed = int(actual_speed)
    if (
        action != expected_action
        or requested_speed != expected_speed
        or state != expected_action
        or speed != int(expected_speed)
    ):
        raise ValueError(f"ACK không khớp lệnh {command}: {raw!r}")
    return ConveyorReply(command, state, speed, raw)


def exchange(host: str, port: int, command: str, timeout_s: float = 0.6) -> ConveyorReply:
    host, port = validate_endpoint(host, port)
    if command != "PING":
        action, separator, speed_text = command.partition(":")
        if (
            not separator
            or not speed_text.isdigit()
            or make_command(action, int(speed_text)) != command
        ):
            raise ValueError("Lệnh băng tải không hợp lệ")
    if not 0 < timeout_s <= 5:
        raise ValueError("Timeout phải trong khoảng 0–5 giây")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
        client.settimeout(timeout_s)
        client.sendto(command.encode("ascii"), (host, port))
        payload, peer = client.recvfrom(512)
    if peer != (host, port):
        raise ValueError(f"Phản hồi từ nguồn khác: {peer}")
    return parse_reply(command, payload)


# ---------------------------------------------------------------------------
# Firmware v2 (docs/esp32_contract.md): STATUS, AUTO and pushed events on 8121.
# ---------------------------------------------------------------------------
_STATUS = re.compile(
    r"^STATUS:(FWD|REV|STOP):(\d{1,3}):SENSOR=([01]):COUNT=(\d+):AUTO=([01]):DELAY=(\d+)"
    r":ISR=(-?\d+):ISL=(-?\d+):UP=(\d+)(?::ASTOP=([01]))?$"
)


@dataclass(frozen=True)
class ConveyorStatus:
    state: str
    speed: int
    sensor: bool
    count: int
    auto: bool
    delay_ms: int
    raw: str
    # Firmware v2.1: the last stop was the AUTO pick stop (the parcel may already have
    # passed the sensor when the stop is delayed). False on v2.0 (field missing).
    autostopped: bool = False


@dataclass(frozen=True)
class ConveyorEvent:
    """``kind`` is SENSOR, AUTOSTOP, WATCHDOG_STOP, LINK_DOWN_STOP or OVERCURRENT_STOP."""

    kind: str
    values: tuple[int, ...]
    raw: str

    @property
    def is_fault(self) -> bool:
        return self.kind in {"WATCHDOG_STOP", "LINK_DOWN_STOP", "OVERCURRENT_STOP"}


def parse_status(payload: bytes | str) -> ConveyorStatus:
    raw = payload.decode("ascii") if isinstance(payload, bytes) else payload
    raw = raw.strip()
    match = _STATUS.fullmatch(raw)
    if match is None:
        raise ValueError(f"STATUS không hợp lệ: {raw!r}")
    state, speed, sensor, count, auto, delay = match.groups()[:6]
    return ConveyorStatus(
        state, int(speed), sensor == "1", int(count), auto == "1", int(delay), raw,
        match.group(10) == "1",
    )


def parse_event(payload: bytes | str) -> ConveyorEvent:
    raw = payload.decode("ascii") if isinstance(payload, bytes) else payload
    raw = raw.strip()
    parts = raw.split(":")
    if len(parts) < 2 or parts[0] != "EVT" or not re.fullmatch(r"[A-Z_]+", parts[1]):
        raise ValueError(f"Sự kiện không hợp lệ: {raw!r}")
    try:
        values = tuple(int(part) for part in parts[2:])
    except ValueError as exc:
        raise ValueError(f"Sự kiện không hợp lệ: {raw!r}") from exc
    if parts[1] == "SENSOR" and (len(values) < 1 or values[0] not in (0, 1)):
        raise ValueError(f"Sự kiện cảm biến không hợp lệ: {raw!r}")
    return ConveyorEvent(parts[1], values, raw)


class ConveyorLink:
    """Blocking command channel (port 8120) plus event listener (port 8121).

    Used by the automatic station cycle from a worker thread. The firmware sends
    events to the IP of the last valid command, so commands must come from this PC.
    """

    def __init__(
        self,
        host: str,
        port: int = 8120,
        event_port: int = 8121,
        timeout_s: float = 0.6,
        retries: int = 2,
        bind_host: str = "0.0.0.0",
    ):
        self.host, self.port = validate_endpoint(host, port)
        self.timeout_s = timeout_s
        self.retries = retries
        self._events = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._events.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._events.bind((bind_host, event_port))
        self.event_port = self._events.getsockname()[1]

    def close(self) -> None:
        self._events.close()

    def __enter__(self) -> ConveyorLink:
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    def _request(self, command: str) -> str:
        last: Exception | None = None
        for _attempt in range(self.retries + 1):
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
                    client.settimeout(self.timeout_s)
                    client.sendto(command.encode("ascii"), (self.host, self.port))
                    payload, peer = client.recvfrom(512)
                if peer[0] != self.host:
                    raise ValueError(f"Phản hồi từ nguồn khác: {peer}")
                raw = payload.decode("ascii").strip()
                if raw.startswith("ERR"):
                    raise ValueError(f"ESP32 từ chối {command}: {raw}")
                return raw
            except (OSError, ValueError, UnicodeError) as exc:
                last = exc
        raise ConnectionError(f"Không nhận phản hồi cho {command}: {last}")

    def drive(self, action: str, speed: int = 0) -> ConveyorReply:
        command = make_command(action, speed)
        return parse_reply(command, self._request(command).encode("ascii"))

    def ping(self) -> None:
        if self._request("PING") != "PONG":
            raise ConnectionError("PING không trả PONG")

    def status(self) -> ConveyorStatus:
        return parse_status(self._request("STATUS"))

    def set_auto(self, enabled: bool, delay_ms: int) -> None:
        if not 0 <= delay_ms <= 10000:
            raise ValueError("Độ trễ dừng phải trong 0–10000 ms")
        expected = f"ACK:AUTO:{1 if enabled else 0}:{delay_ms}"
        raw = self._request(f"AUTO:{1 if enabled else 0}:{delay_ms}")
        if raw != expected:
            raise ValueError(f"ACK AUTO không khớp: {raw!r}")

    def next_event(self, timeout_s: float) -> ConveyorEvent | None:
        """Return the next valid event from the ESP32, or None after ``timeout_s``."""
        self._events.settimeout(max(0.001, timeout_s))
        try:
            payload, peer = self._events.recvfrom(512)
        except TimeoutError:
            return None
        except OSError:
            return None
        if peer[0] != self.host:
            return None
        try:
            return parse_event(payload)
        except (ValueError, UnicodeError):
            return None

    def drain_events(self) -> None:
        while self.next_event(0.001) is not None:
            pass
