"""Watch the conveyor photo sensor through firmware/conveyor_sensor (FW v2.0).

Polls STATUS once per second and prints every EVT:* datagram the ESP32 pushes
to UDP port 8121 (object detected/cleared, auto-stop, watchdog stop).

    python scripts/conveyor_sensor_monitor.py
    python scripts/conveyor_sensor_monitor.py --auto 1 --delay 300   # enable auto-stop
    python scripts/conveyor_sensor_monitor.py --reset-count

The STATUS polls also feed the firmware watchdog. Do not run it alongside the
GUI conveyor tab if you rely on the GUI losing contact to stop the belt.
Windows Firewall may ask to allow Python to receive on UDP 8121.
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))

from machine_vision.controllers.conveyor_udp import validate_endpoint  # noqa: E402


def request(host: str, port: int, command: str, timeout_s: float = 0.6) -> str | None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
        client.settimeout(timeout_s)
        client.sendto(command.encode("ascii"), (host, port))
        try:
            payload, peer = client.recvfrom(512)
        except TimeoutError:
            return None
    if peer != (host, port):
        return None
    return payload.decode("ascii", errors="replace").strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="192.168.2.100")
    parser.add_argument("--port", type=int, default=8120)
    parser.add_argument("--event-port", type=int, default=8121)
    parser.add_argument("--auto", type=int, choices=(0, 1), help="set firmware auto-stop")
    parser.add_argument("--delay", type=int, default=0, help="auto-stop delay in ms")
    parser.add_argument("--reset-count", action="store_true")
    args = parser.parse_args()
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    host, port = validate_endpoint(args.host, args.port)

    pong = request(host, port, "PING")
    print(f"PING -> {pong or 'KHÔNG PHẢN HỒI (kiểm tra cáp LAN / IP laptop 192.168.2.x)'}")
    if pong != "PONG":
        return 1
    if args.reset_count:
        print("RESETCOUNT ->", request(host, port, "RESETCOUNT"))
    if args.auto is not None:
        print("AUTO ->", request(host, port, f"AUTO:{args.auto}:{args.delay}"))

    events = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    events.bind(("0.0.0.0", args.event_port))
    events.settimeout(0.1)
    print(f"Đang theo dõi (sự kiện UDP {args.event_port}). Ctrl+C để thoát.\n")
    last_status = ""
    next_poll = 0.0
    try:
        while True:
            now = time.monotonic()
            if now >= next_poll:
                next_poll = now + 1.0
                status = request(host, port, "STATUS")
                if status is None:
                    print(f"{time.strftime('%H:%M:%S')} STATUS: không phản hồi")
                elif status != last_status:
                    print(f"{time.strftime('%H:%M:%S')} {status}")
                    last_status = status
            try:
                payload, peer = events.recvfrom(512)
            except TimeoutError:
                continue
            if peer[0] == host:
                text = payload.decode("ascii", errors="replace").strip()
                print(time.strftime("%H:%M:%S"), ">>>", text, flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        events.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
