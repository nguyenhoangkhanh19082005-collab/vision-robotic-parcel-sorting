"""Thử riêng một khớp (mặc định J6) với firmware chính robotarmver2 — CHỈ chạy khi người dùng đã đồng ý.

Trình tự (dán 1 mẩu băng dính lên trục/đầu khớp làm dấu để quan sát):
  1. In trạng thái 6 driver (lệnh chữ '?': hở cuộn, chạm mass, quá nhiệt, mất cấu hình).
  2. CHẬM: +90° theo bước 5° mỗi 0.25 s (~20°/s) rồi về 0° — dấu phải quay đúng 1/4 vòng rồi về chỗ cũ.
  3. NHANH: +360° một lệnh rồi về 0° — dấu phải quay đúng 1 vòng, dừng hẳn, không trôi tiếp.
  Đọc trạng thái driver ngay lúc đang quay (cờ hở cuộn ola/olb chỉ tin cậy khi đang chạy chậm).

    E:\\UDMT\\.venv\\Scripts\\python.exe tests\\thu_j6.py --port COM8
    E:\\UDMT\\.venv\\Scripts\\python.exe tests\\thu_j6.py --port COM8 --joint 5   # thử chéo driver J5

Firmware chỉ đếm xung đã phát: nếu firmware báo đúng góc mà dấu trên trục lệch/rung/quay tiếp thì lỗi
nằm ở phần cứng (cặp cuộn dây, jack, driver), không phải phần mềm.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent))
from chan_doan_robot import CMD_MOVE_PC, CMD_STOP, Reader, packet, wait_until  # noqa: E402


def stamp(text: str) -> None:
    print(f"\n[{time.strftime('%H:%M:%S')}] {text}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--joint", type=int, choices=range(1, 7), default=6)
    parser.add_argument("--no-fast", action="store_true", help="bỏ bước quay nhanh 360°")
    args = parser.parse_args()
    k = args.joint - 1

    port = serial.Serial(args.port, 115200, timeout=0.05)
    port.dtr = False
    port.rts = True  # reset ESP32: xem log khởi động + driver được cấu hình lại từ đầu
    time.sleep(0.15)
    port.rts = False
    reader = Reader(port)
    reader.poll(5.0)
    if reader.last_angles is None:
        print("KHÔNG có telemetry → kiểm tra cổng COM / firmware.")
        return 1
    port.write(packet(CMD_STOP, [0.0] * 6))
    reader.poll(0.3)
    home = list(reader.last_angles)

    stamp("Trạng thái driver lúc đứng yên:")
    port.write(b"?")
    reader.poll(1.0)

    def go(value: float) -> None:
        target = list(home)
        target[k] = value
        port.write(packet(CMD_MOVE_PC, target))

    if args.joint in (2, 5):
        print("CẢNH BÁO: J2/J5 giới hạn ~±90..110° — đảm bảo không có vật cản trên hành trình +90°.")
    stamp(f"J{args.joint} CHẬM: {home[k]:+.0f}° → {home[k] + 90:+.0f}° (bước 5°/0.25 s)")
    for step in range(1, 19):
        go(home[k] + 5 * step)
        reader.poll(0.25)
        if step == 9:
            port.write(b"?")  # đọc trạng thái khi đang quay chậm
    wait_until(reader, k, home[k] + 90, timeout=5)
    print(f"  firmware báo: {reader.last_angles[k]:+.2f}° (dấu trên trục phải quay đúng 1/4 vòng)")
    reader.poll(1.5)
    stamp(f"J{args.joint} CHẬM: về {home[k]:+.0f}°")
    for step in range(17, -1, -1):
        go(home[k] + 5 * step)
        reader.poll(0.25)
    wait_until(reader, k, home[k], timeout=5)
    print(f"  firmware báo: {reader.last_angles[k]:+.2f}° (dấu phải về đúng chỗ cũ)")
    reader.poll(2.0)

    if not args.no_fast:
        fast = 360.0 if args.joint == 6 else 45.0  # J1..J5 có giới hạn hành trình nhỏ hơn
        stamp(f"J{args.joint} NHANH: +{fast:.0f}° (1 lệnh)")
        go(home[k] + fast)
        reader.poll(0.4)
        port.write(b"?")
        t = wait_until(reader, k, home[k] + fast, timeout=10)
        print(f"  firmware báo: {reader.last_angles[k]:+.2f}° sau {t:.1f} s (dấu phải quay đúng {fast:.0f}° rồi dừng)")
        reader.poll(1.5)
        stamp(f"J{args.joint} NHANH: về {home[k]:+.0f}°")
        go(home[k])
        t = wait_until(reader, k, home[k], timeout=10)
        print(f"  firmware báo: {reader.last_angles[k]:+.2f}° sau {t:.1f} s")
        reader.poll(1.5)

    port.write(packet(CMD_STOP, [0.0] * 6))
    stamp("Trạng thái driver sau khi thử:")
    port.write(b"?")
    reader.poll(2.5)
    port.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
