"""Thử lần lượt từng khớp qua USB Serial với firmware chính robotarmver2 (KHÔNG hỏi xác nhận —
chỉ chạy sau khi người dùng đã đồng ý trong phiên làm việc).

Mỗi khớp: +DEG độ (từ vị trí hiện tại) → chờ telemetry tới đích → về lại → nghỉ. In giờ bắt đầu
để người quan sát đối chiếu. Firmware chỉ đếm xung đã phát: quan sát thực tế mới xác nhận động cơ quay.

    E:\\UDMT\\.venv\\Scripts\\python.exe tests\\thu_tung_khop.py --port COM8 --deg 10
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent))
from chan_doan_robot import CMD_MOVE_PC, CMD_STOP, Reader, packet, wait_until  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--deg", type=float, default=10.0)
    parser.add_argument("--joints", default="1,2,3,4,5,6")
    parser.add_argument("--pause", type=float, default=2.0, help="nghỉ giữa các khớp (s)")
    args = parser.parse_args()
    delta = max(-30.0, min(30.0, args.deg))

    port = serial.Serial(args.port, 115200, timeout=0.05)
    port.dtr = False
    port.rts = True  # reset ESP32 để xem log khởi động
    time.sleep(0.15)
    port.rts = False
    reader = Reader(port)
    reader.poll(5.0)
    if reader.last_angles is None:
        print("KHÔNG có telemetry → dừng.")
        return 1
    port.write(packet(CMD_STOP, [0.0] * 6))
    reader.poll(0.5)
    home = list(reader.last_angles)
    print(f"Vị trí bắt đầu: {[round(a, 2) for a in home]}")

    for j in [int(x) for x in args.joints.split(",")]:
        k = j - 1
        target = list(home)
        target[k] += delta
        print(f"\n[{time.strftime('%H:%M:%S')}] J{j}: {home[k]:+.1f}° → {target[k]:+.1f}°")
        port.write(packet(CMD_MOVE_PC, target))
        t_go = wait_until(reader, k, target[k], timeout=10)
        reached = reader.last_angles[k]
        others = max(abs(reader.last_angles[i] - home[i]) for i in range(6) if i != k)
        reader.poll(0.5)
        print(f"[{time.strftime('%H:%M:%S')}] J{j}: về {home[k]:+.1f}°")
        port.write(packet(CMD_MOVE_PC, home))
        t_back = wait_until(reader, k, home[k], timeout=10)
        print(f"  → firmware báo: đi tới {reached:+.2f}° sau {t_go:.1f} s, về {reader.last_angles[k]:+.2f}° "
              f"sau {t_back:.1f} s; khớp khác lệch tối đa {others:.2f}°")
        reader.poll(args.pause)

    port.write(packet(CMD_STOP, [0.0] * 6))
    reader.poll(2.5)  # đọc thêm 1 dòng [DIAG] (bước hiện tại phải về như ban đầu)
    print(f"\nKết thúc: {[round(a, 2) for a in reader.last_angles]}")
    port.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
