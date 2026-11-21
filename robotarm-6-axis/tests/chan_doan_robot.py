"""Chẩn đoán kết nối robot qua USB Serial (ESP32-S3 + firmware robotarmver2 có dòng [DIAG]).

Chế độ mặc định CHỈ NGHE (không gửi lệnh): reset board, in log khởi động (TMC2209 UART, W5500,
Flash), các dòng [DIAG] và telemetry góc.

    E:\\UDMT\\.venv\\Scripts\\python.exe tests\\chan_doan_robot.py --port COM8

Thử MỘT khớp (robot SẼ chuyển động — phải gõ YES để xác nhận):

    E:\\UDMT\\.venv\\Scripts\\python.exe tests\\chan_doan_robot.py --port COM8 --jog 1 --deg 5

Đóng GUI robot (hoặc nhấn NGẮT KẾT NỐI) trước khi chạy: mỗi cổng COM chỉ một chương trình mở được.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from donghoc import calc_crc16  # noqa: E402

CMD_STOP, CMD_MOVE_PC = 0, 1


def packet(cmd: int, angles, tool: int = 0) -> bytes:
    raw = struct.pack("<HBB6f", 0xAA55, cmd, tool, *angles)
    return raw + struct.pack("<H", calc_crc16(raw))


class Reader:
    """Tách luồng byte thành gói telemetry 30 byte (header AA 55, CRC16) và dòng chữ."""

    def __init__(self, port: serial.Serial) -> None:
        self.port = port
        self.buf = bytearray()
        self.text = bytearray()
        self.last_angles: list[float] | None = None
        self.last_mode = None
        self.packets = 0
        self.lines: list[str] = []  # các dòng chữ ESP32 đã gửi (để script khác đọc lại, vd. "[CAL] ...")

    def poll(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            self.buf += self.port.read(4096)
            self._parse()

    def _parse(self) -> None:
        i = 0
        while i < len(self.buf):
            if self.buf[i:i + 2] == b"\xaa\x55":
                if len(self.buf) - i < 30:
                    break
                chunk = bytes(self.buf[i:i + 30])
                if calc_crc16(chunk[:28]) == struct.unpack("<H", chunk[28:])[0]:
                    _h, mode, _tool, *angles = struct.unpack("<HBB6f", chunk[:28])
                    self.last_angles, self.last_mode = angles, mode
                    self.packets += 1
                    i += 30
                    continue
            byte = self.buf[i]
            if byte in (10, 13) or 32 <= byte < 127:
                self.text.append(byte)
                if byte == 10:
                    line = self.text.decode("ascii", errors="replace").strip()
                    if line:
                        print("  ESP32 >", line)
                        self.lines.append(line)
                    self.text.clear()
            i += 1
        del self.buf[:i]


def wait_until(reader: Reader, joint: int, target: float, timeout: float = 15.0, tol: float = 0.3) -> float:
    """Chờ tới khi telemetry khớp `joint` đạt `target` (±tol độ) hoặc hết thời gian; trả số giây đã chờ."""

    start = time.time()
    while time.time() - start < timeout:
        reader.poll(0.2)
        if reader.last_angles is not None and abs(reader.last_angles[joint] - target) <= tol:
            break
    return time.time() - start


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--no-reset", action="store_true", help="không reset board khi mở cổng")
    parser.add_argument("--ping", action="store_true",
                        help="gửi 3 lệnh STOP (KHÔNG chuyển động) để kiểm tra ESP32 nhận lệnh USB (rx_ok tăng)")
    parser.add_argument("--jog", type=int, choices=range(1, 7), help="khớp cần thử (1..6)")
    parser.add_argument("--deg", type=float, default=5.0, help="góc thử (độ, tối đa ±90)")
    args = parser.parse_args()

    try:
        port = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as exc:
        print(f"KHÔNG MỞ ĐƯỢC {args.port}: {exc}\n→ Đóng GUI robot / Serial Monitor đang giữ cổng rồi chạy lại.")
        return 2
    if not args.no_reset:
        port.dtr = False
        port.rts = True  # kéo EN xuống → reset ESP32 qua mạch auto-reset của CH343
        time.sleep(0.15)
        port.rts = False
    reader = Reader(port)
    print(f"Đang nghe {args.port} trong {args.seconds:.0f} s (không gửi lệnh)...")
    reader.poll(args.seconds)
    print(f"\nTelemetry hợp lệ: {reader.packets} gói", end="")
    if reader.last_angles is not None:
        print(f" · mode={reader.last_mode & 0x7F} · đã lưu gốc={bool(reader.last_mode & 0x80)}"
              f" · góc={[round(a, 2) for a in reader.last_angles]}")
    else:
        print("\n→ KHÔNG có telemetry: sai cổng COM, firmware chưa chạy, hoặc 'USB CDC On Boot' đang bật"
              " (Serial đi ra cổng USB native thay vì cổng CH343).")
        return 1

    if args.ping:
        print("\nPING: gửi 3 lệnh CMD_STOP (dừng + tắt bơm, không chuyển động). Xem dòng [DIAG] rx_ok tiếp theo:")
        for _ in range(3):
            port.write(packet(CMD_STOP, [0.0] * 6))
            reader.poll(0.2)
        reader.poll(2.5)
        print("→ rx_ok tăng lên ≥ 3: đường lệnh USB PC → ESP32 TỐT. rx_ok = 0: ESP32 không nhận được lệnh;"
              " crc_err tăng: dữ liệu tới nhưng sai khung.")

    if args.jog:
        delta = max(-90.0, min(90.0, args.deg))
        start = list(reader.last_angles)
        target = list(start)
        target[args.jog - 1] += delta
        print(f"\nTHỬ KHỚP J{args.jog}: {start[args.jog - 1]:.1f}° → {target[args.jog - 1]:.1f}°"
              " (các khớp khác giữ nguyên). ROBOT SẼ CHUYỂN ĐỘNG.")
        if input("Vùng làm việc trống, tay sẵn sàng ngắt nguồn động cơ? Gõ YES để chạy: ").strip() != "YES":
            print("Đã hủy.")
            return 0
        port.write(packet(CMD_STOP, [0.0] * 6))  # về MODE_PC (thoát AUTO nếu đang chạy)
        reader.poll(0.3)
        port.write(packet(CMD_MOVE_PC, target))
        elapsed = wait_until(reader, args.jog - 1, target[args.jog - 1])
        moved = reader.last_angles[args.jog - 1] - start[args.jog - 1]
        others = max(abs(reader.last_angles[k] - start[k]) for k in range(6) if k != args.jog - 1)
        print(f"Kết quả J{args.jog}: firmware báo đi {moved:+.2f}° (lệnh {delta:+.2f}°) sau {elapsed:.1f} s; "
              f"khớp khác lệch tối đa {others:.2f}°")
        print("So sánh với quan sát THỰC TẾ: firmware chỉ đếm xung đã phát, không biết động cơ có quay/trượt bước.")
        port.write(packet(CMD_MOVE_PC, start))  # trả về vị trí cũ
        back = wait_until(reader, args.jog - 1, start[args.jog - 1])
        print(f"Đã trả J{args.jog} về {reader.last_angles[args.jog - 1]:+.2f}° sau {back:.1f} s")
    port.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
