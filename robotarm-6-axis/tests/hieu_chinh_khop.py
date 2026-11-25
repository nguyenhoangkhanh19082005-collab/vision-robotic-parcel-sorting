"""Hiệu chỉnh CHIỀU QUAY và TỈ SỐ TRUYỀN từng khớp so với digital twin (người dùng tự chạy, có hỏi đáp).

Robot SẼ chuyển động. Đứng cạnh robot, tay sẵn sàng ngắt nguồn tổ ong. Đóng GUI/Station (giải phóng cổng COM).
Trước khi chạy: đưa robot về tư thế 0 của twin = CHỮ L CHUẨN (từ 15/11/2026, xem dong_hoc_step.py):
cánh tay trên dựng thẳng đứng, cẳng tay nằm ngang chĩa ra trước, cổ tay thẳng hàng với cẳng tay, trục gập cổ tay (J5)
nằm ngang. Dùng thước thủy / ứng dụng đo nghiêng trên điện thoại để đặt, rồi "Lưu Gốc"/Set Zero ở tư thế đó.

Mỗi khớp: quay chậm tới +GÓC (bước 5°), dừng, hỏi: có quay không, cùng chiều twin không, góc thật đo được;
rồi quay về 0 và hỏi có về đúng chỗ không. J6 thêm bài thử "tự quay khi không có xung".
Với firmware từ 15/11/2026 (có lệnh $P/$K/$D/$S): chiều quay và tỉ số truyền mới được GỬI NGAY xuống board, cho thử lại
khớp đó để kiểm chứng, cuối cùng LƯU vào Flash của ESP32 — không cần sửa .ino hay nạp lại.
Firmware cũ hơn: chỉ in đề xuất DIR_SIGNS / GEAR_RATIOS. Luôn ghi file JSON trong reports/.

Cách đo góc thật: J2, J3, J5 (khớp gập) — đặt điện thoại (ứng dụng đo nghiêng) lên khâu, lấy hiệu số trước/sau.
J1, J4, J6 (khớp xoay) — thước đo góc, hoặc đánh dấu rồi đo.

    E:\\UDMT\\.venv\\Scripts\\python.exe tests\\hieu_chinh_khop.py --port COM8
    E:\\UDMT\\.venv\\Scripts\\python.exe tests\\hieu_chinh_khop.py --port COM8 --joints 1,5,6
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from pathlib import Path

import serial

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from chan_doan_robot import CMD_MOVE_PC, CMD_STOP, Reader, packet, wait_until  # noqa: E402

# Chiều DƯƠNG của từng khớp trên digital twin (tính từ mô hình PoE, tư thế 0; hệ base: X ra trước, Z lên)
TWIN_POSITIVE = {
    1: "đế xoay NGƯỢC chiều kim đồng hồ khi nhìn từ trên xuống (đầu hút đi sang TRÁI robot)",
    2: "cánh tay trên CÚI về phía trước (đầu hút đi ra trước và xuống)",
    3: "cẳng tay GẬP XUỐNG (đầu cẳng tay hạ xuống)",
    4: "cẳng tay xoay NGƯỢC chiều kim đồng hồ khi đứng sau robot nhìn dọc cẳng tay ra đầu hút",
    5: "cổ tay NGÓC LÊN (đầu hút hướng lên trên)",
    6: "đầu hút xoay THEO chiều kim đồng hồ khi đứng sau robot nhìn dọc ra đầu hút",
}
TEST_DEG = {1: 30, 2: 20, 3: 20, 4: 45, 5: 30, 6: 90}
CAL_LINE = re.compile(r"\[CAL\] ratio=([-0-9.,]+) sign=([-0-9.,]+)")


def firmware_constants() -> tuple[list[float], list[float]]:
    src = (ROOT / "robotarmver2" / "robotarmver2.ino").read_text(encoding="utf-8")

    def arr(name):
        m = re.search(rf"const float {name}\[6\]\s*=\s*\{{([^}}]*)\}}", src)
        return [float(x) for x in m.group(1).split(",")]

    return arr("GEAR_RATIOS"), arr("DIR_SIGNS")


def ask(prompt: str, choices: str) -> str:
    while True:
        a = input(f"  ? {prompt} [{'/'.join(choices)}]: ").strip().lower()
        if a in choices:
            return a


def ask_float(prompt: str) -> float | None:
    a = input(f"  ? {prompt} (Enter = bỏ qua): ").strip().replace(",", ".")
    try:
        return abs(float(a)) if a else None
    except ValueError:
        return None


def read_cal(port: serial.Serial, reader: Reader) -> tuple[list[float], list[float]] | None:
    """Hỏi firmware tỉ số/chiều đang dùng (lệnh $P). None nếu firmware cũ không hỗ trợ."""
    reader.lines.clear()
    port.write(b"$P\n")
    reader.poll(0.6)
    for line in reversed(reader.lines):
        m = CAL_LINE.search(line)
        if m:
            return [float(x) for x in m.group(1).split(",")], [float(x) for x in m.group(2).split(",")]
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--joints", default="1,2,3,4,5,6")
    args = ap.parse_args()
    joints = [int(j) for j in args.joints.split(",")]
    if input("Robot đang ở tư thế 0 DỰNG THẲNG ĐỨNG, vùng làm việc trống, tay sẵn sàng ngắt nguồn động cơ? Gõ YES để bắt đầu: ").strip() != "YES":
        return 0

    port = serial.Serial(args.port, 115200, timeout=0.05)
    reader = Reader(port)
    reader.poll(1.0)
    if reader.last_angles is None:
        print("KHÔNG có telemetry → sai cổng COM hoặc firmware chưa chạy.")
        return 1
    port.write(packet(CMD_STOP, [0.0] * 6))
    reader.poll(0.3)
    port.write(b"?")  # trạng thái driver: phải đủ 6 dòng OK (nguồn động lực đã bật)
    reader.poll(1.0)

    live = read_cal(port, reader)
    if live:
        ratios, signs = live
        print("Firmware trên board hỗ trợ hiệu chỉnh trực tiếp. Đang dùng:\n  tỉ số =", ratios, "\n  chiều =", signs)
    else:
        ratios, signs = firmware_constants()
        print("Firmware trên board KHÔNG trả lời $P (bản cũ) → chỉ in đề xuất, theo hằng số trong robotarmver2.ino:\n"
              "  GEAR_RATIOS =", ratios, "\n  DIR_SIGNS   =", signs)
    old_r, old_s = list(ratios), list(signs)
    new_r, new_s = list(ratios), list(signs)
    home = list(reader.last_angles)
    results: dict[int, dict] = {}
    notes: dict[int, str] = {}

    def go(k: int, value: float) -> None:
        target = list(home)
        target[k] = value
        port.write(packet(CMD_MOVE_PC, target))

    queue = list(joints)
    while queue:
        j = queue.pop(0)
        k, deg = j - 1, TEST_DEG[j]
        print(f"\n===== J{j}: lệnh +{deg}° (chậm). Chiều dương trên twin: {TWIN_POSITIVE[j]}")
        input("  Nhấn Enter để chạy...")
        for step in range(5, deg + 1, 5):
            go(k, home[k] + step)
            reader.poll(0.2)
            if step == 10:
                port.write(b"?")
        wait_until(reader, k, home[k] + deg, timeout=15)
        reader.poll(0.5)
        fw_deg = reader.last_angles[k] - home[k]
        print(f"  Firmware báo đã đi {fw_deg:+.1f}°")
        r = {"lenh_do": deg, "firmware_bao_do": round(fw_deg, 2), "ti_so_khi_thu": new_r[k], "chieu_khi_thu": new_s[k]}
        r["quay"] = ask("Khớp có quay không? c=có, k=không nhúc nhích, r=chỉ rung/kêu", "ckr")
        if r["quay"] == "c":
            r["cung_chieu_twin"] = ask("Cùng chiều với mô tả twin ở trên?", "cn")
            r["goc_that_do"] = ask_float(f"Góc THẬT khớp đã quay (độ, đo bằng thước đo góc/điện thoại; lệnh {deg}°)")
        if j == 6:
            r["tu_quay_sau_khi_dung"] = ask("J6 còn tiếp tục quay sau khi firmware đã dừng?", "cn")
        go(k, home[k])
        wait_until(reader, k, home[k], timeout=15)
        reader.poll(0.5)
        if r["quay"] == "c":
            r["ve_dung_cho_cu"] = ask("Khớp đã về ĐÚNG vị trí ban đầu?", "cn")
        if j == 6:
            print("  Thử nhiễu: J6 đứng yên (firmware không phát xung J6), J2 chạy ±10° 2 lần. Quan sát J6.")
            for _ in range(2):
                go(1, home[1] + 10); reader.poll(2.5)
                go(1, home[1]); reader.poll(2.5)
            r["j6_tu_quay_khi_j2_chay"] = ask("J6 có tự quay/giật trong lúc J2 chạy?", "cn")
        results[j] = r

        # ----- kết luận cho khớp này -----
        note, changed = "", False
        if r["quay"] != "c":
            note = "KHÔNG QUAY → kiểm tra cơ khí/dây (xem docs/HUONG_DAN_KIEM_TRA_J6.md mục 2–4)" if r["quay"] == "k" \
                else "CHỈ RUNG → nghi đấu nhầm cặp cuộn / đứt 1 dây / kẹt cơ khí"
        else:
            if r.get("cung_chieu_twin") == "n":
                new_s[k] = -new_s[k]
                note += f"đảo chiều → {new_s[k]:+.0f}; "
                changed = True
            if r.get("goc_that_do") and abs(r["goc_that_do"] - deg) > 0.5:
                was = new_r[k]
                new_r[k] = round(was * deg / r["goc_that_do"], 4)
                note += f"tỉ số {was} → {new_r[k]} (lệnh {deg}° / thật {r['goc_that_do']}°); "
                changed = True
            if r.get("ve_dung_cho_cu") == "n":
                note += "KHÔNG về đúng chỗ → mất bước / đai nhảy răng (dòng thấp, kẹt, tốc độ cao); "
        if r.get("tu_quay_sau_khi_dung") == "c" or r.get("j6_tu_quay_khi_j2_chay") == "c":
            note += "J6 TỰ QUAY khi không có xung → dây STEP J6 (GPIO38) hở/lỏng hoặc nhiễu: kiểm tra dây, chung GND, thêm trở 10k kéo xuống GND ở chân STEP driver; "
        notes[j] = note or "OK"
        print(f"  Kết luận J{j}: {notes[j]}")

        # ----- gửi ngay xuống firmware (khớp đã về 0) và cho thử lại -----
        if live and changed:
            port.write(f"$D {j} {new_s[k]:.0f}\n".encode()); reader.poll(0.4)
            port.write(f"$K {j} {new_r[k]}\n".encode()); reader.poll(0.6)
            home = list(reader.last_angles)
            print(f"  → Đã gửi xuống board: J{j} tỉ số {new_r[k]}, chiều {new_s[k]:+.0f}")
            if ask(f"Thử lại J{j} với giá trị mới để kiểm chứng?", "cn") == "c":
                queue.insert(0, j)

    port.write(packet(CMD_STOP, [0.0] * 6))
    port.write(b"?")
    reader.poll(1.5)
    saved = False
    if live and (new_r != old_r or new_s != old_s):
        if ask("LƯU các giá trị đã hiệu chỉnh vào Flash của ESP32 (giữ sau khi tắt nguồn)?", "cn") == "c":
            port.write(b"$S\n")
            reader.poll(0.8)
            saved = True
        else:
            print("  Chưa lưu: giá trị mới sẽ mất khi ESP32 khởi động lại.")
    port.close()

    print("\n================ KẾT QUẢ ================")
    for j in sorted(notes):
        print(f"J{j}: {notes[j]}")
    print("\nGiá trị sau hiệu chỉnh" + (" (ĐÃ LƯU trên board)" if saved else "") + " — nên chép vào robotarmver2.ino làm mặc định:")
    print("const float GEAR_RATIOS[6] = {" + ", ".join(f"{x:g}" for x in new_r) + "};")
    print("const float DIR_SIGNS[6] = {" + ", ".join(f"{x:.1f}" for x in new_s) + "};")
    out = ROOT / "reports" / f"hieu_chinh_khop_{time.strftime('%Y%m%d_%H%M%S')}.json"
    out.write_text(json.dumps({"firmware_cu": {"GEAR_RATIOS": old_r, "DIR_SIGNS": old_s},
                               "de_xuat": {"GEAR_RATIOS": new_r, "DIR_SIGNS": new_s},
                               "gui_truc_tiep_xuong_board": bool(live), "da_luu_flash": saved,
                               "ket_qua": results}, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"Đã lưu: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
