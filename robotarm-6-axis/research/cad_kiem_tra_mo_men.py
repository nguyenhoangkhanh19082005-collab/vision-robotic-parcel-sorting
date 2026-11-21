r"""Đối chiếu firmware (dòng điện, tỉ số truyền) với file lắp ráp Roboter_dong_co_1_27.step:
mô-men trọng lực xấu nhất ở J2..J5 (từ thể tích + trọng tâm chi tiết trong STEP) so với mô-men động cơ có được.
Chạy research\cad_doc_step_lap_rap.py trước (tạo .tmp/step_lap_rap.json).
    E:\UDMT\.venv\Scripts\python.exe research\cad_kiem_tra_mo_men.py
GIẢ ĐỊNH (chưa cân thật): nhựa in PLA 1.24 g/cm3 × hệ số đặc FILL; thép 7.85; nhôm 2.7; khối lượng động cơ theo
datasheet/ước lượng; tải ở đầu hút PAYLOAD kg; hiệu suất hộp số hành tinh 2 cấp 0.80, đai 0.95.
"""
import json, os, sys
import numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
parts = json.load(open(os.path.join(ROOT, ".tmp", "step_lap_rap.json"), encoding="utf-8"))
G = 9.81
PAYLOAD = 0.20
MOTOR_KG = {"Dong_co_moi": 0.50, "STEPPERONLINE": 0.40}  # 42STH38 0.28 + hộp số 36 mm ≈ 0.2; PG5 ≈ 0.40
J5_MOTOR_KG, J6_MOTOR_KG = 0.35, 0.22                     # 17PM-K4xx (datasheet 350 g), 17HS3401 (220 g)


def link_of(p):
    """Khâu mang chi tiết: 0 đế, 1 thân xoay (sau J1), 2 cánh tay trên (sau J2), 3 khuỷu (sau J3), 4 cẳng tay (sau J4),
    5 cổ tay (sau J5), 6 mặt bích (sau J6). None = ốc vít → gán theo chi tiết lớn gần nhất."""
    n, (x, y, z) = p["name"], p["com"]
    if n.startswith(("AX-1_10", "AX-1_11", "De_nang", "Dong_co_moi_J1", "Lager_78")): return 0
    if n.startswith("Riemenscheibe_16mm"): return 0 if x < -120 else 1 if x < 0 else 2 if x < 120 else 4
    if n.startswith("Riemenscheibe_40mm"): return 1 if x < 0 else 5
    if n.startswith(("AX-1", "Dong_co_moi_J2")): return 1
    if n.startswith("Lager_65"): return 1 if x < 50 else 2
    if n.startswith(("AX-2", "Puly_55", "Mat_bich_5mm_J2", "Dong_co_moi_J3")): return 2
    if n.startswith(("AX-3", "Puly_40", "Mat_bich_5mm_J3", "STEPPERONLINE", "Lager_52")): return 3
    if n.startswith(("AX-4", "Lager_32")): return 4
    if n.startswith("Nema17"): return 4 if y > -70 else 5
    if n.startswith(("AX-5", "Lager_19")): return 5
    if n.startswith("AX-6"): return 6
    return None


def mass_of(p, fill):
    n, v = p["name"], p["vol_cm3"]
    for k, m in MOTOR_KG.items():
        if n.startswith(k): return m
    if n.startswith("Nema17"): return J5_MOTOR_KG if p["com"][1] > -70 else J6_MOTOR_KG
    if n.startswith(("AX-", "De_nang", "Puly_", "Riemenscheibe_40", "Riemenscheibe_55")): return v * 1.24 * fill / 1000
    if n.startswith(("Riemenscheibe_16", "Mat_bich")): return v * 2.7 / 1000
    if n.startswith("Lager"): return v * 7.85 * 0.8 / 1000
    return v * 7.85 / 1000  # ốc, đai ốc


def axis_of(pred):
    p = next(q for q in parts if pred(q))
    c = max(p["cyl"], key=lambda c: c["area"])
    d = np.array(c["d"]); return np.array(c["p"]), d / np.linalg.norm(d)


def dist(pt, ax):
    o, d = ax; r = np.asarray(pt) - o
    return np.linalg.norm(r - (r @ d) * d)


anchors = [(np.array(p["com"]), link_of(p)) for p in parts if link_of(p) is not None]
for p in parts:
    p["link"] = link_of(p)
    if p["link"] is None:
        c = np.array(p["com"]); p["link"] = min(anchors, key=lambda a: np.linalg.norm(a[0] - c))[1]

AX = {2: axis_of(lambda q: q["name"].startswith("Puly_55")), 3: axis_of(lambda q: q["name"].startswith("Puly_40")),
      4: axis_of(lambda q: q["name"].startswith("STEPPERONLINE")),
      5: axis_of(lambda q: q["name"].startswith("Riemenscheibe_40mm") and q["com"][0] > 0),
      6: axis_of(lambda q: q["name"].startswith("Nema17") and q["com"][1] < -70)}
ax6 = AX[6]
flange = np.array(next(p["com"] for p in parts if p["name"].startswith("AX-6")))
out6 = ax6[1] if (flange - ax6[0]) @ ax6[1] > 0 else -ax6[1]
tcp = flange + 40.0 * out6  # tải đặt cách tâm mặt bích 40 mm theo trục J6
L2 = dist(AX[3][0], AX[2])  # chiều dài cánh tay trên (khoảng cách trục J2–J3)


def distal(j, fill, payload):
    """(khối lượng kg, trọng tâm mm) của mọi thứ sau khớp j (khâu ≥ j) + tải."""
    ms = [(mass_of(p, fill), np.array(p["com"])) for p in parts if p["link"] >= j] + [(payload, tcp)]
    m = sum(a for a, _ in ms)
    return m, sum(a * c for a, c in ms) / m


def need(fill, payload):
    """Mô-men trọng lực xấu nhất (N·m) tại J2..J5: tay duỗi ngang."""
    r = {}
    m3, c3 = distal(3, fill, payload)
    ms2 = [(mass_of(p, fill), np.array(p["com"])) for p in parts if p["link"] == 2]
    m2 = sum(a for a, _ in ms2); c2 = sum(a * c for a, c in ms2) / m2
    r[2] = G * (m2 * dist(c2, AX[2]) + m3 * (L2 + dist(c3, AX[3]))) / 1000   # cánh tay trên + cẳng tay cùng duỗi ngang
    r[3] = G * m3 * dist(c3, AX[3]) / 1000
    for j in (4, 5):
        m, c = distal(j, fill, payload); r[j] = G * m * dist(c, AX[j]) / 1000
    return r, (m2, m3)


# ---- mô-men có được tại khớp (N·m) theo firmware 14/11: dòng RMS thực / định mức × mô-men giữ datasheet ----
I = {2: 1.49 / 1.68, 3: 1.49 / 1.68, 4: 0.40 / 0.40}
HOLD = {2: 0.7, 3: 0.7, 4: 0.7, 5: 0.8}
DYN = 0.7  # mô-men kéo khi đang chạy ≈ 70 % mô-men giữ (đường cong pull-out, tốc độ thấp)


def have(belt):
    t = {2: 0.36 * I[2] * 27 * 0.80 * belt[2] * 0.95, 3: 0.36 * I[3] * 27 * 0.80 * belt[3] * 0.95, 4: 0.25 * I[4] * 5.18 * 0.90}
    return t


lines = []
P = lines.append
P(f"Trục khớp đọc từ STEP: J2–J3 cách nhau {L2:.0f} mm; J3 → tâm mặt bích {dist(flange, AX[3]):.0f} mm; tải đặt cách trục J3 {dist(tcp, AX[3]):.0f} mm")
for fill in (0.6, 1.0):
    r, (m2, m3) = need(fill, PAYLOAD)
    m4, _ = distal(4, fill, PAYLOAD); m5, _ = distal(5, fill, PAYLOAD)
    P(f"\nNhựa in đặc {fill*100:.0f} %, tải {PAYLOAD*1000:.0f} g: khối lượng cánh tay trên {m2:.2f} kg; sau J3 {m3:.2f} kg; sau J4 {m4:.2f} kg; sau J5 {m5:.2f} kg")
    P("  Mô-men trọng lực xấu nhất (tay duỗi ngang): " + ", ".join(f"J{j} {r[j]:.2f} N·m" for j in (2, 3, 4, 5)))
    r0, _ = need(fill, 0.0)
    P("  Mỗi 100 g tải thêm ở đầu hút: " + ", ".join(f"J{j} +{(r[j]-r0[j])/PAYLOAD*0.1:.2f}" for j in (2, 3, 4, 5)) + " N·m")
    for label, belt in (("đai theo CAD (J2 4.0, J3 3.0)", {2: 4.0, 3: 3.0}), ("đai nếu 16T/40T/55T (J2 3.44, J3 2.5)", {2: 55 / 16, 3: 2.5})):
        t = have(belt)
        P(f"  [{label}]")
        for j in (2, 3, 4):
            P(f"    J{j}: cần {r[j]:.2f} | có khi chạy ≈ {t[j]*DYN:.1f} (hệ số an toàn {t[j]*DYN/r[j]:.1f}) | có khi giữ {HOLD[j]*100:.0f} % ≈ {t[j]*HOLD[j]:.1f} (an toàn {t[j]*HOLD[j]/r[j]:.1f})")
    for label, th in (("J5 nếu cuộn 0.9 A/5.6 Ω (900 mA = 100 %)", 0.50 * 0.99), ("J5 nếu cuộn 1.5 A/2.4 Ω (900 mA = 59 %)", 0.50 * 0.89 / 1.5)):
        t5 = th * 2.5 * 0.95
        P(f"    {label}: cần {r[5]:.2f} | chạy ≈ {t5*DYN:.2f} (an toàn {t5*DYN/r[5]:.1f}) | giữ 80 % ≈ {t5*0.8:.2f} (an toàn {t5*0.8/r[5]:.1f})")

# ---- tải lên hộp số và đai ----
r, _ = need(0.6, PAYLOAD)
P("\nTải cơ khí (nhựa 60 %, tải 200 g, đai theo CAD):")
P(f"  Mô-men trục ra hộp số 1:27 khi động cơ kéo hết sức ở 1.49 A: {0.36*I[2]*27*0.8:.1f} N·m (động cơ) — cần kiểm tra mô-men cho phép của hộp số 36 mm")
P(f"  Mô-men trục ra hộp số chỉ để GIỮ trọng lực: J2 {r[2]/4.0:.2f} N·m, J3 {r[3]/3.0:.2f} N·m")
for j, d_big in ((2, 51.0), (3, 38.1)):
    P(f"  Lực căng đai J{j} do trọng lực (puly lớn Ø{d_big:.0f} mm): {r[j]/(d_big/2000):.0f} N")
txt = "\n".join(lines)
print(txt)
os.makedirs(os.path.join(ROOT, "reports"), exist_ok=True)
open(os.path.join(ROOT, "reports", "kiem_tra_mo_men_theo_CAD.txt"), "w", encoding="utf-8").write(txt + "\n")
