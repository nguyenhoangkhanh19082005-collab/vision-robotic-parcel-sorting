r"""Gá giác hút J6 bản DÀI (21/11/2026) — sửa lỗi ống khí bị gập 90° ở gá cũ (tao_ga_giac_hut_J6.py).

Gá cũ: khoang ống cao 36 mm → đầu đuôi khí chỉ cách mặt đế 21 mm (11 mm khi lò xo nén hết 10 mm) → ống cứng phải bẻ
gần như vuông góc ngay dưới đuôi khí (bán kính uốn ≈ 12 mm) để ra cửa sổ.
Bản dài: giữ nguyên mọi mặt lắp ghép (đế 46 × 45 dày 4, 4 lỗ M3 trên vòng Ø41.54 ở 60/120/240/300°, tấm trên dày 5 có lỗ
Ø12.4 cho ren M12×1 + 4 rãnh chữ U thả vít), chỉ kéo dài khoang ống 36 → 66 mm (+30 mm) và vát góc trong 4 → 6 mm.
Cửa sổ 2 phía theo phương Y (hở hoàn toàn, 36 × 66 mm) cho ống uốn thoải đi ra; khoang cao đủ thò tay/cờ lê siết đai
ốc M12 phía trong và cắm ống vào đuôi khí.
Hệ cục bộ: gốc tại tâm mặt ngoài mặt bích AX-6, Z dọc trục J6 hướng ra (giống gá cũ).
Đầu ra: assets/CAD_giac_hut_dai/ (STEP, STL hướng in, ảnh) + assets/Gui_xuong_in/H_ga_giac_hut_J6_dai/.
    E:\UDMT\.venv\Scripts\python.exe -X utf8 src\cad\tao_ga_giac_hut_J6_dai.py
"""
import json, os, shutil, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import tao_ga_giac_hut_J6 as G0   # dùng lại make_bracket / make_cup_dummy với thông số mới
from tao_chi_tiet_dong_co_1_27 import to_frame, vol, write_step, write_stl
from rut_ngan_lib import mesh_check
from cad_util import load
from cad_khung_dong_co import verts
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec

ROOT = G0.ROOT
OUT = os.path.join(ROOT, "assets", "CAD_giac_hut_dai")
PRINT = os.path.join(ROOT, "assets", "Gui_xuong_in", "H_ga_giac_hut_J6_dai")

# ---------------- thông số mới (mm) ----------------
G0.H_CAV = 66.0                       # khoang ống: 36 → 66
G0.GUSSET = 6.0                       # vát góc trong lớn hơn cho khung cao
G0.H = G0.T_BASE + G0.H_CAV + G0.T_TOP
H, HALF_X, HALF_Y, T_BASE = G0.H, G0.HALF_X, G0.HALF_Y, G0.T_BASE
STROKE = 10.0                         # hành trình lò xo HSP (đuôi khí đi xuống khi giác bị ép)
HOSE_OD = 10.0                        # đường kính ngoài ống giả định (ống 6 × 10, cứng) — dùng để tính bán kính uốn


def build():
    """→ (gá, giác hút mô phỏng, z đầu đuôi khí, z miệng giác) trong hệ gá."""
    b = G0.make_bracket(); cup, z_barb, z_tip = G0.make_cup_dummy()
    return b, cup, z_barb, z_tip


def flange_frame():
    """(tâm mặt ngoài mặt bích AX-6, ma trận R hệ gá) trong hệ lắp ráp 1:27 (cùng cách tính với gá cũ)."""
    parts = json.load(open(os.path.join(ROOT, ".tmp", "step_lap_rap.json"), encoding="utf-8"))
    p = next(q for q in parts if q["name"].startswith("Nema17") and q["com"][1] < -70)
    c = max(p["cyl"], key=lambda c: c["area"]); o6 = np.array(c["p"]); d6 = np.array(c["d"]) / np.linalg.norm(c["d"])
    v = verts(load("AX-6__NAUO1"))
    if (v.mean(0) - o6) @ d6 < 0: d6 = -d6
    face = o6 + d6 * ((v - o6) @ d6).max()
    nuts = [np.array(q["com"]) for q in parts if q["name"].startswith("Mutter_M3") and np.linalg.norm(np.array(q["com"]) - face) < 30]
    r = nuts[0] - face; x = r - (r @ d6) * d6; x /= np.linalg.norm(x)
    return face, np.array([x, np.cross(d6, x), d6])


def max_bend_radius(drop):
    """Bán kính uốn lớn nhất (tâm ống) để ống đi từ đầu đuôi khí xuống, uốn về phía cửa sổ ±Y và ra khỏi gá ở
    y = HALF_Y mà mặt dưới ống vẫn cách mặt đế ≥ 1 mm. drop = khoảng cách đầu đuôi khí → mặt đế.
    Cung tròn bán kính R tiếp tuyến trục tại đầu đuôi khí: tại y = HALF_Y đã đi xuống sqrt(2·R·HALF_Y − HALF_Y²)."""
    h = drop - HOSE_OD / 2 - 1.0
    return (h * h + HALF_Y * HALF_Y) / (2 * HALF_Y) if h > 0 else 0.0


def main():
    os.makedirs(OUT, exist_ok=True); os.makedirs(PRINT, exist_ok=True)
    rep = []
    bracket, cup, z_barb, z_tip = build()
    ok = BRepCheck_Analyzer(bracket).IsValid()
    old_drop, new_drop = 36.0 - (G0.NUT_H + 1.0 + G0.BARB_L), z_barb - T_BASE   # gá cũ: khoang 36, đầu đuôi khí 15 mm dưới mặt trong tấm trên
    rep.append(f"Gá dài: {2*HALF_X:.0f} × {2*HALF_Y:.0f} × {H:.0f} mm (cũ 46 × 45 × 45), khoang ống {G0.H_CAV:.0f} mm (cũ 36), "
               f"thể tích {vol(bracket)/1000:.1f} cm3 (≈ {vol(bracket)/1000*1.24*0.8:.0f} g PLA/PETG), B-rep hợp lệ {ok}")
    rep.append(f"Đầu đuôi khí cách mặt đế: {new_drop:.0f} mm khi giác tự do, {new_drop-STROKE:.0f} mm khi nén hết lò xo "
               f"(gá cũ {old_drop:.0f} / {old_drop-STROKE:.0f} mm)")
    rep.append(f"Bán kính uốn tối đa của ống Ø{HOSE_OD:g} (tâm ống) để ra cửa sổ không cấn đế: tự do {max_bend_radius(new_drop):.0f} mm, "
               f"nén hết {max_bend_radius(new_drop-STROKE):.0f} mm (gá cũ {max_bend_radius(old_drop):.0f} / {max_bend_radius(old_drop-STROKE):.0f} mm)")
    rep.append(f"Miệng giác (chưa nén) cách mặt bích J6 {z_tip:.0f} mm (gá cũ {z_tip-30:.0f}); nén hết {z_tip-STROKE:.0f} mm → tool_length ≈ {z_tip:.0f} mm")
    rep.append("Lắp: 4 vít M3×12 qua đế vào 4 đai ốc chìm của AX-6 (góc 60/120/240/300°, tháo 6 vít M3×10 cắm sẵn); ren M12 kẹp 2 đai ốc "
               "trên tấm trên; cắm ống vào đuôi khí rồi cho ống uốn thoải ra 1 trong 2 cửa sổ hở.")

    write_step([("Ga_giac_hut_J6_dai", bracket)], os.path.join(OUT, "Ga_giac_hut_J6_dai.step"))
    t = gp_Trsf(); t.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), np.pi / 2)   # nằm nghiêng, cửa sổ hướng lên: không cần support
    t2 = gp_Trsf(); t2.SetTranslation(gp_Vec(0, H / 2, HALF_Y))
    printable = BRepBuilderAPI_Transform(BRepBuilderAPI_Transform(bracket, t, True).Shape(), t2, True).Shape()
    stl = os.path.join(OUT, "Ga_giac_hut_J6_dai_huong_in.stl"); write_stl(printable, stl)
    nt, nopen, man, v = mesh_check(stl)
    rep.append(f"STL hướng in: {nt} tam giác, cạnh hở {nopen}, đa tạp {man}, {v:.1f} cm3; in nằm nghiêng (cao in 45 mm), không cần support")
    shutil.copy(stl, PRINT)
    face, R = flange_frame()
    write_step([("AX-6_mat_bich_J6", load("AX-6__NAUO1")), ("Ga_giac_hut_J6_dai", to_frame(bracket, face, R)),
                ("Giac_hut_L60-M12_HSP-40_mo_phong", to_frame(cup, face, R))], os.path.join(OUT, "Lap_rap_ga_giac_hut_J6_dai.step"))
    txt = "\n".join(rep); print(txt)
    open(os.path.join(OUT, "ket_qua_kiem_tra.txt"), "w", encoding="utf-8").write(txt + "\n")

    # ---- ảnh: gá cũ vs gá dài, ống uốn với bán kính lớn nhất khi nén hết ----
    import pyvista as pv
    tmp = os.path.join(ROOT, ".tmp", "v3"); os.makedirs(tmp, exist_ok=True)
    G0.H_CAV, G0.GUSSET = 36.0, 4.0; G0.H = 45.0
    old_b, old_cup, old_zb, _ = build()
    G0.H_CAV, G0.GUSSET = 66.0, 6.0; G0.H = H
    bich = to_frame(load("AX-6__NAUO1"), -R @ face, R.T)
    for n, s in (("ga_cu", old_b), ("giac_cu", old_cup), ("ga_moi", bracket), ("giac_moi", cup), ("bich", bich)):
        write_stl(s, os.path.join(tmp, f"{n}.stl"))

    def hose(zb, drop):
        rb = max(max_bend_radius(drop - STROKE), 3.0); z0 = zb - STROKE          # vẽ ở trạng thái nén hết (khó nhất)
        pts = [(0, 0, zb + 8), (0, 0, z0 + 0.5)] + [(0, rb - rb * np.cos(a), z0 - rb * np.sin(a)) for a in np.linspace(0, np.pi / 2, 30)] + [(0, rb + 40, z0 - rb)]
        return pv.Spline(np.array(pts), 120).tube(radius=HOSE_OD / 2), rb

    p = pv.Plotter(off_screen=True, window_size=(1600, 900), shape=(1, 2))
    for k, (tag, zb, drop, dz) in enumerate((("ga_cu", old_zb, old_drop, 0), ("ga_moi", z_barb, new_drop, 0))):
        p.subplot(0, k)
        p.add_mesh(pv.read(os.path.join(tmp, "bich.stl")).clip(normal="x", origin=(0, 0, 0)), color="#8b949e")
        p.add_mesh(pv.read(os.path.join(tmp, f"{tag}.stl")).clip(normal="x", origin=(0, 0, 0)), color="#f0883e")   # cắt đôi để thấy ống
        p.add_mesh(pv.read(os.path.join(tmp, tag.replace("ga", "giac") + ".stl")), color="#c9d1d9")   # thân ren cố định; ống vẽ ở vị trí đuôi khí khi nén hết
        hm, rb = hose(zb, drop); p.add_mesh(hm, color="#2f81f7")
        p.add_text(("GA CU" if k == 0 else "GA DAI MOI") + f"  (mat cat doc, giac nen het 10 mm)\nong O{HOSE_OD:g}: ban kinh uon toi da ~{rb:.0f} mm",
                   font_size=11, color="black")
        p.camera_position = [(300, 0, 65), (0, 25, 65), (0, 0, -1)]
        p.set_background("white")
    p.link_views(); img = os.path.join(OUT, "so_sanh_ga_cu_va_ga_dai.png"); p.screenshot(img); p.close()
    p = pv.Plotter(off_screen=True, window_size=(1100, 900))
    p.add_mesh(pv.read(os.path.join(tmp, "bich.stl")), color="#8b949e"); p.add_mesh(pv.read(os.path.join(tmp, "ga_moi.stl")), color="#f0883e")
    p.add_mesh(pv.read(os.path.join(tmp, "giac_moi.stl")), color="#c9d1d9"); p.add_mesh(hose(z_barb, new_drop)[0], color="#2f81f7")
    p.camera_position = [(170, 190, -40), (0, 0, 60), (0, 0, -1)]; p.set_background("white")
    p.screenshot(os.path.join(OUT, "ga_giac_hut_dai_3d.png")); p.close()
    print("Đã ghi:", OUT, PRINT)


if __name__ == "__main__":
    main()
