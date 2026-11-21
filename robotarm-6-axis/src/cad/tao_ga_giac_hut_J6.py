r"""Gá in 3D gắn giác hút chân không L60-M12 + HSP-40-2 (ren M12×1.0 dài 15 mm kẹp bằng 2 đai ốc, đuôi khí 6 mm, hành
trình lò xo 10 mm, giác Ø40 cao 22 mm) vào mặt bích J6 (AX-6: đĩa Ø55, 6 lỗ M3 + đai ốc chìm trên vòng Ø41.54).

Hình dạng: khung cổng (đế – 2 vách – tấm trên) đùn theo phương Y → in nằm nghiêng trên Bambu Lab A1 KHÔNG cần support;
2 mặt bên theo phương Y để hở hoàn toàn (cửa sổ 36 × 36 mm) cho ống khí đi ra từ đuôi khí rồi uốn sang ngang.
Hệ cục bộ của gá: gốc tại tâm mặt ngoài mặt bích, Z dọc trục J6 hướng ra, X đi qua 2 lỗ M3 KHÔNG dùng.
Đầu ra: assets/CAD_giac_hut/ (STEP + STL, STL đã xoay theo hướng in) và ảnh reports/cad_giac_hut/.
    E:\UDMT\.venv\Scripts\python.exe src\cad\tao_ga_giac_hut_J6.py
"""
import glob, json, os, sys
import numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "research")); sys.path.insert(0, os.path.dirname(__file__))
from cad_util import CACHE, cylinders, load
from cad_khung_dong_co import verts
from tao_chi_tiet_dong_co_1_27 import box, collide, cyl, to_frame, vol, write_step, write_stl
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon, BRepBuilderAPI_Transform
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCone, BRepPrimAPI_MakePrism
from OCP.gp import gp_Ax1, gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec

OUT = os.path.join(ROOT, "assets", "CAD_giac_hut")
IMG = os.path.join(ROOT, "reports", "cad_giac_hut")

# ---------------- thông số gá (mm) ----------------
PCD_R = 20.77          # bán kính vòng lỗ M3 của mặt bích AX-6 (đo trên CAD)
HALF_X, HALF_Y = 23.0, 22.5   # kích thước ngoài 46 × 45
T_BASE = 4.0           # đế áp mặt bích; vít M3×12 qua đế ăn vào đai ốc chìm trong mặt bích (sâu 8 mm)
H_CAV = 36.0           # khoang cho đuôi khí + đoạn uốn ống (ống 6 mm, bán kính uốn 15 mm, còn hở đế 3 mm)
T_TOP = 5.0            # tấm kẹp giữa 2 đai ốc M12 (ren dài 15 − tấm 5 = 10 mm cho 2 đai ốc)
T_LEG = 5.0            # vách bên
GUSSET = 4.0           # vát góc trong tăng cứng
D_M3, D_HEAD, D_M12 = 3.4, 6.5, 12.4
H = T_BASE + H_CAV + T_TOP
# ---------------- giác hút (ước lượng từ hình của người bán, cần đo lại) ----------------
NUT_H, BARB_L, THREAD_L = 4.0, 10.0, 15.0
BELOW_THREAD, CUP_H, CUP_R = 27.0, 22.0, 20.0   # 52 − 10 − 15 = 27 mm từ cuối ren tới đáy lục giác


def prism_xz(points, y0, y1):
    """Đùn đa giác (x, z) theo phương Y từ y0 tới y1."""
    mk = BRepBuilderAPI_MakePolygon()
    for x, z in points:
        mk.Add(gp_Pnt(x, y0, z))
    mk.Close()
    return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(mk.Wire()).Face(), gp_Vec(0, y1 - y0, 0)).Shape()


def cut(a, b): return BRepAlgoAPI_Cut(a, b).Shape()
def fuse(a, b): return BRepAlgoAPI_Fuse(a, b).Shape()


def make_bracket():
    body = box(-HALF_X, -HALF_Y, 0, HALF_X, HALF_Y, H)
    xi, z0, z1, g = HALF_X - T_LEG, T_BASE, T_BASE + H_CAV, GUSSET
    cavity = prism_xz([(-xi + g, z0), (xi - g, z0), (xi, z0 + g), (xi, z1 - g), (xi - g, z1), (-xi + g, z1), (-xi, z1 - g), (-xi, z0 + g)],
                      -HALF_Y - 1, HALF_Y + 1)
    body = cut(body, cavity)
    body = cut(body, cyl(D_M12 / 2, z1 - 1, H + 1))                       # lỗ M12 cho thân ren giác hút
    for sx in (1, -1):
        for sy in (1, -1):
            x, y = sx * PCD_R * np.cos(np.radians(60)), sy * PCD_R * np.sin(np.radians(60))
            body = cut(body, cyl(D_M3 / 2, -1, T_BASE + 1, x, y))          # lỗ vít M3 trên đế
            slot = fuse(cyl(D_HEAD / 2, z1 - 1, H + 1, x, y),              # rãnh chữ U trên tấm trên: thả vít + đưa lục giác
                        box(x - D_HEAD / 2, min(y, sy * (HALF_Y + 1)), z1 - 1, x + D_HEAD / 2, max(y, sy * (HALF_Y + 1)), H + 1))
            body = cut(body, slot)
    return body


def make_cup_dummy():
    """Mô hình đơn giản của giác hút để kiểm tra khoảng hở (không in)."""
    zt = T_BASE + H_CAV            # mặt trong tấm trên
    top = zt - NUT_H - 1.0         # đầu ren (nhô 1 mm trên đai ốc trong)
    s = cyl(6.0, top, top + THREAD_L)
    s = fuse(s, cyl(3.0, top - BARB_L, top))                               # đuôi khí 6 mm
    s = fuse(s, cyl(10.5, zt - NUT_H, zt)); s = fuse(s, cyl(10.5, H, H + NUT_H))   # 2 đai ốc
    zb = top + THREAD_L + BELOW_THREAD
    s = fuse(s, cyl(4.0, top + THREAD_L, zb)); s = fuse(s, cyl(7.5, zb - 8, zb))
    cone = BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(0, 0, zb), gp_Dir(0, 0, 1)), 10.0, CUP_R, CUP_H).Shape()
    return fuse(s, cone), top - BARB_L, zb + CUP_H


def rotated(shape, point, axis, deg):
    t = gp_Trsf(); t.SetRotation(gp_Ax1(gp_Pnt(*point), gp_Dir(*axis)), np.radians(deg))
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def main():
    os.makedirs(OUT, exist_ok=True); os.makedirs(IMG, exist_ok=True)
    parts = json.load(open(os.path.join(ROOT, ".tmp", "step_lap_rap.json"), encoding="utf-8"))

    def axis_of(pred):
        p = next(q for q in parts if pred(q)); c = max(p["cyl"], key=lambda c: c["area"]); d = np.array(c["d"])
        return np.array(c["p"]), d / np.linalg.norm(d)

    # ---- khung mặt bích J6 trong hệ lắp ráp ----
    o6, d6 = axis_of(lambda q: q["name"].startswith("Nema17") and q["com"][1] < -70)
    ax6 = load("AX-6__NAUO1"); v = verts(ax6)
    if (v.mean(0) - o6) @ d6 < 0: d6 = -d6
    face = o6 + d6 * ((v - o6) @ d6).max()                                 # tâm mặt ngoài mặt bích
    nuts = [np.array(q["com"]) for q in parts if q["name"].startswith("Mutter_M3") and np.linalg.norm(np.array(q["com"]) - face) < 30]
    assert len(nuts) == 6, len(nuts)
    r = nuts[0] - face; x = r - (r @ d6) * d6; x /= np.linalg.norm(x)
    R = np.array([x, np.cross(d6, x), d6])
    ang = sorted(round(float(np.degrees(np.arctan2((n - face) @ R[1], (n - face) @ R[0])))) % 360 for n in nuts)
    report = [f"Mặt bích AX-6: Ø55, 6 đai ốc M3 chìm ở các góc {ang}° (hệ gá), bán kính {np.mean([np.hypot((n-face)@R[0], (n-face)@R[1]) for n in nuts]):.2f} mm"]

    bracket = make_bracket(); cup, z_barb, z_tip = make_cup_dummy()
    report.append(f"Gá: {2*HALF_X:.0f} × {2*HALF_Y:.0f} × {H:.0f} mm, thể tích {vol(bracket)/1000:.1f} cm3 (≈ {vol(bracket)/1000*1.24*0.8:.0f} g PLA/PETG với 4 vách + 25 % infill)")
    report.append(f"Khoang ống khí: rộng {2*(HALF_X-T_LEG):.0f} × cao {H_CAV:.0f} mm, hở 2 phía theo phương Y; đầu đuôi khí cách mặt đế {z_barb - T_BASE:.0f} mm")
    report.append(f"Miệng giác hút (chưa nén) cách mặt bích {z_tip:.0f} mm; nén hết lò xo 10 mm còn {z_tip - 10:.0f} mm → tool_length ≈ {z_tip:.0f} mm (đo lại trên hàng thật)")

    # ---- kiểm tra va chạm: khối bao khi J6 quay (trụ) + giác hút, quét J5 ----
    env = fuse(cyl(np.hypot(HALF_X, HALF_Y) + 0.5, 0.2, H), cyl(CUP_R + 1, H, z_tip))
    env_asm = to_frame(env, face, R)
    o5, d5 = axis_of(lambda q: q["name"].startswith("Riemenscheibe_40mm") and q["com"][0] > 0)
    names = [os.path.basename(f)[:-5] for f in sorted(glob.glob(os.path.join(CACHE, "*.brep")))]
    flange_hw = lambda n: n.startswith(("ISK_M3x10", "Mutter_M3")) and np.linalg.norm(verts(load(n)).mean(0) - face) < 32
    hits0 = [(n, collide(env_asm, load(n))) for n in names if not n.startswith("AX-6") and not flange_hw(n)]
    hits0 = [f"{n} {c:.0f} mm3" for n, c in hits0 if c > 1]
    report.append("Va chạm ở tư thế CAD (J6 quay đủ vòng): " + (", ".join(hits0) if hits0 else "KHÔNG"))
    fixed = [n for n in names if n.startswith(("AX-4", "AX-3", "AX-2", "AX-1", "STEPPERONLINE", "Dong_co")) or (n.startswith("Nema17") and "NAUO21" not in n)]
    worst = {}
    for a in range(-110, 111, 10):
        e = rotated(env_asm, o5, d5, a)
        for n in fixed:
            c = collide(e, load(n))
            if c > 1: worst.setdefault(n.split("__")[0], []).append(a)
    report.append("Va chạm khi quét J5 từ −110° tới +110° (so với tư thế CAD, bước 10°): " +
                  ("; ".join(f"{n} tại J5 = {min(v)}…{max(v)}°" for n, v in worst.items()) if worst else "KHÔNG"))
    report.append("Lưu ý: tháo 6 vít M3×10 đang cắm sẵn ở mặt ngoài mặt bích (giữ lại 6 đai ốc chìm); gá dùng 4 vít M3×12 ở các góc 60°, 120°, 240°, 300°.")

    # ---- xuất file ----
    bracket_asm = to_frame(bracket, face, R)
    write_step([("Ga_giac_hut_J6", bracket)], os.path.join(OUT, "Ga_giac_hut_J6.step"))
    t = gp_Trsf(); t.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), np.pi / 2)   # Y → Z: nằm nghiêng, cửa sổ hướng lên
    t2 = gp_Trsf(); t2.SetTranslation(gp_Vec(0, H / 2, HALF_Y))
    printable = BRepBuilderAPI_Transform(BRepBuilderAPI_Transform(bracket, t, True).Shape(), t2, True).Shape()
    write_stl(printable, os.path.join(OUT, "Ga_giac_hut_J6_huong_in.stl"))
    write_step([("AX-6_mat_bich_J6", ax6), ("Ga_giac_hut_J6", bracket_asm), ("Giac_hut_L60-M12_HSP-40_mo_phong", to_frame(cup, face, R))],
               os.path.join(OUT, "Lap_rap_ga_giac_hut_J6.step"))
    for n, s in (("_ga", bracket), ("_giac", cup), ("_bich", to_frame(ax6, -R @ face, R.T))):   # mặt bích về hệ gá để dựng ảnh
        write_stl(s, os.path.join(ROOT, ".tmp", f"giac_hut{n}.stl"))
    txt = "\n".join(report); print(txt)
    open(os.path.join(OUT, "ket_qua_kiem_tra.txt"), "w", encoding="utf-8").write(txt + "\n")

    # ---- ảnh minh họa (hệ gá) ----
    import pyvista as pv
    tmp = os.path.join(ROOT, ".tmp")
    hose_pts = [(0, 0, z_barb + 8)] + [(0, 15 - 15 * np.cos(a), z_barb - 15 * np.sin(a)) for a in np.linspace(0, np.pi / 2, 12)] + [(0, 60, z_barb - 15)]
    hose = pv.Spline(np.array(hose_pts), 60).tube(radius=3.0)
    for tag, cam in (("tong_the", ((150, 170, -60), (0, 0, 45), (0, 0, -1))), ("cua_so_ong_khi", ((0, 230, 25), (0, 0, 40), (0, 0, -1))),
                     ("mat_tren", ((60, 60, 260), (0, 0, 45), (0, 1, 0)))):
        p = pv.Plotter(off_screen=True, window_size=(1100, 900))
        p.add_mesh(pv.read(os.path.join(tmp, "giac_hut_bich.stl")), color="#8b949e", smooth_shading=True)
        p.add_mesh(pv.read(os.path.join(tmp, "giac_hut_ga.stl")), color="#f0883e", show_edges=False, smooth_shading=False)
        p.add_mesh(pv.read(os.path.join(tmp, "giac_hut_giac.stl")), color="#c9d1d9", opacity=1.0, smooth_shading=True)
        p.add_mesh(hose, color="#58a6ff")
        p.camera_position = list(cam); p.set_background("white")
        p.screenshot(os.path.join(IMG, f"ga_giac_hut_{tag}.png")); p.close()
    print("Đã ghi:", OUT, IMG)


if __name__ == "__main__":
    main()
