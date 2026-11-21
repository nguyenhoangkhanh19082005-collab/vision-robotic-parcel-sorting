r"""Thay động cơ J4 (17HS13-0404S-PG5 có hộp số 5,18:1, ~500 g) bằng MinebeaMitsumi 17PM-K4xx (42 × 42 × 48 mm, 350 g,
trục Ø5 dài 24 mm, 4 lỗ M3 vuông 31 mm, gờ định tâm Ø22 × 2) lắp TRỰC TIẾP, trên bản cẳng tay đã rút ngắn
(src/cad/rut_ngan_cang_tay.py), và thêm gá + giác hút ở mặt bích J6 (src/cad/tao_ga_giac_hut_J6.py).

Chi tiết nhựa phải in (ngoài AX-3_2_rut_ngan và AX-4_4_rut_ngan đã có trong assets/CAD_rut_ngan):
  AX-3_1_rut_ngan_khong_xoay : ống động cơ J4 rút ngắn 9,5 mm nhưng KHÔNG xoay 45° nửa xa (động cơ mới bắt bằng 4 lỗ ở
                               góc vuông nên thân vuông tự nằm song song thành hộp) + rãnh thoát 4 góc thân động cơ
                               → thay cho AX-3_1_rut_ngan.
  AX-3_3_17PM                : tấm gá AX-3_3 thêm 4 lỗ M3 (31 × 31 mm) có hạ bậc đầu vít ở mặt phía cổ tay.
  AX-4_6_truc_5mm            : khớp nối AX-4_6, lỗ trục Ø8 → Ø5,1.
"s" = khoảng cách dọc trục J4 tính từ trục J3 trong bản GỐC; các chi tiết sau đó dời về phía J3 như bản rút ngắn.
Kết quả: assets/CAD_dong_co_J4_17PM/ và reports/dong_co_J4_17PM/
    E:\UDMT\.venv\Scripts\python.exe src\cad\thay_dong_co_J4.py [dung] [kiem] [momen] [anh]     (mặc định tất cả)
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from rut_ngan_lib import *  # noqa: F401,F403
import rut_ngan_kiem_tra as K
from rut_ngan_cang_tay import BIG, CUT_TUBE, D_TUBE, SHIFT_1, SHIFT_2, SHIFT_3, TWIST_PARTS, describe
from tao_chi_tiet_dong_co_1_27 import write_step, write_stl
import tao_ga_giac_hut_J6 as GA
from cad_khung_dong_co import verts
from OCP.BRep import BRep_Builder
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakePrism
from OCP.BRepTools import BRepTools
from OCP.TopoDS import TopoDS_Shape
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Vec

OUT = os.path.join(ROOT, "assets", "CAD_dong_co_J4_17PM"); IMG = os.path.join(ROOT, "reports", "dong_co_J4_17PM")
TMP = os.path.join(ROOT, ".tmp", "j4_17pm")
# ---- động cơ 17PM-K4xx (datasheet MinebeaMitsumi 17PM-K, 2024/02/28) ----
M_SQ, M_L, M_PILOT_D, M_PILOT_H, M_SHAFT_D, M_SHAFT_L, M_HOLE = 42.0, 48.0, 22.0, 2.0, 5.0, 24.0, 31.0
M_CHAMFER = 2.5                  # vát góc thân (datasheet không ghi; nếu góc thật nhọn hơn thì khe hở với ống Ø60 còn ≥ 0,3 mm)
M_CONN = (17.0, 10.0, 6.0)       # đầu nối JST: rộng × dài (từ mặt đuôi) × nhô khỏi thân
M_KG, OLD_KG = 0.350, 0.500      # 17PM-K4xx 350 g (datasheet); động cơ cũ = 350 + 150 g theo số cân của người dùng
S_FACE = 90.0                    # mặt lắp động cơ = mặt gần của tấm gá AX-3_3 (s 90..100)
# ---- tấm gá / khớp nối ----
D_M3, D_CB, H_CB = 3.4, 6.0, 3.2  # lỗ lọt M3, hạ bậc đầu vít Ø6 sâu 3,2 → vít M3×10 ăn ren động cơ 3,2 mm (ren sâu ≥ 4)
BORE = 5.1
RELIEF_SQ = 0.6                  # AX-3_1: nới mỗi phía 0,6 mm quanh thân vuông 42 mm của động cơ
# ---- gá + giác hút ----
CUP_KG = 0.080                   # L60-M12 + HSP-40 + 2 đai ốc + đoạn ống khí (ước lượng, chưa cân)
G = 9.81
log = []


def P(*a):
    s = " ".join(str(x) for x in a); log.append(s); print(s, flush=True)


def prism(points, s0, s1):
    mk = BRepBuilderAPI_MakePolygon()
    for x, y in points: mk.Add(gp_Pnt(x, y, s0))
    mk.Close()
    return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(mk.Wire()).Face(), gp_Vec(0, 0, s1 - s0)).Shape()


def lc(r, x, y, s0, s1):         # trụ dọc trục trong hệ khâu 3 (chưa đưa về hệ lắp ráp)
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, y, s0), gp_Dir(0, 0, 1)), r, s1 - s0).Shape()


def make_motor(conn_sign=+1):
    """Động cơ 17PM-K4xx trong hệ lắp ráp, mặt lắp tại s = S_FACE, thân vuông song song X3/Y3, đầu nối về phía conn_sign·Y3."""
    h, c = M_SQ / 2, M_CHAMFER
    pts = [(h - c, h), (-h + c, h), (-h, h - c), (-h, -h + c), (-h + c, -h), (h - c, -h), (h, -h + c), (h, h - c)]
    s = prism(pts, S_FACE - M_L, S_FACE)
    for sx in (1, -1):
        for sy in (1, -1):
            s = cut(s, lc(1.25, sx * M_HOLE / 2, sy * M_HOLE / 2, S_FACE - 4.5, S_FACE + 1))
    s = fuse(s, lc(M_PILOT_D / 2, 0, 0, S_FACE - 1, S_FACE + M_PILOT_H))
    s = fuse(s, lc(M_SHAFT_D / 2, 0, 0, S_FACE, S_FACE + M_SHAFT_L))
    w, l, t = M_CONN
    y0, y1 = (h - 1, h + t) if conn_sign > 0 else (-h - t, -h + 1)
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    s = fuse(s, BRepPrimAPI_MakeBox(gp_Pnt(-w / 2, y0, S_FACE - M_L), w, y1 - y0, l).Shape())
    return to_frame(unify(s), R3)


def make_screw(x, y, s_head, length, into=-1, R=None, origin=None):
    """Vít M3 đầu trụ đơn giản: mặt tựa đầu vít tại s_head, thân dài length về phía into (−1: về J3)."""
    a, b = (s_head - length, s_head) if into < 0 else (s_head, s_head + length)
    h0, h1 = (s_head, s_head + 3.0) if into < 0 else (s_head - 3.0, s_head)
    s = fuse(lc(1.5, x, y, a, b), lc(2.75, x, y, h0, h1))
    return s if R is None else to_frame(s, R, origin) if origin is not None else to_frame(s, R)


def make_ax31():
    old = loadi(15)
    new = unify(fuse(keep(old, -BIG, CUT_TUBE[0]), shifted(keep(old, CUT_TUBE[1], BIG), D_TUBE), 1e-4))
    P(f"AX-3_1: bỏ lát ống s {CUT_TUBE[0]}..{CUT_TUBE[1]} ({D_TUBE} mm), KHÔNG xoay nửa xa")
    describe("AX-3_1_khong_xoay", old, new)
    new = shifted(new, SHIFT_1)
    # rãnh thoát 4 góc thân động cơ (phòng khi góc thật vuông sắc hơn mô hình): hốc vuông 43,2 mm dọc đoạn thân động cơ
    h = M_SQ / 2 + RELIEF_SQ
    pocket = lbox(R3, -h, h, -h, h, S_FACE - M_L - SHIFT_2, S_FACE - SHIFT_2 + 0.1)
    removed = vol(common(new, pocket)); new = unify(cut(new, pocket))
    P(f"AX-3_1: hốc vuông {2*h:.1f} mm cho thân động cơ, bỏ {removed:.0f} mm3 ở 4 góc; số khối {n_solids(new)}, BRepCheck {valid(new)}")
    assert n_solids(new) == 1 and valid(new)
    return new


def make_plate():
    old = limit_tol(loadi(17)); new = old; h = M_HOLE / 2
    for sx in (1, -1):
        for sy in (1, -1):
            tool = fuse(lc(D_M3 / 2, sx * h, sy * h, S_FACE - 1, S_FACE + 11), lc(D_CB / 2, sx * h, sy * h, S_FACE + 10 - H_CB, S_FACE + 11))
            new = cut(new, to_frame(tool, R3))
    new = unify(new)
    P(f"AX-3_3: thêm 4 lỗ Ø{D_M3} tại (±{h}, ±{h}) + hạ bậc Ø{D_CB} sâu {H_CB} ở mặt s = {S_FACE + 10:.0f}; lỗ giữa Ø22 giữ nguyên (khớp gờ định tâm Ø22 của động cơ); "
          f"bỏ {vol(old) - vol(new):.0f} mm3 (dự tính 4 lỗ ≈ {4 * (np.pi * (D_M3 / 2) ** 2 * (10 - H_CB) + np.pi * (D_CB / 2) ** 2 * H_CB):.0f})")
    describe("AX-3_3_17PM", old, new)
    return shifted(new, SHIFT_2)


def screw_axis_coupling():
    """Trục vít hãm M3 của khớp nối AX-4_6 (chi tiết 32) → (điểm trên trục J4, hướng ra ngoài) trong hệ lắp ráp."""
    c = max(parts[32]["cyl"], key=lambda c: c["area"]); p = np.array(c["p"]); d = np.array(c["d"]); d /= np.linalg.norm(d)
    q = p + d * ((E - p) @ d)                       # điểm trên trục vít gần E nhất
    q = q + 0                                       # (trục vít cắt trục J4)
    s = (q - E) @ F; foot = E + F * s
    out = np.array(parts[32]["com"]) - foot; out -= (out @ F) * F; out /= np.linalg.norm(out)
    off = np.linalg.norm(np.cross(np.array(parts[32]["com"]) - foot, d))
    return foot, out, s, off


def make_coupling():
    old = limit_tol(loadi(26)); foot, out, s, off = screw_axis_coupling()
    new = fuse(old, to_frame(lc(4.3, 0, 0, 105.0, 119.0), R3))
    new = cut(new, to_frame(lc(BORE / 2, 0, 0, 104.0, 120.0), R3))
    hole = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*(foot + out * 1.0)), gp_Dir(*out)), 1.6, 6.5).Shape()
    new = unify(cut(new, hole))
    P(f"AX-4_6: lỗ trục Ø8 → Ø{BORE}; kéo dài lỗ vít hãm M3 (tại s = {s:.1f}, trục vít lệch tâm {off:.2f} mm) vào tới lỗ trục; "
      f"thể tích {vol(old)/1000:.2f} → {vol(new)/1000:.2f} cm3")
    describe("AX-4_6_truc_5mm", old, new)
    return shifted(new, SHIFT_2)


def flange_frame():
    """Tâm mặt ngoài mặt bích J6 + hệ gá (như tao_ga_giac_hut_J6.py), ở bản GỐC."""
    o6, d6 = AXES[5]; v = verts(loadi(29))
    if (v.mean(0) - o6) @ d6 < 0: d6 = -d6
    face = o6 + d6 * ((v - o6) @ d6).max()
    nuts = [i for i, q in enumerate(parts) if q["name"].startswith("Mutter_M3") and np.linalg.norm(np.array(q["com"]) - face) < 30]
    assert len(nuts) == 6, len(nuts)
    r = np.array(parts[nuts[0]]["com"]) - face; x = r - (r @ d6) * d6; x /= np.linalg.norm(x)
    def on_face(q):              # vít M3×10 cắm từ mặt ngoài mặt bích trên vòng lỗ Ø41,54
        r = np.array(q["com"]) - face; ax = r @ d6
        return q["name"].startswith("ISK_M3x10") and abs(ax) < 8 and abs(np.linalg.norm(r - ax * d6) - GA.PCD_R) < 1.0
    screws = [i for i, q in enumerate(parts) if on_face(q)]
    return face, np.array([x, np.cross(d6, x), d6]), screws


def build():
    os.makedirs(OUT, exist_ok=True); os.makedirs(TMP, exist_ok=True)
    P("== Dựng hình ==")
    new = {15: ("AX-3_1_rut_ngan_khong_xoay", make_ax31()), 17: ("AX-3_3_17PM", make_plate()), 26: ("AX-4_6_truc_5mm", make_coupling())}
    for i in TWIST_PARTS:        # 2 kẹp AX-3_4 + 4 vít, 4 đai ốc kẹp: về lại hướng gốc (bản rút ngắn trước đã xoay 45°)
        if i != 17: new[i] = (parts[i]["name"], shifted(loadi(i), SHIFT_2))
    hub = K.new_shape(16)
    cand = {sg: shifted(make_motor(sg), SHIFT_2) for sg in (+1, -1)}
    dd = {sg: min(dist_min(m, hub), dist_min(m, new[15][1])) for sg, m in cand.items()}
    sg = max(dd, key=dd.get)
    P(f"Động cơ 17PM-K4xx: mặt lắp tại s = {S_FACE - SHIFT_2:.1f}, đuôi tại s = {S_FACE - M_L - SHIFT_2:.1f} (bản rút ngắn); "
      f"đầu nối quay về {'+' if sg > 0 else '−'}Y3 (khe hở tới vỏ: +Y3 {dd[1]:.2f} mm, −Y3 {dd[-1]:.2f} mm)")
    extra = [("Dong_co_J4_17PM-K4xx", cand[sg])]
    h = M_HOLE / 2
    for sx in (1, -1):
        for sy in (1, -1):
            extra.append(("ISK_M3x10_dong_co_J4", shifted(make_screw(sx * h, sy * h, S_FACE + 10 - H_CB, 10.0, R=R3), SHIFT_2)))
    face, Rg, flange_screws = flange_frame()
    bracket = shifted(GA.to_frame(GA.make_bracket(), face, Rg), SHIFT_3)
    cup_l, z_barb, z_tip = GA.make_cup_dummy(); cup = shifted(GA.to_frame(cup_l, face, Rg), SHIFT_3)
    extra += [("Ga_giac_hut_J6", bracket), ("Giac_hut_L60-M12_HSP-40_mo_phong", cup)]
    for sx in (1, -1):
        for sy in (1, -1):
            x, y = sx * GA.PCD_R * np.cos(np.radians(60)), sy * GA.PCD_R * np.sin(np.radians(60))
            extra.append(("ISK_M3x12_ga_giac_hut", shifted(GA.to_frame(make_screw(x, y, GA.T_BASE, 12.0), face, Rg), SHIFT_3)))
    P(f"Gá giác hút: bỏ {len(flange_screws)} vít M3×10 cắm sẵn trên mặt bích, thêm 4 vít M3×12; miệng giác cách mặt bích {z_tip:.0f} mm")
    removed = {191, *flange_screws}
    # ---- ghi file ----
    for f in os.listdir(TMP):
        if f.endswith(".brep"): os.remove(os.path.join(TMP, f))      # file trung gian của lần chạy trước
    for i, R in ((15, R3), (17, R3), (26, R3)):
        name, shape = new[i]
        write_step([(name, shape)], os.path.join(OUT, name + ".step")); write_stl(to_local(shape, R), os.path.join(OUT, name + ".stl"))
        nt, nopen, man, v = mesh_check(os.path.join(OUT, name + ".stl"))
        P(f"  {name}.stl: {nt} tam giác, cạnh hở {nopen}, đa tạp {man}, thể tích {v:.1f} cm3")
    write_step([extra[0]], os.path.join(OUT, "Dong_co_17PM-K4xx.step"))
    asm = [new[i] if i in new else (K.NEW_NAMES.get(i, q["name"]), K.new_shape(i)) for i, q in enumerate(parts) if i not in removed] + extra
    write_step(asm, os.path.join(OUT, "Roboter_rut_ngan_17PM_giac_hut.step"))
    for i, (n, s) in new.items(): BRepTools.Write_s(s, os.path.join(TMP, f"new_{i:03d}.brep"))
    for k, (n, s) in enumerate(extra): BRepTools.Write_s(s, os.path.join(TMP, f"extra_{k:02d}_{n}.brep"))
    np.save(os.path.join(TMP, "removed.npy"), np.array(sorted(removed)))
    np.save(os.path.join(TMP, "tool.npy"), np.array([face - F * SHIFT_3, Rg[2], [z_tip, 0, 0]]))
    P(f"Đã ghi {OUT} ({len(asm)} chi tiết trong file lắp ráp)")


# ---------------------------------------------------------------- đọc lại
def rd(f):
    s = TopoDS_Shape(); BRepTools.Read_s(s, f, BRep_Builder()); return s


def loaded():
    import glob
    new = {int(os.path.basename(f)[4:7]): rd(f) for f in glob.glob(os.path.join(TMP, "new_*.brep"))}
    extra = [(os.path.basename(f)[9:-5], rd(f)) for f in sorted(glob.glob(os.path.join(TMP, "extra_*.brep")))]
    return new, extra, set(int(x) for x in np.load(os.path.join(TMP, "removed.npy")))


def shape_c(i, new):             # hình của chi tiết i trong phương án động cơ mới
    return new[i] if i in new else K.new_shape(i)


# ---------------------------------------------------------------- kiểm tra hình học
def check():
    new, extra, removed = loaded(); ex = dict((n, s) for n, s in extra if not n.startswith("ISK"))
    motor = ex["Dong_co_J4_17PM-K4xx"]
    P("\n== Kiểm tra hình học (bản rút ngắn + động cơ 17PM-K4xx) ==")
    P("  Chồng lấn (mm3) / khe hở nhỏ nhất (mm) của động cơ mới với chi tiết xung quanh:")
    around = (16, 15, 17, 26, 25, 120, 121, 11, 10, 111, 177, 56, 57, 58, 59, 126, 127, 128, 129, 18, 19, 33, 34, 35, 36, 32, 134)
    bad = 0
    for b in around:
        sb = limit_tol(shape_c(b, new)); v = interference(motor, sb); d = dist_min(motor, sb)
        vo = interference(limit_tol(loadi(191)), limit_tol(loadi(b)))          # động cơ cũ trong bản gốc
        flag = "   <== CHỒNG LẤN" if v > vo + 5.0 else ""    # vít hãm #32 vẽ ăn vào trục ở cả bản gốc
        bad += bool(flag)
        if v > 0.05 or d < 3.0: P(f"    {parts[b]['name'] + '#' + str(b):28s}: giao {v:6.1f} (động cơ cũ, bản gốc: {vo:6.1f})   khe {d:5.2f}{flag}")
    P("  Chi tiết in mới với chi tiết xung quanh (giao mm3, chỉ in dòng > 0,5):")
    for a, others in ((17, (15, 18, 19, 33, 34, 35, 36, 130, 131, 132, 133, 26)), (26, (25, 32, 134, 106, 107, 108, 109, 17)),
                      (15, (16, 18, 19, 24, 118, 119, 56, 57, 58, 59, 33, 34, 35, 36))):
        for b in others:
            v = interference(limit_tol(new[a]), limit_tol(shape_c(b, new)))
            vo = interference(limit_tol(loadi(a)), limit_tol(loadi(b)))
            flag = "   <== TĂNG" if v > vo + 0.5 else ""
            bad += bool(flag)
            if v > 0.5 or flag: P(f"    {parts[a]['name']}_moi × {K.nm(b):30s}: bản gốc {vo:7.1f} → {v:7.1f}{flag}")
    for n, s in extra:
        if n.startswith("ISK_M3x10"):
            v1, v2 = interference(s, limit_tol(new[17])), interference(s, limit_tol(new[26]))
            if v1 > 0.5 or v2 > 0.5: P(f"    vít động cơ × tấm gá {v1:.1f}, × khớp nối {v2:.1f}   <== CHỒNG LẤN"); bad += 1
    P(f"  Khe hở đầu vít động cơ ↔ khớp nối AX-4_6: {min(dist_min(s, new[26]) for n, s in extra if n.startswith('ISK_M3x10')):.2f} mm")
    P(f"  Trục động cơ nằm trong khớp nối: s {105 - SHIFT_2:.1f}..{S_FACE + M_SHAFT_L - SHIFT_2:.1f} = {S_FACE + M_SHAFT_L - 105:.1f} mm "
      f"(hộp số cũ: 8,0 mm); khe hở bán kính {BORE / 2 - M_SHAFT_D / 2:.2f} mm")
    ga, cup = ex["Ga_giac_hut_J6"], ex["Giac_hut_L60-M12_HSP-40_mo_phong"]
    hits = []
    for i in range(len(parts)):
        if i in removed or K.LINK[i] < 4 or parts[i]["name"].startswith(("AX-6", "Mutter")): continue
        v = interference(ga, limit_tol(shape_c(i, new))) + interference(cup, limit_tol(shape_c(i, new)))
        if v > 0.5: hits.append(f"{K.nm(i)} {v:.1f}")
    P("  Gá + giác hút chồng lấn với chi tiết khác (tư thế CAD): " + (", ".join(hits) if hits else "KHÔNG"))
    bad += len(hits)
    P(f"  → Số chỗ chồng lấn mới: {bad}")


# ---------------------------------------------------------------- mô-men
def masses(case, fill):
    """→ [(kg, trọng tâm, khâu)] cho case: 'goc' (dài gốc, động cơ cũ), 'rn' (rút ngắn, động cơ cũ), 'moi' (rút ngắn, 17PM)."""
    new, extra, removed = loaded(); out = []
    tool = np.load(os.path.join(TMP, "tool.npy")); face, d6, z_tip = tool[0], tool[1], tool[2][0]
    back = F * SHIFT_3 if case == "goc" else 0
    for i, p in enumerate(parts):
        n = p["name"]
        if i in removed and not (i == 191 and case != "moi"): continue
        if case == "goc": c, v = np.array(p["com"]), p["vol_cm3"]
        elif case == "moi" and i in new: c, v = com(new[i]), vol(new[i]) / 1000
        elif K.shift[i] < 0: s = K.new_shape(i); c, v = com(s), vol(s) / 1000
        else: c, v = np.array(p["com"]) - F * K.shift[i], p["vol_cm3"]
        if n.startswith("Dong_co_moi"): m = 0.50
        elif n.startswith("STEPPERONLINE"): m = OLD_KG
        elif n.startswith("Nema17"): m = 0.35 if p["com"][1] > -70 else 0.22
        elif n.startswith(("AX-", "De_nang", "Puly_", "Riemenscheibe_40", "Riemenscheibe_55")): m = v * 1.24 * fill / 1000
        elif n.startswith(("Riemenscheibe_16", "Mat_bich")): m = v * 2.7 / 1000
        elif n.startswith("Lager"): m = v * 7.85 * 0.8 / 1000
        else: m = v * 7.85 / 1000
        out.append((m, c, K.LINK[i]))
    for n, s in extra:
        c = com(s) + back
        if n.startswith("Dong_co_J4"):
            if case == "moi": out.append((M_KG, c - back, 3))
        elif n.startswith("ISK_M3x10"):
            if case == "moi": out.append((vol(s) * 7.85e-6, c - back, 3))
        elif n.startswith("Ga_"): out.append((vol(s) * 1.24e-6 * 0.8, c, 6))
        elif n.startswith("Giac"): out.append((CUP_KG, c, 6))
        else: out.append((vol(s) * 7.85e-6, c, 6))
    return out, face + back + d6 * z_tip


def dist_ax(pt, ax):
    o, d = ax; r = np.asarray(pt) - o; return np.linalg.norm(r - (r @ d) * d)


def rot(p, a, deg):
    T = K.rot4(p, a, deg); return lambda x: T[:3, :3] @ x + T[:3, 3]


def torque():
    P("\n== Mô-men trọng lực xấu nhất (tay duỗi ngang) — có gá + giác hút ở J6, tải đặt tại miệng giác hút ==")
    P(f"  Giả định: động cơ J4 cũ {OLD_KG*1000:.0f} g, 17PM-K4xx {M_KG*1000:.0f} g; giác hút + đai ốc + ống {CUP_KG*1000:.0f} g; "
      "PLA 1,24 g/cm3 × hệ số đặc; các giả định khác như research/cad_kiem_tra_mo_men.py")
    ax2, ax3 = AXES[1], AXES[2]; L2 = dist_ax(ax3[0], ax2)
    names = {"goc": "dài gốc + động cơ J4 cũ", "rn": "rút ngắn + động cơ J4 cũ", "moi": "rút ngắn + 17PM-K4xx  "}
    have = {}
    for label, belt in (("đai theo CAD (4,0 / 3,0)", (4.0, 3.0)), ("đai 16T/55T, 16T/40T (3,44 / 2,5)", (55 / 16, 2.5))):
        have[label] = (0.36 * 27 * 0.80 * belt[0] * 0.95, 0.36 * 27 * 0.80 * belt[1] * 0.95)
    res = {}
    for fill in (0.6, 1.0):
        for pay in (0.0, 0.2, 0.5):
            P(f"  Nhựa đặc {fill*100:.0f} %, tải {pay*1000:.0f} g:")
            for case in ("goc", "rn", "moi"):
                ms, tcp = masses(case, fill)
                m3l = [(m, c) for m, c, l in ms if l >= 3] + [(pay, tcp)]; m3 = sum(m for m, _ in m3l); c3 = sum(m * c for m, c in m3l) / m3
                m2l = [(m, c) for m, c, l in ms if l == 2]; m2 = sum(m for m, _ in m2l); c2 = sum(m * c for m, c in m2l) / m2
                t3 = G * m3 * dist_ax(c3, ax3) / 1000; t2 = G * (m2 * dist_ax(c2, ax2) + m3 * (L2 + dist_ax(c3, ax3))) / 1000
                res[(fill, pay, case)] = (t2, t3)
                b2, b3 = res[(fill, pay, "goc")]
                P(f"    {names[case]}: J2 {t2:5.2f} N·m ({(t2/b2-1)*100:+5.1f} %)   J3 {t3:5.2f} N·m ({(t3/b3-1)*100:+5.1f} %)   "
                  f"sau J3 {m3:.2f} kg, trọng tâm cách J3 {dist_ax(c3, ax3):.0f} mm")
    P("  Mô-men động cơ J2/J3 có được tại khớp (42STH38 0,36 N·m ở ≥ 1,68 A, hộp số 27 η 0,80, đai η 0,95; khi chạy ≈ 70 % mô-men giữ):")
    for label, (h2, h3) in have.items():
        for fill, pay in ((0.6, 0.2), (1.0, 0.5)):
            t2, t3 = res[(fill, pay, "moi")]; o2, o3 = res[(fill, pay, "goc")]
            P(f"    [{label}] nhựa {fill*100:.0f} %, tải {pay*1000:.0f} g: J2 có {h2*0.7:.1f} / cần {t2:.2f} → hệ số {h2*0.7/t2:.1f} (bản gốc {h2*0.7/o2:.1f});  "
              f"J3 có {h3*0.7:.1f} / cần {t3:.2f} → hệ số {h3*0.7/t3:.1f} (bản gốc {h3*0.7/o3:.1f})")
    # ---- J4 lắp trực tiếp ----
    P("\n== Khớp J4 khi bỏ hộp số 5,18:1 (động cơ nối thẳng vào khớp nối) ==")
    ax4 = (E, F); p5, d5 = AXES[4][0] - F * SHIFT_3, AXES[4][1]
    hold_old, hold_new = 0.25 * 5.18 * 0.90, 0.50
    run_old, run_new = hold_old * 0.7, 0.39       # 17PM-K442B/K448B: đáy đường pull-out 24 V ở 400 Hz ≈ 0,39 / 0,44 N·m
    for pay in (0.0, 0.2, 0.5):
        ms, tcp = masses("moi", 0.6); row = []
        for a5 in (0, 45, 90):
            r5 = rot(p5, d5, a5); mv = np.zeros(3); I = 0.0
            for m, c, l in [x for x in ms if x[2] >= 4] + [(pay, tcp, 6)]:
                c = r5(c) if l >= 5 else c; r = c - E; r = r - (r @ F) * F; mv += m * r; I += m * (r @ r)
            row.append((a5, G * np.linalg.norm(mv) / 1000, I * 1e-6))
        P(f"  tải {pay*1000:.0f} g: " + "; ".join(f"J5 gập {a}° → cần {t:.2f} N·m (quán tính tải {I*1e4:.1f} kg·cm2)" for a, t, I in row))
        t = row[-1][1]
        P(f"      J5 gập 90°, dụng cụ nằm ngang: hộp số cũ giữ {hold_old:.2f} / chạy {run_old:.2f} N·m → hệ số {run_old/t:.1f};  "
          f"17PM-K4xx trực tiếp giữ {hold_new:.2f} / chạy {run_new:.2f} N·m → hệ số {run_new/t:.1f}")
        if t < hold_new:
            dth = np.degrees(np.arcsin(t / hold_new)) / 50
            lever = np.linalg.norm(rot(p5, d5, 90)(tcp) - E - ((rot(p5, d5, 90)(tcp) - E) @ F) * F)
            P(f"      độ lệch góc rotor dưới tải này ≈ {dth:.2f}° → miệng giác hút lệch ≈ {np.radians(dth)*lever:.1f} mm (tay đòn {lever:.0f} mm)")
    P(f"  Độ phân giải J4: 16 vi bước → {360/3200:.4f}°/vi bước (hộp số cũ {360/3200/5.181818:.4f}°). Quán tính rotor 75 g·cm2 = 0,075 kg·cm2.")
    P("  Firmware khi lắp động cơ mới: GEAR_RATIOS[3] 5.181818 → 1.0 (hoặc lệnh $K 4 1.0), MOTOR_CURRENTS[3] 400 → 900 mA "
      "(K442B 0,9 A) hoặc tới 1500 mA (K448B 1,5 A); đổi dấu $D 4 nếu quay ngược.")


# ---------------------------------------------------------------- ảnh
def images():
    import pyvista as pv
    os.makedirs(IMG, exist_ok=True)
    new, extra, removed = loaded()

    def stl(shape, name):
        f = os.path.join(TMP, name + ".stl"); write_stl(shape, f); return pv.read(f)

    PAL = {"AX-3_1": "#f0883e", "AX-3_2": "#a371f7", "AX-3_3": "#3fb950", "AX-3_4": "#58a6ff", "AX-4_1": "#db61a2", "AX-4_2": "#e3b341",
           "AX-4_3": "#79c0ff", "AX-4_4": "#56d364", "AX-4_5": "#d2a8ff", "AX-4_6": "#ffc680", "AX-5": "#ffa198", "AX-6": "#8957e5",
           "Ga_": "#f0883e", "Giac": "#30363d", "Dong_co_J4": "#d1242f"}

    def color(n):
        for k, c in PAL.items():
            if n.startswith(k): return c
        if n.startswith(("Dong_co", "Nema", "STEPPER")): return "#d1242f"
        if n.startswith("Lager"): return "#1f6feb"
        if n.startswith(("Riemen", "Puly")): return "#1a7f37"
        return "#57606a" if n.startswith(("ISK", "Mutter")) else "#c9d1d9"

    def scene(case):
        items = []
        for i, p in enumerate(parts):
            if K.LINK[i] < 2: continue
            if case == "moi" and i in removed: continue
            if case == "moi" and i in new: items.append((p["name"], stl(new[i], f"img_new_{i}")))
            else: items.append((p["name"], pv.read(K.stl_of(i, True))))
        if case == "moi": items += [(n, stl(s, f"img_extra_{k}")) for k, (n, s) in enumerate(extra)]
        return items

    def local(m):
        m = m.copy(); m.points = (m.points - E) @ R3.T; return m

    # 1. mặt cắt dọc qua trục J4 (nhìn dọc trục J3): động cơ cũ ↔ mới
    p = pv.Plotter(off_screen=True, window_size=(1800, 1300), shape=(2, 1))
    for row, case in enumerate(("rn", "moi")):
        p.subplot(row, 0)
        for n, m in scene(case):
            m = local(m)
            if m.bounds[5] < -50 or m.bounds[4] > 150: continue
            c = m.clip(normal=(1, 0, 0), origin=(0.01, 0, 0), invert=True)
            if c.n_points: p.add_mesh(c, color=color(n), smooth_shading=False)
        p.add_text("Rut ngan + dong co cu 17HS13-PG5 (500 g)" if case == "rn" else "Rut ngan + 17PM-K4xx lap truc tiep (350 g)", font_size=13, color="black")
        p.camera_position = [(400, 0, 50), (0, 0, 50), (0, 1, 0)]; p.enable_parallel_projection(); p.camera.parallel_scale = 62
    p.set_background("white"); p.screenshot(os.path.join(IMG, "cat_doc_dong_co_J4_cu_moi.png")); p.close()
    # 2. toàn cảnh cẳng tay + gá giác hút
    p = pv.Plotter(off_screen=True, window_size=(1800, 1100))
    for n, m in scene("moi"):
        m = local(m); op = 0.35 if n.startswith(("AX-3_1", "AX-3_2", "AX-4_4")) else 1.0
        p.add_mesh(m, color=color(n), opacity=op, smooth_shading=False)
    p.camera_position = [(330, 230, -120), (0, 0, 110), (0, 1, 0)]; p.set_background("white")
    p.screenshot(os.path.join(IMG, "toan_canh_17PM_giac_hut.png")); p.close()
    # 3. ba chi tiết in mới
    p = pv.Plotter(off_screen=True, window_size=(1800, 700), shape=(1, 3))
    for k, (i, t) in enumerate(((15, "AX-3_1_rut_ngan_khong_xoay"), (17, "AX-3_3_17PM"), (26, "AX-4_6_truc_5mm"))):
        p.subplot(0, k); p.add_mesh(pv.read(os.path.join(OUT, t + ".stl")), color=color(parts[i]["name"]), show_edges=False)
        p.add_text(t, font_size=12, color="black"); p.view_isometric()
    p.set_background("white"); p.screenshot(os.path.join(IMG, "chi_tiet_in_moi.png")); p.close()
    P(f"Ảnh: {IMG}")


if __name__ == "__main__":
    steps = sys.argv[1:] or ["dung", "kiem", "momen", "anh"]
    for name, fn in (("dung", build), ("kiem", check), ("momen", torque), ("anh", images)):
        if name in steps:
            fn()
            open(os.path.join(TMP, f"log_{name}.txt"), "w", encoding="utf-8").write("\n".join(log) + "\n"); log.clear()
    os.makedirs(OUT, exist_ok=True)
    txt = "".join(open(os.path.join(TMP, f"log_{n}.txt"), encoding="utf-8").read() for n in ("dung", "kiem", "momen") if os.path.exists(os.path.join(TMP, f"log_{n}.txt")))
    open(os.path.join(OUT, "ket_qua_kiem_tra.txt"), "w", encoding="utf-8").write(txt)
