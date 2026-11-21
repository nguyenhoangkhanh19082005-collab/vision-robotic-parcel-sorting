r"""Rút ngắn cẳng tay (trục J3 → tâm cổ tay 267,0 → 235,5 mm) bằng cách bỏ khoảng trống, GIỮ NGUYÊN mọi linh kiện cơ khí
(động cơ, hộp số, ổ bi, puly, đai, vít) và kiểu dáng các chi tiết in. Chỉ 3 chi tiết nhựa phải in lại:

  AX-3_2 (hộp trục J3)   : bỏ lát cổ hộp s 36..42 (6 mm) + vát gờ chặn ổ bi 1 mm ở chỗ thân động cơ J4 đi qua.
  AX-3_1 (ống động cơ J4): bỏ lát ống trơn s 75..84,5 (9,5 mm) và XOAY nửa ống phía xa 45° quanh trục. Tấm gá AX-3_3,
                           hai kẹp AX-3_4 và động cơ J4 xoay theo → thân vuông của động cơ nằm THẲNG (cạnh song song
                           thành hộp) thay vì nằm chéo. Nằm chéo thì góc động cơ vướng ổ bi trục J3 nên không lùi được.
  AX-4_4 (bầu cẳng tay)  : dựng lại đoạn bầu rỗng s 154,5..187 (32,5 mm) thành 16,5 mm; thành dày hơn bản gốc.

"s" = khoảng cách dọc trục J4 tính từ trục J3 (mm). Kết quả: assets/CAD_rut_ngan/ (STEP từng chi tiết ở vị trí lắp ráp,
STL nằm dọc trục Z để in, file lắp ráp Roboter_rut_ngan.step).
    E:\UDMT\.venv\Scripts\python.exe src\cad\rut_ngan_xuat_brep.py      (một lần, tạo .tmp/brep_1_27)
    E:\UDMT\.venv\Scripts\python.exe src\cad\rut_ngan_cang_tay.py
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from rut_ngan_lib import *  # noqa: F401,F403
from tao_chi_tiet_dong_co_1_27 import write_step, write_stl
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire
from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCP.GC import GC_MakeArcOfCircle
from OCP.BRepTools import BRepTools

OUT = os.path.join(ROOT, "assets", "CAD_rut_ngan")
CUT_HUB = (36.0, 42.0)          # AX-3_2: lát cổ hộp (tiết diện không đổi, trước hốc đai ốc M3 ở s 42,7..45)
CUT_TUBE = (75.0, 84.5)         # AX-3_1: lát ống trơn Ø65/Ø60 (giữa chân loe s<74,5 và kẹp AX-3_4 s 85..105)
TUBE_TWIST = 45.0               # AX-3_1: xoay nửa ống phía xa (độ, quanh trục J4) → động cơ J4 nằm thẳng
RELIEF = dict(x0=-22.0, x1=-20.5, y0=-23.0, y1=23.0, s0=11.0, s1=37.0)   # vát gờ chặn ổ bi phía −X của AX-3_2
FUN_S0, FUN_B, FUN_NEW = 154.5, 187.0, 16.5   # AX-4_4: đoạn bầu cũ s 154,5..187 → dài 16,5 mm
FUN_PROX_CUT = 155.0            # giữ nguyên phần gốc tới s = 155 (cổ tròn sau vách ngăn); đoạn mới lồng vào 0,5 mm
FUN_TAIL = 0.8                  # đoạn mới lồng vào phần xa 0,8 mm
FUN_PROUD, FUN_INSIDE = 0.3, 0.5   # đoạn mới dày hơn bản gốc: 0,3 mm ra ngoài, 0,5 mm vào trong (hai khối lồng nhau, hàn chắc)
FUN_WALL_EXTRA = 1.5            # thành trong lùi thêm theo phương X (mm) để bề dày vuông góc ≥ 2,5 mm ở chỗ dốc ~57°
EASE_U0 = 0.12
D_HUB = CUT_HUB[1] - CUT_HUB[0]; D_TUBE = CUT_TUBE[1] - CUT_TUBE[0]; D_FUN = (FUN_B - FUN_S0) - FUN_NEW
SHIFT_1, SHIFT_2, SHIFT_3 = D_HUB, D_HUB + D_TUBE, D_HUB + D_TUBE + D_FUN    # 6 / 15,5 / 31,5 mm
TWIST_PARTS = (17, 18, 19, 33, 34, 35, 36, 130, 131, 132, 133)   # tấm gá, 2 kẹp, 4 vít + 4 đai ốc kẹp: xoay theo nửa ống
NEW_NAMES = {15: "AX-3_1_rut_ngan", 16: "AX-3_2_rut_ngan", 24: "AX-4_4_rut_ngan"}
log = []


def P(*a):
    s = " ".join(str(x) for x in a); log.append(s); print(s, flush=True)


def describe(name, old, new, expect=None):
    """In thể tích, kiểm tra 1 khối. expect (mm3): thể tích phải có — đối chiếu bằng LƯỚI tam giác kín, vì thể tích tính từ
    mặt CAD của chi tiết có dung sai lớn trong file gốc (AX-4_4) lệch ~0,7 %."""
    tmp = os.path.join(ROOT, ".tmp", f"rn_check_{name}.stl"); write_stl(new, tmp)
    nt, nopen, man, vm = mesh_check(tmp)
    P(f"  {name}: thể tích {vol(old)/1000:.1f} → {vm:.1f} cm3 (lưới kín: cạnh hở {nopen}, đa tạp {man}), số khối {n_solids(new)}, "
      f"BRepCheck gốc {valid(old)} → mới {valid(new)}")
    assert n_solids(new) == 1 and nopen == 0 and man and (valid(new) or not valid(old)), name
    if expect is not None:
        assert abs(vm * 1000 - expect) < 0.005 * expect, (name, vm, expect)


# ---------------------------------------------------------------- AX-3_2
def make_ax32():
    old = loadi(16)
    relief = lbox(R3, RELIEF["x0"], RELIEF["x1"], RELIEF["y0"], RELIEF["y1"], RELIEF["s0"], RELIEF["s1"])
    removed = vol(common(old, relief))
    new = slab_cut(cut(old, relief), *CUT_HUB)
    P(f"AX-3_2: bỏ lát s {CUT_HUB[0]}..{CUT_HUB[1]} ({D_HUB} mm); vát gờ ổ bi phía −X về x = {RELIEF['x0']} (bỏ {removed:.0f} mm3)")
    describe("AX-3_2", old, new)
    return new


# ---------------------------------------------------------------- AX-3_1
def make_ax31():
    old = loadi(15)
    prox = keep(old, -BIG, CUT_TUBE[0]); dist = rotated_about_f(shifted(keep(old, CUT_TUBE[1], BIG), D_TUBE), TUBE_TWIST)
    new = unify(fuse(prox, dist, 1e-4))
    P(f"AX-3_1: bỏ lát ống s {CUT_TUBE[0]}..{CUT_TUBE[1]} ({D_TUBE} mm), nửa xa xoay {TUBE_TWIST:+.0f}° quanh trục")
    describe("AX-3_1", old, new)
    return shifted(new, SHIFT_1)


# ---------------------------------------------------------------- động cơ J4
def motor_hole_angle():
    """Góc (độ, hệ khâu 3, 0..90) của lỗ ren M3 trên mặt hộp số PG5 trong mô hình CAD. Lỗ tấm gá AX-3_3 ở 0/90/180/270°."""
    for r, p, d, a in cylinders(loadi(191)):
        q = p - E
        if abs(d @ F) > 0.99 and abs(r - 1.5) < 0.01 and abs(np.hypot(q @ X3, q @ Y3) - 14.0) < 0.05:
            return float(np.degrees(np.arctan2(q @ Y3, q @ X3))) % 90.0
    raise RuntimeError("không thấy lỗ ren hộp số")


# ---------------------------------------------------------------- AX-4_4
def ease(u, u0=EASE_U0):
    """0→1, vào êm (dốc 0 ở u=0) rồi tuyến tính: cổ tròn chuyển tiếp trơn sang phần bầu mới."""
    return u * u / (2 * u0 * (1 - u0 / 2)) if u < u0 else (u - u0 / 2) / (1 - u0 / 2)


def fit_rr(loop, outer):
    """Chữ nhật bo góc (cx, hx, hy, rc) khớp một vòng mặt cắt. Góc của bản gốc không tròn hẳn (lệch ≤ 0,5 mm) nên
    chọn rc sao cho hình khớp BAO vòng ngoài (outer=True) hoặc NẰM TRONG vòng trong (outer=False)."""
    cx = (loop[:, 0].min() + loop[:, 0].max()) / 2; hx = np.ptp(loop[:, 0]) / 2; hy = np.ptp(loop[:, 1]) / 2
    top = loop[loop[:, 1] > loop[:, 1].max() - 2e-3]; side = loop[loop[:, 0] > loop[:, 0].max() - 2e-3]
    rcx = hx - np.ptp(top[:, 0]) / 2; rcy = hy - np.ptp(side[:, 1]) / 2
    return np.array([cx, hx, hy, min(rcx, rcy) if outer else max(rcx, rcy)])


def rr_points(q, n=64):
    """Điểm trên chữ nhật bo góc q = (cx, hx, hy, rc), ngược chiều kim đồng hồ."""
    cx, hx, hy, rc = q; pts = []
    for sx, sy, a0 in ((1, 1, 0), (-1, 1, 90), (-1, -1, 180), (1, -1, 270)):
        c = np.array([cx + sx * (hx - rc), sy * (hy - rc)])
        pts += [c + rc * np.array([np.cos(a), np.sin(a)]) for a in np.radians(np.linspace(a0, a0 + 90, n))]
    return np.array(pts)


def rr_wire(q, s):
    """Dây kín 4 đoạn thẳng + 4 cung tròn (hệ khâu 4) tại mặt phẳng s; mọi trạm cùng thứ tự cạnh để loft thẳng hàng."""
    cx, hx, hy, rc = q
    def pt(x, y): return gp_Pnt(*(E + X4 * x + Y4 * y + F * s))
    k = rc * (1 - np.sqrt(0.5)); mk = BRepBuilderAPI_MakeWire()
    corners = [((cx + hx, hy - rc), (cx + hx - k, hy - k), (cx + hx - rc, hy)), ((cx - hx + rc, hy), (cx - hx + k, hy - k), (cx - hx, hy - rc)),
               ((cx - hx, -hy + rc), (cx - hx + k, -hy + k), (cx - hx + rc, -hy)), ((cx + hx - rc, -hy), (cx + hx - k, -hy + k), (cx + hx, -hy + rc))]
    prev = corners[-1][2]
    for a, m, b in corners:
        mk.Add(BRepBuilderAPI_MakeEdge(pt(*prev), pt(*a)).Edge())
        mk.Add(BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(pt(*a), pt(*m), pt(*b)).Value()).Edge()); prev = b
    assert mk.IsDone()
    return mk.Wire()


def loft(stations):
    """Khối loft thẳng (ruled) qua các trạm [(q, s)]."""
    ts = BRepOffsetAPI_ThruSections(True, True, 1e-6); ts.CheckCompatibility(False)
    for q, s in stations: ts.AddWire(rr_wire(q, s))
    ts.Build(); assert ts.IsDone()
    return ts.Shape()


def make_ax44():
    old = loadi(24)

    def outline(s):
        l = sorted(loops(section_edges(old, s, R4), 0.2), key=lambda l: -np.ptp(l[:, 0])); assert len(l) >= 2, s
        return l

    ao = fit_rr(outline(155.0)[0], True); ai = fit_rr(outline(156.0)[1], False)     # cổ tròn sau vách ngăn: không đổi s 154..156
    bo = fit_rr(outline(FUN_B)[0], True); bi = fit_rr(outline(FUN_B)[1], False)
    co = fit_rr(outline(FUN_B + FUN_TAIL)[0], True); ci = fit_rr(outline(FUN_B + FUN_TAIL)[1], False)
    ao[0] = ai[0] = 0.0

    def station(u):
        """(biên ngoài, biên trong) của đoạn bầu mới tại u (0..1); u > 1: mặt cắt lồng vào phần xa."""
        if u <= 1.0:
            w = ease(u); qo = (1 - w) * ao + w * bo; qi = (1 - w) * ai + w * bi
        else:
            qo, qi = co, ci
        t = min(1.0, u / EASE_U0); e = FUN_WALL_EXTRA * t * t * (3 - 2 * t)        # bù bề dày thành hai bên ±X (đủ từ u = 0,12)
        return qo + [0, FUN_PROUD, FUN_PROUD, FUN_PROUD], qi - [0, FUN_INSIDE + e, FUN_INSIDE, FUN_INSIDE]

    us = [0.0, 0.03, 0.06, 0.09, EASE_U0, 1.0]                # sau u = 0,12 biên dạng đổi tuyến tính → một đoạn loft thẳng là đủ
    ss = [FUN_S0 + u * FUN_NEW for u in us] + [FUN_S0 + FUN_NEW + FUN_TAIL]
    st = [station(u) for u in us] + [station(2.0)]
    funnel = cut(loft([(o, s) for (o, i), s in zip(st, ss)]), loft([(i, s) for (o, i), s in zip(st, ss)]))
    assert valid(funnel) and n_solids(funnel) == 1
    P(f"        (file STEP gốc: dung sai đỉnh của AX-4_4 tới {max_tol(old):.1f} mm → siết về 0,001 mm trước khi hàn)")
    prox = limit_tol(keep(old, -BIG, FUN_PROX_CUT, R4)); dist = limit_tol(shifted(keep(old, FUN_B, BIG, R4), D_FUN))
    for nm_, sh in (("funnel", funnel), ("prox", prox), ("dist", dist)):       # cho rut_ngan_kiem_tra.py
        BRepTools.Write_s(sh, os.path.join(ROOT, ".tmp", f"rn_{nm_}.brep"))
    v1, v2 = vol(common(prox, funnel)), vol(common(dist, funnel))
    assert v1 > 100 and v2 > 100, (v1, v2)                    # hai đầu đoạn mới phải lồng vào phần gốc
    new = fuse(fuse(prox, funnel), dist)
    expect = vol(prox) + vol(funnel) + vol(dist) - v1 - v2
    # bề dày thành nhỏ nhất: khoảng cách từ điểm mặt trong tới mặt ngoài (lấy mẫu dày dọc trục)
    grid = np.vstack([np.column_stack([rr_points(station(u)[0]), np.full(256, FUN_S0 + u * FUN_NEW)]) for u in np.linspace(0, 1, 120)])
    thick = []
    for u in (0.0, 0.06, 0.12, 0.3, 0.5, 0.7, 0.9, 1.0):
        q = np.column_stack([rr_points(station(u)[1]), np.full(256, FUN_S0 + u * FUN_NEW)])
        thick.append((u, min(np.min(np.linalg.norm(grid - p, axis=1)) for p in q)))
    k = 1 / (1 - EASE_U0 / 2)
    slope = [np.degrees(np.arctan(k * abs((bo[0] + bo[1]) - (ao[0] + ao[1])) / FUN_NEW)), np.degrees(np.arctan(k * abs((bo[0] - bo[1]) - (ao[0] - ao[1])) / FUN_NEW))]
    P(f"AX-4_4: đoạn bầu s {FUN_S0}..{FUN_B} ({FUN_B - FUN_S0} mm) → {FUN_NEW} mm (ngắn {D_FUN} mm); "
      f"độ dốc thành so với trục: +X {slope[0]:.1f}°, −X {slope[1]:.1f}° (bản gốc ≈ 37°)")
    P("        bề dày thành nhỏ nhất dọc đoạn bầu mới (mm; bản gốc 2,5): " + ", ".join(f"u={u:.2f}:{d:.2f}" for u, d in thick))
    describe("AX-4_4", old, new, expect)
    return shifted(new, SHIFT_2)


# ---------------------------------------------------------------- lắp ráp
def chain_shift(i):
    """Độ dời (mm, về phía trục J3) của chi tiết i trong bản rút ngắn; None nếu là chi tiết dựng mới."""
    q = parts[i]; c = np.array(q["com"]) - E; s = c @ F; r = np.linalg.norm(c - s * F)
    if not ((np.array(q["com"]) - P2) @ (P3 - P2) / np.linalg.norm(P3 - P2) > 100 and r < 90): return 0.0
    if i in NEW_NAMES: return None
    if s < CUT_HUB[0]: return 0.0
    if s < 60: return SHIFT_1
    if s < 150: return SHIFT_2
    assert s > FUN_B, (i, q["name"], s)
    return SHIFT_3


def build():
    """→ ({số: (tên, hình mới)}, góc xoay động cơ J4). Chi tiết không đổi không có trong dict."""
    new = {15: make_ax31(), 16: make_ax32(), 24: make_ax44()}
    clock = (TUBE_TWIST - motor_hole_angle() + 45.0) % 90.0 - 45.0     # lỗ ren hộp số trùng lỗ tấm gá (đã xoay theo nửa ống)
    P(f"Động cơ J4: xoay {clock:+.1f}° quanh trục so với mô hình CAD (lỗ ren hộp số trùng lỗ tấm gá AX-3_3 đã xoay {TUBE_TWIST:+.0f}°) "
      f"→ thân vuông song song thành hộp")
    out = {i: (NEW_NAMES[i], s) for i, s in new.items()}
    for i, q in enumerate(parts):
        d = chain_shift(i)
        if d is None or d == 0.0: continue
        s = loadi(i)
        if i == 191: s = rotated_about_f(s, clock)
        if i in TWIST_PARTS: s = rotated_about_f(s, TUBE_TWIST)
        out[i] = (q["name"], shifted(s, d))
    return out, clock


def main():
    os.makedirs(OUT, exist_ok=True)
    L = np.linalg.norm(WRIST - E)
    P(f"Rút ngắn cẳng tay: {D_HUB} + {D_TUBE} + {D_FUN} = {SHIFT_3} mm; trục J3 → tâm cổ tay {L:.1f} → {L - SHIFT_3:.1f} mm")
    new, clock = build()
    for i, R in ((15, R3), (16, R3), (24, R4)):
        name, shape = new[i]
        write_step([(name, shape)], os.path.join(OUT, name + ".step"))
        write_stl(to_local(shape, R), os.path.join(OUT, name + ".stl"))
        nt, nopen, man, v = mesh_check(os.path.join(OUT, name + ".stl"))
        P(f"  {name}.stl: {nt} tam giác, cạnh hở {nopen}, đa tạp {man}, thể tích lưới {v:.1f} cm3 (khối CAD {vol(shape)/1000:.1f})")
    for i, (name, shape) in new.items():
        BRepTools.Write_s(shape, os.path.join(ROOT, ".tmp", f"rn_new_{i:03d}.brep"))
    asm = [new[i] if i in new else (q["name"], loadi(i)) for i, q in enumerate(parts)]
    write_step(asm, os.path.join(OUT, "Roboter_rut_ngan.step"))
    np.save(os.path.join(ROOT, ".tmp", "rn_shift.npy"), np.array([(-1.0 if chain_shift(i) is None else chain_shift(i)) for i in range(len(parts))]))
    P(f"Số chi tiết dời chỗ: {len(new) - len(NEW_NAMES)}; đã ghi {OUT}")
    open(os.path.join(OUT, "ket_qua_dung_hinh.txt"), "w", encoding="utf-8").write("\n".join(log) + "\n")


if __name__ == "__main__":
    main()
