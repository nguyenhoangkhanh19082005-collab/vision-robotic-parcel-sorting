r"""VỎ J2 IN LẠI — dời 2 trụ căng đai ra GIỮA nhánh đai (f ≈ 0.52) để dùng rãnh trượt chữ U giống J3 (20/11/2026).

Lý do: trụ căng bản cũ (f = 0.28, cách điểm đai rời puly lớn 19.6 mm) quá sát puly 55 mm → gờ của rãnh trượt đâm vào
vành puly Ø55. Ở f ≈ 0.52 (≈ 37 mm) rãnh trượt cách vành ≥ 4 mm kể cả khi ép 10 mm.
Thiết kế (giữ đúng kiểu trụ căng bản cũ người dùng đã quen):
  - Trụ Ø11 mọc từ thành vỏ về phía nhánh đai, trục vít M4 nằm ở w = −4 (cao độ lỗ cũ → dùng lại rãnh trượt lệch tâm 2/3 mm).
  - Vít M4 luồn qua KHE giữa trụ vít nắp (Ø12, có hốc đai ốc M3) và trụ vít ghép vỏ; nghiêng vài độ so với pháp tuyến đai
    (chọn hướng có khe rộng nhất). Lỗ Ø4.4 xuyên trụ + thành; hốc lục giác M4 (7.3, sâu 3.5) ở mặt ngoài thành.
  - Chân đỡ đặc từ trụ xuống đáy vỏ (in không cần support).
  - Vỏ nền = AX-1_3 gốc (KHÔNG có 2 trụ căng cũ).
Kiểm tra (gọn): khối hợp lệ + 1 solid, lưới STL kín, phần thêm không chạm chi tiết khác, rãnh trượt (e = 2 và 3 mm) ép
0/5/10 mm và thân vít không chạm gì, không che hốc đai ốc M3 của trụ vít nắp.
    E:\UDMT\.venv\Scripts\python.exe -X utf8 src\cad\tao_vo_J2_cang_dai_giua_nhanh.py
"""
import os, sys, shutil
import numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.dirname(__file__)); sys.path.insert(0, os.path.join(ROOT, "research"))
import rut_ngan_lib as L
from cad_util import cylinders, local_frame, faces
from tao_chi_tiet_dong_co_1_27 import write_step, collide
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakePrism
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform, BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_IN, TopAbs_VERTEX
from OCP.TopExp import TopExp_Explorer
from OCP.BRep import BRep_Tool
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Ax2, gp_Ax3, gp_Dir, gp_Lin, gp_Pnt, gp_Trsf, gp_Vec

OUT = os.path.join(ROOT, "assets", "CAD_vo_J2_cang_dai_giua_nhanh")
PRINT = os.path.join(ROOT, "assets", "Gui_xuong_in", "G_vo_J2_cang_dai_giua_nhanh")
NAME = "AX-1_3_cang_dai_giua_nhanh_J2"
WAX = -4.0              # trục vít (= cao độ lỗ cũ; rãnh trượt lệch tâm e đưa đai lên đúng chỗ)
RB_ = 5.5               # bán kính trụ Ø11
LEG_HW = 4.8            # nửa bề rộng chân đỡ (< RB_: tránh mặt bên chân TIẾP TUYẾN mặt trụ → phép ghép OCC lỗi)
Z0 = 9.0                # mặt đầu trụ cách lưng đai (rãnh trượt cao 6.8 → còn 2.2 mm khi chưa ép)
BORE_D, HEX_AF, HEX_DEPTH = 4.4, 7.3, 3.5
RF, RS_FL = 27.5, 8.0   # bán kính vành puly lớn / puly nhỏ
rB, rS = 25.25 + 1.4, 6.0 + 1.4
LOG = []


def P(*a):
    s = " ".join(str(x) for x in a); print(s); LOG.append(s)


def verts(s):
    e = TopExp_Explorer(s, TopAbs_VERTEX); out = []
    while e.More():
        p = BRep_Tool.Pnt_s(TopoDS.Vertex(e.Current())); out.append((p.X(), p.Y(), p.Z())); e.Next()
    return np.array(out)


def frame_shape(shape, origin, ex, ez):
    t = gp_Trsf(); t.SetTransformation(gp_Ax3(gp_Pnt(*origin), gp_Dir(*ez), gp_Dir(*ex)), gp_Ax3())
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def hex_prism(af, z0, z1):
    r = af / np.sqrt(3); poly = BRepBuilderAPI_MakePolygon()
    for k in range(6):
        a = np.radians(60 * k + 30); poly.Add(gp_Pnt(r * np.cos(a), r * np.sin(a), z0))
    poly.Close()
    return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(poly.Wire()).Face(), gp_Vec(0, 0, z1 - z0)).Shape()


def box(x0, y0, z0, x1, y1, z1): return BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), x1 - x0, y1 - y0, z1 - z0).Shape()
def zcyl(r, z0, z1, x=0.0, y=0.0): return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, y, z0), gp_Dir(0, 0, 1)), r, z1 - z0).Shape()


def shoe(e):
    """Rãnh trượt chữ U (tao_ranh_truot_cang_dai.py): x dọc đai, y lệch tâm (+ = phía miệng vỏ), z hướng vào đai, đế z = 0."""
    CH, LIP_T, LIP_H, Ls, H0, CROWN, RC, SD, SDEP, SW = 7.0, 1.6, 3.0, 16.0, 5.0, 1.8, 20.0, 4.5, 5.5, 2.2
    yc = e; y_lo = min(yc - CH / 2 - LIP_T, -(SD / 2 + SW)); y_hi = yc + CH / 2 + LIP_T; top = H0 + CROWN; zc = top - RC
    ycyl = lambda r: BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, y_lo - 1, zc), gp_Dir(0, 1, 0)), r, y_hi - y_lo + 2).Shape()
    b = BRepAlgoAPI_Common(box(-Ls / 2, y_lo, 0, Ls / 2, y_hi, top), ycyl(RC)).Shape()
    b = L.fuse(b, box(-Ls / 2, y_lo, 0, Ls / 2, y_hi, H0 - 0.5))
    for a, bb in ((y_lo, yc - CH / 2), (yc + CH / 2, y_hi)):
        b = L.fuse(b, BRepAlgoAPI_Common(box(-Ls / 2, a, 0, Ls / 2, bb, top + LIP_H), ycyl(RC + LIP_H)).Shape())
    return L.cut(b, zcyl(SD / 2, -0.1, SDEP))


# ---------------- hệ trục puly lớn J2 (bản lắp 1:27) ----------------
Bp = L.loadi(190); c = max(cylinders(Bp), key=lambda c: c[0]); R, o = local_frame(c[1], c[2] / np.linalg.norm(c[2]))
cs = max(cylinders(L.loadi(184)), key=lambda c: c[3]); C2 = ((np.asarray(cs[1]) - o) @ R.T)[:2]
to3 = lambda uv, w=WAX: o + R[0] * uv[0] + R[1] * uv[1] + R[2] * w
v3 = lambda d2: R[0] * d2[0] + R[1] * d2[1]
H = L.loadi(3)
INT = IntCurvesFace_ShapeIntersector(); INT.Load(H, 1e-4)


def hits(p, d, tmax=80.0):
    INT.Perform(gp_Lin(gp_Pnt(*p), gp_Dir(*d)), 0.0, tmax); return sorted(INT.WParameter(i) for i in range(1, INT.NbPnt() + 1))


def inside(p): return BRepClass3d_SolidClassifier(H, gp_Pnt(*p), 1e-4).State() == TopAbs_IN


def ok_fuse(s, t):
    """Ghép t vào s; chỉ nhận nếu ra đúng 1 solid, hợp lệ (BRepCheck) và thể tích không giảm."""
    r = L.fuse(s, t)
    return r if (L.n_solids(r) == 1 and BRepCheck_Analyzer(r).IsValid() and L.vol(r) >= L.vol(s) - 1e-3) else None


P(f"vỏ gốc AX-1_3: {L.vol(H) / 1000:.2f} cm3 (trùng bản cad_cache 98.83 cm3), hợp lệ {BRepCheck_Analyzer(H).IsValid()}")
P(f"puly lớn ở gốc, puly nhỏ tâm {np.round(C2, 1)}, khoảng cách tâm {np.linalg.norm(C2):.1f} mm")
# chi tiết lân cận (trừ vỏ); puly nhỏ dời +3 mm theo trục (lắp thực tế: đế puly bằng mặt con thoi)
near = {}
for i, q in enumerate(L.parts):
    cc = np.array(q["com"]) - o; w_ = cc @ R[2]; r_ = np.linalg.norm(cc - w_ * R[2])
    if r_ < 120 and -95 < w_ < 45 and i != 3:
        s = L.loadi(i)
        if i == 184:
            t = gp_Trsf(); t.SetTranslation(gp_Vec(*(R[2] * 3.0))); s = BRepBuilderAPI_Transform(s, t, True).Shape()
        near[i] = s
# hốc đai ốc M3 của trụ vít nắp (lỗ r1.75 ở w −1..4.8, trụ r6 ở w −5..4.8): vùng nhét đai ốc từ dưới lên phải để trống
nut_cols = []
acc = {}
for fc in faces(H):
    sf = BRepAdaptor_Surface(fc)
    if sf.GetType() != GeomAbs_Cylinder:
        continue
    cy = sf.Cylinder(); d_ = cy.Axis().Direction(); d_ = np.array([d_.X(), d_.Y(), d_.Z()])
    if abs(abs(d_ @ R[2]) - 1) > 1e-3:
        continue
    p_ = cy.Axis().Location(); c_ = ((np.array([p_.X(), p_.Y(), p_.Z()]) - o) @ R.T)[:2]; wv = (verts(fc) - o) @ R[2]
    k = (round(c_[0], 1), round(c_[1], 1), round(cy.Radius(), 2)); a_ = acc.setdefault(k, [c_, cy.Radius(), wv.min(), wv.max()])
    a_[2] = min(a_[2], wv.min()); a_[3] = max(a_[3], wv.max())
for c_, r_, w0, w1 in acc.values():
    if abs(r_ - 6.0) < 0.01 and w0 > -6 and w1 < 6 and np.linalg.norm(c_) > 40:
        nut_cols.append(c_)
P("trụ vít nắp (có hốc đai ốc M3) gần nhánh đai:", [tuple(np.round(c_, 1)) for c_ in nut_cols])

# ---------------- chọn vị trí + hướng vít cho từng nhánh ----------------
d = np.linalg.norm(C2); e = C2 / d
placed = []
for sg in (+1, -1):
    a = np.arccos((rB - rS) / d); n = np.array([[np.cos(sg * a), -np.sin(sg * a)], [np.sin(sg * a), np.cos(sg * a)]]) @ e
    T1, T2 = rB * n, C2 + rS * n; Lb = np.linalg.norm(T2 - T1); tb = (T2 - T1) / Lb
    best = None
    for f in np.arange(0.50, 0.5451, 0.005):
        q = T1 + f * (T2 - T1)
        for tilt in np.arange(-15, 15.01, 0.5):
            th = np.radians(tilt); out = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]) @ n; sd = -out
            pts = np.array([q + p_ * sd + s_ * sd + t_ * tb for p_ in (0, 5, 10) for s_ in np.linspace(-6.8, 3.0, 8) for t_ in np.linspace(-8, 8, 9)])
            cb = np.min(np.linalg.norm(pts, axis=1)) - RF; csm = np.min(np.linalg.norm(pts - C2, axis=1)) - RS_FL
            if cb < 4.0 or csm < 4.0:
                continue
            side = np.array([-out[1], out[0]])
            h0 = [x for x in hits(to3(q), v3(out)) if x > 8.8]
            if len(h0) != 2 or h0[1] - h0[0] < 2.5:
                continue
            margin = 0.0
            for rr in (3.0, 3.4, 3.8, 4.2, 4.6, 5.0):
                ok = all(len([x for x in hits(to3(q + rr * np.cos(ang) * side, WAX + rr * np.sin(ang)), v3(out)) if x > 8.8]) == 2
                         for ang in np.radians(np.arange(0, 360, 45)))
                if not ok:
                    break
                margin = rr
            if margin < 3.4:
                continue
            key = (margin, -abs(f - 0.52), -abs(tilt))
            if best is None or key > best[0]:
                best = (key, dict(sg=sg, f=f, tilt=tilt, q=q, out=out, sd=sd, side=side, tin=h0[0], tout=h0[1], margin=margin,
                                  cb=cb, cs=csm, dist=f * Lb, T1=T1, T2=T2, tb=tb))
    assert best, f"nhánh {sg}: không tìm được hướng vít"
    placed.append(best[1])
    b = best[1]
    P(f"nhánh {sg:+d}: f = {b['f']:.3f} (cách điểm đai rời puly lớn {b['dist']:.1f} mm), vít nghiêng {b['tilt']:+.1f}° so với pháp tuyến đai, "
      f"khe quanh trục vít ≥ {b['margin']:.1f} mm (lỗ Ø4.4 → nhựa quanh lỗ ≥ {b['margin'] - BORE_D / 2:.1f} mm), "
      f"lưng đai → mặt trong thành {b['tin']:.1f} mm, mặt ngoài {b['tout']:.1f} mm; rãnh trượt (ép 0–10) cách vành puly lớn "
      f"{b['cb']:.1f} mm, puly nhỏ {b['cs']:.1f} mm")

# ---------------- dựng trụ + chân đỡ ----------------
Hn = H; added_all = []
for b in placed:
    ez = v3(b["out"]); ex = v3(b["side"]); ey = np.cross(ez, ex)
    if ey @ R[2] < 0:
        ex = -ex; ey = -ey
    base = to3(b["q"])
    loc = lambda x, y, z: base + ex * x + ey * y + ez * z
    # mặt cuối trụ: lùi 0.4 mm trước MẶT NGOÀI CÙNG thấp nhất trên đĩa Ø11 → không lòi ra ngoài vỏ (thành xiên + hốc giữa
    # trụ vít và thành nên không chôn phẳng được; phần đĩa nằm trong hốc là mặt trong, không phải mặt ngoài)
    smp = [(0, 0)] + [(rr * np.cos(t), rr * np.sin(t)) for rr in (3.0, 4.5, 5.5) for t in np.radians(np.arange(0, 360, 22.5))]
    last = []
    for x, y in smp:
        hh = [h for h in hits(loc(x, y, 0.0), ez, 80.0) if h > Z0]
        if hh:
            last.append(hh[-1])
    # Thành xiên giao gờ bo trụ vít: mặt cuối trụ nằm trong khoảng ~24.8–26 mm làm phép ghép OCC hỏng (0 solid / khối lỗi)
    # → dò từ vị trí xa nhất lùi dần 0.1 mm tới khi ghép ra 1 khối hợp lệ.
    z_end = None
    for z in np.arange(min(last) - 0.4, b["tin"] - 4.0, -0.1):
        if ok_fuse(H, frame_shape(zcyl(RB_, Z0, z), base, ex, ez)) is not None:
            z_end = z; break
    assert z_end, "không tìm được mặt cuối trụ ghép hợp lệ"
    # GÂN VUÔNG 7.2×7.2 quanh lỗ vít nối mặt cuối trụ vào thành (vít được đỡ liền tới thành): đầu gân chôn vào thành
    # ≥ 0.3 mm ở tâm, lùi ≥ 0.4 mm trước mặt ngoài tại 4 góc + 4 cạnh; dò độ dài ghép hợp lệ
    bsmp = [(sx * 3.6, sy * 3.6) for sx in (-1, 0, 1) for sy in (-1, 0, 1)]
    blast = []
    for x, y in bsmp:
        hh = [h for h in hits(loc(x, y, 0.0), ez, 80.0) if h > Z0]
        if hh:
            blast.append(hh[-1])
    z_br = None
    if b["tin"] + 0.3 > z_end:
        for z in np.arange(min(min(blast) - 0.4, b["tin"] + 1.5), b["tin"] + 0.3, -0.1):
            if ok_fuse(H, frame_shape(box(-3.6, -3.6, z_end - 0.5, 3.6, 3.6, z), base, ex, ez)) is not None:
                z_br = z; break
    # chân đỡ dài tới mặt trong gần nhất ở các tầng dưới trục (+0.5 ăn vào thành / trụ vít)
    zl = []
    for y in (-1.0, -4.0, -8.0, -12.0):
        for x in (-LEG_HW, 0.0, LEG_HW):
            hh = [h for h in hits(loc(x, y, 0.0), ez, 80.0) if h > Z0]
            if hh:
                zl.append(hh[0])
    zl1 = min(min(zl) + 0.5, z_end)
    # đáy vỏ ngay dưới chân đỡ (lưới 3 × 5 điểm trong phạm vi chân): mặt trên / mặt dưới đáy
    tops, bots = [], []
    for x in (-LEG_HW + 0.3, 0.0, LEG_HW - 0.3):
        for z in np.linspace(Z0 + 0.3, zl1 - 0.8, 5):
            hh = hits(loc(x, -RB_ - 0.5, z), -R[2], 60.0)
            if len(hh) >= 2:
                tops.append(WAX - RB_ - 0.5 - hh[0]); bots.append(WAX - RB_ - 0.5 - hh[1])
    ybot = min(tops) - 0.3                       # ăn vào đáy ≥ 0.3 mm ở mọi điểm
    if ybot < max(bots) + 0.8:                   # không được xuyên thủng mặt dưới đáy
        ybot = max(bots) + 0.8
    ybot -= WAX
    pieces = [("chân đỡ", frame_shape(box(-LEG_HW, ybot, Z0, LEG_HW, 0.0, zl1), base, ex, ez)),
              ("trụ Ø11", frame_shape(zcyl(RB_, Z0, z_end), base, ex, ez))]
    if z_br:
        pieces.append(("gân nối thành", frame_shape(box(-3.6, -3.6, z_end - 0.5, 3.6, 3.6, z_br), base, ex, ez)))
    boss = pieces[0][1]
    for _n, pc in pieces[1:]:
        boss = L.fuse(boss, pc)
    # lỗ vít xuyên trụ + thành; KHE THẢ ĐAI ỐC M4 từ trên xuống (miệng vỏ): lục giác đỉnh dọc ±w, 2 cạnh phẳng ±x → đai ốc
    # trượt xuống giữa 2 cạnh phẳng rồi nằm khít đáy chữ V; khe ở z = ZN..ZN+3.5 (bên trong trụ, cách mặt đầu 3 mm)
    ZN = Z0 + 3.0
    bore = frame_shape(zcyl(BORE_D / 2, Z0 - 1.0, max(last) + 6.0), base, ex, ez)
    hexp = frame_shape(L.fuse(hex_prism(HEX_AF, ZN, ZN + HEX_DEPTH), box(-HEX_AF / 2, 0.0, ZN, HEX_AF / 2, RB_ + 12.0, ZN + HEX_DEPTH)),
                       base, ex, ez)
    v_cut_old = L.vol(L.common(hexp, H))
    up = hits(loc(0.0, RB_ + 0.1, ZN + HEX_DEPTH / 2), R[2], 80.0)
    b.update(slot_cut_old=v_cut_old, slot_free_up=(up[0] if up else None))
    b.update(boss=boss, bore=bore, hexp=hexp)
    added_all.append(L.cut(L.cut(boss, bore), hexp))
    b.update(z_end=z_end, z_br=z_br, ybot=ybot, zl1=zl1, ex=ex, ey=ey, ez=ez, base=base, pieces=pieces, blast=blast)
    P(f"  trụ nhánh {b['sg']:+d}: mặt cuối trụ Ø11 ở {z_end:.1f} mm (mặt trong thành tại trục vít {b['tin']:.1f}); "
      + (f"gân vuông 7.2 mm nối vào thành tới {z_br:.1f} mm" if z_br else "trụ đã chạm thành, không cần gân"))
    P(f"  khe thả đai ốc M4: cách lưng đai {ZN:.1f}–{ZN + HEX_DEPTH:.1f} mm; ăn vào vỏ gốc {b['slot_cut_old']:.1f} mm3; "
      f"phía trên khe trống {('%.1f mm' % b['slot_free_up']) if b['slot_free_up'] else 'tới hết (không vướng gì)'}")
    P(f"  trụ nhánh {b['sg']:+d}: Ø11 từ {Z0:.1f} → {z_end:.1f} mm (cách mặt ngoài thành ≥ 0.4 mm), chân đỡ xuống đáy "
      f"(w = {WAX + ybot:.1f}, đáy dày {min(np.array(tops) - np.array(bots)):.1f}–{max(np.array(tops) - np.array(bots)):.1f} mm), "
      f"chân dài tới {zl1:.1f} mm")
added = L.fuse(*added_all) if len(added_all) == 2 else added_all[0]


from OCP.BRepTools import BRepTools


def assemble(zb_of):
    """Ghép chân → trụ → gân (độ dài gân zb_of(b)), khoét lỗ + khe; trả về khối nếu hợp lệ VÀ lưới STL kín."""
    s = H
    for b in placed:
        pcs = list(b["pieces"][:2])
        zb = zb_of(b)
        if zb is not None:
            pcs.append(("gân nối thành", frame_shape(box(-3.6, -3.6, b["z_end"] - 0.5, 3.6, 3.6, zb), b["base"], b["ex"], b["ez"])))
        for nm, pc in pcs:
            s = ok_fuse(s, pc)
            if s is None:
                return None, None
        b["pcs_used"] = pcs; b["zb_used"] = zb
    for b in placed:
        s = L.cut(L.cut(s, b["bore"]), b["hexp"])
    if not (L.n_solids(s) == 1 and BRepCheck_Analyzer(s).IsValid()):
        return None, None
    p = os.path.join(ROOT, "..", ".tmp", "dai", "kt_luoi.stl"); BRepTools.Clean_s(s); BRepMesh_IncrementalMesh(s, 0.05, False, 0.3, True)
    w = StlAPI_Writer(); w.ASCIIMode = False; w.Write(s, p)
    return s, L.mesh_check(p)[1]


# Đầu gân nằm trong thành ngay sau mặt trong để lại mặt ngăn ẩn → lỗ vít cắt qua sinh cạnh không đa tạp (lưới hở).
# Thử gân dài dần; chọn bản ngắn nhất có lưới kín (đầu gân có thể nhô ra ngoài mặt thành xiên một chút ở góc thấp).
Hn = None
for lab, rule in (("không nhô", lambda b: b["z_br"]), ("tin+1.4", lambda b: b["tin"] + 1.4), ("tin+1.6", lambda b: b["tin"] + 1.6),
                  ("tin+1.8", lambda b: b["tin"] + 1.8), ("tin+2.2", lambda b: b["tin"] + 2.2), ("sát mặt ngoài", lambda b: b["tout"] - 0.6)):
    s, nop = assemble(rule)
    P(f"  gân {lab}: " + ("ghép lỗi" if s is None else f"cạnh hở lưới {nop}"))
    if s is not None and nop == 0:
        Hn = s
        for b in placed:
            P(f"  → nhánh {b['sg']:+d}: gân tới {b['zb_used']:.1f} mm; đầu gân nhô khỏi mặt ngoài thành tối đa "
              f"{max(0.0, b['zb_used'] - min(b['blast'])):.1f} mm (ở góc phía thành thấp)")
        break
assert Hn is not None, "không có phương án gân nào cho lưới kín"
added_all = []
for b in placed:
    bs = b["pcs_used"][0][1]
    for _n, pc in b["pcs_used"][1:]:
        bs = L.fuse(bs, pc)
    added_all.append(L.cut(L.cut(bs, b["bore"]), b["hexp"]))
added = L.fuse(*added_all) if len(added_all) == 2 else added_all[0]

# ---------------- kiểm tra ----------------
P("\n=== KIỂM TRA ===")
P(f"vỏ mới: hợp lệ {BRepCheck_Analyzer(Hn).IsValid()}, số solid {L.n_solids(Hn)}, thể tích {L.vol(H) / 1000:.2f} → {L.vol(Hn) / 1000:.2f} cm3")
bad = [(L.parts[i]["name"][:28], round(collide(added, s), 2)) for i, s in near.items() if collide(added, s) > 0.05]
P("phần thêm chạm chi tiết khác:", bad or "KHÔNG")
# hốc đai ốc M3 trụ vít nắp: vùng Ø7.5 dưới trụ (từ đáy lên w = −5) phải trống
blk = []
for c_ in nut_cols:
    pr = frame_shape(zcyl(3.75, -25.0, -5.0, *c_), o, R[0], R[2])
    v = collide(added, pr)
    if v > 0.05:
        blk.append((tuple(np.round(c_, 1)), round(v, 1)))
P("che lối nhét đai ốc M3 của trụ vít nắp:", blk or "KHÔNG")
# rãnh trượt e = 2 và 3 mm, ép 0 / 5 / 10 mm; thân vít M4 (Ø4) từ đầu vít trong lỗ rãnh trượt ra ngoài vỏ 15 mm
for b in placed:
    for e_ in (2.0, 3.0):
        Sh = shoe(e_)
        for p_ in (0.0, 5.0, 10.0):
            org = b["base"] + b["ez"] * (6.8 - p_)               # đế rãnh trượt cách lưng đai 6.8 mm, ép vào p mm
            tb3 = v3(b["tb"]); xax = tb3 - (tb3 @ b["ez"]) * b["ez"]; xax /= np.linalg.norm(xax)
            if np.cross(-b["ez"], xax) @ R[2] < 0:             # y rãnh trượt (= z × x) phải hướng lên miệng vỏ (+w)
                xax = -xax
            sh = frame_shape(Sh, org, xax, -b["ez"])            # z rãnh trượt hướng vào đai
            hit = [(L.parts[i]["name"][:22], round(collide(sh, s), 1)) for i, s in near.items() if collide(sh, s) > 0.05]
            vh = collide(sh, Hn)
            dmin = BRepExtrema_DistShapeShape(sh, near[190]); dmin.Perform()
            P(f"  nhánh {b['sg']:+d} rãnh trượt e={e_:g} ép {p_:4.1f}: chạm {hit or 'không'}; chạm vỏ {vh:.1f} mm3; cách puly lớn {dmin.Value():.1f} mm")
        rod = frame_shape(zcyl(2.0, 6.8 - 5.5 - 10.0, b["tout"] + 15.0), b["base"], b["ex"], b["ez"])
    hit = [(L.parts[i]["name"][:22], round(collide(rod, s), 1)) for i, s in near.items() if collide(rod, s) > 0.05]
    P(f"  nhánh {b['sg']:+d} thân vít (ép 0→10, thò ra ngoài 15 mm): chạm {hit or 'không'}")
    P(f"  nhánh {b['sg']:+d} vít cần: chạm lưng đai ↔ ép 10 mm = M4 dài {b['tout'] - 1.3:.0f} ↔ {b['tout'] - 1.3 + 10:.0f} mm "
      f"(tính từ mặt ngoài vỏ, đầu vít nằm ngoài) → dùng M4×40")

# ---------------- xuất file ----------------
os.makedirs(OUT, exist_ok=True); os.makedirs(PRINT, exist_ok=True)
write_step([(NAME, Hn)], os.path.join(OUT, NAME + ".step"))
from OCP.BRepTools import BRepTools
stl = os.path.join(OUT, NAME + ".stl")
for defl, ang in ((0.05, 0.3), (0.03, 0.2), (0.08, 0.4), (0.02, 0.15), (0.1, 0.5)):   # chọn bộ tham số cho lưới KÍN
    BRepTools.Clean_s(Hn); BRepMesh_IncrementalMesh(Hn, defl, False, ang, True)
    w = StlAPI_Writer(); w.ASCIIMode = False; assert w.Write(Hn, stl)
    nt, nop, man, vv = L.mesh_check(stl)
    P(f"STL (lệch {defl} mm, góc {ang}): {nt} tam giác, cạnh hở {nop}, đa tạp {man}, {vv:.2f} cm3")
    if nop == 0:
        break
P(f"→ lưới kín: {nop == 0}")
shutil.copy(stl, PRINT)
open(os.path.join(OUT, "ket_qua_kiem_tra.txt"), "w", encoding="utf-8").write("\n".join(LOG) + "\n")
print("xong:", OUT)
