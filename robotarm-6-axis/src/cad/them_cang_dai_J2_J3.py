r"""Thêm KHỐI TRỤ CĂNG ĐAI vào vỏ đai J2 (AX-1_3) và J3 (AX-2_1).

Người dùng đang chèn mẩu đũa (J2 ≈ 31–33 mm, J3 ≈ 35–37 mm) từ thành vỏ ép vào LƯNG đai (mặt trơn) để căng đai.
Thay bằng: khối trụ rỗng mọc từ thành vỏ về phía nhánh đai, trục vuông góc nhánh đai và nằm trên mặt phẳng giữa đai.
  - Lỗ xuyên Ø4.4 cho que Ø4 / vít M4 luồn từ NGOÀI vỏ vào, đẩy thẳng vào lưng đai, cố định bằng keo AB.
  - Hốc lục giác M4 (7.3 mm, sâu 3.5) ở mặt ngoài: tùy chọn dùng vít M4 + đai ốc để chỉnh được độ căng (keo AB khóa sau).
  - Đầu khối trụ cách lưng đai (lúc chưa ép) 3 mm, đủ cho đai rung; que thò ra khỏi khối trụ đè đai vào trong.
  - Khối trụ có chân đỡ đặc xuống đáy vỏ (tiết diện chữ D) → in không cần support, cứng hơn trụ treo.
Mỗi nhánh đai (2 nhánh/khớp) được tìm 1 vị trí: thành vỏ phẳng, không trúng trụ vít, gần giữa nhánh nhất.

Đầu ra: assets/CAD_cang_dai_J2_J3/ (STEP + STL từng vỏ, file lắp ráp tổng, ảnh mặt cắt, kết quả kiểm tra) và bản sao
STL vào assets/Gui_xuong_in/C_cang_dai_J2_J3/.
    E:\UDMT\.venv\Scripts\python.exe -X utf8 src\cad\them_cang_dai_J2_J3.py
"""
import glob, os, sys, shutil
import numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "research")); sys.path.insert(0, os.path.dirname(__file__))
from cad_util import CACHE, cylinders, load, local_frame
from tao_chi_tiet_dong_co_1_27 import write_step, write_stl, collide, vol
from OCP.BRep import BRep_Tool, BRep_Builder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse, BRepAlgoAPI_Section
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace, BRepBuilderAPI_Transform
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakePrism, BRepPrimAPI_MakeCylinder
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.GCPnts import GCPnts_UniformAbscissa
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDataStd import TDataStd_Name
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TopAbs import TopAbs_EDGE, TopAbs_VERTEX
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Ax2, gp_Ax3, gp_Dir, gp_Lin, gp_Pln, gp_Pnt, gp_Trsf, gp_Vec
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

OUT = os.path.join(ROOT, "assets", "CAD_cang_dai_J2_J3")
PRINT = os.path.join(ROOT, "assets", "Gui_xuong_in", "C_cang_dai_J2_J3")
ASM_IN = os.path.join(ROOT, "assets", "CAD_dong_co_J4_17PM", "Roboter_rut_ngan_17PM_giac_hut.step")

BORE_D = 4.4        # lỗ cho que Ø4 / vít M4 (+0.4 bù co lỗ khi in FDM)
BOSS_D = 11.0       # đường kính khối trụ (thành 3.3 mm)
HEX_AF = 7.3        # hốc đai ốc M4 (7.0 + 0.3)
HEX_DEPTH = 3.5
TIP_GAP = 3.0       # đầu khối trụ cách lưng đai lúc chưa ép
BELT_T = 1.4        # dày đai GT2 (lưng đai = đáy răng puly + 1.4)
# joint: (vỏ, puly lớn, động cơ, w giữa đai, r đáy răng puly lớn, puly nhỏ) — w/r đo từ cắt lát puly trong CAD
JOINTS = {"J2": ("AX-1_3__NAUO163", "Riemenscheibe_55mm__NAUO140", "STEPPERONLINE*NAUO156", -4.0, 25.25, 6.0),
          "J3": ("AX-2_1__NAUO124", "Riemenscheibe_40mm__NAUO120", "STEPPERONLINE*NAUO123", 5.5, 18.8, 6.0)}
STICK = {"J2": "31–33", "J3": "35–37"}
W_ROD = {"J2": -4.0, "J3": 5.0}
# J2 nhánh 0: thành phẳng duy nhất nằm sát puly lớn (f ≈ 0.18) → chân khối trụ chạm ổ bi 65x50x7 và que chạm vành puly
# 55 mm khi ép; giữa nhánh vướng 2 trụ vít. Bỏ — 1 que ở nhánh 1 là đủ căng cả vòng đai.
SKIP = set()   # J3: mép vỏ ở w = 10, đai 1.5–9.5 → hạ trục que 0.5 mm
LOG = []
WHY = []
LAST_HOLE = [None]


def P(*a):
    s = " ".join(str(x) for x in a); print(s); LOG.append(s)


def verts(shape):
    out, e = [], TopExp_Explorer(shape, TopAbs_VERTEX)
    while e.More():
        p = BRep_Tool.Pnt_s(TopoDS.Vertex(e.Current())); out.append([p.X(), p.Y(), p.Z()]); e.Next()
    return np.array(out)


def ray_hits(inter, p, d, tmax=80.0):
    """Tham số t (mm) các giao điểm của tia p + t·d với vỏ, tăng dần."""
    inter.Perform(gp_Lin(gp_Pnt(*p), gp_Dir(*d)), 0.0, tmax)
    return sorted(inter.WParameter(i) for i in range(1, inter.NbPnt() + 1))


def frame_shape(shape, origin, ex, ez):
    """Hình dựng trong hệ cục bộ (x = ex, z = ez, gốc origin) → hệ lắp ráp."""
    t = gp_Trsf(); t.SetTransformation(gp_Ax3(gp_Pnt(*origin), gp_Dir(*ez), gp_Dir(*ex)), gp_Ax3())
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def hex_prism(af, z0, z1):
    r = af / np.sqrt(3)
    poly = BRepBuilderAPI_MakePolygon()
    for k in range(6):
        a = np.radians(60 * k + 30); poly.Add(gp_Pnt(r * np.cos(a), r * np.sin(a), z0))
    poly.Close()
    return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(poly.Wire()).Face(), gp_Vec(0, 0, z1 - z0)).Shape()


def fuse(a, b): return BRepAlgoAPI_Fuse(a, b).Shape()
def cut(a, b): return BRepAlgoAPI_Cut(a, b).Shape()


def unify(s):
    u = ShapeUpgrade_UnifySameDomain(s, True, True, True); u.Build(); return u.Shape()


def tangents(c1, r1, c2, r2):
    d = c2 - c1; L = np.linalg.norm(d); e = d / L; out = []
    for s in (+1, -1):
        a = np.arccos((r1 - r2) / L)
        n = np.array([[np.cos(s * a), -np.sin(s * a)], [np.sin(s * a), np.cos(s * a)]]) @ e
        out.append((c1 + r1 * n, c2 + r2 * n, n))
    return out


def section_uv(shape, o, R, w):
    sec = BRepAlgoAPI_Section(shape, gp_Pln(gp_Pnt(*(o + R[2] * w)), gp_Dir(*R[2]))); sec.Build()
    segs, e = [], TopExp_Explorer(sec.Shape(), TopAbs_EDGE)
    while e.More():
        c = BRepAdaptor_Curve(TopoDS.Edge(e.Current())); a = GCPnts_UniformAbscissa(c, 0.4)
        if a.IsDone():
            pts = np.array([[c.Value(a.Parameter(i)).X(), c.Value(a.Parameter(i)).Y(), c.Value(a.Parameter(i)).Z()]
                            for i in range(1, a.NbPoints() + 1)]) - o
            segs.append(np.c_[pts @ R[0], pts @ R[1]])
        e.Next()
    return segs


others = [os.path.basename(f)[:-5] for f in glob.glob(os.path.join(CACHE, "*.brep"))]
new_parts, figs = {}, {}
for j, (hname, pul, motor, W, rB, rS) in JOINTS.items():
    H = load(hname)
    Pu = load(pul); c = max(cylinders(Pu), key=lambda c: c[0]); ax = c[2] / np.linalg.norm(c[2])
    R, o = local_frame(c[1], ax)
    vM = verts(load(motor)).mean(0)
    small = min((n for n in others if n.startswith("Riemenscheibe_16mm")), key=lambda n: np.linalg.norm(verts(load(n)).mean(0) - vM))
    cs = max(cylinders(load(small)), key=lambda c: c[3]); cS = ((cs[1] - o) @ R.T)[:2]
    inter = IntCurvesFace_ShapeIntersector(); inter.Load(H, 1e-4)
    to3 = lambda uv, w=W: o + R[0] * uv[0] + R[1] * uv[1] + R[2] * w
    P(f"\n===== {j}: vỏ {hname.split('__')[0]}, puly lớn {pul.split('__')[0]} / puly nhỏ {small.split('__')[0]}, "
      f"khoảng cách tâm {np.linalg.norm(cS):.1f} mm, mặt phẳng giữa đai w = {W} =====")
    Hn, placed = H, []
    W = W_ROD[j]                               # trục que (có thể lệch khỏi giữa đai để khối trụ không vượt mép vỏ)
    to3 = lambda uv, w=W: o + R[0] * uv[0] + R[1] * uv[1] + R[2] * w
    rb = BOSS_D / 2
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    # lỗ vít / hốc đai ốc của vỏ (trục song song trục puly; mặt trụ quét ≥ 180° — loại góc bo R2–R3 ở chân trụ vít):
    # (tâm uv, bán kính, w_min, w_max). Chỉ lỗ có đoạn w chồng lên khối trụ (từ đáy tới mép hở) mới bị khối trụ che.
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder
    from OCP.BRepTools import BRepTools
    from cad_util import faces as _faces
    acc = {}
    for fc in _faces(H):
        sf = BRepAdaptor_Surface(fc)
        if sf.GetType() != GeomAbs_Cylinder:
            continue
        cy = sf.Cylinder(); r_ = cy.Radius(); d_ = cy.Axis().Direction(); d_ = np.array([d_.X(), d_.Y(), d_.Z()])
        if abs(abs(d_ @ R[2]) - 1) > 1e-3 or not 1.3 <= r_ <= 4.5:
            continue
        p_ = cy.Axis().Location(); c_ = ((np.array([p_.X(), p_.Y(), p_.Z()]) - o) @ R.T)[:2]
        u0, u1, _v0, _v1 = BRepTools.UVBounds_s(fc)
        wv = (verts(fc) - o) @ R[2]
        key = (round(c_[0], 1), round(c_[1], 1), round(r_, 2))
        a_ = acc.setdefault(key, [c_, r_, 0.0, wv.min(), wv.max()])
        a_[2] += u1 - u0; a_[3] = min(a_[3], wv.min()); a_[4] = max(a_[4], wv.max())
    holes = [(a_[0], a_[1], a_[3], a_[4]) for a_ in acc.values() if a_[2] >= np.pi - 1e-3]
    P("  lỗ vít/hốc đai ốc (tâm uv, r, w): " + ", ".join(f"({c[0]:.0f},{c[1]:.0f}) r{r:.1f} w{w0:.0f}..{w1:.0f}" for c, r, w0, w1 in holes))

    def seg_dist(c, q, d2, t0, t1):
        t = np.clip((c - q) @ d2, t0, t1); return np.linalg.norm(q + t * d2 - c)

    def hole_clear(q, d2, tin, tout, wlo=-1e9, whi=1e9):
        """Khe hở nhỏ nhất (mm) giữa khối trụ/lỗ que/hốc đai ốc và các lỗ vít của vỏ (âm = cắt vào)."""
        m = 1e9
        for c, r, w0, w1 in holes:
            if w1 < wlo or w0 > whi:
                continue
            m_old = m
            m = min(m, seg_dist(c, q, d2, TIP_GAP, tin) - (r + rb),
                    seg_dist(c, q, d2, tin, tout) - (r + BORE_D / 2),
                    seg_dist(c, q, d2, tout - HEX_DEPTH, tout) - (r + HEX_AF / np.sqrt(3)))
            if m < m_old:
                LAST_HOLE[0] = (round(c[0]), round(c[1]), round(r, 1), round(w0), round(w1))
        return m

    def build_boss(q, d2, e2):
        """Khối trụ cho que đi qua điểm q (trên lưng đai) theo hướng d2 (ra ngoài). None nếu thành không phù hợp."""
        n3 = R[0] * d2[0] + R[1] * d2[1]; t3 = R[0] * e2[0] + R[1] * e2[1]
        # lớp thành NGOÀI CÙNG trên mỗi tia (trong 40 mm): (mặt trong, mặt ngoài) = 2 giao điểm cuối; vành trụ đai ốc /
        # gân nằm giữa đai và thành sẽ được khối trụ bao luôn (kiểm tra lỗ vít ở dưới đảm bảo không che lỗ nào)
        c0 = ray_hits(inter, to3(q), n3, 40.0)
        if len(c0) < 2 or c0[0] - TIP_GAP < 8.0 or c0[-2] > 37.0 or c0[-1] - c0[-2] > 20.0 or len(c0) % 2:
            WHY.append(("thanh", c0[:4])); return None
        c0 = c0[-2:]
        # 5 tia (tâm + 4 biên) → lớp thành ngoài cùng; mặt ngoài thành coi như phẳng cục bộ (khớp mặt phẳng, sai số
        # ≤ 1 mm). Khối trụ kéo xuyên thành rồi CẮT theo mặt phẳng đó lùi vào 0.3 mm → liền vào thành, không lòi ra ngoài
        # kể cả khi thành xiên so với trục que.
        rays = [(0.0, 0.0, c0[0], c0[1])]
        for dl, dw in ((-rb, 0), (rb, 0), (0, -rb + 1), (0, rb - 1)):
            h = ray_hits(inter, to3(q) + t3 * dl + R[2] * dw, n3, 40.0)
            if not h:
                continue                      # tia đi qua mép hở của vỏ (khối trụ sẽ bị cắt ngang mép)
            if len(h) < 2 or len(h) % 2 or h[0] < TIP_GAP + 1 or h[-1] - h[-2] < 1.8:
                WHY.append(("bien", dl, dw, h[:4])); return None
            rays.append((dl, dw, h[-2], h[-1]))
        if len(rays) < 4:
            WHY.append(("it_tia", len(rays))); return None
        pts_out = np.array([to3(q) + t3 * dl + R[2] * dw + n3 * to for dl, dw, ti, to in rays])
        cen = pts_out.mean(0); _u, _s, vt = np.linalg.svd(pts_out - cen); pn = vt[2] if vt[2] @ n3 > 0 else -vt[2]
        plane_ok = np.abs((pts_out - cen) @ pn).max() <= 1.0 and pn @ n3 >= np.cos(np.radians(60))
        if plane_ok:
            z1 = max(r_[3] for r_ in rays) + 2.0  # kéo xuyên quá mặt ngoài rồi cắt theo mặt phẳng
        else:
            # thành cong: mặt cuối phẳng dừng trước mặt ngoài gần nhất 0.3 mm; tâm + ≥ 3/5 tia phải liền vào thành
            z1 = min(r_[3] for r_ in rays) - 0.3
            att = sum(r_[2] + 0.8 <= z1 for r_ in rays)
            if not (rays[0][2] + 0.8 <= z1 and att >= 3):
                WHY.append(("thanh_cong", att)); return None
        if max(r_[2] for r_ in rays) - TIP_GAP > 34.0:
            WHY.append(("dai", max(r_[2] for r_ in rays))); return None
        tin, tout = c0[0], c0[1]
        base = to3(q); mid = base + n3 * (TIP_GAP + tin) / 2
        d_side = {sg: (ray_hits(inter, mid, sg * R[2], 80.0) or [None])[0] for sg in (+1, -1)}
        wall_pt = base + n3 * (tin + tout) / 2
        rim = {sg: (ray_hits(inter, wall_pt, sg * R[2], 80.0) or [0.0])[0] for sg in (+1, -1)}
        sides = [sg for sg in (+1, -1) if d_side[sg] is not None]
        if not sides:
            return None
        floor_sg = min(sides, key=lambda sg: d_side[sg]); open_sg = -floor_sg
        floor_d = d_side[floor_sg]; open_d = min(rim[open_sg] - 0.4, rb)
        if open_d < 3.5 or floor_d > 30:
            WHY.append(("day/mep", floor_d, open_d)); return None
        wspan = sorted([W + floor_sg * (floor_d + 1.5), W + open_sg * open_d])
        clear = hole_clear(q, d2, tin, tout, *wspan)
        if clear < 1.5:                       # ≥ 1.5 mm nhựa giữa khối trụ và lỗ vít
            WHY.append(("lo_vit", round(clear, 1), LAST_HOLE[0])); return None
        ez, ex = n3, t3; ey = np.cross(ez, ex)
        ys = 1.0 if ey @ (floor_sg * R[2]) > 0 else -1.0
        z0 = TIP_GAP
        cyl = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, z0), gp_Dir(0, 0, 1)), rb, z1 - z0).Shape()
        yf = ys * (floor_d + 1.0)
        # chân đỡ chỉ ăn 0.5 mm vào mặt trong thành (không chạm góc bo ngoài thành–đáy)
        zl1 = min(min(r_[2] for r_ in rays) + 0.5, z1)
        leg = BRepPrimAPI_MakeBox(gp_Pnt(-rb, min(0.0, yf), z0), BOSS_D, abs(yf), zl1 - z0).Shape()
        yo = -ys * open_d
        keep = BRepPrimAPI_MakeBox(gp_Pnt(-50, min(yo, yf), z0 - 1), 100, abs(yf - yo), z1 - z0 + 2).Shape()
        boss = frame_shape(BRepAlgoAPI_Common(fuse(cyl, leg), keep).Shape(), base, ex, ez)
        if plane_ok:   # nửa không gian phía trong mặt ngoài thành (lùi 0.3 mm)
            pe = np.cross(pn, R[2]); pe /= np.linalg.norm(pe)
            half = frame_shape(BRepPrimAPI_MakeBox(gp_Pnt(-200, -200, -400), 400, 400, 400).Shape(), cen - pn * 0.3, pe, pn)
            boss = BRepAlgoAPI_Common(boss, half).Shape()
        bore = frame_shape(BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, z0 - 1), gp_Dir(0, 0, 1)), BORE_D / 2, tout + 6 - z0).Shape(), base, ex, ez)
        hexp = frame_shape(hex_prism(HEX_AF, tout - HEX_DEPTH, tout + 4), base, ex, ez)
        # lỗ + hốc đai ốc không được ăn vào trụ vít/gân: vùng Ø khối trụ quanh trục, từ mặt trong ra ngoài, chỉ được
        # chứa đúng 1 lớp thành (thể tích ≈ diện tích × bề dày, cho phép xiên)
        env = frame_shape(BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, tin - 0.5), gp_Dir(0, 0, 1)), rb, tout - tin + 4).Shape(), base, ex, ez)
        v_env = vol(BRepAlgoAPI_Common(env, H).Shape())
        if v_env > np.pi * rb ** 2 * (tout - tin + 4) * 1.6 + 30:
            WHY.append(("env", round(v_env))); return None
        rod = frame_shape(BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -10), gp_Dir(0, 0, 1)), 2.0, tout + 10).Shape(), base, ex, ez)
        from OCP.BRepOffsetAPI import BRepOffsetAPI_MakeOffsetShape
        for n in others:
            if n == hname:
                continue
            S = load(n)
            if n.startswith("Lager") and collide(boss, S) > 0.05 and collide(rod, S) <= 0.05:
                # chân đỡ chạm ổ bi → khoét tránh ổ bi, chừa khe 0.5 mm
                off = BRepOffsetAPI_MakeOffsetShape(); off.PerformBySimple(S, 0.5)
                boss = cut(boss, off.Shape())
            if collide(boss, S) > 0.05 or collide(rod, S) > 0.05:
                WHY.append(("va_cham", n.split("__")[0])); return None
        return dict(clear=clear, boss=boss, bore=bore, hexp=hexp, tin=tin, tout=tout, floor_d=floor_d, rim=rim[open_sg], base=base,
                    ez=ez, z=(z0, z1))

    for k, (p1, p2, n2) in enumerate(tangents(np.zeros(2), rB + BELT_T, cS, rS + BELT_T)):
        L = np.linalg.norm(p2 - p1); tdir = (p2 - p1) / L
        cands = sorted(((f, a) for f in np.arange(0.12, 0.80, 0.02) for a in range(-25, 26, 5)),
                       key=lambda fa: abs(fa[0] - 0.5) + abs(fa[1]) / 50)
        res = None
        for f, a in cands:
            q = p1 + f * (p2 - p1); th = np.radians(a)
            rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
            d2, e2 = rot @ n2, rot @ tdir
            WHY.clear(); res = build_boss(q, d2, e2)
            if os.environ.get("CHAN_DOAN") and j == os.environ["CHAN_DOAN"] and abs(a) <= 10:
                print("   chẩn đoán", j, k, round(f, 3), a, WHY)
            if res:
                break
        if not res:
            P(f"  nhánh {k}: KHÔNG tìm được chỗ đặt (thành phẳng, không trụ vít, không va chạm) — bỏ qua"); continue
        before = vol(Hn)
        Hn = cut(cut(fuse(Hn, res["boss"]), res["bore"]), res["hexp"])
        tin, tout = res["tin"], res["tout"]
        P(f"  nhánh {k}: f = {f:.3f} ({f * L:.1f} mm dọc nhánh từ điểm rời puly lớn, 0.5 = giữa nhánh), que nghiêng {a:+d}° so với "
          f"pháp tuyến đai; lưng đai → mặt trong thành {tin:.1f} mm, → mặt ngoài {tout:.1f} mm; đáy vỏ cách trục que "
          f"{res['floor_d']:.1f} mm, mép hở cách trục que {res['rim']:.1f} mm; khối trụ dài {res['z'][1] - res['z'][0]:.1f} mm, "
          f"thêm {(vol(Hn) - before) / 1000:+.2f} cm3; cách lỗ vít gần nhất ≥ {res['clear']:.1f} mm")
        P(f"           chiều dài que tính từ MẶT NGOÀI vỏ: chạm lưng đai = {tout:.1f} mm; ép vào 5 / 8 / 10 mm = "
          f"{tout + 5:.1f} / {tout + 8:.1f} / {tout + 10:.1f} mm (mẩu đũa đang dùng {STICK[j]} mm)")
        placed.append(dict(k=k, f=f, q=q, n2=d2, tin=tin, tout=tout, boss=res["boss"], base=res["base"], ez=res["ez"]))
    Hu = unify(Hn)
    if BRepCheck_Analyzer(Hu).IsValid(): Hn = Hu
    P(f"  vỏ gốc hợp lệ: {BRepCheck_Analyzer(H).IsValid()}; vỏ mới hợp lệ: {BRepCheck_Analyzer(Hn).IsValid()}, thể tích {vol(H) / 1000:.2f} → {vol(Hn) / 1000:.2f} cm3")
    # va chạm phần vật liệu thêm với mọi chi tiết khác (đứng yên trong tư thế CAD)
    added = cut(Hn, H)
    hit = []
    for n in others:
        if n == hname:
            continue
        v = collide(added, load(n))
        if v > 0.05:
            hit.append(f"{n.split('__')[0]} {v:.2f} mm3")
    P("  phần thêm chạm chi tiết khác: " + (", ".join(hit) if hit else "KHÔNG"))
    # phần thêm có lòi ra NGOÀI vỏ không: điểm lưới của phần thêm nằm ngoài vật liệu vỏ cũ mà ≥ 5/8 tia ngang (trong mặt
    # phẳng đai) không gặp vỏ cũ → nằm ngoài vỏ (trong lòng vỏ thì tia nào cũng gặp thành)
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
    from OCP.TopAbs import TopAbs_IN, TopAbs_ON
    BRepMesh_IncrementalMesh(added, 0.3, False, 0.5, True)
    pts = verts(added)
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopAbs import TopAbs_FACE as _F
    ex_ = TopExp_Explorer(added, _F)
    while ex_.More():
        tri = BRep_Tool.Triangulation_s(TopoDS.Face(ex_.Current()), TopLoc_Location())
        if tri is not None:
            pts = np.vstack([pts, [[tri.Node(i).X(), tri.Node(i).Y(), tri.Node(i).Z()] for i in range(1, tri.NbNodes() + 1)]])
        ex_.Next()
    worst = 0.0
    for pt in pts:
        cl = BRepClass3d_SolidClassifier(H, gp_Pnt(*pt), 1e-4)
        if cl.State() in (TopAbs_IN, TopAbs_ON):
            continue
        miss = sum(not ray_hits(inter, pt, R[0] * np.cos(a_) + R[1] * np.sin(a_), 150.0) for a_ in np.radians(np.arange(0, 360, 45)))
        if miss >= 5:
            dd = BRepExtrema_DistShapeShape(BRepBuilderAPI_MakeVertex(gp_Pnt(*pt)).Vertex(), H); dd.Perform()
            worst = max(worst, dd.Value())
    P(f"  phần thêm lòi ra ngoài vỏ: {'KHÔNG' if worst < 0.05 else f'CÓ, tối đa {worst:.2f} mm'} ({len(pts)} điểm kiểm tra)")
    # que Ø4 (đến khi ép 10 mm) có chạm puly/động cơ/vít không
    for pl in placed:
        rod = frame_shape(BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -10), gp_Dir(0, 0, 1)), 2.0, pl["tout"] + 10).Shape(),
                          pl["base"], np.cross(pl["ez"], R[2]), pl["ez"])
        rh = [n.split("__")[0] for n in others if n not in (hname,) and collide(rod, load(n)) > 0.05]
        P(f"  que nhánh {pl['k']} (ép 10 mm) chạm: " + (", ".join(rh) if rh else "KHÔNG (chỉ chạm lưng đai)"))
    new_parts[j] = (hname, Hn, placed, H, R, o, W, cS, rB, rS)

# ---------------- ghi file ----------------
os.makedirs(OUT, exist_ok=True); os.makedirs(PRINT, exist_ok=True)
NEW_NAME = {"J2": "AX-1_3_cang_dai_J2", "J3": "AX-2_1_cang_dai_J3"}
for j, (hname, Hn, *_r) in new_parts.items():
    write_step([(NEW_NAME[j], Hn)], os.path.join(OUT, NEW_NAME[j] + ".step"))
    write_stl(Hn, os.path.join(OUT, NEW_NAME[j] + ".stl"))
    shutil.copy(os.path.join(OUT, NEW_NAME[j] + ".stl"), PRINT)

# file lắp ráp tổng: thay 2 vỏ trong bản rút ngắn + 17PM + giác hút
doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
rd = STEPCAFControl_Reader(); rd.SetNameMode(True); rd.ReadFile(ASM_IN); rd.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()); labs = TDF_LabelSequence(); st.GetFreeShapes(labs)
asm, replaced = [], []
for i in range(1, labs.Length() + 1):
    lab = labs.Value(i); nm = TDataStd_Name()
    name = nm.Get().ToExtString() if lab.FindAttribute(TDataStd_Name.GetID_s(), nm) else f"part_{i}"
    shp = st.GetShape_s(lab)
    for j, (hname, Hn, _p, H, *_r) in new_parts.items():
        if name.startswith(hname.split("__")[0]) and abs(vol(shp) - vol(H)) < 1.0 and np.allclose(verts(shp).mean(0), verts(H).mean(0), atol=0.05):
            shp, name = Hn, NEW_NAME[j]; replaced.append(name); break
    asm.append((name, shp))
P(f"\nLắp ráp tổng: {len(asm)} chi tiết, đã thay: {replaced}")
assert len(replaced) == len(new_parts), "không tìm thấy vỏ gốc trong file lắp ráp"
write_step(asm, os.path.join(OUT, "Roboter_rut_ngan_17PM_giac_hut_cang_dai.step"))

# ảnh mặt cắt tại mặt phẳng đai
fig, axs = plt.subplots(1, 2, figsize=(14, 9))
for a, (j, (hname, Hn, placed, H, R, o, W, cS, rB, rS)) in zip(axs, new_parts.items()):
    for s in section_uv(Hn, o, R, W): a.plot(s[:, 0], s[:, 1], "-", color="tab:orange", lw=1)
    t = np.linspace(0, 2 * np.pi, 200)
    for cc, r in ((np.zeros(2), rB), (cS, rS)):
        a.plot(cc[0] + r * np.cos(t), cc[1] + r * np.sin(t), "k-", lw=0.7)
    for k, (p1, p2, n2) in enumerate(tangents(np.zeros(2), rB + BELT_T, cS, rS + BELT_T)):
        a.plot([p1[0], p2[0]], [p1[1], p2[1]], "-", color="0.3", lw=2)
    for pl in placed:
        q, n2 = pl["q"], pl["n2"]
        a.plot([q[0] - n2[0] * 8, q[0] + n2[0] * (pl["tout"] + 6)], [q[1] - n2[1] * 8, q[1] + n2[1] * (pl["tout"] + 6)], "b--", lw=1.2)
        a.annotate(f"que Ø4\nchạm đai: {pl['tout']:.1f} mm\n(từ mặt ngoài)", q + n2 * (pl["tout"] + 7), fontsize=8, color="b")
    a.set_aspect("equal"); a.grid(lw=0.3)
    a.set_title(f"{j} — {NEW_NAME[j]} cắt tại mặt phẳng giữa đai\n(xám = đai, xanh nét đứt = que đẩy vào lưng đai)")
fig.tight_layout(); fig.savefig(os.path.join(OUT, "mat_cat_cang_dai.png"), dpi=110); plt.close(fig)

with open(os.path.join(OUT, "ket_qua_kiem_tra.txt"), "w", encoding="utf-8") as fo:
    fo.write("\n".join(LOG) + "\n")
P(f"Đã ghi {OUT} và {PRINT}")
