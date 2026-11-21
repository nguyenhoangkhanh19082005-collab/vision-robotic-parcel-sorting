r"""CẬP NHẬT LẮP RÁP THEO THỰC TẾ (20/11/2026) — bản 1:27 (động cơ J1–J3 mới) + động cơ J4 cũ + gá giác hút J6:

1. MẶT BÍCH 5 mm chuyển sang PHÍA KHÂU (vùng xanh người dùng chỉ): vành Ø22 áp lên mặt đỉnh trụ chứa đai ốc M5
   (J2: AX-2_5 tại w = −27.5; J3: AX-3_2 tại w = −24.5), moay-ơ hướng ra ngoài (−w). Mặt bích nối TRỤC (bu lông M5) với
   KHÂU: puly kẹp bu lông bằng vít hãm của puly → bu lông → vít hãm moay-ơ mặt bích → 4 vít M3 → khâu.
   - Mặt đỉnh hiện chỉ phẳng Ø11 (J2) / Ø19 (J3), mép bo cong → thêm ĐẾ PHẲNG Ø23 (vành Ø22 + 0.5 mm mỗi bên), giữ
     nguyên hốc đai ốc M5 (đai ốc vẫn nằm dưới vành mặt bích).
   - 4 lỗ mồi Ø2.5 sâu 7 mm trên vòng Ø16 cho vít M3×8 (vành 2.1 mm + 5.9 mm ăn vào nhựa).
   - Bu lông M5×25 không đủ dài để vít hãm moay-ơ (cách mặt vành 8.75 mm) kẹp được → đổi M5×35.
   - Mặt bích cũ (ngoài puly lớn) bỏ đi; puly lớn và nắp AX-2_3 trở về bản gốc (không cần lỗ/khoét cho mặt bích).
2. PULY NHỎ J2, J3 nâng +3.0 mm theo trục (đế puly bằng mặt trên "con thoi" AX-1_6, như lắp thực tế).
3. Vỏ đai AX-1_3 / AX-2_1 và nắp AX-2_3 bản căng đai v2 (them_cang_dai_J2_J3_v2.py).
4. Gá giác hút J6 (tao_ga_giac_hut_J6.py).
Đầu ra: assets/CAD_v2_dai_that_mat_bich/ và assets/Gui_xuong_in/E_cang_dai_v2_mat_bich/.
    E:\UDMT\.venv\Scripts\python.exe -X utf8 src\cad\cap_nhat_mat_bich_v2.py
"""
import os, sys, shutil
import numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "research")); sys.path.insert(0, os.path.dirname(__file__))
from rut_ngan_lib import parts, loadi, vol, cut, fuse, common, unify
from cad_util import cylinders, local_frame, load, bbox
from tao_chi_tiet_dong_co_1_27 import write_step, write_stl, collide
from OCP.BRep import BRep_Builder
from OCP.BRepTools import BRepTools
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.STEPControl import STEPControl_Reader
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDataStd import TDataStd_Name
from OCP.collections import Sequence_TDF_Label
from OCP.TopoDS import TopoDS_Shape
from OCP.gp import gp_Ax2, gp_Ax3, gp_Dir, gp_Lin, gp_Pnt, gp_Trsf, gp_Vec

OUT = os.path.join(ROOT, "assets", "CAD_v2_dai_that_mat_bich")
PRINT = os.path.join(ROOT, "assets", "Gui_xuong_in", "E_cang_dai_v2_mat_bich")
V2 = os.path.join(ROOT, "assets", "CAD_cang_dai_J2_J3_v2")
DW_SMALL = 3.0          # puly nhỏ nâng theo trục
PAD_R = 11.5            # đế phẳng Ø23 cho vành mặt bích Ø22
PAD_RIN = 5.5           # không chạm hốc đai ốc M5 (góc lục giác r ≤ 5.33)
PILOT_D, PCD_R = 2.5, 8.0
PILOT_DEPTH = {"J2": 7.0, "J3": 6.5}   # J3: nhựa sau mặt đỡ chỉ 8 mm → lỗ 6.5, còn 1.5 mm; vít M3×8 ăn 5.9 mm
BOLT_EXT = 10.0         # M5×25 → M5×35
LOG = []


def P(*a):
    s = " ".join(str(x) for x in a); print(s); LOG.append(s)


def rd(p):
    s = TopoDS_Shape(); BRepTools.Read_s(s, p, BRep_Builder()); return s


def frame(shape, origin, ez, ex):
    t = gp_Trsf(); t.SetTransformation(gp_Ax3(gp_Pnt(*origin), gp_Dir(*ez), gp_Dir(*ex)), gp_Ax3())
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def moved(shape, v):
    t = gp_Trsf(); t.SetTranslation(gp_Vec(*v)); return BRepBuilderAPI_Transform(shape, t, True).Shape()


def zcyl(r, z0, z1, x=0.0, y=0.0):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, y, z0), gp_Dir(0, 0, 1)), r, z1 - z0).Shape()


name_of = {i: q["name"] for i, q in enumerate(parts)}
idx = lambda pref: [i for i, n in name_of.items() if n.startswith(pref)]
r = STEPControl_Reader(); r.ReadFile(os.path.join(ROOT, "assets", "CAD_mat_bich", "mat_bich_5mm_goc.step")); r.TransferRoots()
FL = r.OneShape()   # cục bộ: mặt vành z = 0, vành tới 2.1, moay-ơ tới 11.95, 4 lỗ M3 tại (±8,0),(0,±8), vít hãm ở z = 8.75

# chỉ số trong lắp ráp 1:27 (research/.tmp/step_lap_rap.json)
J = {"J2": dict(big=190, small=184, link=14, bolt=112, nut=178, old_fl=196, seat=-27.5, floor=-23.5, orig_pul="Riemenscheibe_55mm__NAUO140"),
     "J3": dict(big=187, small=183, link=16, bolt=111, nut=177, old_fl=197, seat=-24.5, floor=-20.5, orig_pul="Riemenscheibe_40mm__NAUO120")}
replace, remove, add = {}, set(), []
for j, d in J.items():
    c = max(cylinders(loadi(d["big"])), key=lambda c: c[0]); ax = c[2] / np.linalg.norm(c[2])
    R, o = local_frame(c[1], ax); d["R"], d["o"] = R, o
    P(f"\n===== {j} =====")
    # --- puly lớn về bản gốc
    pg = load(d["orig_pul"])
    lo1, hi1 = bbox(pg); lo2, hi2 = bbox(loadi(d["big"]))
    P(f"  puly lớn gốc {d['orig_pul'].split('__')[0]} thay bản khoan lỗ mặt bích; lệch hộp bao {np.abs(np.r_[lo1 - lo2, hi1 - hi2]).max():.3f} mm")
    replace[d["big"]] = (d["orig_pul"].split("__")[0] + f"_{j}", pg)
    remove.add(d["old_fl"])
    # --- puly nhỏ nâng +3
    ps = moved(loadi(d["small"]), R[2] * DW_SMALL)
    replace[d["small"]] = (name_of[d["small"]] + f"_{j}_nang_{DW_SMALL:g}mm", ps)
    # --- khâu: đế phẳng + lỗ mồi
    org = o + R[2] * d["seat"]; ez = -R[2]; ex = R[0]          # hệ mặt bích: z = 0 tại mặt đỡ, +z ra ngoài khâu
    L0 = loadi(d["link"])
    depth_pad = d["seat"] - d["floor"]                           # âm: đế đi vào khâu (+w) 4 mm
    pad = frame(cut(zcyl(PAD_R, depth_pad, 0.0), zcyl(PAD_RIN, depth_pad - 1, 1)), org, ez, ex)
    pil = None
    for k in range(4):
        a = np.radians(90 * k)
        h = frame(zcyl(PILOT_D / 2, -PILOT_DEPTH[j], 0.5, PCD_R * np.cos(a), PCD_R * np.sin(a)), org, ez, ex)
        pil = h if pil is None else fuse(pil, h)
    L1 = cut(fuse(L0, pad), pil)
    Lu = unify(L1)
    if BRepCheck_Analyzer(Lu).IsValid(): L1 = Lu
    P(f"  {name_of[d['link']]}: đế phẳng Ø{2 * PAD_R:g} (thêm {(vol(fuse(L0, pad)) - vol(L0)):.0f} mm3) + 4 lỗ mồi Ø{PILOT_D} sâu {PILOT_DEPTH[j]:g} "
      f"→ hợp lệ {BRepCheck_Analyzer(L1).IsValid()}, thể tích {vol(L0) / 1000:.2f} → {vol(L1) / 1000:.2f} cm3")
    # nhựa còn lại sau đáy lỗ mồi
    it = IntCurvesFace_ShapeIntersector(); it.Load(L1, 1e-4)
    p_ = org + R[0] * PCD_R + R[2] * (PILOT_DEPTH[j] + 0.01)
    it.Perform(gp_Lin(gp_Pnt(*p_), gp_Dir(*R[2])), 0, 40); hh = sorted(it.WParameter(i) for i in range(1, it.NbPnt() + 1))
    P(f"  nhựa còn lại sau đáy lỗ mồi: {hh[0] if hh else 0:.2f} mm")
    lname = name_of[d["link"]] + f"_mat_bich_{j}"
    replace[d["link"]] = (lname, L1)
    # --- mặt bích mới
    fl = frame(FL, org, ez, ex)
    add.append((f"Mat_bich_5mm_{j}_phia_khau", fl))
    # --- bu lông M5×35
    bo = loadi(d["bolt"])
    wb = [((v - o) @ R[2]) for v in [bbox(bo)[0], bbox(bo)[1]]]
    tip = min(cc[1] for cc in [(0, 0)]) if False else None
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_VERTEX
    from OCP.TopoDS import TopoDS
    from OCP.BRep import BRep_Tool
    ws = []
    e = TopExp_Explorer(bo, TopAbs_VERTEX)
    while e.More():
        pp = BRep_Tool.Pnt_s(TopoDS.Vertex(e.Current())); ws.append((np.array([pp.X(), pp.Y(), pp.Z()]) - o) @ R[2]); e.Next()
    tip = min(ws)
    ext = frame(zcyl(2.5, 0, BOLT_EXT + 0.01), o + R[2] * (tip + 0.01), -R[2], R[0])
    bo2 = fuse(bo, ext)
    replace[d["bolt"]] = (f"ISK_M5x35_DIN912_{j}", bo2)
    hub_end = d["seat"] - 11.95
    P(f"  bu lông: đầu mũi w {tip:.2f} → {tip - BOLT_EXT:.2f}; moay-ơ mặt bích w {d['seat']:.2f} … {hub_end:.2f}; vít hãm tại w "
      f"{d['seat'] - 8.75:.2f} → bu lông {'ĐỦ' if tip - BOLT_EXT < d['seat'] - 8.75 - 1.5 else 'THIẾU'} chiều dài (mũi {'thò' if tip - BOLT_EXT < hub_end else 'nằm trong'} "
      f"{abs(tip - BOLT_EXT - hub_end):.2f} mm so với đầu moay-ơ)")
    d["fl"], d["L1"], d["bo2"], d["pad"], d["ps"] = fl, L1, bo2, pad, ps
    # vít hãm moay-ơ (z = 8.75, 2 lỗ theo phương chéo): khoảng trống từ mặt moay-ơ (r = 5) ra tới khâu để luồn lục giác
    itL = IntCurvesFace_ShapeIntersector(); itL.Load(L1, 1e-4)
    free = []
    for ang in range(0, 360, 15):
        a_ = np.radians(ang); dv = R[0] * np.cos(a_) - R[1] * np.sin(a_)
        p0 = org + ez * 8.75 + dv * 5.0
        itL.Perform(gp_Lin(gp_Pnt(*p0), gp_Dir(*dv)), 0, 200); hh = [itL.WParameter(i) for i in range(1, itL.NbPnt() + 1)]
        free.append((min(hh) if hh else 999.0, ang))
    fmin, fmax = min(free), max(free)
    P(f"  vít hãm moay-ơ: khoảng trống hướng kính tới thành khâu {fmin[0]:.1f} mm (hướng {fmin[1]}°) … "
      f"{'thông ra ngoài' if fmax[0] > 900 else f'{fmax[0]:.1f} mm'} (hướng {fmax[1]}°) — xoay mặt bích để vít hãm nằm ở hướng thoáng")

# --- vỏ / nắp căng đai v2
NV = {3: ("AX-1_3_cang_dai_J2_v2", "AX-1_3_cang_dai_J2_v2.brep"), 10: ("AX-2_1_cang_dai_J3_v2", "AX-2_1_cang_dai_J3_v2.brep"),
      12: ("AX-2_3_ranh_cang_dai_J3", "AX-2_3_ranh_cang_dai_J3.brep")}
for i, (nm, f) in NV.items():
    s = rd(os.path.join(V2, f))
    replace[i] = (nm, s)
    P(f"  [{i}] {name_of[i]} → {nm} (hợp lệ {BRepCheck_Analyzer(s).IsValid()})")
# --- gá giác hút J6
doc = TDocStd_Document(TCollection_ExtendedString("x"))
rd_ = STEPCAFControl_Reader(); rd_.SetNameMode(True); rd_.ReadFile(os.path.join(ROOT, "assets", "CAD_giac_hut", "Lap_rap_ga_giac_hut_J6.step")); rd_.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()); labs = Sequence_TDF_Label(); st.GetFreeShapes(labs)
for i in range(1, labs.Length() + 1):
    lab = labs.Value(i); nm = TDataStd_Name()
    name = nm.Get().ToExtString() if lab.FindAttribute(TDataStd_Name.GetID_s(), nm) else f"x{i}"
    if not name.startswith("AX-6"):
        add.append((name, st.GetShape_s(lab))); P(f"  thêm {name}")

# ---------------- lắp ráp + kiểm tra va chạm ----------------
asm = []
for i in range(len(parts)):
    if i in remove:
        continue
    asm.append((i, *replace.get(i, (name_of[i], loadi(i)))))
for nm, s in add:
    asm.append((None, nm, s))
# gá giác hút: 6 vít M3×10 cắm sẵn ở mặt ngoài mặt bích AX-6 phải tháo (gá dùng 4 vít M3×12 qua đế vào đai ốc chìm)
ga = next(s for _, n, s in asm if n == "Ga_giac_hut_J6")
drop = [k for k, (i, n, s) in enumerate(asm) if n.startswith("ISK_M3x10") and collide(ga, s) > 0.5]
asm = [x for k, x in enumerate(asm) if k not in drop]
P(f"  bỏ {len(drop)} vít M3×10 ở mặt ngoài mặt bích J6 (vướng gá giác hút — theo thiết kế gá)")
P(f"\nLắp ráp: {len(asm)} chi tiết")
# chồng lấn của các chi tiết mới/sửa với mọi chi tiết khác (bỏ cặp tiếp xúc có chủ đích)
PREF = ("Mat_bich_5mm", "ISK_M5x35", "AX-2_5_mat", "AX-3_2_mat", "Riemenscheibe_16mm_J",
        "AX-1_3_cang", "AX-2_1_cang", "AX-2_3_ranh", "Ga_giac", "Giac_hut")
CHECK = [k for k, (_, n, _) in enumerate(asm) if n.startswith(PREF)]
ALLOW = {frozenset(("Ga_giac_hut_J6", "Giac_hut_L60-M12_HSP-40_mo_phong"))}
seen = set()
for kn in CHECK:
    i_n, n, sn = asm[kn]
    hits = []
    for km, (i_m, m, s) in enumerate(asm):
        if km == kn or frozenset((kn, km)) in seen or frozenset((n, m)) in ALLOW:
            continue
        seen.add(frozenset((kn, km)))
        v = collide(sn, s)
        if v > 0.5:
            pre = collide(loadi(i_n), loadi(i_m)) if i_n is not None and i_m is not None else 0.0
            hits.append(f"{m} {v:.1f} mm3" + (f" (có sẵn trong bản gốc: {pre:.1f} mm3)" if pre > 0.5 else " (MỚI)"))
    P(f"  {n}: chồng lấn {'KHÔNG' if not hits else ', '.join(hits)}")

# ---------------- dây đai ở mặt phẳng thật (puly nhỏ đã nâng) ----------------
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
P("\nDây đai GT2 6 mm ở mặt phẳng thật (giữa vùng răng puly nhỏ đã nâng):")
for j, d in J.items():
    R, o = d["R"], d["o"]
    cs = max(cylinders(d["ps"]), key=lambda c: c[3]); cS = ((cs[1] - o) @ R.T)[:2]
    rB = max(c_[0] for c_ in cylinders(loadi(d["big"])) if c_[0] < 26) if j == "J2" else 18.8
    rB = 25.25 if j == "J2" else 18.8
    w0 = -7.5 + DW_SMALL + 0.75
    def ring(c, r_):
        return cut(zcyl(r_ + 1.4, w0, w0 + 6.0, c[0], c[1]), zcyl(r_, w0 - 1, w0 + 7, c[0], c[1]))
    bl = fuse(ring(np.zeros(2), rB), ring(cS, 6.0))
    Lc = np.linalg.norm(cS); e_ = cS / Lc
    for sg in (+1, -1):
        a_ = np.arccos((rB - 6.0) / Lc); n_ = np.array([[np.cos(sg * a_), -np.sin(sg * a_)], [np.sin(sg * a_), np.cos(sg * a_)]]) @ e_
        p1 = n_ * rB; p2 = cS + n_ * 6.0; t_ = (p2 - p1) / np.linalg.norm(p2 - p1)
        bx = BRepPrimAPI_MakeBox(gp_Pnt(0, -0.0, 0), float(np.linalg.norm(p2 - p1)), 1.4, 6.0).Shape()
        y_ = np.array([-t_[1], t_[0]]); flip = 1.0 if y_ @ n_ > 0 else -1.0
        tr = gp_Trsf(); tr.SetTransformation(gp_Ax3(gp_Pnt(p1[0], p1[1], w0), gp_Dir(0, 0, flip), gp_Dir(t_[0], t_[1], 0)), gp_Ax3())
        bx = BRepBuilderAPI_Transform(bx, tr, True).Shape()
        if flip < 0:
            bx = moved(bx, np.array([0, 0, 6.0]))
        bl = fuse(bl, bx)
    blw = frame(bl, o, R[2], R[0])
    hits = []
    for _, m, s_ in asm:
        v = collide(blw, s_)
        if v > 0.5: hits.append(f"{m} {v:.0f} mm3")
    P(f"  {j}: đai w {w0:.2f} … {w0 + 6:.2f} (vùng răng puly lớn −8.0 … 0.0, puly nhỏ đã nâng −4.5 … +3.0) chạm: {hits if hits else 'KHÔNG'}")

os.makedirs(OUT, exist_ok=True); os.makedirs(PRINT, exist_ok=True)
for j, d in J.items():
    lname = replace[d["link"]][0]
    write_step([(lname, d["L1"])], os.path.join(OUT, lname + ".step")); write_stl(d["L1"], os.path.join(OUT, lname + ".stl"))
    shutil.copy(os.path.join(OUT, lname + ".stl"), PRINT)
write_step([(n, s) for _, n, s in asm], os.path.join(OUT, "Roboter_1_27_giac_hut_cang_dai_v2_mat_bich.step"))
with open(os.path.join(OUT, "ket_qua_kiem_tra.txt"), "w", encoding="utf-8") as fo:
    fo.write("\n".join(LOG) + "\n")
P(f"Đã ghi {OUT}")
