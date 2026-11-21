r"""Tạo các chi tiết in 3D đã sửa để thay động cơ J1/J2/J3 bằng 42STH38-1684A + hộp số hành tinh
36JXF27G (1:27, mặt hộp số Ø35.9, 4 lỗ ren M2.5 trên vòng Ø28, trục vuông 6 mm dài 14.5 mm).

Đầu vào: cache .brep của Roboter.STEP (research/cad_xuat_brep.py). Đầu ra (hệ tọa độ lắp ráp gốc, mm) trong
assets/CAD_dong_co_1_27/: từng chi tiết .step + .stl và file lắp ráp Roboter_dong_co_1_27.step để mở bằng
Fusion 360 (Insert > Insert Mesh / Open). Kết quả đã kiểm tra va chạm in ra cuối chương trình.

Các sửa đổi (lý do: xem docs/THAY_DONG_CO_J1_J3_HOP_SO_1_27.md):
  1. AX-2_2  : khoét hốc phía sau động cơ J3 (động cơ mới dài hơn PG5 12.3 mm, va vào thành vỏ khâu 2).
  2. AX-1_11 : thêm 4 lỗ Ø3.0 (vít M2.5) trên vòng Ø28 để bắt mặt hộp số J1 (giữ nguyên lỗ NEMA cũ).
  3. De_nang_J1 (MỚI): đế nâng ôm đúng biên dạng đáy AX-1_10, vì động cơ J1 mới thò xuống dưới mặt đáy.
  AX-1_6 (gá J2/J3) GIỮ NGUYÊN: hốc Ø37 + 4 lỗ Ø3.5 trên vòng Ø28 đã khớp hộp số Ø36 (dùng vít M2.5 + long đền).

    E:\UDMT\.venv\Scripts\python.exe src\cad\tao_chi_tiet_dong_co_1_27.py
"""

from __future__ import annotations

import glob
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "research"))
from cad_khung_dong_co import MotorFrame, verts  # noqa: E402
from cad_util import CACHE, cylinders, faces, load, local_frame  # noqa: E402

from OCP.BRepAdaptor import BRepAdaptor_Surface  # noqa: E402
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse  # noqa: E402
from OCP.BRepBndLib import BRepBndLib  # noqa: E402
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform  # noqa: E402
from OCP.BRepFilletAPI import BRepFilletAPI_MakeChamfer  # noqa: E402
from OCP.BRepGProp import BRepGProp  # noqa: E402
from OCP.BRepMesh import BRepMesh_IncrementalMesh  # noqa: E402
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakePrism  # noqa: E402
from OCP.Bnd import Bnd_Box  # noqa: E402
from OCP.GeomAbs import GeomAbs_Plane  # noqa: E402
from OCP.GProp import GProp_GProps  # noqa: E402
from OCP.IFSelect import IFSelect_RetDone  # noqa: E402
from OCP.Interface import Interface_Static  # noqa: E402
from OCP.STEPCAFControl import STEPCAFControl_Writer  # noqa: E402
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer  # noqa: E402
from OCP.StlAPI import StlAPI_Writer  # noqa: E402
from OCP.TCollection import TCollection_ExtendedString  # noqa: E402
from OCP.TDataStd import TDataStd_Name  # noqa: E402
from OCP.TDocStd import TDocStd_Document  # noqa: E402
from OCP.XCAFDoc import XCAFDoc_DocumentTool  # noqa: E402
from OCP.gp import gp_Ax2, gp_Ax3, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec  # noqa: E402

OUT = os.path.join(ROOT, "assets", "CAD_dong_co_1_27")

# ---- Động cơ mới (mm, theo ảnh người bán — đo lại khi nhận hàng) ----
GEAR_D, GEAR_L = 35.9, 34.1          # hộp số Ø35.9, dài 34.1 (từ mặt đầu ra tới mặt bích động cơ)
MOTOR_BODY, MOTOR_L = 42.3, 39.5     # thân NEMA17 42.3 vuông, dài 39.5 (42STH38)
SHAFT_W, SHAFT_L = 6.0, 14.5         # trục ra vuông 6, dài 14.5
HOLE_PCD, HOLE_M = 28.0, 2.5         # 4 lỗ ren M2.5 trên vòng Ø28 ở mặt đầu ra
CLEAR = 1.5                          # khe hở quanh động cơ khi khoét chi tiết in
M25_CLEAR_D = 3.0                    # lỗ xuyên cho vít M2.5 trên nhựa in (co ngót ~0.1–0.2 mm)
RISER_GAP = 5.0                      # khe hở dưới đuôi động cơ J1 tới mặt bàn


def vol(shape) -> float:
    p = GProp_GProps(); BRepGProp.VolumeProperties_s(shape, p); return p.Mass()


def to_frame(shape, origin, R):
    """Hệ cục bộ (x=R[0], y=R[1], z=R[2], gốc origin) → hệ lắp ráp."""
    ax = gp_Ax3(gp_Pnt(*origin), gp_Dir(*R[2]), gp_Dir(*R[0]))
    t = gp_Trsf(); t.SetTransformation(ax, gp_Ax3())
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def cyl(r, z0, z1, x=0.0, y=0.0):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, y, z0), gp_Dir(0, 0, 1)), r, z1 - z0).Shape()


def box(x0, y0, z0, x1, y1, z1):
    return BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), x1 - x0, y1 - y0, z1 - z0).Shape()


def new_motor_local(detail=True):
    """Động cơ mới, hệ cục bộ: trục +Z, mặt đầu ra hộp số z = 0, lỗ M2.5 ở các góc chéo (45°)."""
    gear = cyl(GEAR_D / 2, -GEAR_L, 0)
    s = MOTOR_BODY / 2
    body = box(-s, -s, -GEAR_L - MOTOR_L, s, s, -GEAR_L)
    if detail:  # vát 4 cạnh dọc thân như động cơ thật
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopAbs import TopAbs_EDGE
        from OCP.TopoDS import TopoDS
        ch = BRepFilletAPI_MakeChamfer(body)
        e = TopExp_Explorer(body, TopAbs_EDGE)
        while e.More():
            edge = TopoDS.Edge(e.Current())
            p = verts(edge)  # 2 đỉnh; cạnh dọc thân có cùng x, y
            if np.allclose(p[0, :2], p[-1, :2], atol=1e-6):
                ch.Add(4.0, edge)
            e.Next()
        body = ch.Shape()
    shaft = box(-SHAFT_W / 2, -SHAFT_W / 2, 0, SHAFT_W / 2, SHAFT_W / 2, SHAFT_L)
    m = BRepAlgoAPI_Fuse(BRepAlgoAPI_Fuse(gear, body).Shape(), shaft).Shape()
    if detail:
        for k in range(4):
            a = np.radians(45 + 90 * k)
            m = BRepAlgoAPI_Cut(m, cyl(HOLE_M / 2 * 0.85, -6, 0.01, HOLE_PCD / 2 * np.cos(a), HOLE_PCD / 2 * np.sin(a))).Shape()
    return m


def motor_envelope_local(clear=CLEAR):
    """Khối bao động cơ (để khoét chi tiết in): thân vuông + hộp số trụ, nới rộng `clear`."""
    s = MOTOR_BODY / 2 + clear
    body = box(-s, -s, -GEAR_L - MOTOR_L - clear, s, s, -GEAR_L + 0.01)
    gear = cyl(GEAR_D / 2 + clear, -GEAR_L - 0.01, 0)
    return BRepAlgoAPI_Fuse(body, gear).Shape()


RISER_WALL = 6.0                     # thành đế nâng (đế gốc chỉ ~3 mm)
TAB = (16.0, 18.0, 5.0)              # tai bắt vít: nhô ra, rộng, dày (mm)
TAB_HOLE_D = 4.5                     # lỗ vít M4 bắt xuống bàn


def make_riser(bottom_face, up, axis_point, height):
    """Đế nâng: khung theo đúng biên ngoài đáy AX-1_10, thành dày RISER_WALL, cao `height`, 4 tai M4."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
    from OCP.BRepOffsetAPI import BRepOffsetAPI_MakeOffset
    from OCP.BRepTools import BRepTools
    from OCP.GeomAbs import GeomAbs_Arc

    outer = BRepTools.OuterWire_s(bottom_face)
    filled = BRepBuilderAPI_MakeFace(outer, True).Face()
    off = BRepOffsetAPI_MakeOffset(filled, GeomAbs_Arc)
    off.Perform(-RISER_WALL)
    inner = BRepBuilderAPI_MakeFace(_first_wire(off.Shape()), True).Face()
    vec = gp_Vec(*(-up * height))
    frame = BRepAlgoAPI_Cut(BRepPrimAPI_MakePrism(filled, vec).Shape(),
                            BRepPrimAPI_MakePrism(inner, vec).Shape()).Shape()
    # 4 tai ở 2 cạnh thẳng (khung cục bộ quanh trục J1: u ngang, v dọc phần thẳng của chữ U)
    R, o = local_frame(axis_point, up)
    pts = (verts(filled) - o)
    u, v = pts @ R[0], pts @ R[1]
    w_bot = float(((verts(filled) - o) @ R[2]).mean()) - height
    tabs = None
    for su in (-1, 1):
        for vv in (v.min() + 0.45 * (v.max() - v.min()), v.max() - 15):
            x_edge = u.max() if su > 0 else u.min()
            x0, x1 = sorted([x_edge - su * 1.0, x_edge + su * TAB[0]])
            t = box(x0, vv - TAB[1] / 2, w_bot, x1, vv + TAB[1] / 2, w_bot + TAB[2])
            t = BRepAlgoAPI_Cut(t, cyl(TAB_HOLE_D / 2, w_bot - 1, w_bot + TAB[2] + 1,
                                       x_edge + su * TAB[0] * 0.55, vv)).Shape()
            tabs = t if tabs is None else BRepAlgoAPI_Fuse(tabs, t).Shape()
    return BRepAlgoAPI_Fuse(frame, to_frame(tabs, o, R)).Shape()


def _first_wire(shape):
    from OCP.TopAbs import TopAbs_WIRE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS
    return TopoDS.Wire(TopExp_Explorer(shape, TopAbs_WIRE).Current())


def frame_of(mf, w_shift=0.0):
    """Gốc & trục của động cơ cũ (mặt đầu ra tại w = 0) dịch dọc trục w_shift mm."""
    origin = mf.o + mf.R[2] * (mf.w0 + w_shift)
    return origin, mf.R


def write_step(shapes_named, path):
    """Ghi STEP lắp ráp có tên từng chi tiết (Fusion 360 hiện đúng tên component)."""
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    for name, shape in shapes_named:
        lab = st.AddShape(shape, False)
        TDataStd_Name.Set_s(lab, TCollection_ExtendedString(name))
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    w = STEPCAFControl_Writer(); w.SetNameMode(True)
    w.Transfer(doc, STEPControl_AsIs)
    assert w.Write(path) == IFSelect_RetDone, path


def write_stl(shape, path):
    BRepMesh_IncrementalMesh(shape, 0.05, False, 0.3, True)
    w = StlAPI_Writer(); w.ASCIIMode = False
    assert w.Write(shape, path), path


def collide(a, b) -> float:
    ba, bb = Bnd_Box(), Bnd_Box()
    BRepBndLib.Add_s(a, ba); BRepBndLib.Add_s(b, bb)
    if ba.IsOut(bb):
        return 0.0
    return vol(BRepAlgoAPI_Common(a, b).Shape())


def main() -> None:
    os.makedirs(OUT, exist_ok=True)
    report = []

    # ---------------- khung động cơ cũ ----------------
    mf2 = MotorFrame("STEPPERONLINE*NAUO156", 3.0, 18.0)   # J2 (PG5 + đai 20T→80T)
    mf3 = MotorFrame("STEPPERONLINE*NAUO123", 3.0, 18.0)   # J3 (PG5 + đai 20T→60T)
    mf1 = MotorFrame("Nema17*NAUO167", 2.5, 11.0)          # J1 (Nema17 + đai 20T→60T)
    # J1: mặt hộp số áp vào mặt dưới tấm gá AX-1_11 (= mặt bích Nema17 cũ, thấp hơn đỉnh gờ định tâm 1.9 mm)
    J1_SHIFT = -1.9
    o1, R1 = frame_of(mf1, J1_SHIFT)
    o2, R2 = frame_of(mf2)
    o3, R3 = frame_of(mf3)
    motor_local = new_motor_local()
    motors = {"J1": to_frame(motor_local, o1, R1), "J2": to_frame(motor_local, o2, R2), "J3": to_frame(motor_local, o3, R3)}

    # ---------------- 1. AX-2_2: khoét hốc đuôi động cơ J3 ----------------
    ax22 = load("AX-2_2__NAUO125")
    ax22_new = BRepAlgoAPI_Cut(ax22, to_frame(motor_envelope_local(), o3, R3)).Shape()
    report.append(f"AX-2_2: bớt {(vol(ax22) - vol(ax22_new)) / 1000:.2f} cm3 (hốc đuôi động cơ J3)")

    # ---------------- 2. AX-1_11: 4 lỗ M2.5 trên vòng Ø28 ----------------
    ax111 = load("AX-1_11__NAUO195")
    holes = None
    for k in range(4):
        a = np.radians(45 + 90 * k)  # cùng phương chéo với lỗ NEMA cũ (r = 21.9) — còn đủ thịt (kiểm tra bản đồ vật liệu)
        h = cyl(M25_CLEAR_D / 2, -3.0, 12.0, HOLE_PCD / 2 * np.cos(a), HOLE_PCD / 2 * np.sin(a))
        holes = h if holes is None else BRepAlgoAPI_Fuse(holes, h).Shape()
    ax111_new = BRepAlgoAPI_Cut(ax111, to_frame(holes, o1, R1)).Shape()
    # hộp số Ø35.9 áp mặt dưới tấm: khoét nhẹ phần nhựa chạm thân (nếu có) với khe hở
    ax111_new = BRepAlgoAPI_Cut(ax111_new, to_frame(motor_envelope_local(0.5), o1, R1)).Shape()
    report.append(f"AX-1_11: bớt {(vol(ax111) - vol(ax111_new)):.0f} mm3 (4 lỗ Ø{M25_CLEAR_D} trên vòng Ø{HOLE_PCD:.0f})")

    # ---------------- 3. Đế nâng J1 ----------------
    # trục J1 = trục ổ bi 78x60x10; đáy đế = mặt phẳng thấp nhất của AX-1_10 vuông góc trục J1
    lager = load("Lager_78x60x10__NAUO190")
    c = max(cylinders(lager), key=lambda c: c[0])
    up = c[2] / np.linalg.norm(c[2])
    if up @ R1[2] < 0:
        up = -up
    base = load("AX-1_10__NAUO194")
    bottom, wmin = None, 1e9
    for f in faces(base):
        s = BRepAdaptor_Surface(f)
        if s.GetType() != GeomAbs_Plane:
            continue
        n = s.Plane().Axis().Direction(); n = np.array([n.X(), n.Y(), n.Z()])
        if abs(abs(n @ up) - 1) > 1e-3:
            continue
        w = (verts(f) @ up).mean()
        if w < wmin:
            wmin, bottom = w, f
    motor_low = (verts(motors["J1"]) @ up).min()
    protrude = wmin - motor_low
    riser_h = float(np.ceil(protrude + RISER_GAP))
    riser = make_riser(bottom, up, c[1], riser_h)
    riser = BRepAlgoAPI_Cut(riser, to_frame(motor_envelope_local(3.0), o1, R1)).Shape()
    report.append(f"De_nang_J1: động cơ J1 mới thò {protrude:.1f} mm dưới đáy đế → đế nâng cao {riser_h:.0f} mm "
                  f"(khe hở đuôi động cơ ≥ {riser_h - protrude:.1f} mm), thể tích {vol(riser) / 1000:.0f} cm3")

    # ---------------- kiểm tra va chạm lắp ráp mới ----------------
    replaced = {"AX-2_2__NAUO125": ax22_new, "AX-1_11__NAUO195": ax111_new}
    removed = ("NAUO156", "NAUO123", "NAUO167")  # 2 PG5 + Nema17 J1 cũ
    # vít M3 bắt PG5 cũ vào AX-1_6 (sẽ thay bằng M2.5) và puly 16 mm (lỗ trục phải đổi) bỏ qua khi kiểm tra
    skip_check = ("ISK", "Mutter", "Riemenscheibe_16mm")
    parts = []
    for f in sorted(glob.glob(os.path.join(CACHE, "*.brep"))):
        n = os.path.basename(f)[:-5]
        if any(r in n for r in removed):
            continue
        parts.append((n, replaced.get(n) or load(n)))
    parts.append(("De_nang_J1", riser))
    for j, m in motors.items():
        hits = [(collide(m, s), n) for n, s in parts if not any(k in n for k in skip_check)]
        hits = [(v, n) for v, n in hits if v > 1.0]
        report.append(f"Kiểm tra va chạm động cơ {j}: " + ("KHÔNG va chạm" if not hits else
                      ", ".join(f"{n} {v:.0f} mm3" for v, n in hits)))
    t_ax22 = min(collide(motors["J3"], ax22_new), 1e9)
    report.append(f"AX-2_2 mới ∩ động cơ J3: {t_ax22:.1f} mm3")

    # ---------------- xuất file ----------------
    outs = {"AX-2_2_sua_J3": ax22_new, "AX-1_11_sua_J1_M2.5": ax111_new, "De_nang_J1": riser,
            "Dong_co_42STH38_hop_so_1_27": motor_local}
    for name, shape in outs.items():
        write_step([(name, shape)], os.path.join(OUT, name + ".step"))
        write_stl(shape, os.path.join(OUT, name + ".stl"))
    assembly = [(n.split("__")[0] + ("_sua" if n in replaced else ""), s) for n, s in parts]
    assembly += [(f"Dong_co_moi_{j}", m) for j, m in motors.items()]
    write_step(assembly, os.path.join(OUT, "Roboter_dong_co_1_27.step"))

    with open(os.path.join(OUT, "ket_qua_kiem_tra.txt"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(report) + "\n")
    print("\n".join(report))
    print("Đã ghi:", OUT)


if __name__ == "__main__":
    main()
