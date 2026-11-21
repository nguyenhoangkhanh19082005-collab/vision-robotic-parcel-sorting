r"""Hàm dùng chung cho việc rút ngắn cẳng tay (rut_ngan_cang_tay.py, rut_ngan_kiem_tra.py).

Hệ cục bộ của cẳng tay (mm): gốc E = điểm trên trục J4 gần trục J3 nhất, Z = f = hướng trục J4 (khuỷu → cổ tay),
X = trục J3 (khâu 3) hoặc trục J5 (khâu 4), Y = Z × X. "s" = tọa độ dọc trục J4 tính từ trục J3.
Chi tiết đọc từ .tmp/brep_1_27 (tạo bằng rut_ngan_xuat_brep.py), số thứ tự trùng .tmp/step_lap_rap.json.
"""
import glob, json, os, sys
import numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "research")); sys.path.insert(0, os.path.dirname(__file__))
from cad_util import bbox, cylinders, planes  # noqa: E402
from OCP.BRep import BRep_Builder
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse, BRepAlgoAPI_Section
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.BRepTools import BRepTools
from OCP.GCPnts import GCPnts_QuasiUniformDeflection
from OCP.GProp import GProp_GProps
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.TopAbs import TopAbs_EDGE, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Shape
from OCP.gp import gp_Ax1, gp_Ax2, gp_Ax3, gp_Dir, gp_Pln, gp_Pnt, gp_Trsf, gp_Vec
try:
    from OCP.TopTools import TopTools_ListOfShape
except ImportError:  # OCP 8: các kiểu collection chuyển sang OCP.collections
    from OCP.collections import List_TopoDS_Shape as TopTools_ListOfShape

parts = json.load(open(os.path.join(ROOT, ".tmp", "step_lap_rap.json"), encoding="utf-8"))
BREP = os.path.join(ROOT, ".tmp", "brep_1_27")


def _axis_of(pred):
    p = next(q for q in parts if pred(q)); c = max(p["cyl"], key=lambda c: c["area"]); d = np.array(c["d"])
    return np.array(c["p"]), d / np.linalg.norm(d)


def _closest(pa, da, pb, db):
    w = pa - pb; a = da @ da; b = da @ db; c = db @ db; d = da @ w; e = db @ w
    return pa + (b * e - c * d) / (a * c - b * b) * da


# trục khớp trong hệ STEP (điểm, hướng)
AXES = [_axis_of(lambda q: q["name"].startswith("Lager_78")), _axis_of(lambda q: q["name"].startswith("Puly_55")),
        _axis_of(lambda q: q["name"].startswith("Puly_40")), _axis_of(lambda q: q["name"].startswith("STEPPERONLINE")),
        _axis_of(lambda q: q["name"].startswith("Riemenscheibe_40mm") and q["com"][0] > 0),
        _axis_of(lambda q: q["name"].startswith("Nema17") and q["com"][1] < -70)]
(P2, D2), (P3, D3), (P4, D4), (P5, D5) = AXES[1], AXES[2], AXES[3], AXES[4]
E = _closest(P4, D4, P3, D3)                 # khuỷu: điểm trên trục J4 gần trục J3 nhất
WRIST = _closest(P4, D4, P5, D5)             # tâm cổ tay
F = (WRIST - E) / np.linalg.norm(WRIST - E)  # hướng cẳng tay
X3 = D3; Y3 = np.cross(F, X3)                                # hệ khâu 3 (X = hướng trục J3 như trong STEP: +X là phía puly J3)
X4 = D5 - (D5 @ F) * F; X4 /= np.linalg.norm(X4); Y4 = np.cross(F, X4)   # hệ khâu 4
R3 = np.array([X3, Y3, F]); R4 = np.array([X4, Y4, F])


def loadi(i):
    f = glob.glob(os.path.join(BREP, f"{i:03d}_*.brep"))
    assert f, f"thiếu {i} trong {BREP} — chạy src/cad/rut_ngan_xuat_brep.py trước"
    s = TopoDS_Shape(); BRepTools.Read_s(s, f[0], BRep_Builder()); return s


def vol(shape):
    g = GProp_GProps(); BRepGProp.VolumeProperties_s(shape, g); return g.Mass()


def area(shape):
    g = GProp_GProps(); BRepGProp.SurfaceProperties_s(shape, g); return g.Mass()


def com(shape):
    g = GProp_GProps(); BRepGProp.VolumeProperties_s(shape, g); c = g.CentreOfMass(); return np.array([c.X(), c.Y(), c.Z()])


def n_solids(shape):
    n = 0; ex = TopExp_Explorer(shape, TopAbs_SOLID)
    while ex.More(): n += 1; ex.Next()
    return n


def valid(shape):
    return BRepCheck_Analyzer(shape).IsValid()


def to_frame(shape, R, origin=None):
    """Hệ cục bộ (x=R[0], y=R[1], z=R[2], gốc origin) → hệ lắp ráp."""
    origin = E if origin is None else origin
    t = gp_Trsf(); t.SetTransformation(gp_Ax3(gp_Pnt(*origin), gp_Dir(*R[2]), gp_Dir(*R[0])), gp_Ax3())
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def to_local(shape, R, origin=None):
    """Hệ lắp ráp → hệ cục bộ (để xuất STL nằm dọc trục Z)."""
    origin = E if origin is None else origin
    t = gp_Trsf(); t.SetTransformation(gp_Ax3(), gp_Ax3(gp_Pnt(*origin), gp_Dir(*R[2]), gp_Dir(*R[0])))
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def shifted(shape, d):
    """Tịnh tiến d mm về phía trục J3 (ngược hướng cẳng tay)."""
    t = gp_Trsf(); t.SetTranslation(gp_Vec(*(-F * d)))
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def rotated_about_f(shape, deg):
    t = gp_Trsf(); t.SetRotation(gp_Ax1(gp_Pnt(*E), gp_Dir(*F)), np.radians(deg))
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def lbox(R, x0, x1, y0, y1, s0, s1):
    return to_frame(BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, s0), x1 - x0, y1 - y0, s1 - s0).Shape(), R)


def lcyl(R, r, x, y, s0, s1):
    return to_frame(BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, y, s0), gp_Dir(0, 0, 1)), r, s1 - s0).Shape(), R)


def _bool(cls, a, b, fuzzy):
    op = cls(); la = TopTools_ListOfShape(); la.Append(a); lb = TopTools_ListOfShape(); lb.Append(b)
    op.SetArguments(la); op.SetTools(lb)
    if fuzzy: op.SetFuzzyValue(fuzzy)
    op.Build(); assert op.IsDone(), cls.__name__
    return op.Shape()


def common(a, b, fuzzy=0.0): return _bool(BRepAlgoAPI_Common, a, b, fuzzy)
def cut(a, b, fuzzy=0.0): return _bool(BRepAlgoAPI_Cut, a, b, fuzzy)
def fuse(a, b, fuzzy=0.0): return _bool(BRepAlgoAPI_Fuse, a, b, fuzzy)


def limit_tol(shape, tol=1e-3):
    """Siết dung sai đỉnh/cạnh. File STEP gốc có chi tiết (AX-4_4) dung sai đỉnh tới 4,9 mm → phép cộng/trừ khối coi các
    mặt cách nhau vài mm là trùng nhau và trả kết quả rỗng. Sửa tại chỗ, trả lại chính hình đó."""
    from OCP.ShapeFix import ShapeFix_ShapeTolerance
    ShapeFix_ShapeTolerance().LimitTolerance(shape, 1e-7, tol)
    return shape


def max_tol(shape):
    from OCP.ShapeAnalysis import ShapeAnalysis_ShapeTolerance
    from OCP.TopAbs import TopAbs_VERTEX
    return ShapeAnalysis_ShapeTolerance().Tolerance(shape, 1, TopAbs_VERTEX)


def mesh_check(stl_path):
    """(số tam giác, số cạnh hở, đa tạp?, thể tích cm3) của lưới STL — lưới in được phải kín (0 cạnh hở)."""
    import pyvista as pv
    m = pv.read(stl_path).clean(tolerance=1e-6)
    return m.n_cells, m.n_open_edges, bool(m.is_manifold), abs(m.volume) / 1000


def unify(shape):
    u = ShapeUpgrade_UnifySameDomain(shape, True, True, True); u.Build(); return u.Shape()


BIG = 400.0


def keep(shape, s0, s1, R=None):
    """Phần của chi tiết nằm trong s0 ≤ s ≤ s1."""
    return common(shape, lbox(R3 if R is None else R, -BIG, BIG, -BIG, BIG, s0, s1))


def slab_cut(shape, s0, s1):
    """Bỏ lát s0..s1 (tiết diện ở hai mặt cắt phải giống nhau), kéo phần xa về phía trục J3 cho khít rồi hàn lại."""
    prox = keep(shape, -BIG, s0); dist = shifted(keep(shape, s1, BIG), s1 - s0)
    return unify(fuse(prox, dist, 1e-4))


def dist_min(a, b):
    return BRepExtrema_DistShapeShape(a, b).Value()


def interference(a, b):
    """Thể tích giao nhau (mm3); 0 nếu hộp bao không chạm."""
    la, ha = bbox(a); lb, hb = bbox(b)
    if np.any(la > hb) or np.any(lb > ha): return 0.0
    try:
        return vol(common(a, b))
    except Exception:
        return float("nan")


def section_edges(shape, s, R=None):
    """Các cạnh của mặt cắt ngang tại s → danh sách mảng điểm (hệ cục bộ x, y)."""
    R = R3 if R is None else R
    sec = BRepAlgoAPI_Section(shape, gp_Pln(gp_Pnt(*(E + F * s)), gp_Dir(*F)), False); sec.Build()
    out = []; ex = TopExp_Explorer(sec.Shape(), TopAbs_EDGE)
    while ex.More():
        c = BRepAdaptor_Curve(TopoDS.Edge(ex.Current())); d = GCPnts_QuasiUniformDeflection(c, 0.01)
        p = np.array([[d.Value(i).X(), d.Value(i).Y(), d.Value(i).Z()] for i in range(1, d.NbPoints() + 1)]) - E
        out.append(np.column_stack([p @ R[0], p @ R[1]])); ex.Next()
    return out


def loops(edges, tol=0.02):
    """Nối các cạnh rời thành vòng kín (mảng điểm x, y)."""
    edges = [e for e in edges if len(e) >= 2]; out = []
    while edges:
        cur = edges.pop(0)
        while np.linalg.norm(cur[0] - cur[-1]) > tol or len(cur) < 4:
            for k, e in enumerate(edges):
                if np.linalg.norm(e[0] - cur[-1]) < tol: cur = np.vstack([cur, e[1:]]); edges.pop(k); break
                if np.linalg.norm(e[-1] - cur[-1]) < tol: cur = np.vstack([cur, e[::-1][1:]]); edges.pop(k); break
            else:
                break
        if np.linalg.norm(cur[0] - cur[-1]) <= tol: out.append(cur[:-1])
    return out


def resample(loop, n):
    """Lấy n điểm cách đều theo chiều dài cung, ngược chiều kim đồng hồ, bắt đầu ở giao với tia +x qua tâm vòng."""
    p = np.asarray(loop, float)
    a = 0.5 * np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1])
    if a < 0: p = p[::-1]
    c = (p.min(0) + p.max(0)) / 2; q = np.vstack([p, p[:1]])
    # điểm bắt đầu: cắt tia y = c_y, x > c_x
    start = None
    for i in range(len(p)):
        y0, y1 = q[i, 1] - c[1], q[i + 1, 1] - c[1]
        if y0 <= 0 < y1 and max(q[i, 0], q[i + 1, 0]) > c[0]:
            t = -y0 / (y1 - y0); start = (i, q[i] + t * (q[i + 1] - q[i])); break
    assert start is not None
    i, p0 = start
    chain = np.vstack([p0, q[i + 1:-1], q[:i + 1], p0])
    seg = np.linalg.norm(np.diff(chain, axis=0), axis=1); L = np.concatenate([[0], np.cumsum(seg)])
    t = np.linspace(0, L[-1], n, endpoint=False)
    return np.column_stack([np.interp(t, L, chain[:, 0]), np.interp(t, L, chain[:, 1])])
