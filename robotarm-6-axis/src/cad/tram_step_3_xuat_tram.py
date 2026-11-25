r"""Bước 3/3: ghép toàn bộ trạm phân loại thành MỘT file STEP lắp ráp (khối đặc B-rep, đơn vị mm, hệ BASE robot:
gốc ở chân đế trên trục J1, Z lên, X hướng ra băng tải) → assets/CAD_tram_tong/Tram_phan_loai_tong.step

Thành phần (nhóm = component cấp 1 trong STEP):
  Robot_6_truc   194 chi tiết lắp ráp v3, đặt ở tư thế q của bước 2 (mặc định HOME = 0, dựng thẳng đứng)
  Bang_tai       dựng lại theo SortingStation/sorting_station/conveyor_model.py + station.yaml (layout.conveyor)
  Thung_*        3 thùng hở miệng theo layout.bins;  Kien_hang   kiện mẫu tại điểm gắp (layout.pick_y)
  Mat_ban        tấm đế (ƯỚC LƯỢNG) tại cao độ mặt đặt băng tải
  Den_ring       đầu đèn ring từ file OBJ người dùng (bỏ chân tripod), lưới → khối đặc nhiều mặt phẳng
  Webcam         C260 WebCam Assembly.STEP (người dùng cung cấp), đặt giữa vòng đèn, ống kính nhìn xuống
Chạy sau bước 1, 2:
    E:\UDMT\.venv\Scripts\python.exe -X utf8 src\cad\tram_step_3_xuat_tram.py
"""
import json, math, os, sys
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, r"E:\UDMT\SortingStation")
from tram_step_1_doc_robot import read_parts, SRC as ROBOT_STEP
from sorting_station.conveyor_model import ConveyorSpec

from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeSolid,
                                BRepBuilderAPI_Sewing, BRepBuilderAPI_Transform)
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_ColorSurf, XCAFDoc_DocumentTool

OUT_DIR = os.path.join(ROOT, "assets", "CAD_tram_tong"); os.makedirs(OUT_DIR, exist_ok=True)
TMP = os.path.join(ROOT, ".tmp", "tram_step")
STATION_YAML = r"E:\UDMT\SortingStation\config\station.yaml"
NGUON = os.path.join(OUT_DIR, "nguon")                       # file nguồn người dùng cung cấp (chép vào dự án)
WEBCAM_STEP = os.path.join(NGUON, "Webcam_C260.step")
RING_OBJ = os.path.join(NGUON, "Den_ring.obj")
GIA_STEP = os.path.join(NGUON, "Gia_camera_den_cam_bien.step")  # PHANLOAIHANG (Fusion 360) của nhóm

# ---- Giả định chưa đo (sửa ở đây rồi chạy lại) ----
RING_OUTER_MM = 200.0        # đường kính ngoài vòng đèn (23/11/2026: 260 → 200 theo yêu cầu, ƯỚC LƯỢNG)
TABLE_SIZE = (900.0, 800.0, 10.0)   # tấm đế (ƯỚC LƯỢNG), tâm XY đặt giữa robot và băng tải
# Gá (hệ file STEP gá, mm): X dọc băng (+X về CUỐI băng, phía cảm biến), Y ngang băng (2 chân ở Y=0 và Y=124,
# tâm khoảng trống Y=62.5), Z lên, đáy chân Z=-15. Đầu thanh đế phía cảm biến (X=+92.5) trùng cuối băng tải.
GIA_FOOT_END_X = 92.5
GIA_BOTTOM_Z = -15.0
GIA_MID_Y = 62.5
CAM_X = -35.0                # tâm ống kính webcam trên gá: nhích về ĐẦU băng so với thanh ngang (X=0) — ƯỚC LƯỢNG
ARM_TOP_Z, ARM_BOT_Z = 297.0, 292.0   # tay đòn trên cùng của gá (khối s6)
SENSOR_AXIS = (68.1, 99.2)   # tâm vòng kẹp M18 (X, Z) trên gá, trục theo Y
SENSOR_STEP = os.path.join(NGUON, "Cam_bien_E18-D80NK.step")   # E18-D80NK (CC BY-4.0, M. Rafli Ardiansyah)
NUT_STEP = os.path.join(NGUON, "Dai_oc_E18-D80NK.step")
SENSOR_FRONT_YG = 14.0       # mặt mắt quang nhô vào trong vòng kẹp tới Y gá = 14 mm (vòng kẹp Y gá −5..5)


# ---------------------------------------------------------------- tiện ích hình học
def trsf_from(T):
    t = gp_Trsf(); T = np.asarray(T, float)
    t.SetValues(*T[0, :4], *T[1, :4], *T[2, :4]); return t


def moved(shape, T):
    return BRepBuilderAPI_Transform(shape, trsf_from(T), True).Shape()


def box(x0, x1, y0, y1, z0, z1):
    return BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), gp_Pnt(x1, y1, z1)).Shape()


def cyl(center, direction, r, h):
    d = np.asarray(direction, float) / np.linalg.norm(direction); c = np.asarray(center, float) - d * h / 2
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*c), gp_Dir(*d)), r, h).Shape()


def mesh_solid(V, F):
    """Lưới tam giác kín → khối đặc (mặt phẳng). Lưới hở thì trả về vỏ (shell) đã khâu."""
    sew = BRepBuilderAPI_Sewing(1e-3)
    for f in F:
        p = [gp_Pnt(*V[i]) for i in f]
        if np.linalg.norm(np.cross(V[f[1]] - V[f[0]], V[f[2]] - V[f[0]])) < 1e-9:
            continue
        poly = BRepBuilderAPI_MakePolygon(*p, True)
        face = BRepBuilderAPI_MakeFace(poly.Wire(), True)
        if face.IsDone():
            sew.Add(face.Face())
    sew.Perform(); sh = sew.SewedShape()
    try:
        ms = BRepBuilderAPI_MakeSolid(TopoDS.Shell(sh))
        if ms.IsDone():
            return ms.Shape()
    except Exception:
        pass
    return sh


def hexcol(h):
    h = h.lstrip("#"); return tuple(int(h[i:i + 2], 16) / 255 for i in (0, 2, 4))


# ---------------------------------------------------------------- các thành phần
CUP_DUMMY = "Giac_hut_L60-M12_HSP-40_mo_phong"


def tcp_mm(q_deg):
    """Khung TCP (miệng giác hút, z hướng ra ngoài) ở tư thế q, mm, hệ base — cùng RobotModel của twin."""
    from sorting_station.station_config import install_import_paths, load_station_config, resolve_paths
    cfg = load_station_config(); install_import_paths(resolve_paths(cfg))
    from sorting_station.robot_model import build_model_from_robot_project
    model = build_model_from_robot_project(resolve_paths(cfg).robot_root, tool_length_m=0.126)
    T = np.array(model.fk_deg(q_deg), float); T[:3, 3] *= 1000.0
    return T


def robot_group():
    place = json.load(open(os.path.join(TMP, "robot_dat.json"), encoding="utf-8"))
    parts = read_parts(ROBOT_STEP)
    assert len(parts) == len(place["parts"])
    cols = ["#9aa4ad", "#e8eaed", "#e8eaed", "#d8dde2", "#e8eaed", "#d8dde2", "#3b82f6"]
    out = []
    tcp = tcp_mm(place["q_deg"])
    for (name, shp), pl in zip(parts, place["parts"]):
        assert name == pl["name"]
        if name == CUP_DUMMY:                   # 24/11/2026: khối nón mô phỏng → giác hút L60-M12 + DP-40 chi tiết
            out += [(n, moved(sh, tcp), c) for n, sh, c in suction_cup_parts()]
            continue
        c = "#2b2f33" if name.startswith(("Nema17", "Dong_co", "STEPPERONLINE")) else cols[pl["link"]]
        out.append((name, moved(shp, pl["T"]), c))
    return out, place["q_deg"]


def station_layout():
    cfg = yaml.safe_load(open(STATION_YAML, encoding="utf-8"))
    lay = cfg["layout"]; conv = lay["conveyor"]
    spec = ConveyorSpec.from_config(conv.get("model"))
    cx, cy = (1000.0 * float(v) for v in conv["center"])
    base_z = 1000.0 * float(conv.get("base_z", 0.0))
    return cfg, lay, spec, cx, cy, base_z


def conveyor_group(spec, cx, cy, bz):
    s = spec; hw = s.belt_width / 2; ry, rz = s.roller_y, s.roller_center_z; r_out = s.roller_radius + s.belt_thickness
    P = []
    belt = box(-hw, hw, -ry, ry, s.belt_top_height - s.belt_thickness, s.belt_top_height)
    P.append(("Bang_tai_mat_tren", belt, "#1f6b3a"))
    P.append(("Bang_tai_nhanh_hoi", box(-hw, hw, -ry, ry, rz - r_out, rz - s.roller_radius), "#1f6b3a"))
    for sign, lab in ((1, "ra"), (-1, "vao")):
        ring = BRepAlgoAPI_Cut(cyl((0, sign * ry, rz), (1, 0, 0), r_out, s.belt_width),
                               cyl((0, sign * ry, rz), (1, 0, 0), s.roller_radius, s.belt_width + 2)).Shape()
        half = box(-hw - 1, hw + 1, (sign * ry) if sign > 0 else (sign * ry - r_out - 1),
                   (sign * ry + r_out + 1) if sign > 0 else (sign * ry), rz - r_out - 1, rz + r_out + 1)
        from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
        P.append((f"Bang_tai_om_rulo_{lab}", BRepAlgoAPI_Common(ring, half).Shape(), "#1f6b3a"))
        P.append((f"Rulo_{lab}", cyl((0, sign * ry, rz), (1, 0, 0), s.roller_radius, s.belt_width + 2), "#b9c0c7"))
    rail_y = s.length_total / 2 - s.cap_length; rail_top = s.belt_top_height - s.belt_thickness - 1
    for side, lab in ((-1, "trai"), (1, "phai")):
        inner = hw + 1
        x0, x1 = sorted((side * inner, side * (inner + s.rail_thickness)))
        P.append((f"Thanh_nhom_{lab}", box(x0, x1, -rail_y, rail_y, rail_top - s.rail_height, rail_top), "#b9c0c7"))
        x0, x1 = sorted((side * inner, side * (inner + s.cap_thickness)))
        for end, el in ((-1, "vao"), (1, "ra")):
            yi = end * (s.length_total / 2 - s.cap_length); y0, y1 = sorted((yi, end * ry))
            P.append((f"Nap_dau_{lab}_{el}", box(x0, x1, y0, y1, rz - s.cap_height / 2, rz + s.cap_height / 2), "#6b7076"))
            P.append((f"Nap_dau_{lab}_{el}_bo_tron", cyl(((x0 + x1) / 2, end * ry, rz), (1, 0, 0), s.cap_height / 2, x1 - x0), "#6b7076"))
    leg_top = rail_top - s.rail_height
    for y in s.leg_positions:
        for side, lab in ((-1, "trai"), (1, "phai")):
            inner = hw + 1
            x0, x1 = sorted((side * inner, side * (inner - s.leg_thickness)))
            P.append((f"Chan_{lab}_{y:+.0f}", box(x0, x1, y - s.leg_width / 2, y + s.leg_width / 2, 0, leg_top + 10), "#9aa3ab"))
            fx0, fx1 = sorted((side * inner, side * (inner - s.leg_foot_length)))
            P.append((f"Chan_{lab}_{y:+.0f}_de", box(fx0, fx1, y - s.leg_width / 2, y + s.leg_width / 2, 0, s.leg_thickness), "#9aa3ab"))
    T = np.eye(4); T[:3, 3] = (cx, cy, bz)
    return [(n, moved(sh, T), c) for n, sh, c in P]


def bins_group(lay):
    out = []
    for key, b in lay["bins"].items():
        bx, by, bz = (1000.0 * float(v) for v in b["center"]); sx, sy, sz = (1000.0 * float(v) for v in b["size"])
        t = 4.0
        outer = box(bx - sx / 2, bx + sx / 2, by - sy / 2, by + sy / 2, bz, bz + sz)
        inner = box(bx - sx / 2 + t, bx + sx / 2 - t, by - sy / 2 + t, by + sy / 2 - t, bz + t, bz + sz + 1)
        out.append((f"Thung_{key}", BRepAlgoAPI_Cut(outer, inner).Shape(), b.get("color", "#8b9aab")))
    return out


def gia_transform(cx, conv_end_y, table_z):
    """Ma trận hệ gá (mm) → hệ base (mm): X gá → +Y base (dọc băng về cuối), Y gá → ∓X base (ngang băng), Z giữ.
    Tâm khoảng trống giữa 2 chân (Y gá = 62.5) nằm trên đường tâm băng; đầu thanh đế X = +92.5 trùng cuối băng;
    đáy chân (Z = -15) đặt trên mặt bàn."""
    T = np.eye(4)
    T[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)   # quay 90° quanh Z (vòng kẹp cảm biến ở phía +X base)
    T[:3, 3] = (cx + GIA_MID_Y, conv_end_y - GIA_FOOT_END_X, table_z - GIA_BOTTOM_Z)
    return T


def gia_group(T):
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_SOLID
    names = ["Thanh_de_phia_cam_bien", "Chan_khung_1", "Thanh_ngang_tren", "Thanh_de_2", "Chan_khung_2",
             "Gia_cam_bien_M18", "Tay_don_webcam_den"]
    out = []
    for _n, shp in read_parts(GIA_STEP):
        e = TopExp_Explorer(shp, TopAbs_SOLID); k = 0
        while e.More():
            out.append((f"Gia_{names[k] if k < len(names) else k}", moved(e.Current(), T), "#7a7f86")); k += 1; e.Next()
    return out


def at(T, p):
    return (T @ np.append(np.asarray(p, float), 1.0))[:3]


def sensor_group(T):
    """Cảm biến quang E18-D80NK (STEP thật: trục X, mắt quang ở X = +2.5, thân ren M18 tới X = −53.8) luồn qua vòng kẹp
    Ø18.5 của gá, mắt quang hướng vào băng (+Y gá); 2 đai ốc kẹp hai mặt vòng kẹp."""
    x, z = SENSOR_AXIS
    R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)      # X cảm biến → +Y gá, Y cảm biến → −X gá, Z giữ
    def place(shape, x_s):                                          # điểm (x_s, 0, 0) của cảm biến ↔ trục vòng kẹp
        L = np.eye(4); L[:3, :3] = R; L[:3, 3] = (x, x_s, z)
        return moved(shape, T @ L)
    out = []
    for n, shp in read_parts(SENSOR_STEP):
        L = np.eye(4); L[:3, :3] = R; L[:3, 3] = (x, SENSOR_FRONT_YG - 2.5, z)
        out.append(("Cam_bien_E18-D80NK", moved(shp, T @ L), "#f2b705"))
    nut = read_parts(NUT_STEP)[0][1]                                # đai ốc dày 8.8 mm, tâm tại gốc
    for i, yg in enumerate((5.0 + 4.4, -5.0 - 4.4)):
        L = np.eye(4); L[:3, :3] = R; L[:3, 3] = (x, yg, z)
        out.append((f"Dai_oc_E18_{i + 1}", moved(nut, T @ L), "#1b1d20"))
    return out


def webcam_group(T):
    """Webcam C260: ống kính nhìn thẳng xuống, kẹp + thân treo dưới tay đòn trên cùng của gá, tâm ống kính tại
    (CAM_X, giữa 2 chân) — nhích về ĐẦU băng so với thanh ngang để thanh ngang không che."""
    parts = read_parts(WEBCAM_STEP)
    Tg = T[:3, :3]
    # hệ webcam: +Z = hướng ống kính, +Y = lên (thân trên kẹp). Ống kính → -Z base; +Y webcam → về ĐẦU băng (-X gá)
    ex, ey, ez = Tg @ [0, -1, 0], Tg @ [-1, 0, 0], np.array([0.0, 0.0, -1.0])
    ex = np.cross(ey, ez)
    R = np.column_stack([ex, ey, ez])
    # webcam cao nhất ở z_webcam = -84.8 (đuôi kẹp) → sau xoay là điểm cao nhất; đặt sát đáy tay đòn
    lens = np.array([0.0, 19.0, 3.0])                     # tâm mặt ống kính (hệ webcam, ƯỚC LƯỢNG từ hộp bao)
    top_drop = 3.0 + 84.8                                   # từ mặt ống kính lên đỉnh đuôi kẹp
    target = at(T, (CAM_X, GIA_MID_Y, ARM_BOT_Z - top_drop))
    W = np.eye(4); W[:3, :3] = R; W[:3, 3] = target - R @ lens
    return [(f"Webcam_{n}", moved(s, W), "#1b1d20") for n, s in parts], target


def ring_light_group(T, cam_lens):
    """Vòng đèn ring (OBJ, vùng 0 = vòng) nằm ngang, đồng tâm webcam khi nhìn từ trên, đáy vòng sát mặt trên tay đòn
    (ngang tầm webcam). Bỏ ống đứng, kẹp, chân tripod. Thang đo theo RING_OUTER_MM."""
    import pyvista as pv
    m = pv.read(RING_OBJ).clean()
    conn = m.connectivity(extraction_mode="all")
    sub = conn.threshold([0, 0], scalars="RegionId").extract_surface(algorithm="dataset_surface").triangulate()
    p = np.asarray(sub.points, float); f = sub.faces.reshape(-1, 4)[:, 1:]
    k = RING_OUTER_MM / (p[:, 1].max() - p[:, 1].min())
    # mặt phẳng vòng: pháp tuyến = véc-tơ riêng nhỏ nhất của hiệp phương sai
    c0 = p.mean(0); w, v = np.linalg.eigh(np.cov((p - c0).T)); n = v[:, 0]
    a = np.cross(n, [0, 0, 1.0]); s = np.linalg.norm(a); cang = n[2]
    if s < 1e-9:
        R = np.eye(3)
    else:
        a /= s; K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        R = np.eye(3) + K * s + K @ K * (1 - cang)              # quay n → +Z
    q = (R @ (p - c0).T).T * k
    zmin = q[:, 2].min()
    arm_top = at(T, (0, 0, ARM_TOP_Z))[2]
    V = q + np.array([cam_lens[0], cam_lens[1], arm_top - zmin + 1.0])
    return [("Den_ring_vong", mesh_solid(V, f), "#f4f4f4")], V.mean(0), k


# ---------------------------------------------------------------- ghi STEP có cây lắp ráp + màu
def write_assembly(groups, path):
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()); ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    top = st.NewShape(); TDataStd_Name.Set_s(top, TCollection_ExtendedString("Tram_phan_loai_tong"))
    for gname, items in groups:
        g = st.NewShape(); TDataStd_Name.Set_s(g, TCollection_ExtendedString(gname))
        for name, shp, col in items:
            lab = st.AddShape(shp, False)
            TDataStd_Name.Set_s(lab, TCollection_ExtendedString(name))
            ct.SetColor(lab, Quantity_Color(*hexcol(col), Quantity_TOC_RGB), XCAFDoc_ColorSurf)
            comp = st.AddComponent(g, lab, TopLoc_Location())
            TDataStd_Name.Set_s(comp, TCollection_ExtendedString(name))
        comp = st.AddComponent(top, g, TopLoc_Location()); TDataStd_Name.Set_s(comp, TCollection_ExtendedString(gname))
    st.UpdateAssemblies()
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    Interface_Static.SetCVal_s("write.step.schema", "AP214IS")
    w = STEPCAFControl_Writer(); w.SetNameMode(True); w.SetColorMode(True)
    assert w.Transfer(doc, STEPControl_AsIs)
    assert w.Write(path) == IFSelect_RetDone, path


def cone(z0, z1, r0, r1):
    """Nón cụt trục Z từ z0 (bán kính r0) tới z1 (bán kính r1), z1 > z0."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCone
    return BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(0, 0, z0), gp_Dir(0, 0, 1)), r0, r1, z1 - z0).Shape()


def hexagon(z0, z1, af):
    """Lục giác đều cạnh đối cạnh `af`, trục Z, cao z0..z1."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
    r = af / math.sqrt(3.0)
    poly = BRepBuilderAPI_MakePolygon()
    for k in range(6):
        a = math.radians(60 * k)
        poly.Add(gp_Pnt(r * math.cos(a), r * math.sin(a), z0))
    poly.Close()
    face = BRepBuilderAPI_MakeFace(poly.Wire(), True).Face()
    return BRepPrimAPI_MakePrism(face, gp_Vec(0, 0, z1 - z0)).Shape()


def suction_cup_parts():
    """Giác hút L60-M12 + HSP-40-2 (DP-40) theo ảnh shop (mm): hệ cục bộ gốc tại MIỆNG giác, +Z hướng ra ngoài
    (= trục z của TCP RobotModel), thân đi về −Z. Tổng 74 = cốc 22 + thân 52 (ren M12×1 15 mm, lò xo 10 mm, núm Ø6)."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
    P = []
    # cốc silicon 2 tầng: miệng Ø40 → cổ Ø16
    cup = BRepAlgoAPI_Fuse(cone(-6, 0, 14.0, 20.0), cone(-13, -6, 12.5, 14.0)).Shape()
    cup = BRepAlgoAPI_Fuse(cup, cone(-18, -13, 13.0, 12.5)).Shape()
    cup = BRepAlgoAPI_Fuse(cup, cone(-22, -18, 8.0, 13.0)).Shape()
    cup = BRepAlgoAPI_Cut(cup, cone(-19, 0.01, 6.0, 18.5)).Shape()       # lòng cốc
    P.append(("Giac_hut_coc_DP-40", cup, "#e3ecf2"))
    P.append(("Giac_hut_luc_giac_duoi", hexagon(-32, -22, 12.0), "#c0c4c8"))
    P.append(("Giac_hut_truc_truot", cyl((0, 0, -44), (0, 0, 1), 4.0, 24.0), "#c0c4c8"))
    for k in range(5):                                                       # lò xo: 5 vòng dây Ø1.2
        t = gp_Trsf(); t.SetTranslation(gp_Vec(0, 0, -44 + 2.0 * k))
        ring = BRepPrimAPI_MakeTorus(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 5.6, 0.6).Shape()
        P.append((f"Giac_hut_lo_xo_{k + 1}", BRepBuilderAPI_Transform(ring, t, True).Shape(), "#9aa3ab"))
    P.append(("Giac_hut_than_ren_M12x1", cyl((0, 0, -55), (0, 0, 1), 6.0, 18.0), "#c0c4c8"))
    P.append(("Giac_hut_dai_oc_M12_ngoai", hexagon(-52, -46, 19.0), "#c0c4c8"))
    P.append(("Giac_hut_dai_oc_M12_trong", hexagon(-65, -59, 19.0), "#c0c4c8"))
    P.append(("Giac_hut_num_ong_6mm", BRepAlgoAPI_Fuse(cyl((0, 0, -67), (0, 0, 1), 3.5, 4.0),
                                                       cone(-74, -69, 2.4, 3.2)).Shape(), "#c0c4c8"))
    return P


# ---------------------------------------------------------------- cụm khí nén giác hút (24/11/2026)
# Van AirTAC 4V210-08 (5/2, 1 cuộn 12 V DC, cổng 1/4", nối nhanh PC602 ống Ø6, giảm thanh BSL01) — file nguồn chỉ có
# SolidWorks (.SLDPRT) nên DỰNG LẠI theo kích thước catalog, ƯỚC LƯỢNG ±3 mm. Bơm hút DC (đầu bơm trắng + động cơ
# 370) đặt trong đế in 3D — kích thước ƯỚC LƯỢNG từ ảnh. Giác hút L60-M12 + DP-40 đã có trong Robot_6_truc (khâu 6).
# Vị trí đặt trên bàn phía sau robot (X âm) — ƯỚC LƯỢNG, chưa có ảnh bố trí thật.
VALVE_ORIGIN = (-170.0, 20.0)     # mm, hệ base: góc đầu nắp trái van; thân van chạy dọc +Y
PUMP_ORIGIN = (-180.0, -160.0)    # mm, hệ base: mặt đầu bơm; bơm chạy dọc +Y


def valve_parts():
    """Van 4V210-08 nằm nghiêng (mặt cổng hướng ngang): hệ cục bộ X dọc van, cổng A/B phía +Y, P/R/S phía −Y, Z lên."""
    P = []
    P.append(("Van_than_nhom", box(0, 72, -16, 16, 0, 22), "#d9dde2"))
    P.append(("Van_nap_trai", box(-10, 0, -16, 16, 0, 22), "#1f2328"))
    P.append(("Van_dau_dan_huong", box(72, 88, -16, 16, 0, 22), "#1f2328"))
    P.append(("Van_cuon_dien_12V", box(88, 118, -15, 15, 0, 30), "#1f2328"))
    P.append(("Van_giac_cam_DIN", box(91, 118, -13.5, 13.5, 30, 56), "#1f2328"))
    P.append(("Van_ong_luon_cap", cyl((126, 0, 43), (1, 0, 0), 7.0, 16.0), "#1f2328"))
    for x, lab in ((22.0, "A"), (50.0, "B")):
        P.append((f"Noi_nhanh_PC602_{lab}", cyl((x, 23, 11), (0, 1, 0), 7.0, 14.0), "#c9a227"))
        P.append((f"Noi_nhanh_PC602_{lab}_vong", cyl((x, 32, 11), (0, 1, 0), 6.0, 4.0), "#2563eb"))
    P.append(("Noi_nhanh_PC602_P", cyl((36, -23, 11), (0, 1, 0), 7.0, 14.0), "#c9a227"))
    P.append(("Noi_nhanh_PC602_P_vong", cyl((36, -32, 11), (0, 1, 0), 6.0, 4.0), "#2563eb"))
    for x, lab in ((12.0, "R"), (60.0, "S")):
        P.append((f"Giam_thanh_BSL01_{lab}", cyl((x, -25, 11), (0, 1, 0), 5.5, 18.0), "#b08d57"))
    return P


def pump_parts():
    """Bơm hút DC: đầu bơm trắng Ø44 + động cơ 370 Ø27.6, nằm ngang trong đế in 3D; 2 núm ống Ø5 trên đầu bơm."""
    P = []
    P.append(("Bom_de_in_3D", box(-6, 76, -30, 30, 0, 8), "#f4f4f4"))
    for x in (6.0, 56.0):
        P.append((f"Bom_yen_do_{int(x)}", box(x, x + 12, -24, 24, 8, 22), "#f4f4f4"))
    P.append(("Bom_dau_bom", cyl((17, 0, 30), (1, 0, 0), 22.0, 34.0), "#f4f4f4"))
    P.append(("Bom_dong_co_370", cyl((53, 0, 30), (1, 0, 0), 13.8, 38.0), "#b8bcc2"))
    for x in (9.0, 25.0):
        P.append((f"Bom_num_ong_{int(x)}", cyl((x, 0, 57), (0, 0, 1), 2.5, 10.0), "#f4f4f4"))
    return P


def vacuum_group(table_z):
    R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)      # X cục bộ → +Y base
    out = []
    for parts, (ox, oy) in ((valve_parts(), VALVE_ORIGIN), (pump_parts(), PUMP_ORIGIN)):
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = (ox, oy, table_z)
        out += [(n, moved(s, T), c) for n, s, c in parts]
    return out


def main():
    cfg, lay, spec, cx, cy, base_z = station_layout()
    belt_top = base_z + spec.belt_top_height
    robot, q = robot_group()
    T = gia_transform(cx, cy + spec.length_total / 2, base_z)
    webcam, lens = webcam_group(T)
    ring, ring_center, k = ring_light_group(T, lens)
    pick_y = 1000.0 * float(lay.get("pick_y", 0.0)); px, py, pz = (1000.0 * float(v) for v in lay["parcel_size"])
    parcel = [("Kien_hang_mau", box(cx - px / 2, cx + px / 2, pick_y - py / 2, pick_y + py / 2, belt_top, belt_top + pz), "#d29922")]
    tx, ty, tz = TABLE_SIZE; mx, my = cx / 2, cy
    table = [("Mat_ban", box(mx - tx / 2, mx + tx / 2, my - ty / 2, my + ty / 2, base_z - tz, base_z), "#f2f2f2")]
    groups = [("Robot_6_truc", robot), ("Bang_tai", conveyor_group(spec, cx, cy, base_z)),
              ("Thung_phan_loai", bins_group(lay)), ("Kien_hang", parcel), ("Mat_ban", table),
              ("Gia_camera_den_cam_bien", gia_group(T)), ("Cam_bien", sensor_group(T)),
              ("Den_ring", ring), ("Webcam", webcam), ("Cum_giac_hut_bom_van", vacuum_group(base_z))]
    path = os.path.join(OUT_DIR, "Tram_phan_loai_tong.step")
    write_assembly(groups, path)
    info = [f"STEP: {path} ({os.path.getsize(path) / 1e6:.1f} MB)",
            f"Robot q = {q} độ; {len(robot)} chi tiết",
            f"Đèn ring: thang đo {k:.3f} mm/đv OBJ, tâm vòng (mm, base) = {np.round(ring_center, 1)}, "
            f"cao hơn mặt băng {ring_center[2] - belt_top:.0f} mm",
            f"Webcam: tâm ống kính (mm, base) = {np.round(lens, 1)}, cao hơn mặt băng {lens[2] - belt_top:.0f} mm",
            f"Cảm biến M18: trục tại Y base = {at(T, (SENSOR_AXIS[0], 0, 0))[1]:.1f} mm, cao hơn mặt băng "
            f"{at(T, (0, 0, SENSOR_AXIS[1]))[2] - belt_top:.0f} mm",
            "Số khối: " + ", ".join(f"{g} {len(it)}" for g, it in groups)]
    open(os.path.join(OUT_DIR, "ket_qua.txt"), "w", encoding="utf-8").write("\n".join(info) + "\n")
    print("\n".join(info))
    # xuất kèm lưới cho bước render kiểm tra
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.StlAPI import StlAPI_Writer
    os.makedirs(os.path.join(TMP, "stl"), exist_ok=True)
    # lưới thiết bị cho digital twin SortingStation (mm, hệ base) — cell_twin.py đọc manifest.json
    eq_dir = r"E:\UDMT\SortingStation\assets\station_equipment"; os.makedirs(eq_dir, exist_ok=True)
    manifest = []
    for gname, items in groups:
        if gname not in ("Gia_camera_den_cam_bien", "Cam_bien", "Den_ring", "Webcam", "Cum_giac_hut_bom_van"):
            continue
        for name, shp, col in items:
            BRepMesh_IncrementalMesh(shp, 0.3, False, 0.4, True)
            fn = f"{name}.stl"; StlAPI_Writer().Write(shp, os.path.join(eq_dir, fn))
            manifest.append({"file": fn, "group": gname, "color": col})
    json.dump({"note": "Xuất bởi robotarm-6-axis/src/cad/tram_step_3_xuat_tram.py — mm, hệ base robot. Đổi "
                       "layout băng tải trong station.yaml thì chạy lại script này.", "parts": manifest},
              open(os.path.join(eq_dir, "manifest.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    for gname, items in groups:
        for i, (name, shp, col) in enumerate(items):
            BRepMesh_IncrementalMesh(shp, 0.5, False, 0.5, True)
            w = StlAPI_Writer(); w.Write(shp, os.path.join(TMP, "stl", f"{gname}__{i:03d}__{col.lstrip('#')}.stl"))


if __name__ == "__main__":
    main()
