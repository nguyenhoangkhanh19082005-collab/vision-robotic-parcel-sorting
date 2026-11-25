"""Sinh part in 3D "góc quây sọt" (25/11/2026): 4 cột góc chữ L dán xuống tấm bìa để quây 1 ô làm sọt phân loại.

Mỗi part = vách chữ L đứng + đế (mặt bích) chìa RA NGOÀI để dán keo/băng keo 2 mặt (bên trong sọt phẳng, kiện rơi
không vướng) + 2 gân tam giác phía ngoài cho vách cứng + 2 lỗ Ø3.5 trên đế (bắt vít/ghim nếu không muốn dán).
Mép trên phía trong vát 45° để kiện trượt vào trong khi thả lệch. 4 part giống hệt nhau — xoay 90° cho từng góc.
Đơn vị mm. Chạy: python src/cad/tao_goc_quay_sot.py [cạnh_tay] [chiều_cao]  → assets/goc_quay_sot/
"""
import os
import sys

from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakePrism
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse, BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCP.gp import gp_Pnt, gp_Vec, gp_Ax2, gp_Dir

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = os.path.join(ROOT, "assets", "goc_quay_sot")

# ---------------- THAM SỐ ----------------
A = 50.0       # chiều dài mỗi cánh chữ L, đo từ góc TRONG của sọt
H = 50.0       # chiều cao vách (kiện cao 50 mm thả từ trên xuống: 40–60 mm là đủ giữ)
T = 3.0        # bề dày vách
F = 15.0       # bề rộng đế chìa ra ngoài (diện tích dán keo)
FT = 2.4       # bề dày đế
VAT = 4.0      # vát mép trên phía trong 45° (dẫn kiện vào sọt)
G_H = 0.6      # gân cao = 60 % vách
G_T = 3.0      # bề dày gân
HOLE = 3.5     # lỗ vít đế


def box(x0, y0, z0, x1, y1, z1):
    return BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), gp_Pnt(x1, y1, z1)).Shape()


def prism(pts, vec):
    poly = BRepBuilderAPI_MakePolygon()
    for p in pts:
        poly.Add(gp_Pnt(*p))
    poly.Close()
    face = BRepBuilderAPI_MakeFace(poly.Wire()).Face()
    return BRepPrimAPI_MakePrism(face, gp_Vec(*vec)).Shape()


def fuse(*shapes):
    s = shapes[0]
    for o in shapes[1:]:
        s = BRepAlgoAPI_Fuse(s, o).Shape()
    return s


def cut(s, *tools):
    for t in tools:
        s = BRepAlgoAPI_Cut(s, t).Shape()
    return s


def make_part(a, h):
    # Góc trong của sọt tại gốc tọa độ; lòng sọt là góc phần tư x > 0, y > 0. Vách nằm ở x < 0 / y < 0.
    wall_x = box(-T, -T, 0, a, 0, h)                         # cánh dọc trục X
    wall_y = box(-T, -T, 0, 0, a, h)                         # cánh dọc trục Y
    flange = fuse(box(-T - F, -T - F, 0, a, -T, FT),         # đế ngoài cánh X
                  box(-T - F, -T - F, 0, -T, a, FT))         # đế ngoài cánh Y
    gh = h * G_H
    gx = a * 0.55                                            # gân đặt khoảng giữa cánh
    rib_x = prism([(gx, -T, FT), (gx, -T - F, FT), (gx, -T, gh)], (G_T, 0, 0))
    rib_y = prism([(-T, gx, FT), (-T - F, gx, FT), (-T, gx, gh)], (0, G_T, 0))
    body = fuse(wall_x, wall_y, flange, rib_x, rib_y)
    # vát 45° mép trên phía trong (mặt y = 0 của cánh X, mặt x = 0 của cánh Y): cắt bằng lăng trụ tam giác
    v = min(VAT, T - 0.8)
    ch_x = prism([(0, 0.01, h - v), (0, 0.01, h + 0.01), (0, -v, h + 0.01)], (a + 1, 0, 0))
    ch_y = prism([(0.01, 0, h - v), (0.01, 0, h + 0.01), (-v, 0, h + 0.01)], (0, a + 1, 0))
    holes = [BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, y, -1), gp_Dir(0, 0, 1)), HOLE / 2, FT + 2).Shape()
             for (x, y) in [(a - 8, -T - F / 2), (-T - F / 2, a - 8)]]
    return cut(body, ch_x, ch_y, *holes)


def export(shape, base):
    BRepMesh_IncrementalMesh(shape, 0.05, False, 0.3, True)
    StlAPI_Writer().Write(shape, base + ".stl")
    st = STEPControl_Writer()
    st.Transfer(shape, STEPControl_AsIs)
    st.Write(base + ".step")


def preview(stl, png, a):
    import pyvista as pv
    m = pv.read(stl)
    p = pv.Plotter(off_screen=True, window_size=(1400, 700), shape=(1, 2))
    p.subplot(0, 0)
    p.add_text("1 part (view from inside the bin)", font_size=11)
    p.add_mesh(m, color="#4a90d9")
    p.camera_position = "iso"
    p.reset_camera()
    p.subplot(0, 1)
    p.add_text("4 parts on board: bin 160 x 140 mm, parcel 90 x 120 x 50", font_size=11)
    W, D = 160.0, 140.0
    p.add_mesh(pv.Box(bounds=(-40, W + 40, -40, D + 40, -3, 0)), color="#d9c9a3")
    for ang, (tx, ty) in [(0, (0, 0)), (90, (W, 0)), (180, (W, D)), (270, (0, D))]:
        p.add_mesh(m.copy().rotate_z(ang, point=(0, 0, 0)).translate((tx, ty, 0)), color="#4a90d9")
    p.add_mesh(pv.Box(bounds=(35, 125, 10, 130, 0, 50)), color="#c8873a", opacity=0.6)  # kiện 90 x 120 x 50
    p.camera_position = "iso"
    p.reset_camera()
    p.set_background("white")
    p.screenshot(png)


if __name__ == "__main__":
    a = float(sys.argv[1]) if len(sys.argv) > 1 else A
    h = float(sys.argv[2]) if len(sys.argv) > 2 else H
    os.makedirs(OUT, exist_ok=True)
    base = os.path.join(OUT, f"goc_quay_sot_A{a:g}_H{h:g}")
    export(make_part(a, h), base)
    preview(base + ".stl", base + "_xem_truoc.png", a)
    print("Đã ghi:", base + ".stl / .step / _xem_truoc.png")
    print(f"Cánh {a:g} mm (từ góc trong), cao {h:g} mm, vách {T:g} mm, đế chìa ngoài {F:g} mm; "
          f"chiếm chỗ ngoài {a + T + F:g} × {a + T + F:g} mm")
