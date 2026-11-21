r"""Đặt mô hình đơn giản của động cơ mới 42STH38-1684A + hộp số 36JXF27G (1:27) vào vị trí động cơ J1/J2/J3
trong Roboter.STEP và kiểm tra va chạm với mọi chi tiết khác (thể tích giao nhau).

Kích thước theo ảnh người bán (mm): thân 42x42 dài 39.5; hộp số Ø35.9 dài 34.1 (tính cả tấm bích nhôm);
trục ra vuông 6x6 dài 14.5; 4 lỗ M2.5 trên vòng Ø28. Gốc w = 0 tại mặt đầu ra hộp số.
    E:\UDMT\.venv\Scripts\python.exe research\cad_kiem_tra_dong_co_moi.py
"""
import glob, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from cad_util import CACHE, load
from cad_khung_dong_co import MotorFrame
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf, gp_Ax3
from OCP.BRepTools import BRepTools

GEAR_L, GEAR_D, MOTOR_L, BODY = 34.1, 35.9, 39.5, 42.3
SHAFT_L, SHAFT_W = 14.5, 6.0


def new_motor_local():
    """Động cơ mới trong hệ cục bộ: trục +Z, mặt đầu ra hộp số tại z = 0."""
    gear = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -GEAR_L), gp_Dir(0, 0, 1)), GEAR_D / 2, GEAR_L).Shape()
    body = BRepPrimAPI_MakeBox(gp_Pnt(-BODY / 2, -BODY / 2, -GEAR_L - MOTOR_L), BODY, BODY, MOTOR_L).Shape()
    shaft = BRepPrimAPI_MakeBox(gp_Pnt(-SHAFT_W / 2, -SHAFT_W / 2, 0), SHAFT_W, SHAFT_W, SHAFT_L).Shape()
    s = BRepAlgoAPI_Fuse(gear, body).Shape()
    return BRepAlgoAPI_Fuse(s, shaft).Shape()


def place(shape, mf, w_offset=0.0):
    """Đưa hình từ hệ cục bộ (z = w) sang hệ CAD theo khung động cơ mf."""
    o = mf.o + mf.R[2] * (mf.w0 + w_offset)
    ax = gp_Ax3(gp_Pnt(*o), gp_Dir(*mf.R[2]), gp_Dir(*mf.R[0]))
    t = gp_Trsf(); t.SetTransformation(ax, gp_Ax3())
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def vol(shape):
    p = GProp_GProps(); BRepGProp.VolumeProperties_s(shape, p); return p.Mass()


def check(label, mf, exclude, w_offset=0.0, export=None):
    nm = place(new_motor_local(), mf, w_offset)
    if export:
        BRepTools.Write_s(nm, export)
    print(f"== {label}: động cơ mới (vol {vol(nm)/1000:.0f} cm3)")
    hits = []
    for f in sorted(glob.glob(os.path.join(CACHE, "*.brep"))):
        name = os.path.basename(f)[:-5]
        if any(x in name for x in exclude):
            continue
        part = load(name)
        try:
            v = vol(BRepAlgoAPI_Common(nm, part).Shape())
        except Exception as e:  # noqa
            v = -1
        if v > 1.0:
            hits.append((v, name))
    for v, n in sorted(hits, reverse=True):
        print(f"   VA CHAM {v:8.1f} mm3  {n}")
    if not hits:
        print("   khong va cham")


if __name__ == "__main__":
    out = os.path.join(os.path.dirname(__file__), "cad_cache_moi"); os.makedirs(out, exist_ok=True)
    mf2 = MotorFrame("STEPPERONLINE*NAUO156", 3.0, 18.0)
    check("J2 (thay PG5 NAUO156, giữ nguyên tấm gá AX-1_6)", mf2, ["NAUO156"], export=os.path.join(out, "moi_J2.brep"))
    mf3 = MotorFrame("STEPPERONLINE*NAUO123", 3.0, 18.0)
    check("J3 (thay PG5 NAUO123, giữ nguyên tấm gá AX-1_6)", mf3, ["NAUO123"], export=os.path.join(out, "moi_J3.brep"))
    mf1 = MotorFrame("Nema17*NAUO167", 2.5, 11.0)
    # J1: mặt đầu ra hộp số đặt tại mặt bích Nema17 cũ (w=0) để puly giữ nguyên cao độ
    check("J1 (thay Nema17 NAUO167, mặt hộp số tại mặt bích cũ)", mf1, ["NAUO167"], export=os.path.join(out, "moi_J1.brep"))
