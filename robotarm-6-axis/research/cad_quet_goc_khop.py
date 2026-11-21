r"""Quét góc khớp J2/J3: quay động cơ (cũ PG5 và mới 1:27) ngược quanh trục khớp — tương đương quay
khâu phía sau — và tìm dải góc không va chạm với các chi tiết của khâu quay. So sánh cũ/mới.
    E:\UDMT\.venv\Scripts\python.exe research\cad_quet_goc_khop.py
"""
import glob, os, re, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from cad_util import CACHE, load, cylinders
from cad_khung_dong_co import MotorFrame
from cad_kiem_tra_dong_co_moi import new_motor_local, place, vol
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf

LINK2_PLUS = r"^(AX-[2-6]|Riemenscheibe_55mm|STEPPERONLINE.*NAUO(123|81)|Nema17.*NAUO(42|21)|Riemenscheibe_(16|40)mm__NAUO(161|120|39|38)|Lager_(65x50x7__NAUO10[01]|52x40|32x|19x5))"
LINK3_PLUS = r"^(AX-[3-6]|STEPPERONLINE.*NAUO81|Nema17.*NAUO(42|21)|Riemenscheibe_(16|40)mm__NAUO(39|38)|Riemenscheibe_40mm__NAUO120|Lager_(52x40|32x|19x5))"


def box(shape):
    b = Bnd_Box(); BRepBndLib.Add_s(shape, b); return b


def joint_axis(pulley_pattern, axis_hint):
    for r, p, d, a in sorted(cylinders(load(pulley_pattern)), key=lambda c: -c[3]):
        if abs(abs(d @ axis_hint) - 1) < 1e-3:
            return p, d / np.linalg.norm(d)


def sweep(label, motor_shape, axis_p, axis_d, group_re, angles):
    parts = [(n, load(n)) for n in (os.path.basename(f)[:-5] for f in glob.glob(os.path.join(CACHE, "*.brep")))
             if re.search(group_re, n)]
    bad = []
    for ang in angles:
        t = gp_Trsf(); t.SetRotation(gp_Ax1(gp_Pnt(*axis_p), gp_Dir(*axis_d)), np.radians(-ang))
        m = BRepBuilderAPI_Transform(motor_shape, t, True).Shape()
        mb = box(m)
        hit = []
        for n, s in parts:
            if mb.IsOut(box(s)):
                continue
            if vol(BRepAlgoAPI_Common(m, s).Shape()) > 1.0:
                hit.append(n.split("__")[0])
        if hit:
            bad.append((ang, sorted(set(hit))))
    ok = [a for a in angles if a not in [b[0] for b in bad]]
    print(f"== {label}: va chạm ở {len(bad)}/{len(angles)} góc")
    for a, h in bad:
        print(f"   {a:+5.0f}°: {', '.join(h)}")
    return ok


if __name__ == "__main__":
    angles = list(range(-180, 181, 10))
    for motor, pulley, group, name in [("NAUO156", "Riemenscheibe_55mm__NAUO140", LINK2_PLUS, "J2"),
                                       ("NAUO123", "Riemenscheibe_40mm__NAUO120", LINK3_PLUS, "J3")]:
        mf = MotorFrame(f"STEPPERONLINE*{motor}", 3.0, 18.0)
        p, d = joint_axis(pulley, mf.R[2])
        old = load(f"STEPPERONLINE*{motor}")
        sweep(f"{name} PG5 hiện tại (góc tương đối tư thế CAD)", old, p, d, group, angles)
        sweep(f"{name} động cơ mới 1:27", place(new_motor_local(), mf), p, d, group, angles)
