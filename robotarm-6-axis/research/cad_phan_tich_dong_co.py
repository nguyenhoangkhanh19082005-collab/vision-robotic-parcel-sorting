"""In hình học động cơ trong hệ tọa độ riêng của nó: w dọc trục ra (về phía trục ra), gốc tại mặt
đầu ra hộp số; mỗi mặt trụ: bán kính, khoảng cách tâm lỗ tới trục (r_pcd), góc, đoạn dọc trục [w0, w1]."""
import sys
import numpy as np
sys.path.insert(0, __import__("os").path.dirname(__file__))
from cad_util import load, cylinders, planes, faces, local_frame
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box


def face_w_range(face, R, o):
    b = Bnd_Box(); BRepBndLib.AddOptimal_s(face, b, False, False)
    lo, hi = b.CornerMin(), b.CornerMax()
    pts = np.array([[x, y, z] for x in (lo.X(), hi.X()) for y in (lo.Y(), hi.Y()) for z in (lo.Z(), hi.Z())])
    w = (pts - o) @ R[2]
    return w.min(), w.max()


def analyse(pattern, shaft_r_hint):
    m = load(pattern)
    cyl = cylinders(m)
    # trục chính = hướng của mặt trụ lớn nhất; gốc = điểm trên trục của trục ra (bán kính ~ shaft)
    big = max(cyl, key=lambda c: c[3])
    axis = big[2] / np.linalg.norm(big[2])
    shaft = min((c for c in cyl if abs(c[0] - shaft_r_hint) < 0.6 and abs(abs(c[2] @ axis) - 1) < 1e-3),
                key=lambda c: -c[3])
    pts = np.array([shaft[1]])
    R, o = local_frame(shaft[1], axis)
    # hướng w về phía đầu trục ra: đầu trục ra là phía có mặt trụ trục
    from cad_util import faces as _f
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder
    lo_all, hi_all = 1e9, -1e9
    rows = []
    for f in faces(m):
        s = BRepAdaptor_Surface(f)
        if s.GetType() != GeomAbs_Cylinder:
            continue
        c = s.Cylinder(); ax = c.Axis(); d = np.array([ax.Direction().X(), ax.Direction().Y(), ax.Direction().Z()])
        if abs(abs(d @ axis) - 1) > 1e-3:
            continue
        p = np.array([ax.Location().X(), ax.Location().Y(), ax.Location().Z()])
        rel = p - o; uv = np.array([rel @ R[0], rel @ R[1]])
        w0, w1 = face_w_range(f, R, o)
        rows.append((round(c.Radius(), 2), round(float(np.hypot(*uv)), 2), round(float(np.degrees(np.arctan2(uv[1], uv[0]))), 1), round(w0, 1), round(w1, 1)))
        lo_all, hi_all = min(lo_all, w0), max(hi_all, w1)
    # chiều: đầu trục ở w lớn
    sw = [r for r in rows if abs(r[0] - shaft_r_hint) < 0.6 and r[1] < 0.5]
    flip = np.mean([r[4] for r in sw]) < (lo_all + hi_all) / 2
    if flip:
        rows = [(r[0], r[1], (r[2] + 180) % 360 - 180, -r[4], -r[3]) for r in rows]
        lo_all, hi_all = -hi_all, -lo_all
    print(f"== {pattern}: truc {axis.round(3)}  doan doc truc [{lo_all:.1f}, {hi_all:.1f}] mm (tong {hi_all-lo_all:.1f})")
    for r in sorted(set(rows), key=lambda r: (r[3], r[0])):
        print(f"  r={r[0]:6.2f} (D{2*r[0]:5.2f})  cach truc={r[1]:6.2f}  goc={r[2]:7.1f}  w=[{r[3]:6.1f},{r[4]:6.1f}]")


if __name__ == "__main__":
    analyse(sys.argv[1], float(sys.argv[2]))
