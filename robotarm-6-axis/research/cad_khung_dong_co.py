"""Phân tích các chi tiết quanh 1 động cơ trong hệ tọa độ của động cơ (w = trục, hướng về đầu trục ra;
w = 0 tại mặt đầu ra hộp số/mặt bích). Dùng đỉnh (vertex) để lấy cao độ chính xác."""
import sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from cad_util import load, faces, local_frame
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.TopAbs import TopAbs_VERTEX
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool


def verts(shape):
    e = TopExp_Explorer(shape, TopAbs_VERTEX); out = []
    while e.More():
        p = BRep_Tool.Pnt_s(TopoDS.Vertex(e.Current())); out.append([p.X(), p.Y(), p.Z()]); e.Next()
    return np.array(out)


class MotorFrame:
    def __init__(self, motor_pattern, shaft_r, flange_r):
        m = load(motor_pattern)
        cyl = []
        for f in faces(m):
            s = BRepAdaptor_Surface(f)
            if s.GetType() == GeomAbs_Cylinder:
                c = s.Cylinder(); a = c.Axis()
                cyl.append((c.Radius(), np.array([a.Location().X(), a.Location().Y(), a.Location().Z()]),
                            np.array([a.Direction().X(), a.Direction().Y(), a.Direction().Z()]), f))
        shaft = [c for c in cyl if abs(c[0] - shaft_r) < 0.05]
        axis = shaft[0][2] / np.linalg.norm(shaft[0][2])
        R, o = local_frame(shaft[0][1], axis)
        sv = np.vstack([verts(c[3]) for c in shaft]) - o
        fv = np.vstack([verts(c[3]) for c in cyl if abs(c[0] - flange_r) < 0.05]) - o
        sw, fw = sv @ R[2], fv @ R[2]
        if np.mean(sw) < np.mean(fw):  # đầu trục ra phải nằm ở w lớn
            R = np.array([R[0], -R[1], -R[2]]); sw, fw = -sw, -fw
        self.R, self.o = R, o
        # gốc w = mặt đầu ra hộp số (đầu của thân trụ flange_r phía trục ra)
        self.w0 = fw.max()
        self.shaft_tip = sw.max() - self.w0
        self.motor = m

    def to_local(self, pts):
        rel = np.atleast_2d(pts) - self.o
        return np.column_stack([rel @ self.R[0], rel @ self.R[1], rel @ self.R[2] - self.w0])

    def describe(self, shape, label, rmax=40):
        print(f"--- {label}")
        rows = set()
        for f in faces(shape):
            s = BRepAdaptor_Surface(f)
            v = self.to_local(verts(f))
            if s.GetType() == GeomAbs_Cylinder:
                c = s.Cylinder(); a = c.Axis()
                d = np.array([a.Direction().X(), a.Direction().Y(), a.Direction().Z()])
                if abs(abs(d @ self.R[2]) - 1) > 1e-3:
                    continue
                p = self.to_local([a.Location().X(), a.Location().Y(), a.Location().Z()])[0]
                rr = np.hypot(p[0], p[1])
                if rr > rmax:
                    continue
                rows.add(("TRU", round(c.Radius() * 2, 2), round(rr, 2), round(np.degrees(np.arctan2(p[1], p[0])), 0),
                          round(v[:, 2].min(), 2), round(v[:, 2].max(), 2)))
            elif s.GetType() == GeomAbs_Plane:
                n = s.Plane().Axis().Direction(); n = np.array([n.X(), n.Y(), n.Z()])
                if abs(abs(n @ self.R[2]) - 1) < 1e-3 and np.hypot(v[:, 0], v[:, 1]).min() < rmax:
                    rows.add(("PHANG", round(v[:, 2].mean(), 2), round(np.hypot(v[:, 0], v[:, 1]).min(), 1),
                              round(np.hypot(v[:, 0], v[:, 1]).max(), 1), 0, 0))
        for r in sorted(rows, key=lambda r: (r[0], r[4] if r[0] == "TRU" else r[1])):
            if r[0] == "TRU":
                print(f"  tru  D={r[1]:6.2f} cach_truc={r[2]:6.2f} goc={r[3]:6.0f}  w=[{r[4]:7.2f},{r[5]:7.2f}]")
            else:
                print(f"  phang w={r[1]:7.2f}  r_tu {r[2]:5.1f} den {r[3]:5.1f}")


if __name__ == "__main__":
    mf = MotorFrame(sys.argv[1], float(sys.argv[2]), float(sys.argv[3]))
    print(f"Dau truc ra: w = {mf.shaft_tip:.2f} mm so voi mat dau ra")
    for pat in sys.argv[4:]:
        mf.describe(load(pat), pat)
