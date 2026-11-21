"""Liệt kê động cơ, puly, dây đai trong Roboter.STEP kèm tâm hộp bao (hệ tọa độ CAD, mm) và
khớp gần nhất (JOINT_POINTS của robot_simulation.py, đổi sang mm) — để xác định J6 dẫn động trực tiếp
hay qua đai. Chỉ đọc file CAD.

    E:\\UDMT\\.venv\\Scripts\\python.exe research\\cad_tim_truyen_dong_j6.py "E:\\robot công nghiệp\\Bài_thi_2026\\Roboter.STEP"
"""

import re
import sys

import numpy as np
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_DocumentTool

JOINTS_M = [
    (-0.1544785, 0.1454204, -0.0507776),
    (-0.1538267, 0.1394184, 0.0824138),
    (-0.1373280, 0.1339924, 0.2348735),
    (-0.2154871, 0.0110950, 0.2594626),
    (-0.2646521, -0.0790322, 0.2838017),
    (-0.2774943, -0.1114704, 0.2813030),
]
PATTERN = re.compile(r"riemen|scheibe|pulley|nema|17hs|motor|belt|zahn|gt2|welle|getriebe", re.I)


def name_of(label):
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return ""


def main(path):
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(path) != IFSelect_RetDone:
        raise SystemExit("Không đọc được STEP")
    reader.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    st.GetFreeShapes(roots)
    found = []

    def walk(label, loc, path_names):
        nm = name_of(label)
        ref = label
        if st.IsReference_s(label):
            from OCP.TDF import TDF_Label
            ref = TDF_Label()
            st.GetReferredShape_s(label, ref)
            loc = loc.Multiplied(st.GetLocation_s(label))
            nm = nm or name_of(ref)
        pname = name_of(ref)
        full = path_names + [pname or nm]
        if PATTERN.search(pname or "") or PATTERN.search(nm or ""):
            shape = st.GetShape_s(ref).Moved(loc)
            box = Bnd_Box()
            BRepBndLib.Add_s(shape, box)
            if not box.IsVoid():
                lo, hi = box.CornerMin(), box.CornerMax()
                x0, y0, z0, x1, y1, z1 = lo.X(), lo.Y(), lo.Z(), hi.X(), hi.Y(), hi.Z()
                c = np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
                found.append((pname or nm, nm, c, np.array([x1 - x0, y1 - y0, z1 - z0])))
            return  # không đi sâu vào bên trong động cơ/puly
        if st.IsAssembly_s(ref):
            comps = TDF_LabelSequence()
            st.GetComponents_s(ref, comps)
            for i in range(1, comps.Length() + 1):
                walk(comps.Value(i), loc, full)

    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [])

    centers = np.array([c for _, _, c, _ in found])
    # Thử 2 đơn vị cho JOINT_POINTS (m → mm) — in khoảng cách tới khớp gần nhất
    joints = np.array(JOINTS_M) * 1000.0
    for part, inst, c, size in sorted(found, key=lambda f: f[0]):
        d = np.linalg.norm(joints - c, axis=1)
        k = int(np.argmin(d))
        print(f"{part[:40]:40s} {inst[:14]:14s} tam=({c[0]:7.1f},{c[1]:7.1f},{c[2]:7.1f}) "
              f"kich_thuoc=({size[0]:5.1f},{size[1]:5.1f},{size[2]:5.1f})  gan J{k + 1} ({d[k]:.0f} mm)")
    print(f"\n{len(found)} chi tiết. Khớp (mm): " + "; ".join(
        f"J{i + 1}=({p[0]:.0f},{p[1]:.0f},{p[2]:.0f})" for i, p in enumerate(joints)))
    _ = centers


if __name__ == "__main__":
    main(sys.argv[1])
