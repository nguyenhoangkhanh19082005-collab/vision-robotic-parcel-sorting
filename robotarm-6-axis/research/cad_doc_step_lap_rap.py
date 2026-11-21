r"""Đọc file lắp ráp STEP (mặc định assets/CAD_dong_co_1_27/Roboter_dong_co_1_27.step) bằng XCAF: lấy tên, thể tích,
trọng tâm, hộp bao và các mặt trụ lớn của từng chi tiết → .tmp/step_lap_rap.json (dùng cho cad_kiem_tra_mo_men.py).
    E:\UDMT\.venv\Scripts\python.exe research\cad_doc_step_lap_rap.py
"""
import json, os, sys
import numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))
from cad_util import bbox, cylinders
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool

path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "assets", "CAD_dong_co_1_27", "Roboter_dong_co_1_27.step")
doc = TDocStd_Document(TCollection_ExtendedString("x"))
rd = STEPCAFControl_Reader(); rd.SetNameMode(True)
assert rd.ReadFile(path) == IFSelect_RetDone, path
rd.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
labels = TDF_LabelSequence(); st.GetFreeShapes(labels)
out = []
for i in range(1, labels.Length() + 1):
    lab = labels.Value(i)
    name = TDataStd_Name()
    nm = lab.FindAttribute(TDataStd_Name.GetID_s(), name) and name.Get().ToExtString() or f"?{i}"
    shp = st.GetShape_s(lab)
    g = GProp_GProps(); BRepGProp.VolumeProperties_s(shp, g)
    c = g.CentreOfMass(); lo, hi = bbox(shp)
    cyl = sorted(cylinders(shp), key=lambda c: -c[3])[:6]
    out.append({"name": nm, "vol_cm3": g.Mass() / 1000, "com": [c.X(), c.Y(), c.Z()], "min": lo.tolist(), "max": hi.tolist(),
                "cyl": [{"r": r, "p": p.tolist(), "d": d.tolist(), "area": a} for r, p, d, a in cyl]})
dst = os.path.join(ROOT, ".tmp", "step_lap_rap.json")
json.dump(out, open(dst, "w", encoding="utf-8"), ensure_ascii=False)
print(len(out), "chi tiết →", dst)
for o in out:
    if not o["name"].startswith(("ISK", "Mutter", "DIN", "ISO")):
        print(f"{o['name']:40s} {o['vol_cm3']:8.1f} cm3  com=({o['com'][0]:7.1f},{o['com'][1]:7.1f},{o['com'][2]:7.1f})")
