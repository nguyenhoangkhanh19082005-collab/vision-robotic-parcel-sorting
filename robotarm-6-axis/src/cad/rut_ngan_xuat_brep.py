r"""Xuất từng chi tiết của Roboter_dong_co_1_27.step (đã đặt vào vị trí lắp ráp, mm) ra .tmp/brep_1_27/<số>_<tên>.brep
và .tmp/stl_1_27/<số>_<tên>.stl để phân tích/dựng ảnh nhanh. Số thứ tự trùng với .tmp/step_lap_rap.json.
    E:\UDMT\.venv\Scripts\python.exe src\cad\rut_ngan_xuat_brep.py
"""
import os, re, sys
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
from OCP.BRepTools import BRepTools
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.collections import Sequence_TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool

path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "assets", "CAD_dong_co_1_27", "Roboter_dong_co_1_27.step")
doc = TDocStd_Document(TCollection_ExtendedString("x"))
rd = STEPCAFControl_Reader(); rd.SetNameMode(True)
assert rd.ReadFile(path) == IFSelect_RetDone, path
rd.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
labels = Sequence_TDF_Label(); st.GetFreeShapes(labels)
for i in range(1, labels.Length() + 1):
    lab = labels.Value(i); name = TDataStd_Name()
    nm = lab.FindAttribute(TDataStd_Name.GetID_s(), name) and name.Get().ToExtString() or f"x{i}"
    key = f"{i-1:03d}_" + re.sub(r"[^A-Za-z0-9_.-]+", "_", nm)[:40]
    shp = st.GetShape_s(lab)
    BRepTools.Write_s(shp, os.path.join(ROOT, ".tmp", "brep_1_27", key + ".brep"))
    BRepMesh_IncrementalMesh(shp, 0.15, False, 0.4, True)
    w = StlAPI_Writer(); w.ASCIIMode = False; w.Write(shp, os.path.join(ROOT, ".tmp", "stl_1_27", key + ".stl"))
print(labels.Length(), "chi tiết")
