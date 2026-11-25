r"""Bước 1/3 xuất STEP toàn trạm: đọc lắp ráp robot v3 (XCAF), lưu đỉnh lưới thô của từng chi tiết → .tmp/tram_step/robot_parts.npz
    E:\UDMT\.venv\Scripts\python.exe src\cad\tram_step_1_doc_robot.py
"""
import os, sys
import numpy as np
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.TopLoc import TopLoc_Location

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SRC = os.path.join(ROOT, "assets", "CAD_lap_rap_v3", "Roboter_v3_J4_cu_bau_ngan_giac_hut_dai.step")
OUT = os.path.join(ROOT, ".tmp", "tram_step"); os.makedirs(OUT, exist_ok=True)


def read_parts(path):
    doc = TDocStd_Document(TCollection_ExtendedString("x"))
    rd = STEPCAFControl_Reader(); rd.SetNameMode(True)
    assert rd.ReadFile(path) == IFSelect_RetDone, path
    rd.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    labels = TDF_LabelSequence(); st.GetFreeShapes(labels)
    out = []
    for i in range(1, labels.Length() + 1):
        lab = labels.Value(i); name = TDataStd_Name()
        nm = lab.FindAttribute(TDataStd_Name.GetID_s(), name) and name.Get().ToExtString() or f"part_{i}"
        out.append((nm, st.GetShape_s(lab)))
    return out


def mesh_points(shape, defl=1.0):
    BRepMesh_IncrementalMesh(shape, defl, False, 0.5, True)
    pts = []
    e = TopExp_Explorer(shape, TopAbs_FACE)
    while e.More():
        f = TopoDS.Face(e.Current()); loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f, loc)
        if tri is not None:
            tr = loc.Transformation()
            for k in range(1, tri.NbNodes() + 1):
                p = tri.Node(k).Transformed(tr); pts.append((p.X(), p.Y(), p.Z()))
        e.Next()
    return np.unique(np.round(np.array(pts, float), 2), axis=0) if pts else np.zeros((0, 3))


if __name__ == "__main__":
    parts = read_parts(SRC)
    names, arrs = [], {}
    for i, (nm, shp) in enumerate(parts):
        names.append(nm); arrs[f"p{i}"] = mesh_points(shp)
    np.savez(os.path.join(OUT, "robot_parts.npz"), names=np.array(names), **arrs)
    print(len(parts), "chi tiết; tổng đỉnh", sum(len(a) for a in arrs.values()))
