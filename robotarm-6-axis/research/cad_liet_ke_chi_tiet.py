r"""Liệt kê toàn bộ chi tiết (lá) của Roboter.STEP: tên, tâm & kích thước hộp bao (mm, hệ CAD), thể tích.
Lưu kết quả JSON để các bước phân tích sau dùng lại. Chỉ đọc file CAD.
    E:\UDMT\.venv\Scripts\python.exe research\cad_liet_ke_chi_tiet.py <Roboter.STEP> <out.json>
"""
import json, sys
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_DocumentTool


def name_of(label):
    attr = TDataStd_Name()
    return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else ""


def main(path, out):
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    reader = STEPCAFControl_Reader(); reader.SetNameMode(True)
    assert reader.ReadFile(path) == IFSelect_RetDone
    reader.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = Sequence_TDF_Label(); st.GetFreeShapes(roots)
    parts = []

    def walk(label, loc, parent):
        inst = name_of(label); ref = label
        if st.IsReference_s(label):
            ref = TDF_Label(); st.GetReferredShape_s(label, ref)
            loc = loc.Multiplied(st.GetLocation_s(label))
        pname = name_of(ref)
        if st.IsAssembly_s(ref):
            comps = Sequence_TDF_Label(); st.GetComponents_s(ref, comps)
            for i in range(1, comps.Length() + 1):
                walk(comps.Value(i), loc, parent + [pname])
            return
        shape = st.GetShape_s(ref).Moved(loc)
        box = Bnd_Box(); BRepBndLib.Add_s(shape, box)
        if box.IsVoid():
            return
        lo, hi = box.CornerMin(), box.CornerMax()
        props = GProp_GProps(); BRepGProp.VolumeProperties_s(shape, props)
        c = props.CentreOfMass()
        parts.append({"part": pname, "inst": inst, "parent": "/".join(parent[1:]),
                      "min": [lo.X(), lo.Y(), lo.Z()], "max": [hi.X(), hi.Y(), hi.Z()],
                      "com": [c.X(), c.Y(), c.Z()], "vol_cm3": props.Mass() / 1000.0})

    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [])
    json.dump(parts, open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(len(parts), "chi tiet ->", out)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
