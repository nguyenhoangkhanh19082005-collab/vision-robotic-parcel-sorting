r"""Xuất từng chi tiết (đã đặt vào vị trí lắp ráp, hệ CAD mm) của Roboter.STEP ra file .brep để phân tích
nhanh mà không phải đọc lại STEP 25 MB. Tên file: <part>__<NAUO>.brep. Chỉ đọc file CAD gốc.
    E:\UDMT\.venv\Scripts\python.exe research\cad_xuat_brep.py <Roboter.STEP> research\cad_cache
"""
import os, re, sys
from OCP.BRepTools import BRepTools
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


def main(path, out_dir):
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    reader = STEPCAFControl_Reader(); reader.SetNameMode(True)
    assert reader.ReadFile(path) == IFSelect_RetDone
    reader.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = Sequence_TDF_Label(); st.GetFreeShapes(roots)
    count = {}

    def walk(label, loc, top_inst):
        inst = name_of(label); ref = label
        if st.IsReference_s(label):
            ref = TDF_Label(); st.GetReferredShape_s(label, ref)
            loc = loc.Multiplied(st.GetLocation_s(label))
        pname = name_of(ref)
        top = top_inst or (inst if st.IsReference_s(label) else "")
        if st.IsAssembly_s(ref) and not re.search(r"Nema17|STEPPERONLINE", pname):
            comps = Sequence_TDF_Label(); st.GetComponents_s(ref, comps)
            for i in range(1, comps.Length() + 1):
                walk(comps.Value(i), loc, top_inst)
            return
        safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", pname)[:40]
        key = f"{safe}__{inst or 'x'}"
        count[key] = count.get(key, 0) + 1
        if count[key] > 1:
            key += f"_{count[key]}"
        BRepTools.Write_s(st.GetShape_s(ref).Moved(loc), os.path.join(out_dir, key + ".brep"))

    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), "")
    print(sum(count.values()), "file ->", out_dir)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
