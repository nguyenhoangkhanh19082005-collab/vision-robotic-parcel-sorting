r"""Tách gá giác hút + giác hút L60-M12 / HSP-40 (DP-40) từ lắp ráp robot v3 thành STL trong HỆ FILE MESH khâu 6
(mm, cùng hệ assets/ROBOTARM2_description/meshes/link_6_1.stl) → twin GUI robot và twin trạm vẽ đúng hình đầu hút
thay khối trụ. Cần chạy bước 1, 2 trước.
    E:\UDMT\.venv\Scripts\python.exe -X utf8 src\cad\tram_step_5_dung_cu_J6.py
"""
import json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, r"E:\UDMT\SortingStation")
from tram_step_1_doc_robot import read_parts, SRC
from tram_step_3_xuat_tram import moved, suction_cup_parts, tcp_mm
from sorting_station.station_config import install_import_paths, load_station_config, resolve_paths
cfg = load_station_config(); paths = resolve_paths(cfg); install_import_paths(paths)
from sorting_station.robot_model import build_model_from_robot_project
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer

OUT = os.path.join(ROOT, "assets", "ROBOTARM2_description", "meshes")
PARTS = {"Ga_giac_hut_J6_dai": "link_6_tool_ga.stl"}

place = json.load(open(os.path.join(ROOT, ".tmp", "tram_step", "robot_dat.json"), encoding="utf-8"))
model = build_model_from_robot_project(paths.robot_root, tool_length_m=0.126)
T6 = model.link_transforms_base(np.radians(place["q_deg"]))[6].copy(); T6[:3, 3] *= 1000.0   # file mm → base mm
inv6 = np.linalg.inv(T6)
done = []
for (name, shp), pl in zip(read_parts(SRC), place["parts"]):
    if name in PARTS:
        assert pl["link"] == 6, name
        s = moved(shp, inv6 @ np.array(pl["T"]))      # STEP → base → hệ file mesh khâu 6
        BRepMesh_IncrementalMesh(s, 0.1, False, 0.3, True)
        StlAPI_Writer().Write(s, os.path.join(OUT, PARTS[name])); done.append(PARTS[name])
# giác hút L60-M12 + DP-40 chi tiết (24/11/2026) — đặt theo TCP rồi về hệ file mesh khâu 6
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Compound
comp = TopoDS_Compound(); bb = BRep_Builder(); bb.MakeCompound(comp)
Tc = inv6 @ tcp_mm(place["q_deg"])
for _n, sh, _c in suction_cup_parts():
    bb.Add(comp, moved(sh, Tc))
BRepMesh_IncrementalMesh(comp, 0.1, False, 0.3, True)
StlAPI_Writer().Write(comp, os.path.join(OUT, "link_6_tool_giac_hut.stl")); done.append("link_6_tool_giac_hut.stl")
print("Đã xuất:", done, "→", OUT)
