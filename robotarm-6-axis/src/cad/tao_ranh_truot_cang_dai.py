r"""RÃNH TRƯỢT CĂNG ĐAI LỆCH TÂM — chụp lên đầu vít M4 đang lắp ở vỏ J2/J3 (20/11/2026).

Vấn đề: đai GT2 6 mm (J3: 230-2GT-6) nằm CAO HƠN lỗ vít căng 0.5–1 con ốc M4 (≈ 2–3 mm) vì puly nhỏ được nâng bằng
mặt "con thoi". Vít tì thẳng → đai trượt khỏi điểm tì.
Giải pháp: con trượt nhựa có RÃNH giữ đai, tâm rãnh LỆCH LÊN e mm so với trục vít:
  - rãnh rộng 7.0 (đai 6 + 1), 2 gờ chặn dày 1.6 cao 3.0 (nhô hơn mặt răng đai 1.6 mm) → đai không thể trượt ra;
  - đáy rãnh cong R20 dọc chiều chạy của đai, 2 đầu thấp dần → đai vào/ra êm, không vấp;
  - lỗ Ø4.5 không ren sâu 5.5 ở mặt dưới chụp đầu vít M4 → vặn vít đẩy con trượt + đai vào trong (căng hết cỡ),
    con trượt không xoay theo vít; chính đai nằm trong rãnh giữ cho con trượt đúng hướng;
  - phía đế dày hơn (bên dưới) = phía ĐÁY vỏ; rãnh lệch về phía MIỆNG vỏ (nắp).
Xuất 2 cỡ lệch tâm e = 2.0 và 3.0 mm (chọn theo khoảng lệch đo được giữa tâm đai và tâm lỗ vít).
    E:\UDMT\.venv\Scripts\python.exe -X utf8 src\cad\tao_ranh_truot_cang_dai.py
"""
import os, sys, shutil
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.dirname(__file__)); sys.path.insert(0, os.path.join(ROOT, "research"))
from tao_chi_tiet_dong_co_1_27 import write_step, vol
from rut_ngan_lib import mesh_check
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

BELT_W = 6.0
CH = BELT_W + 1.0          # rãnh 7.0
LIP_T, LIP_H = 1.6, 3.0    # gờ chặn
L = 16.0                   # dài dọc đai
H0, CROWN, RC = 5.0, 1.8, 20.0
SOCK_D, SOCK_DEPTH = 4.5, 5.5
SOCK_WALL = 2.2            # thành quanh lỗ vít


def box(x0, y0, z0, x1, y1, z1):
    return BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), x1 - x0, y1 - y0, z1 - z0).Shape()


def make(e):
    """x dọc đai, y = hướng lệch tâm (lên phía miệng vỏ), z hướng vào đai. Trục vít tại x = y = 0."""
    yc = e                                       # tâm rãnh
    y_lo = min(yc - CH / 2 - LIP_T, -(SOCK_D / 2 + SOCK_WALL))
    y_hi = yc + CH / 2 + LIP_T
    top = H0 + CROWN; zc = top - RC
    ycyl = lambda r: BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, y_lo - 1, zc), gp_Dir(0, 1, 0)), r, y_hi - y_lo + 2).Shape()
    body = BRepAlgoAPI_Common(box(-L / 2, y_lo, 0, L / 2, y_hi, top), ycyl(RC)).Shape()
    body = BRepAlgoAPI_Fuse(body, box(-L / 2, y_lo, 0, L / 2, y_hi, H0 - 0.5)).Shape()
    for a, b in ((y_lo, yc - CH / 2), (yc + CH / 2, y_hi)):           # 2 gờ bám theo cung
        lip = BRepAlgoAPI_Common(box(-L / 2, a, 0, L / 2, b, top + LIP_H), ycyl(RC + LIP_H)).Shape()
        body = BRepAlgoAPI_Fuse(body, lip).Shape()
    sock = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -0.1), gp_Dir(0, 0, 1)), SOCK_D / 2, SOCK_DEPTH + 0.1).Shape()
    return BRepAlgoAPI_Cut(body, sock).Shape(), y_lo, y_hi, top


OUT = os.path.join(ROOT, "assets", "Ranh_truot_cang_dai"); PR = os.path.join(ROOT, "assets", "Gui_xuong_in", "F_ranh_truot_cang_dai")
os.makedirs(OUT, exist_ok=True); os.makedirs(PR, exist_ok=True)
for e in (2.0, 3.0):
    s, y_lo, y_hi, top = make(e)
    name = f"Ranh_truot_cang_dai_lech_{e:g}mm_M4"
    write_step([(name, s)], os.path.join(OUT, name + ".step"))
    BRepMesh_IncrementalMesh(s, 0.01, False, 0.03, True)
    w = StlAPI_Writer(); w.ASCIIMode = False; assert w.Write(s, os.path.join(OUT, name + ".stl"))
    shutil.copy(os.path.join(OUT, name + ".stl"), PR)
    n, op, man, v = mesh_check(os.path.join(OUT, name + ".stl"))
    print(f"{name}: {L:g} × {y_hi - y_lo:.1f} mm, cao {top + LIP_H:.1f} mm; tâm rãnh cao hơn trục vít {e:g} mm; "
          f"gờ trên cách trục vít {y_hi:.1f} mm, đế dưới {-y_lo:.1f} mm; hợp lệ {BRepCheck_Analyzer(s).IsValid()}, "
          f"lưới kín {op == 0}, {v * 1000:.0f} mm3")
