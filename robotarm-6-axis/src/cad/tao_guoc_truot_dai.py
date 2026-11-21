r"""GUỐC TRƯỢT CĂNG ĐAI chụp lên đầu vít M4 (J2, J3).

Vấn đề: đầu vít M4 tì vào lưng đai GT2 6 mm — đai hẹp, khi khớp quay đai xê dịch ngang và tuột khỏi điểm tì.
Giải pháp: guốc nhựa có RÃNH (đáy rãnh cong lồi dọc theo đai + 2 gờ chặn hai bên) đội lên đầu vít:
  - rãnh rộng 7.0 mm (đai 6 mm + 1 mm hở) → đai không thể trượt ra khỏi guốc;
  - đáy rãnh cong R20 dọc chiều chạy của đai → đai ôm êm, không có cạnh vấp, hai đầu thấp dần (dẫn đai vào);
  - lỗ Ø4.5 không ren ở mặt dưới, sâu 5.5 mm: guốc TỰ XOAY trên đầu vít → gờ rãnh bị chính đai kéo thẳng hàng,
    vặn vít để căng đai mà guốc không xoay theo.
Toạ độ: x dọc đai, y ngang đai, z hướng vào đai (mặt tì ở trên). In: mặt có lỗ đặt xuống bàn in, rãnh hướng lên.
Đầu ra: assets/Guoc_truot_dai/ (STEP + STL) và bản sao STL ở assets/Gui_xuong_in/D_guoc_truot_dai/.
    E:\UDMT\.venv\Scripts\python.exe -X utf8 src\cad\tao_guoc_truot_dai.py [bề_rộng_đai_mm]
"""
import os, sys, shutil
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.dirname(__file__)); sys.path.insert(0, os.path.join(ROOT, "research"))
from tao_chi_tiet_dong_co_1_27 import write_step, write_stl, vol
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

BELT_W = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0   # GT2 6 mm (đai đang dùng 230-2GT-6)
CH = BELT_W + 1.0      # rãnh: hở 0.5 mm mỗi bên (bù sai số in)
H0 = 5.0               # đế
RC = 20.0              # bán kính cong đáy rãnh dọc đai
SOCK_D, SOCK_DEPTH = 4.5, 5.5   # lỗ chụp đầu vít M4 (không ren, xoay tự do)
# 20/11/2026 — 2 biến thể (gờ dày 1.6 → rộng 10.2 mm: J3 ở đỉnh rãnh vít còn cách nắp AX-2_3 0.3 mm):
#   J3: dài 18, gờ cao 3.5 (nhô hơn mặt răng đai 2.1 mm), vồng 2.0
#   J2: dài 14, gờ cao 2.5 (nhô hơn mặt răng đai 1.1 mm), vồng 1.5 — đặt gần puly 55 mm, gờ cao/dài sẽ chạm vành puly
VARIANTS = {"J3": dict(L=18.0, LIP_T=1.6, LIP_H=3.5, CROWN=2.0), "J2": dict(L=14.0, LIP_T=1.6, LIP_H=2.5, CROWN=1.5)}


def box(x0, y0, z0, x1, y1, z1):
    return BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), x1 - x0, y1 - y0, z1 - z0).Shape()


def make_shoe(L, LIP_T, LIP_H, CROWN):
    W = CH + 2 * LIP_T
    def ycyl(r, zc):  # trụ trục y (dài qua hết bề ngang), tâm tại x = 0, z = zc
        return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, -W, zc), gp_Dir(0, 1, 0)), r, 2 * W).Shape()
    top = H0 + CROWN
    zc = top - RC                                   # tâm cung đáy rãnh
    body = BRepAlgoAPI_Common(box(-L / 2, -W / 2, 0, L / 2, W / 2, top), ycyl(RC, zc)).Shape()
    body = BRepAlgoAPI_Fuse(body, box(-L / 2, -W / 2, 0, L / 2, W / 2, H0 - 0.5)).Shape()   # đế phẳng đủ cao ở 2 đầu
    for y0, y1 in ((-W / 2, -CH / 2), (CH / 2, W / 2)):                                         # 2 gờ bám theo cung
        lip = BRepAlgoAPI_Common(box(-L / 2, y0, 0, L / 2, y1, top + LIP_H), ycyl(RC + LIP_H, zc)).Shape()
        body = BRepAlgoAPI_Fuse(body, lip).Shape()
    sock = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -0.1), gp_Dir(0, 0, 1)), SOCK_D / 2, SOCK_DEPTH + 0.1).Shape()
    return BRepAlgoAPI_Cut(body, sock).Shape(), W, top


OUT = os.path.join(ROOT, "assets", "Guoc_truot_dai"); PR = os.path.join(ROOT, "assets", "Gui_xuong_in", "E_cang_dai_v2_mat_bich")
os.makedirs(OUT, exist_ok=True); os.makedirs(PR, exist_ok=True)
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
for j, v in VARIANTS.items():
    shoe, W, top = make_shoe(**v)
    name = f"Guoc_truot_dai_{j}_{BELT_W:g}mm_M4"
    write_step([(name, shoe)], os.path.join(OUT, name + ".step"))
    BRepMesh_IncrementalMesh(shoe, 0.01, False, 0.03, True)   # lưới mịn: đáy rãnh cong không gãy khúc
    _w = StlAPI_Writer(); _w.ASCIIMode = False; assert _w.Write(shoe, os.path.join(OUT, name + ".stl"))
    shutil.copy(os.path.join(OUT, name + ".stl"), PR)
    L, LIP_H = v["L"], v["LIP_H"]
    print(f"{name}: {L:g} × {W:g} mm, cao {top + LIP_H:g} mm (đáy rãnh giữa {top:g} mm), rãnh {CH:g} mm, gờ {LIP_H:g} mm, "
          f"lỗ Ø{SOCK_D} sâu {SOCK_DEPTH}; thể tích {vol(shoe):.0f} mm3; hợp lệ {BRepCheck_Analyzer(shoe).IsValid()}; "
          f"đáy rãnh 2 đầu thấp hơn giữa {RC - (RC ** 2 - (L / 2) ** 2) ** 0.5:.2f} mm")
