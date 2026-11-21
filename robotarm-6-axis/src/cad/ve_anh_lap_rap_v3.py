import os, sys, numpy as np
sys.path.insert(0, r"E:\UDMT\robotarm-6-axis\src\cad")
import cap_nhat_lap_rap_v3 as V
from rut_ngan_lib import com, AXES
import pyvista as pv
asm = V.read_named(os.path.join(V.OUT, "Roboter_v3_J4_cu_bau_ngan_giac_hut_dai.step"))
NEW = ("AX-1_3_cang_dai_giua_nhanh_J2", "AX-2_1_cang_dai_J3", "AX-2_3", "Ga_giac_hut_J6_dai", "Giac_hut_L60-M12_HSP-40_mo_phong")
up = AXES[0][1]; up = -up if up[1] > 0 else up
col = {"moi": "#f0883e", "bau": "#a371f7", "lui": "#58a6ff", "giu": "#c9d1d9"}
pts = []
pl = pv.Plotter(off_screen=True, window_size=(1600, 1100))
for k, (n, s) in enumerate(asm):
    if n in NEW: t = "moi"
    elif n == "AX-4_4_rut_ngan": t = "bau"
    else:
        sc, arm = V.s_of(com(s)); t = "lui" if (arm and sc > V.FUN_S0 - V.D_FUN + 0.5) else "giu"
    m = pv.read(os.path.join(r"E:\UDMT\robotarm-6-axis\.tmp\v3\mesh", f"{k:03d}.stl")); pl.add_mesh(m, color=col[t]); pts.append(m.center)
c = np.mean(pts, 0)
side = np.cross(up, AXES[1][1]); 
pl.camera.focal_point = c; pl.camera.position = c + 900 * (0.75 * AXES[1][1] / np.linalg.norm(AXES[1][1]) + 0.55 * side / np.linalg.norm(side) + 0.35 * up)
pl.camera.up = up; pl.camera.zoom(0.95)
pl.add_text("CAM = chi tiet moi/doi (vo J2 cang giua nhanh, vo J3 ban 18/11, nap J3 goc, ga giac hut dai)\nTIM = bau AX-4_4 rut ngan   XANH = cum co tay lui 16 mm (J5, J6)   XAM = giu nguyen (J1-J3 dong co 1:27, J4 cu)", font_size=10, color="black")
pl.set_background("white"); pl.screenshot(os.path.join(V.OUT, "anh_lap_rap_v3.png"))
