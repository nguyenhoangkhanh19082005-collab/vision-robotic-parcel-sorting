r"""Render kiểm tra file STEP toàn trạm (từ lưới STL bước 3 xuất kèm) → assets/CAD_tram_tong/tram_phan_loai_tong.png
    E:\UDMT\robotarm-6-axis\.venv\Scripts\python.exe src\cad\tram_step_4_render.py
"""
import glob, os
import pyvista as pv
from PIL import Image
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
files = sorted(glob.glob(os.path.join(ROOT, ".tmp", "tram_step", "stl", "*.stl")))
views = [((1.6, -1.3, 1.1), "iso", (0.12, -0.1, 0.2)), ((0.15, -1.9, 0.35), "truoc", (0.12, -0.1, 0.2)),
         ((0.15, -0.15, 2.2), "tren", (0.12, -0.1, 0.2)), ((0.75, -0.55, 0.42), "gia", (0.22, 0.0, 0.12)),
         ((-0.6, -0.55, 0.3), "bom_van", (-0.19, -0.06, -0.03)), ((0.3, -0.35, 0.85), "dau_hut", (0.0, 0.0, 0.72))]
ims = []
for pos, name, foc in views:
    pl = pv.Plotter(off_screen=True, window_size=(900, 800))
    for f in files:
        col = "#" + os.path.basename(f).split("__")[2][:6]
        pl.add_mesh(pv.read(f).scale(0.001, inplace=False), color=col, smooth_shading=True)
    pl.camera.position = pos; pl.camera.focal_point = foc; pl.camera.up = (0, 0, 1) if name != "tren" else (0, 1, 0)
    pl.set_background("white"); pl.enable_anti_aliasing("ssaa")
    ims.append(Image.fromarray(pl.screenshot(return_img=True))); pl.close()
out = Image.new("RGB", (900 * len(ims), 800), "white")
for i, im in enumerate(ims): out.paste(im, (900 * i, 0))
dst = os.path.join(ROOT, "assets", "CAD_tram_tong", "tram_phan_loai_tong.png"); out.save(dst); print(dst)
