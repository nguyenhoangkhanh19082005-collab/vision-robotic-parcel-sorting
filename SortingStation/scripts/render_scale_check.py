"""Ảnh đối chiếu tỉ lệ robot (CAD × robot.scale) với băng tải thật — chiếu song song, lưới 50 mm.

Dùng:  E:\\UDMT\\.venv\\Scripts\\python.exe scripts\\render_scale_check.py
Kết quả: reports\\scale_check.png
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main() -> int:
    from sorting_station.station_config import install_import_paths, load_station_config, resolve_paths

    config = load_station_config()
    paths = resolve_paths(config)
    install_import_paths(paths)

    import pyvista as pv

    from sorting_station.cell_state import CellLayout
    from sorting_station.cell_twin import FONT_FILE, ROBOT_COLORS, ROBOT_MESHES
    from sorting_station.conveyor_model import build_parts
    from sorting_station.robot_model import build_model_from_robot_project

    robot_cfg = config["robot"]
    scale = float(robot_cfg.get("scale", 1.0))
    model = build_model_from_robot_project(paths.robot_root, tool_length_m=robot_cfg["tool_length_mm"] / 1000.0,
                                           scale=scale)
    layout = CellLayout.from_config(config["layout"])
    mesh_dir = paths.robot_root / "assets" / "ROBOTARM2_description" / "meshes"

    plotter = pv.Plotter(off_screen=True, window_size=(1600, 900))
    plotter.set_background("#ffffff")
    # Robot tư thế 0 (chữ L), đặt cạnh băng tải trên cùng mặt phẳng để so kích thước trực quan
    for name, color, matrix in zip(ROBOT_MESHES, ROBOT_COLORS, model.link_transforms_base(np.zeros(6))):
        mesh = pv.read(str(mesh_dir / f"{name}.stl"))
        mesh.points *= 0.001 * scale
        plotter.add_mesh(mesh.transform(matrix, inplace=False), color=color, smooth_shading=True)
    # Băng tải xoay dọc trục X, đặt bên trái robot (chỉ cho ảnh này) để nhìn ngang thấy cả hai
    conveyor_x = -0.42
    for _name, mesh, color in build_parts(layout.conveyor_spec, scale=0.001):
        placed = mesh.rotate_z(90.0, inplace=False).translate((conveyor_x, 0.0, layout.conveyor_base_z),
                                                               inplace=False)
        plotter.add_mesh(placed, color=color, smooth_shading=True)

    # Lưới 50 mm phía sau (mặt phẳng X–Z)
    y_back = 0.20
    xs = np.arange(-0.65, 0.40, 0.05)
    zs = np.arange(0.0, 0.50, 0.05)
    for x in xs:
        plotter.add_mesh(pv.Line((x, y_back, 0.0), (x, y_back, zs[-1])),
                         color="#c0c6cc" if round(x * 1000) % 100 else "#8a939c", line_width=1)
    for z in zs:
        plotter.add_mesh(pv.Line((xs[0], y_back, z), (xs[-1], y_back, z)),
                         color="#c0c6cc" if round(z * 1000) % 100 else "#8a939c", line_width=1)
    labels = [(xs[0] - 0.03, y_back, z) for z in zs[::2]]
    plotter.add_point_labels(np.array(labels), [f"{z * 1000:.0f}" for z in zs[::2]], font_size=12,
                             show_points=False, shape=None, text_color="#333333", font_file=FONT_FILE)

    tcp = model.fk(np.zeros(6))[:3, 3]
    shoulder = (model.points[1] - model.cad_from_base[:3, 3]) @ model.axes[0]
    text = (
        f"robot.scale = {scale:.3f}  (1.0 = đúng CAD Roboter.STEP)\n"
        f"Robot: trục vai cao {shoulder * 1000:.0f} mm · TCP ở tư thế 0 cao {tcp[2] * 1000:.0f} mm\n"
        f"Băng tải: dài {layout.conveyor_spec.length_total:.0f} mm · mặt băng cao "
        f"{layout.conveyor_spec.belt_top_height:.0f} mm · rộng {layout.conveyor_spec.belt_width:.0f} mm\n"
        "Lưới 50 mm (đường đậm 100 mm), chiếu song song — không méo phối cảnh"
    )
    plotter.add_text(text, position="upper_left", font_size=11, color="#222222", font_file=FONT_FILE)
    plotter.enable_parallel_projection()
    plotter.camera_position = [(-0.12, -1.5, 0.22), (-0.12, 0.0, 0.22), (0.0, 0.0, 1.0)]
    plotter.reset_camera()
    plotter.camera.zoom(1.25)
    out = ROOT / "reports" / "scale_check.png"
    plotter.screenshot(str(out))
    plotter.close()
    print("Đã ghi:", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
