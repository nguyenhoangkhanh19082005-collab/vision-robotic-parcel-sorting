"""Dựng mô hình băng tải từ station.yaml (layout.conveyor.model) và xuất STL, GLB, ảnh xem trước.

Dùng:  E:\\UDMT\\.venv\\Scripts\\python.exe scripts\\build_conveyor_model.py
Kết quả: assets\\conveyor\\conveyor.stl (mm, dùng cho CAD/in 3D), conveyor.glb (m, màu theo chi tiết),
         conveyor_preview.png
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main() -> int:
    import pyvista as pv

    from sorting_station.conveyor_model import ConveyorSpec, build_merged, build_parts
    from sorting_station.station_config import load_station_config

    config = load_station_config()
    spec = ConveyorSpec.from_config(config.get("layout", {}).get("conveyor", {}).get("model"))
    out = ROOT / "assets" / "conveyor"
    out.mkdir(parents=True, exist_ok=True)

    merged = build_merged(spec, scale=1.0)
    merged.save(str(out / "conveyor.stl"))

    plotter = pv.Plotter(off_screen=True, window_size=(1400, 900))
    plotter.set_background("#f2f4f7")
    for _name, mesh, color in build_parts(spec, scale=0.001):
        plotter.add_mesh(mesh, color=color, smooth_shading=True)
    plotter.export_gltf(str(out / "conveyor.glb"))
    plotter.camera_position = [(0.42, -0.48, 0.30), (0.0, 0.0, 0.045), (0.0, 0.0, 1.0)]
    plotter.add_axes()
    plotter.screenshot(str(out / "conveyor_preview.png"))
    plotter.close()

    xmin, xmax, ymin, ymax, zmin, zmax = merged.bounds
    print(f"Kích thước bao (mm): rộng {xmax - xmin:.1f} × dài {ymax - ymin:.1f} × cao {zmax - zmin:.1f}")
    print(f"Mặt băng cao {spec.belt_top_height:.1f} mm, rộng {spec.belt_width:.1f} mm")
    for name in ("conveyor.stl", "conveyor.glb", "conveyor_preview.png"):
        print("Đã ghi:", out / name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
