"""Mô hình 3D tham số hóa của băng tải thật (không gồm động cơ, puly, đai truyền).

Kích thước mặc định ước lượng từ ảnh trong ``assets/conveyor_photos`` (21/10/2026):
- 05_tu_tren.jpg có thước dây: dài tổng ≈ 400 mm, mặt băng rộng ≈ 100 mm, rộng ngoài ≈ 122 mm.
- 01_mat_ben_trai.jpg (tỉ lệ ≈ 3.6 px/mm theo chiều dài 400 mm): mặt băng cao ≈ 82 mm so với tấm đế,
  thanh nhôm cao ≈ 25 mm, nắp đầu bo tròn cao ≈ 24 mm, chân ở ≈ 115 mm và ≈ 280 mm tính từ đầu trái.
- 03_dau_vao.jpg / 07_goc_45_truoc_phai.jpg: chân là thanh thép dẹt chữ L, chân đế gập vào trong.
Giá trị nào chưa đo bằng thước đều ghi "ước lượng" — sửa trong ``layout.conveyor.model`` của station.yaml.

Hệ tọa độ cục bộ (mm): Y dọc băng tải (0 ở giữa), X ngang băng (0 ở đường tâm), Z = 0 ở mặt đặt chân.
"""

from __future__ import annotations

from dataclasses import dataclass, fields

BELT_COLOR = "#1f6b3a"
FRAME_COLOR = "#b9c0c7"
CAP_COLOR = "#6b7076"
LEG_COLOR = "#9aa3ab"


@dataclass(frozen=True)
class ConveyorSpec:
    length_total: float = 400.0      # mm, đầu nắp này tới đầu nắp kia (đo bằng thước)
    belt_width: float = 100.0        # mm (đo bằng thước)
    belt_top_height: float = 82.0    # mm so với mặt đặt chân (ước lượng từ ảnh)
    belt_thickness: float = 2.0
    roller_radius: float = 10.0      # ước lượng
    rail_height: float = 25.0        # thanh nhôm định hình hai bên (ước lượng)
    rail_thickness: float = 9.0
    cap_length: float = 65.0         # nắp đầu băng (in 3D), tính cả phần bo tròn
    cap_thickness: float = 11.0
    cap_height: float = 24.0
    leg_positions: tuple[float, float] = (-85.0, 80.0)  # vị trí chân dọc Y (mm, từ tâm)
    leg_width: float = 15.0
    leg_thickness: float = 1.5
    leg_foot_length: float = 30.0

    @staticmethod
    def from_config(section: dict | None) -> "ConveyorSpec":
        if not section:
            return ConveyorSpec()
        known = {f.name for f in fields(ConveyorSpec)}
        values = {}
        for key, value in section.items():
            if key not in known:
                raise ValueError(f"Tham số mô hình băng tải không hợp lệ: {key}")
            values[key] = tuple(float(v) for v in value) if isinstance(value, (list, tuple)) else float(value)
        spec = ConveyorSpec(**values)
        if spec.roller_radius * 2 >= spec.belt_top_height:
            raise ValueError("Bán kính rulô quá lớn so với chiều cao mặt băng")
        return spec

    # Các đại lượng suy ra -------------------------------------------------
    @property
    def roller_center_z(self) -> float:
        return self.belt_top_height - self.belt_thickness - self.roller_radius

    @property
    def roller_y(self) -> float:
        """Tâm rulô dọc Y: nắp đầu bo tròn đồng tâm với rulô."""
        return self.length_total / 2.0 - self.cap_height / 2.0

    @property
    def outer_width(self) -> float:
        return self.belt_width + 2.0 * (self.cap_thickness + 1.0)


def build_parts(spec: ConveyorSpec, scale: float = 0.001) -> list[tuple[str, object, str]]:
    """Trả về danh sách (tên, mesh PyVista, màu). ``scale`` mặc định đổi mm → m."""

    import pyvista as pv  # noqa: PLC0415 - chỉ cần khi dựng hình

    s = spec
    half_w = s.belt_width / 2.0
    ry, rz = s.roller_y, s.roller_center_z
    r_out = s.roller_radius + s.belt_thickness
    parts: list[tuple[str, object, str]] = []

    def box(name, x0, x1, y0, y1, z0, z1, color):
        parts.append((name, pv.Box((x0, x1, y0, y1, z0, z1)), color))

    # Băng tải: mặt trên, nhánh hồi phía dưới, hai đoạn ôm rulô (nửa ống trụ)
    box("belt_top", -half_w, half_w, -ry, ry, s.belt_top_height - s.belt_thickness, s.belt_top_height, BELT_COLOR)
    box("belt_return", -half_w, half_w, -ry, ry, rz - r_out, rz - s.roller_radius, BELT_COLOR)
    for sign, label in ((1.0, "out"), (-1.0, "in")):
        wrap = pv.CylinderStructured(
            radius=[s.roller_radius, r_out], height=s.belt_width, center=(0.0, sign * ry, rz),
            direction=(1.0, 0.0, 0.0), theta_resolution=40, z_resolution=2,
        ).extract_surface(algorithm="dataset_surface")
        # Chỉ giữ nửa ống phía ngoài đầu băng (nửa trong bị mặt băng che)
        wrap = wrap.clip(normal=(0.0, -sign, 0.0), origin=(0.0, sign * ry, rz))
        parts.append((f"belt_wrap_{label}", wrap, BELT_COLOR))
        roller = pv.Cylinder(center=(0.0, sign * ry, rz), direction=(1.0, 0.0, 0.0),
                             radius=s.roller_radius, height=s.belt_width + 2.0, resolution=32)
        parts.append((f"roller_{label}", roller, FRAME_COLOR))

    # Thanh nhôm hai bên (giữa hai nắp đầu)
    rail_y = s.length_total / 2.0 - s.cap_length
    rail_top = s.belt_top_height - s.belt_thickness - 1.0
    for side in (-1.0, 1.0):
        inner = half_w + 1.0
        x0, x1 = sorted((side * inner, side * (inner + s.rail_thickness)))
        box(f"rail_{'L' if side < 0 else 'R'}", x0, x1, -rail_y, rail_y, rail_top - s.rail_height, rail_top,
            FRAME_COLOR)

    # Nắp đầu (in 3D): khối hộp + đầu bo tròn đồng tâm rulô, 4 chiếc
    for side in (-1.0, 1.0):
        inner = half_w + 1.0
        x0, x1 = sorted((side * inner, side * (inner + s.cap_thickness)))
        for end in (-1.0, 1.0):
            y_inner = end * (s.length_total / 2.0 - s.cap_length)
            y0, y1 = sorted((y_inner, end * ry))
            tag = f"cap_{'L' if side < 0 else 'R'}_{'in' if end < 0 else 'out'}"
            box(tag, x0, x1, y0, y1, rz - s.cap_height / 2.0, rz + s.cap_height / 2.0, CAP_COLOR)
            nose = pv.Cylinder(center=((x0 + x1) / 2.0, end * ry, rz), direction=(1.0, 0.0, 0.0),
                               radius=s.cap_height / 2.0, height=x1 - x0, resolution=32)
            parts.append((tag + "_nose", nose, CAP_COLOR))

    # Chân: thanh thép dẹt bắt vào mặt trong thanh nhôm, chân đế gập vào trong
    leg_top = rail_top - s.rail_height
    for y in s.leg_positions:
        for side in (-1.0, 1.0):
            inner = half_w + 1.0
            x0, x1 = sorted((side * inner, side * (inner - s.leg_thickness)))
            y0, y1 = y - s.leg_width / 2.0, y + s.leg_width / 2.0
            tag = f"leg_{'L' if side < 0 else 'R'}_{y:+.0f}"
            box(tag, x0, x1, y0, y1, 0.0, leg_top + 10.0, LEG_COLOR)  # chồng 10 mm lên thanh nhôm (bắt vít)
            fx0, fx1 = sorted((side * inner, side * (inner - s.leg_foot_length)))
            box(tag + "_foot", fx0, fx1, y0, y1, 0.0, s.leg_thickness, LEG_COLOR)

    if scale != 1.0:
        parts = [(name, mesh.scale(scale, inplace=False), color) for name, mesh, color in parts]
    return parts


def build_merged(spec: ConveyorSpec, scale: float = 0.001):
    """Một mesh duy nhất (tam giác) để xuất STL."""

    import pyvista as pv  # noqa: PLC0415

    merged = pv.merge([mesh.triangulate() for _name, mesh, _color in build_parts(spec, scale)])
    return merged.extract_surface(algorithm="dataset_surface").triangulate()
