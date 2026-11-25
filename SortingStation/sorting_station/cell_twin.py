"""Digital twin 3D của cả trạm (PyVista/VTK): băng tải + giá camera + thùng phân loại + robot mesh CAD.

- Robot vẽ bằng 7 mesh STL thật, đặt theo ``RobotModel.link_transforms_base`` (cùng công thức
  với RobotSimulationWidget) nên tư thế trên twin trùng với GUI robot.
- Widget giữ API của ``machine_vision.ui.digital_twin.DigitalTwinWidget`` (set_process_state,
  set_conveyor_command_state, set_view, set_layer_visible, ...) để thay thế trực tiếp vào GUI
  MachineVision mà không phải sửa mã nguồn gốc.
- Nhiều view có thể cùng hiển thị một ``CellState``; view bị ẩn (tab khác) không render.
"""

from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget

from machine_vision.ui.digital_twin import ProcessStage
from sorting_station.cell_state import CellState
from sorting_station.conveyor_model import build_parts
from sorting_station.robot_model import RobotModel

try:
    import pyvista as pv
    from pyvistaqt import QtInteractor

    HAS_PYVISTA = True
except Exception:  # pragma: no cover - phụ thuộc môi trường
    pv = None
    QtInteractor = None
    HAS_PYVISTA = False

ROBOT_MESHES = (
    "base_link",
    "link_1_1",
    "link_2_1",
    "link_3_1",
    "link_4_1",
    "link_5_1",
    "link_6_1",
)
ROBOT_COLORS = ("#59636e", "#c8ced6", "#e88924", "#d9781d", "#c8ced6", "#bd6518", "#f4a460")
DECISION_COLORS = {"PASS": "#26a269", "REJECT": "#f85149", "MANUAL_REVIEW": "#d29922"}
TOOL_COLORS = {0: "#8b9aab", 1: "#2ee59d", 2: "#ffab00"}
_MESH_CACHE: dict[str, object] = {}


def _unicode_font() -> str | None:
    """Font TTF có dấu tiếng Việt (font mặc định của VTK thiếu glyph → mất chữ Ă, Ơ, Ư...)."""

    import os

    windows = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts"
    for name in ("segoeui.ttf", "tahoma.ttf", "arial.ttf"):
        if (windows / name).is_file():
            return str(windows / name)
    for candidate in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",):
        if Path(candidate).is_file():
            return candidate
    return None


FONT_FILE = _unicode_font()


def _load_mesh(path: Path, scale: float = 1.0):
    key = f"{path}|{scale}"
    if key not in _MESH_CACHE:
        mesh = pv.read(str(path))
        mesh.points *= 0.001 * scale  # STL theo mm (URDF scale 0.001) × tỉ lệ robot thật/CAD
        _MESH_CACHE[key] = mesh
    return _MESH_CACHE[key]


EQUIPMENT_DIR = Path(__file__).resolve().parents[1] / "assets" / "station_equipment"


def _load_equipment() -> list[tuple[object, str]]:
    """Gá camera + webcam + đèn ring + cảm biến (STL mm, hệ base robot) theo manifest.json; thiếu thư mục → []."""
    import json

    manifest = EQUIPMENT_DIR / "manifest.json"
    if not manifest.is_file():
        return []
    out = []
    for part in json.loads(manifest.read_text(encoding="utf-8")).get("parts", []):
        path = EQUIPMENT_DIR / part["file"]
        if path.is_file():
            out.append((_load_mesh(path), part.get("color", "#7a7f86")))
    return out


def _translation(xyz) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, 3] = xyz
    return matrix


class CellTwinView(QWidget):
    """View PyVista của CellState, tương thích API DigitalTwinWidget của MachineVision."""

    VIEW_PRESETS = {
        # (vị trí camera, tiêu điểm, hướng lên) — mét, hệ base robot
        "isometric": ((1.05, -1.15, 0.85), (0.10, -0.05, 0.16), (0.0, 0.0, 1.0)),
        "top": ((0.08, -0.05, 1.45), (0.08, -0.05, 0.0), (1.0, 0.0, 0.0)),
        "front": ((1.15, -0.05, 0.28), (0.10, -0.05, 0.15), (0.0, 0.0, 1.0)),
        "robot": ((0.62, -0.55, 0.55), (0.05, 0.0, 0.20), (0.0, 0.0, 1.0)),
    }

    def __init__(
        self,
        scene: CellState,
        model: RobotModel,
        mesh_dir: Path,
        compact: bool = False,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.scene = scene
        self.model = model
        self.mesh_dir = Path(mesh_dir)
        self.compact = compact
        self.layer_visibility = {"equipment": True, "vision_zones": True, "telemetry_labels": True}
        self.plotter = None
        self._actors: dict[str, list] = {"equipment": [], "vision_zones": []}
        self._robot_actors: list = []
        self._cleats: list = []
        self._label_actor = None
        self._label_text: tuple = ()
        self._belt_phase = 0.0
        self._last_tick = time.monotonic()
        self._dirty = True
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.setMinimumSize(320 if compact else 620, 280 if compact else 390)
        if not HAS_PYVISTA:
            layout.addWidget(self._fallback("Chưa cài PyVista/pyvistaqt — không dựng được twin 3D"))
            return
        try:
            self.plotter = QtInteractor(self)
            layout.addWidget(self.plotter.interactor)
            self._build_scene()
        except Exception as exc:  # pragma: no cover - lỗi OpenGL/driver
            self.plotter = None
            layout.addWidget(self._fallback(f"Lỗi khởi tạo twin 3D: {exc}"))
            return
        self.set_view("isometric")
        scene.changed.connect(self._mark_dirty)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(40)

    # ------------------------------------------------------------------ API MV
    @property
    def stage(self) -> ProcessStage:
        return self.scene.stage

    @property
    def decision(self) -> str | None:
        return self.scene.decision

    @property
    def conveyor_running(self) -> bool:
        return self.scene.conveyor_state in {"FWD", "REV"}

    @property
    def robot_angles(self) -> list[int]:
        return [int(round(v)) for v in self.scene.robot_display_deg()]

    def set_process_state(self, stage, decision=None, conveyor_running=None) -> None:
        self.scene.set_process(stage, decision)

    def set_conveyor_command_state(self, state: str) -> None:
        self.scene.set_conveyor(state)

    def set_joint_angle(self, joint: int, angle) -> None:
        if not 0 <= joint < 6:
            raise IndexError("Robot joint index must be from 0 to 5")
        command = list(self.scene.robot_command_deg)
        command[joint] = float(angle)
        self.scene.set_robot_command(command)

    def set_view(self, preset: str) -> None:
        if preset not in self.VIEW_PRESETS:
            raise ValueError(f"Unknown camera preset: {preset}")
        if self.plotter is None:
            return
        self.plotter.camera_position = list(self.VIEW_PRESETS[preset])
        self.plotter.camera.view_angle = 30.0
        self._dirty = True

    def reset_view(self) -> None:
        self.set_view("isometric")

    def set_layer_visible(self, layer: str, visible: bool) -> None:
        if layer not in self.layer_visibility:
            raise ValueError(f"Unknown Digital Twin layer: {layer}")
        self.layer_visibility[layer] = bool(visible)
        for actor in self._actors.get(layer, []):
            actor.SetVisibility(bool(visible))
        if layer == "equipment":
            for actor in self._robot_actors + self._cleats:
                actor.SetVisibility(bool(visible))
        self._label_text = ()  # buộc vẽ lại nhãn
        self._dirty = True

    def close_view(self) -> None:
        if self.plotter is not None:
            self._timer.stop()
            try:
                self.plotter.close()
            except Exception:
                pass
            self.plotter = None

    # ------------------------------------------------------------- dựng cảnh
    def _fallback(self, text: str) -> QLabel:
        label = QLabel(text)
        label.setWordWrap(True)
        label.setStyleSheet("color:#e0ad47; background:#080d12; padding:12px;")
        return label

    def _add(self, layer: str, mesh, **kwargs):
        actor = self.plotter.add_mesh(mesh, **kwargs)
        self._actors.setdefault(layer, []).append(actor)
        return actor

    def _build_scene(self) -> None:
        p = self.plotter
        lay = self.scene.layout
        p.set_background("#080d12", top="#13202c")
        if not self.compact:
            p.enable_anti_aliasing("ssaa")
            p.add_axes(line_width=2, labels_off=False)

        # Sàn lưới (m)
        floor = pv.Plane(center=(0.05, -0.15, -0.001), direction=(0, 0, 1), i_size=1.4, j_size=1.4,
                         i_resolution=14, j_resolution=14)
        p.add_mesh(floor, color="#1b2833", style="wireframe", line_width=1)

        # Băng tải: mô hình dựng theo kích thước thật (conveyor_model.py, không gồm động cơ)
        cx, w, ztop = lay.conveyor_center_x, lay.conveyor_width, lay.belt_top_z
        cy = (lay.conveyor_start_y + lay.conveyor_end_y) / 2.0
        for _name, mesh, color in build_parts(lay.conveyor_spec, scale=0.001):
            placed = mesh.translate((cx, cy, lay.conveyor_base_z), inplace=False)
            self._add("equipment", placed, color=color, smooth_shading=True)
        # Vạch mờ trên mặt băng (chỉ để thấy băng đang chạy; băng thật trơn)
        self._belt_flat_y0 = cy - lay.conveyor_spec.roller_y / 1000.0
        self._belt_flat_len = 2.0 * lay.conveyor_spec.roller_y / 1000.0
        for _ in range(6):
            cleat = pv.Box((cx - w / 2 + 0.008, cx + w / 2 - 0.008, -0.0015, 0.0015, ztop, ztop + 0.0006))
            self._cleats.append(p.add_mesh(cleat, color="#3f9a63"))
        self._cleat_spacing = self._belt_flat_len / len(self._cleats)

        # Camera (+ giá đỡ nếu đã lắp: layout.camera.show_mount)
        cam_z = ztop + lay.camera_height_above_belt
        equipment = _load_equipment() if lay.camera_show_mount else []
        if equipment:
            # 23/11/2026: gá camera/đèn/cảm biến thật của nhóm + webcam C260 + đèn ring + cảm biến M18 (mesh mm, hệ base,
            # xuất từ STEP toàn trạm — robotarm-6-axis/src/cad/tram_step_3_xuat_tram.py)
            for mesh, color in equipment:
                self._add("equipment", mesh, color=color, smooth_shading=True)
        elif lay.camera_show_mount:
            for x in (cx - w / 2 - 0.05, cx + w / 2 + 0.05):
                self._add("equipment", pv.Box((x - 0.01, x + 0.01, lay.camera_y - 0.01, lay.camera_y + 0.01,
                                               0.0, cam_z + 0.04)), color="#647787")
            self._add("equipment", pv.Box((cx - w / 2 - 0.06, cx + w / 2 + 0.06, lay.camera_y - 0.012,
                                           lay.camera_y + 0.012, cam_z + 0.03, cam_z + 0.05)), color="#708596")
        if not equipment:
            self._add("equipment", pv.Box((cx - 0.04, cx + 0.04, lay.camera_y - 0.03, lay.camera_y + 0.03,
                                           cam_z, cam_z + 0.03)), color="#2f81f7")
        # Vùng nhìn camera (frustum) — layer vision_zones. Góc nhìn webcam ≈ 58° × 46° → nửa vùng = 0.55 h × 0.42 h
        h = lay.camera_height_above_belt
        half_x, half_y = min(0.13, 0.55 * h), min(0.10, 0.42 * h)
        corners = [(cx - half_x, lay.camera_y - half_y), (cx + half_x, lay.camera_y - half_y),
                   (cx + half_x, lay.camera_y + half_y), (cx - half_x, lay.camera_y + half_y)]
        apex = np.array([cx, lay.camera_y, cam_z])
        for x, y in corners:
            self._add("vision_zones", pv.Line(apex, (x, y, ztop + 0.001)), color="#2f81f7", line_width=1)
        rect = [(x, y, ztop + 0.002) for x, y in corners]
        for a, b in zip(rect, rect[1:] + rect[:1]):
            self._add("vision_zones", pv.Line(a, b), color="#f85149", line_width=2)
        # Điểm gắp
        pick = lay.parcel_on_belt(lay.pick_y)
        self._add("vision_zones", pv.Disc(center=(pick[0], pick[1], ztop + 0.002), inner=0.035, outer=0.04,
                                          normal=(0, 0, 1), c_res=32), color="#f0a04b")

        # Thùng phân loại (hộp hở miệng)
        self._bin_tops: dict[str, tuple] = {}
        for spec in lay.bins:
            bx, by, bz = spec.center
            sx, sy, sz = spec.size
            t = 0.004
            for box in (
                (bx - sx / 2, bx + sx / 2, by - sy / 2, by + sy / 2, bz, bz + t),
                (bx - sx / 2, bx - sx / 2 + t, by - sy / 2, by + sy / 2, bz, bz + sz),
                (bx + sx / 2 - t, bx + sx / 2, by - sy / 2, by + sy / 2, bz, bz + sz),
                (bx - sx / 2, bx + sx / 2, by - sy / 2, by - sy / 2 + t, bz, bz + sz),
                (bx - sx / 2, bx + sx / 2, by + sy / 2 - t, by + sy / 2, bz, bz + sz),
            ):
                self._add("equipment", pv.Box(box), color=spec.color, opacity=0.85)
            self._bin_tops[spec.key] = (bx, by, bz + sz + 0.03)

        # Kiện hàng
        sx, sy, sz = lay.parcel_size
        self._parcel = p.add_mesh(pv.Box((-sx / 2, sx / 2, -sy / 2, sy / 2, -sz / 2, sz / 2)),
                                  color="#d29922", show_edges=True, edge_color="#f4d58d")
        self._parcel_label_box = p.add_mesh(
            pv.Box((-0.052, 0.052, -0.037, 0.037, sz / 2, sz / 2 + 0.001)), color="#f3f7fb")

        # Robot (mesh CAD)
        for name, color in zip(ROBOT_MESHES, ROBOT_COLORS):
            path = self.mesh_dir / f"{name}.stl"
            mesh = _load_mesh(path, self.model.mesh_scale) if path.is_file() else pv.Box((-0.02, 0.02, -0.02, 0.02, -0.02, 0.02))
            self._robot_actors.append(p.add_mesh(mesh, color=color, smooth_shading=True))
        # 24/11/2026: gá giác hút + giác hút L60-M12 / DP-40 thật (tách từ lắp ráp v3, hệ file mesh khâu 6 —
        # robotarm-6-axis/src/cad/tram_step_5_dung_cu_J6.py). Thiếu file → khối trụ dài tool_length như cũ.
        ga_path, cup_path = self.mesh_dir / "link_6_tool_ga.stl", self.mesh_dir / "link_6_tool_giac_hut.stl"
        self._tool_on_link6 = ga_path.is_file() and cup_path.is_file()
        if self._tool_on_link6:
            self._tool_ga = p.add_mesh(_load_mesh(ga_path, self.model.mesh_scale), color="#3b82f6", smooth_shading=True)
            self._tool = p.add_mesh(_load_mesh(cup_path, self.model.mesh_scale), color=TOOL_COLORS[0], smooth_shading=True)
        else:
            cup_length = max(self.model.tool_length_m, 0.005)
            cup = pv.Cylinder(center=(0, 0, -cup_length / 2), direction=(0, 0, 1), radius=0.011, height=cup_length)
            self._tool_ga = None
            self._tool = p.add_mesh(cup, color=TOOL_COLORS[0])
        self._robot_actors.append(self._tool)
        if self._tool_ga is not None:
            self._robot_actors.append(self._tool_ga)

        self._overlay_name = "cell_overlay"
        self._apply_state()

    # ----------------------------------------------------------- cập nhật
    def _mark_dirty(self) -> None:
        self._dirty = True

    def _tick(self) -> None:
        now = time.monotonic()
        dt, self._last_tick = now - self._last_tick, now
        running = self.scene.conveyor_state in {"FWD", "REV"}
        if running:
            direction = 1.0 if self.scene.conveyor_state == "FWD" else -1.0
            speed = 0.02 + 0.06 * min(self.scene.conveyor_speed, 255) / 255.0  # minh họa, chưa có encoder
            self._belt_phase += direction * speed * dt
            self._dirty = True
        if not self._dirty or self.plotter is None or not self.isVisible():
            return
        self._dirty = False
        self._apply_state()
        self.plotter.render()

    def _apply_state(self) -> None:
        scene, lay = self.scene, self.scene.layout
        q_deg = scene.robot_display_deg()
        transforms = self.model.link_transforms_base(np.radians(q_deg))
        for actor, matrix in zip(self._robot_actors[:7], transforms):
            actor.user_matrix = matrix
        tcp = self.model.fk(np.radians(q_deg))
        if self._tool_on_link6:
            self._tool.user_matrix = transforms[6]
            self._tool_ga.user_matrix = transforms[6]
        else:
            self._tool.user_matrix = tcp
        self._tool.prop.color = TOOL_COLORS.get(scene.tool_state, "#8b9aab")

        # Vạch băng tải
        y0, length = self._belt_flat_y0, self._belt_flat_len
        for index, actor in enumerate(self._cleats):
            y = y0 + (index * self._cleat_spacing + self._belt_phase) % length
            actor.user_matrix = _translation((0.0, y, 0.0))

        # Kiện: gắn vào đầu hút hoặc theo vị trí trong CellState
        if scene.parcel_attached:
            approach = tcp[:3, 2]
            center = tcp[:3, 3] + approach * (lay.parcel_size[2] / 2.0)
        else:
            center = np.asarray(scene.parcel_position)
        color = DECISION_COLORS.get(scene.decision or "", "#d29922")
        visible = scene.parcel_visible and self.layer_visibility["equipment"]
        for actor in (self._parcel, self._parcel_label_box):
            actor.user_matrix = _translation(center)
            actor.SetVisibility(visible)
        self._parcel.prop.color = color

        self._update_labels(tcp)
        self._update_overlay(q_deg)

    def _update_labels(self, tcp: np.ndarray) -> None:
        scene, lay = self.scene, self.scene.layout
        if not self.layer_visibility["telemetry_labels"]:
            if self._label_actor is not None:
                self.plotter.remove_actor(self._label_actor)
                self._label_actor = None
                self._label_text = ()
            return
        points, texts = [], []
        cam_z = lay.belt_top_z + lay.camera_height_above_belt + 0.08
        points.append((lay.conveyor_center_x, lay.camera_y, cam_z))
        texts.append("CAMERA / VISION" if lay.camera_show_mount else "CAMERA (vị trí dự kiến)")
        points.append((lay.conveyor_center_x, lay.conveyor_start_y + 0.05, lay.belt_top_z + 0.05))
        texts.append(f"BĂNG TẢI: {scene.conveyor_state}")
        for spec in lay.bins:
            points.append(self._bin_tops[spec.key])
            texts.append(f"{spec.label} ({scene.bin_counts.get(spec.key, 0)})")
        points.append(tuple(tcp[:3, 3] + np.array([0.0, 0.0, 0.06])))
        texts.append("TCP")
        key = tuple(texts)
        if key == self._label_text and self._label_actor is not None:
            return
        if self._label_actor is not None:
            self.plotter.remove_actor(self._label_actor)
        self._label_actor = self.plotter.add_point_labels(
            np.array(points), texts, font_size=9 if self.compact else 11, text_color="#d9e5f0",
            shape_color="#0b1722", shape_opacity=0.75, point_size=1, show_points=False,
            always_visible=True, reset_camera=False, font_file=FONT_FILE,
        )
        self._label_text = key

    def _update_overlay(self, q_deg) -> None:
        scene = self.scene
        source = "ACT" if scene.actual_is_fresh() else "CMD"
        link = scene.robot_link if scene.robot_link == "ONLINE" else "MÔ PHỎNG"
        tool = {0: "OFF", 1: "HÚT", 2: "NHẢ"}.get(scene.tool_state, "?")
        joints = " ".join(f"{v:+.0f}" for v in q_deg)
        lines = [
            f"BĂNG TẢI: {scene.conveyor_state}  ·  STAGE: {scene.stage.value}  ·  KQ: {scene.decision or '—'}",
            f"ROBOT: {link} ({source})  ·  ĐẦU HÚT: {tool}",
            f"J[°]: {joints}",
            f"CHU TRÌNH: {scene.sequence_text}",
        ]
        if self.compact:
            lines = lines[:2] + lines[3:]
        self.plotter.add_text("\n".join(lines), position="upper_left", font_size=7 if self.compact else 9,
                              color="#b8c5d1", name=self._overlay_name, font_file=FONT_FILE)
