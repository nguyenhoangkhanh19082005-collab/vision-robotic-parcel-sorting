"""
robot_simulation.py
====================
Module mô phỏng robot 6 trục từ file URDF, sử dụng PyVista.
Nhúng vào frame `frm_main_simulation` của giao diện PyQt6.
"""

import sys
import os
import math
import numpy as np
from xml.etree import ElementTree as ET

if sys.platform == "win32":
    try:
        if hasattr(sys.stdout, 'reconfigure'):
            sys.stdout.reconfigure(encoding='utf-8', errors='replace')
        if hasattr(sys.stderr, 'reconfigure'):
            sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

try:
    import pyvista as pv
    from pyvistaqt import QtInteractor
    HAS_PYVISTA = True
except ImportError:
    pv = None
    QtInteractor = None
    HAS_PYVISTA = False

try:
    import vtkmodules.vtkCommonCore as vtk_core
    vtk_core.vtkObject.GlobalWarningDisplayOff()
    import vtkmodules.vtkCommonMath as vtk_math
except Exception:
    vtk_math = None

# ─────────────────────────────────────────────────────────────────────────────
# Hằng số
# ─────────────────────────────────────────────────────────────────────────────
JOINT_ORDER = [
    "Revolute 8",   # J1 - Base       (continuous)
    "Revolute 9",   # J2 - Shoulder   (revolute)
    "Revolute 10",  # J3 - Elbow      (revolute)
    "Revolute 11",  # J4 - Wrist 1    (continuous)
    "Revolute 12",  # J5 - Wrist 2    (revolute)
    "Revolute 13",  # J6 - Wrist 3    (continuous)
]

LINK_COLORS = [
    "#B0B0B0",  # base_link
    "#C0C8D0",  # link_1_1
    "#A8C0D8",  # link_2_1
    "#90B8D0",  # link_3_1
    "#78B0C8",  # link_4_1
    "#60A8C0",  # link_5_1
    "#F4A460",  # link_6_1
]

# Tọa độ tâm trục và vector trục quay chuẩn xác từ mô hình cơ khí CAD (Đơn vị: mét)
JOINT_POINTS = [
    np.array([-0.1544785,  0.1454204, -0.0507776]), # Joint 1 (Base - Khớp 1)
    np.array([-0.1538267,  0.1394184,  0.0824138]), # Joint 2 (Shoulder - Vai)
    np.array([-0.1373280,  0.1339924,  0.2348735]), # Joint 3 (Elbow - Khuỷu)
    np.array([-0.2154871,  0.0110950,  0.2594626]), # Joint 4 (Forearm Roll - Xoay cẳng tay)
    np.array([-0.2646521, -0.0790322,  0.2838017]), # Joint 5 (Wrist Pitch - Gập cổ tay Pw)
    np.array([-0.2774943, -0.1114704,  0.2813030]), # Joint 6 (Tool Flange - Đầu hút)
]

JOINT_AXES = [
    np.array([-0.000797, -0.083636,  0.996496]),
    np.array([ 0.872438, -0.487070, -0.040182]),
    np.array([ 0.872438, -0.487070, -0.040182]),
    np.array([ 0.466352,  0.854273, -0.229639]),
    np.array([-0.815770,  0.515717,  0.261830]),
    np.array([-0.540858, -0.840599, -0.029428]),
]

# 15/11/2026 — Hình học chính xác lấy từ file lắp ráp STEP (dong_hoc_step.py, sinh bởi src/cad/tinh_dong_hoc_tu_step.py):
# - JOINT_POINTS cũ ở trên lệch trục khớp thật 3–13 mm vì mesh từng khâu của bản xuất URDF chưa ghép khít;
# - tư thế 0 cũ là tư thế vẽ CAD (cánh tay trên nghiêng 5.9°, cẳng tay chúc lên 17.5°, J4 xoay 13.2°, cổ tay gập 15.5°).
# Tư thế 0 = CHỮ L ĐẦU HÚT CHÚC XUỐNG từ 24/11/2026 (cánh tay trên đứng, cẳng tay ngang, trục J6 chỉ xuống). Dựng thẳng đứng (22/11) = J3 −90°, J5 +90°.
# LINK_MESH_TRANSFORMS: ma trận đặt lại mesh từng khâu cho khớp với điểm/trục mới. Hằng số cũ giữ làm dự phòng.
LINK_MESH_TRANSFORMS: dict = {}
try:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import dong_hoc_step as _geo
    JOINT_POINTS, JOINT_AXES, LINK_MESH_TRANSFORMS = _geo.JOINT_POINTS, _geo.JOINT_AXES, _geo.LINK_MESH_TRANSFORMS
except ImportError:
    print("[robot_simulation] Không tìm thấy dong_hoc_step.py — dùng hình học cũ (kém chính xác).")


# 24/11/2026: dụng cụ gắn khâu 6 (gá giác hút + giác hút L60-M12 / DP-40), STL mm trong HỆ FILE mesh link_6_1
# (xuất bởi src/cad/tram_step_5_dung_cu_J6.py) → dùng chung tỉ lệ + LINK_MESH_TRANSFORMS + biến đổi của link_6_1.
TOOL_MESHES = {"link_6_tool_ga": ("link_6_1", "#3B82F6"), "link_6_tool_giac_hut": ("link_6_1", "#9CA3AF")}
TOOL_STATE_COLORS = {0: "#9CA3AF", 1: "#22C55E", 2: "#F59E0B"}


def rot_axis_4x4(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    norm = np.linalg.norm(axis)
    if norm < 1e-9 or abs(angle_rad) < 1e-9:
        return np.eye(4)
    u = axis / norm
    ct = math.cos(angle_rad)
    st = math.sin(angle_rad)
    cross = np.array([
        [0.0, -u[2], u[1]],
        [u[2], 0.0, -u[0]],
        [-u[1], u[0], 0.0]
    ])
    R = ct * np.eye(3) + st * cross + (1.0 - ct) * np.outer(u, u)
    T = np.eye(4)
    T[:3, :3] = R
    return T

def make_trans_4x4(xyz: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = xyz
    return T

def rotation_matrix_axis_angle(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    t = 1 - c
    x, y, z = axis
    return np.array([
        [t*x*x + c,   t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z, t*y*y + c,   t*y*z - s*x],
        [t*x*z - s*y, t*y*z + s*x, t*z*z + c  ],
    ])

def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    Rz = rotation_matrix_axis_angle(np.array([0, 0, 1]), yaw)
    Ry = rotation_matrix_axis_angle(np.array([0, 1, 0]), pitch)
    Rx = rotation_matrix_axis_angle(np.array([1, 0, 0]), roll)
    return Rz @ Ry @ Rx

def make_transform(rot: np.ndarray, trans: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = trans
    return T

def parse_xyz(text: str) -> np.ndarray:
    return np.array([float(v) for v in text.strip().split()])

def parse_rpy(text: str) -> tuple:
    vals = [float(v) for v in text.strip().split()]
    return tuple(vals)

class URDFParser:
    def __init__(self, urdf_path: str):
        self.urdf_path = os.path.abspath(urdf_path)
        self.urdf_dir  = os.path.dirname(self.urdf_path)
        self.tree      = ET.parse(self.urdf_path)
        self.root      = self.tree.getroot()

        self.links:  dict = {}
        self.joints: dict = {}
        self._parse()

    def _resolve_mesh_path(self, filename: str) -> str | None:
        if filename.startswith("package://"):
            rest = filename[len("package://"):]
            parts = rest.split("/", 1)
            rel_path = parts[1] if len(parts) > 1 else rest

            candidate = os.path.join(self.urdf_dir, rel_path)
            if os.path.isfile(candidate):
                return candidate

            basename = os.path.basename(rel_path)
            for root, _, files in os.walk(self.urdf_dir):
                if basename in files:
                    return os.path.join(root, basename)

            return None

        if os.path.isabs(filename):
            return filename if os.path.isfile(filename) else None
        candidate = os.path.join(self.urdf_dir, filename)
        return candidate if os.path.isfile(candidate) else None

    def _parse(self):
        for link_el in self.root.findall("link"):
            name = link_el.get("name")
            info = {"mesh_path": None, "visual_xyz": np.zeros(3), "visual_rpy": (0, 0, 0)}

            visual = link_el.find("visual")
            if visual is not None:
                origin = visual.find("origin")
                if origin is not None:
                    xyz_str = origin.get("xyz", "0 0 0")
                    rpy_str = origin.get("rpy", "0 0 0")
                    info["visual_xyz"] = parse_xyz(xyz_str)
                    info["visual_rpy"] = parse_rpy(rpy_str)

                geom = visual.find("geometry")
                if geom is not None:
                    mesh_el = geom.find("mesh")
                    if mesh_el is not None:
                        raw = mesh_el.get("filename", "")
                        scale_str = mesh_el.get("scale", "1 1 1")
                        info["mesh_path"] = self._resolve_mesh_path(raw)
                        info["mesh_scale"] = parse_xyz(scale_str)

            self.links[name] = info

        for joint_el in self.root.findall("joint"):
            name      = joint_el.get("name")
            jtype     = joint_el.get("type", "fixed")
            parent_el = joint_el.find("parent")
            child_el  = joint_el.find("child")
            origin_el = joint_el.find("origin")
            axis_el   = joint_el.find("axis")

            parent = parent_el.get("link") if parent_el is not None else None
            child  = child_el.get("link")  if child_el  is not None else None

            xyz = np.zeros(3)
            rpy = (0.0, 0.0, 0.0)
            if origin_el is not None:
                xyz = parse_xyz(origin_el.get("xyz", "0 0 0"))
                rpy = parse_rpy(origin_el.get("rpy", "0 0 0"))

            axis = np.array([0.0, 0.0, 1.0])
            if axis_el is not None:
                axis = parse_xyz(axis_el.get("xyz", "0 0 1"))

            self.joints[name] = {
                "type": jtype, "parent": parent, "child": child,
                "origin_xyz": xyz, "origin_rpy": rpy, "axis": axis,
            }

class RobotSimulationWidget:
    def __init__(
        self,
        parent_frame,
        urdf_dir:  str = "assets/ROBOTARM2_description",
        urdf_file: str = "PUBY.urdf",
        background: str = "#EBEDF3",
    ):
        self.parent_frame  = parent_frame
        self.urdf_path     = os.path.join(urdf_dir, urdf_file)
        self.background    = background

        self._parser:       URDFParser | None = None
        self._plotter      = None
        self._joint_angles: list[float] = [0.0] * 6
        self._actors:       dict = {}
        self._meshes:       dict = {}
        self._ready:        bool = False

    def setup(self):
        if not HAS_PYVISTA or QtInteractor is None:
            print("[RobotSim] Thư viện PyVista/QtInteractor chưa được cài đặt. Hiển thị thông báo Fallback.")
            self._show_fallback()
            return

        try:
            self._parser = URDFParser(self.urdf_path)
        except Exception as e:
            print(f"[RobotSim] Lỗi đọc URDF: {e}")
            self._show_fallback()
            return

        layout = self.parent_frame.layout()
        if layout is None:
            from PyQt6.QtWidgets import QHBoxLayout
            layout = QHBoxLayout(self.parent_frame)
            layout.setContentsMargins(0, 0, 0, 0)

        self._plotter = QtInteractor(self.parent_frame)
        self._plotter.set_background(self.background)
        layout.addWidget(self._plotter.interactor)

        self._load_meshes()
        self._build_scene()
        self._ready = True

        self._plotter.camera_position = "iso"
        self._plotter.reset_camera()
        try:
            self._plotter.render()
        except Exception:
            pass
        print("[RobotSim] Khởi tạo thành công.")

    def update_joints(self, angles_deg: list[float]):
        if not self._ready or not self._plotter:
            return
        if len(angles_deg) != 6:
            return

        try:
            # Kiểm tra widget interactor có còn hiển thị không trước khi re-render
            if hasattr(self._plotter, 'interactor') and self._plotter.interactor.isVisible():
                self._joint_angles = [math.radians(a) for a in angles_deg]
                self._update_actors()
                self._plotter.render()
        except Exception:
            pass

    def get_plotter(self):
        return self._plotter

    def close(self):
        if self._plotter:
            try:
                self._plotter.close()
            except Exception:
                pass

    def _load_meshes(self):
        for link_name, info in self._parser.links.items():
            mesh_path = info.get("mesh_path")
            if mesh_path and os.path.isfile(mesh_path):
                try:
                    mesh = pv.read(mesh_path)
                    scale = info.get("mesh_scale", np.ones(3))
                    if not np.allclose(scale, 1.0):
                        mesh.points *= scale
                    pre = LINK_MESH_TRANSFORMS.get(link_name)
                    if pre is not None:  # ghép khít + tư thế 0 chữ L (mesh đã ở đơn vị mét)
                        mesh.transform(np.asarray(pre, dtype=float), inplace=True)
                    self._meshes[link_name] = mesh
                except Exception as e:
                    self._meshes[link_name] = self._make_fallback_mesh(link_name)
            else:
                self._meshes[link_name] = self._make_fallback_mesh(link_name)
        # dụng cụ khâu 6 (nếu có file)
        info6 = self._parser.links.get("link_6_1", {})
        mesh6 = info6.get("mesh_path")
        for tool_name, (parent, _color) in TOOL_MESHES.items():
            path = os.path.join(os.path.dirname(mesh6), tool_name + ".stl") if mesh6 else ""
            if not (path and os.path.isfile(path)):
                continue
            try:
                mesh = pv.read(path)
                scale = info6.get("mesh_scale", np.ones(3))
                if not np.allclose(scale, 1.0):
                    mesh.points *= scale
                pre = LINK_MESH_TRANSFORMS.get(parent)
                if pre is not None:
                    mesh.transform(np.asarray(pre, dtype=float), inplace=True)
                self._meshes[tool_name] = mesh
            except Exception:
                pass

    def set_tool_state(self, state: int):
        """Tô màu giác hút theo trạng thái: xám TẮT, xanh HÚT, cam NHẢ."""
        actor = self._actors.get("link_6_tool_giac_hut")
        if actor is None:
            return
        try:
            actor.prop.color = TOOL_STATE_COLORS.get(int(state), "#9CA3AF")
            if self._plotter is not None:
                self._plotter.render()
        except Exception:
            pass

    def _make_fallback_mesh(self, link_name: str):
        if pv is None:
            return None
        box = pv.Box(bounds=(-0.05, 0.05, -0.02, 0.02, -0.05, 0.05))
        return box

    def _build_scene(self):
        self._plotter.clear()
        self._actors.clear()

        self._plotter.add_axes()

        link_names = list(self._parser.links.keys())
        for idx, link_name in enumerate(link_names):
            mesh = self._meshes.get(link_name)
            if mesh is None:
                continue
            color = LINK_COLORS[idx] if idx < len(LINK_COLORS) else "#AAAAAA"
            actor = self._plotter.add_mesh(
                mesh.copy(),
                color=color,
                smooth_shading=True,
                show_edges=False,
                opacity=1.0,
                name=link_name,
            )
            self._actors[link_name] = actor
        for tool_name, (_parent, color) in TOOL_MESHES.items():
            mesh = self._meshes.get(tool_name)
            if mesh is not None:
                self._actors[tool_name] = self._plotter.add_mesh(mesh.copy(), color=color, smooth_shading=True,
                                                                 name=tool_name)

        self._update_actors()

    def _compute_link_transforms(self) -> dict[str, np.ndarray]:
        link_names = list(self._parser.links.keys()) if self._parser else [
            "base_link", "link_1_1", "link_2_1", "link_3_1", "link_4_1", "link_5_1", "link_6_1"
        ]
        transforms: dict[str, np.ndarray] = {link_names[0]: np.eye(4)}
        T_curr = np.eye(4)
        for i in range(min(6, len(link_names) - 1)):
            p = JOINT_POINTS[i]
            axis = JOINT_AXES[i]
            angle = self._joint_angles[i] if i < len(self._joint_angles) else 0.0
            M_i = make_trans_4x4(p) @ rot_axis_4x4(axis, angle) @ make_trans_4x4(-p)
            T_curr = T_curr @ M_i
            transforms[link_names[i + 1]] = T_curr.copy()
        return transforms

    def _update_actors(self):
        transforms = self._compute_link_transforms()

        for link_name, actor in self._actors.items():
            parent = TOOL_MESHES.get(link_name, (link_name,))[0]
            T_final = transforms.get(parent, np.eye(4))
            if hasattr(actor, "user_matrix"):
                actor.user_matrix = T_final
            elif hasattr(actor, "SetUserMatrix"):
                if vtk_math is not None:
                    vtk_matrix = vtk_math.vtkMatrix4x4()
                else:
                    import vtkmodules.vtkCommonMath as vtk_math_local
                    vtk_matrix = vtk_math_local.vtkMatrix4x4()
                for row in range(4):
                    for col in range(4):
                        vtk_matrix.SetElement(row, col, T_final[row, col])
                actor.SetUserMatrix(vtk_matrix)

    def _show_fallback(self):
        from PyQt6.QtWidgets import QLabel, QHBoxLayout
        from PyQt6.QtCore import Qt

        layout = self.parent_frame.layout()
        if layout is None:
            layout = QHBoxLayout(self.parent_frame)

        lbl = QLabel("⚠ Không tải được URDF\nKiểm tra thư mục meshes")
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet("color: #cc0000; font-size: 13px;")
        layout.addWidget(lbl)

