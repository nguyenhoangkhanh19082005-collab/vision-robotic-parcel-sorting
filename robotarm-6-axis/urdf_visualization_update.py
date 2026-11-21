import sys
import os
import numpy as np
import pyvista as pv
import vtk

from pyvistaqt import QtInteractor
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QSlider, QLabel
from PyQt6.QtCore import Qt

from yourdfpy import URDF


class RobotDigitalTwin(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("6-DOF Robot Simulation (Fixed & Smooth)")
        self.setGeometry(100, 100, 1200, 800)

        # =========================
        # 1. LOAD URDF
        # =========================
        urdf_path = "assets/ROBOTARM2_description/PUBY.urdf"
        self.robot = URDF.load(urdf_path)

        self.actuated_joints = [
            j for j in self.robot.robot.joints
            if j.type in ['revolute', 'continuous']
        ]

        # =========================
        # 2. UI
        # =========================
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        self.plotter = QtInteractor(self)
        main_layout.addWidget(self.plotter.interactor, stretch=3)

        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        main_layout.addWidget(control_panel, stretch=1)

        self.sliders = []
        for joint in self.actuated_joints:
            lbl = QLabel(f"{joint.name} (deg)")
            control_layout.addWidget(lbl)

            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(-180, 180)
            slider.setValue(0)
            slider.valueChanged.connect(self.update_robot_pose)

            control_layout.addWidget(slider)
            self.sliders.append(slider)

        control_layout.addStretch()

        # =========================
        # 3. LOAD MESH + AXES
        # =========================
        self.actors = {}
        self.axes_actors = {}

        colors = ['gray', 'royalblue', 'orange', 'green', 'purple', 'cyan', 'red']
        color_idx = 0

        base_dir = os.path.dirname(os.path.dirname(urdf_path))

        for link in self.robot.robot.links:
            link_name = link.name

            # ===== AXES =====
            axes = vtk.vtkAxesActor()
            axes.SetTotalLength(0.08, 0.08, 0.08)
            self.plotter.renderer.AddActor(axes)
            self.axes_actors[link_name] = axes

            # ===== MESH =====
            if len(link.visuals) > 0:
                mesh_filename = link.visuals[0].geometry.mesh.filename

                if mesh_filename.startswith("package://"):
                    parts = mesh_filename.split("/")
                    relative_path = os.path.join(*parts[2:])
                    full_mesh_path = os.path.join(base_dir, relative_path)
                else:
                    full_mesh_path = mesh_filename

                try:
                    mesh = pv.read(full_mesh_path)

                    # scale mm → m
                    mesh.points *= 0.001

                    actor = self.plotter.add_mesh(
                        mesh,
                        color=colors[color_idx % len(colors)],
                        smooth_shading=True,
                        opacity=0.7
                    )

                    self.actors[link_name] = actor
                    color_idx += 1

                except Exception as e:
                    print(f"Lỗi load mesh {link_name}: {e}")

        self.plotter.show_grid()
        self.plotter.enable_eye_dome_lighting()

        self.update_robot_pose()

    # =========================
    # HELPER
    # =========================
    def np_to_vtk_matrix(self, mat_np):
        vtk_mat = vtk.vtkMatrix4x4()
        for i in range(4):
            for j in range(4):
                vtk_mat.SetElement(i, j, mat_np[i, j])
        return vtk_mat

    # =========================
    # UPDATE POSE (FIX CHUẨN)
    # =========================
    def update_robot_pose(self):
        q_rad = [np.radians(slider.value()) for slider in self.sliders]

        # config joint
        cfg = {}
        for i, joint in enumerate(self.actuated_joints):
            cfg[joint.name] = q_rad[i]

        self.robot.update_cfg(cfg)

        # update từng link
        for link in self.robot.robot.links:
            link_name = link.name

            transform, _ = self.robot.scene.graph.get(link_name)

            # ===== AXES (LINK FRAME) =====
            if link_name in self.axes_actors:
                vtk_mat = self.np_to_vtk_matrix(transform)
                self.axes_actors[link_name].SetUserMatrix(vtk_mat)

            # ===== MESH (VISUAL FRAME) =====
            if link_name in self.actors:
                if len(link.visuals) > 0 and link.visuals[0].origin is not None:
                    visual_offset = link.visuals[0].origin
                else:
                    visual_offset = np.eye(4)

                final_transform = transform @ visual_offset

                vtk_mat = self.np_to_vtk_matrix(final_transform)

                # 🔥 FIX QUAN TRỌNG: apply cho MESH
                self.actors[link_name].SetUserMatrix(vtk_mat)

        self.plotter.render()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotDigitalTwin()
    window.show()
    sys.exit(app.exec())