import sys
import os
import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QSlider, QLabel
from PyQt6.QtCore import Qt

from yourdfpy import URDF

class RobotDigitalTwin(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("6-DOF Robot Simulation (Bản Hoàn Chỉnh)")
        self.setGeometry(100, 100, 1200, 800)

        # =========================================================
        # 1. NẠP FILE URDF
        # =========================================================
        urdf_path = "assets/ROBOTARM2_description/puby.urdf"
        self.robot = URDF.load(urdf_path)
        self.actuated_joints = [j for j in self.robot.robot.joints if j.type in ['revolute', 'continuous']]

        # =========================================================
        # 2. XÂY DỰNG GIAO DIỆN CƠ BẢN
        # =========================================================
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
            lbl = QLabel(f"{joint.name} (Độ):")
            control_layout.addWidget(lbl)
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(-180, 180)
            slider.setValue(0)
            slider.valueChanged.connect(self.update_robot_pose)
            control_layout.addWidget(slider)
            self.sliders.append(slider)
        control_layout.addStretch()

        # =========================================================
        # 3. NẠP MÔ HÌNH 3D VÀO KHÔNG GIAN
        # =========================================================
        self.actors = {}
        colors = ['gray', 'royalblue', 'orange', 'green', 'purple', 'cyan', 'red']
        color_idx = 0
        base_dir = os.path.dirname(os.path.dirname(urdf_path)) 

        for link in self.robot.robot.links:
            link_name = link.name
            if len(link.visuals) > 0:
                mesh_filename = link.visuals[0].geometry.mesh.filename
                if mesh_filename.startswith("package://"):
                    parts = mesh_filename.split("/")
                    # Cắt chuỗi để lấy đúng đường dẫn: meshes/link_...
                    relative_path = os.path.join(*parts[3:]) 
                    full_mesh_path = os.path.join(base_dir, relative_path)
                else:
                    full_mesh_path = mesh_filename

                try:
                    mesh = pv.read(full_mesh_path)
                    
                    # CHÌA KHÓA: Thu nhỏ 1000 lần trực tiếp trên mảng đỉnh
                    # Tuyệt đối không dùng mesh.scale() hay mesh.rotate_x()
                    mesh.points = mesh.points * 0.001

                    actor = self.plotter.add_mesh(
                        mesh, 
                        color=colors[color_idx % len(colors)], 
                        smooth_shading=True, 
                        copy_mesh=False
                    )
                    self.actors[link_name] = actor
                    color_idx += 1
                except Exception as e:
                    print(f"Lỗi nạp file cho {link_name}: {e}")

        self.plotter.show_grid()
        self.plotter.enable_eye_dome_lighting()
        
        # Khởi tạo vị trí ban đầu
        self.update_robot_pose()

    def update_robot_pose(self):
        q_rad = [np.radians(slider.value()) for slider in self.sliders]
        
        # 1. Báo cho URDF biết góc quay hiện tại
        cfg = {}
        for i, joint in enumerate(self.actuated_joints):
            cfg[joint.name] = q_rad[i]
        
        # Thư viện tự động giải toàn bộ ma trận Động học (bỏ qua hàm đệ quy thủ công)
        self.robot.update_cfg(cfg)

        # 2. Cập nhật giao diện 3D
        for link in self.robot.robot.links:
            link_name = link.name
            if link_name in self.actors:
                
                # Lấy ma trận vị trí tuyệt đối của Link trong không gian
                transform, _ = self.robot.scene.graph.get(link_name)
                
                # Lấy ma trận bù trừ hình ảnh của file CAD
                if len(link.visuals) > 0 and link.visuals[0].origin is not None:
                    visual_offset = link.visuals[0].origin
                else:
                    visual_offset = np.eye(4)
                
                # Nhân gộp: Vị trí toán học của khớp * Sai số tâm của file 3D
                final_transform = transform @ visual_offset
                self.actors[link_name].user_matrix = final_transform

        self.plotter.update()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotDigitalTwin()
    window.show()
    sys.exit(app.exec())