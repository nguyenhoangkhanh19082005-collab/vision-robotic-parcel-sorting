import sys
import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QSlider, QLabel
from PyQt6.QtCore import Qt

# Nạp class tính toán D-H của bạn
from donghoc import kinematic, dh_params


class RobotDigitalTwin(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("6-DOF Robot Simulation (D-H Matrix & PyVista)")
        self.setGeometry(100, 100, 1200, 800)

        # 1. Khởi tạo Engine Động học của bạn
        self.kin = kinematic(dh_params)
        # THÊM DÒNG NÀY: Lưu lại vị trí các khớp lúc q = 0 (Home)
        _, self.frames_home = self.kin.fk(np.zeros(6), return_all_frames=True)
        # 2. Xây dựng Giao diện PyQt6
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # --- Khung render 3D bên trái ---
        self.plotter = QtInteractor(self)
        main_layout.addWidget(self.plotter.interactor, stretch=3)

        # --- Bảng điều khiển Slider bên phải ---
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        main_layout.addWidget(control_panel, stretch=1)

        self.sliders = []
        for i in range(6):
            lbl = QLabel(f"Khớp {i+1} (Độ):")
            control_layout.addWidget(lbl)

            slider = QSlider(Qt.Orientation.Horizontal)
            # Giới hạn góc tùy theo thiết kế của bạn
            slider.setRange(-180, 180)
            slider.setValue(0)
            slider.valueChanged.connect(self.update_robot_pose)

            control_layout.addWidget(slider)
            self.sliders.append(slider)

        control_layout.addStretch()

        # 3. Nạp dữ liệu 3D (Tệp STL)
        self.actors = []
        self._load_stl_models()

        # Hiển thị lưới mặt đất và setup ánh sáng
        self.plotter.show_grid()
        self.plotter.enable_eye_dome_lighting()

        # Cập nhật vị trí ban đầu
        self.update_robot_pose()

    def _load_stl_models(self):
        """
        Nạp các file STL. Lưu ý: Trục tọa độ gốc (0,0,0) của mỗi file STL 
        phải được đặt trùng với tâm quay của khớp đó trên phần mềm CAD.
        """
        # Danh sách file STL (Hãy thay bằng đường dẫn tới file của bạn)
        stl_files = [
            "assets/CAD/base_link.stl",  # Base (Đế)
            "assets/CAD/link_1.stl",
            "assets/CAD/link_2.stl",
            "assets/CAD/link_3.stl",
            "assets/CAD/link_4.stl",
            "assets/CAD/link_5.stl",
            "assets/CAD/link_6.stl"
        ]

        colors = ['gray', 'royalblue', 'orange',
                  'green', 'purple', 'cyan', 'red']

        for i, file_path in enumerate(stl_files):
            try:
                mesh = pv.read(file_path)
            except Exception as e:
                print(
                    f"Không tìm thấy {file_path}, tạo khối hình trụ thay thế...")
                # Tạo hình giả lập nếu bạn chưa có sẵn file STL
                mesh = pv.Cylinder(direction=(0, 0, 1),
                                   radius=0.05, height=0.15)

            # copy_mesh=False cực kỳ quan trọng. Nó yêu cầu PyVista tham chiếu thẳng
            # vào dữ liệu RAM ban đầu thay vì tạo bản sao mỗi khi frame thay đổi.
            actor = self.plotter.add_mesh(
                mesh, color=colors[i], smooth_shading=True, copy_mesh=False)
            self.actors.append(actor)

    def update_robot_pose(self):
        q_deg = [slider.value() for slider in self.sliders]
        q_rad = np.radians(q_deg)

        try:
            T, frames = self.kin.fk(q_rad, True)

            # Base (luôn đứng im ở vị trí gốc)
            self.actors[0].user_matrix = np.eye(4)

            # Link 1 → 6: Nhân ma trận hiện tại với ma trận NGHỊCH ĐẢO của vị trí Home
            for i in range(6):
                T_current = frames[i+1]
                T_home_inv = np.linalg.inv(self.frames_home[i+1])
                
                # Công thức bù trừ: Kéo vật thể về gốc -> Xoay theo D-H -> Đẩy ra vị trí mới
                self.actors[i+1].user_matrix = T_current @ T_home_inv

            self.plotter.update()

        except Exception as e:
            print(f"Lỗi động học: {e}")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotDigitalTwin()
    window.show()
    sys.exit(app.exec())
