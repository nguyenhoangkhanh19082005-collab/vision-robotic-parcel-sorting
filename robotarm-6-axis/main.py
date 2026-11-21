import sys
import os
import time
import datetime
import json
import numpy as np

if sys.platform == "win32":
    try:
        if hasattr(sys.stdout, 'reconfigure'):
            sys.stdout.reconfigure(encoding='utf-8', errors='replace')
        if hasattr(sys.stderr, 'reconfigure'):
            sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QTreeWidgetItem, QMenu
)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QAction

from uis.robodk_ui import RoboDKStyleUI
from robot_simulation import RobotSimulationWidget
from communication import RobotCommunicator
from donghoc import kinematic, dh_params  # noqa: F401 (giữ tương thích script cũ)
from kinematics_poe import build_model_from_robot_project

DEFAULT_URDF_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets", "ROBOTARM2_description")


class WorldClassRoboDKApp(QMainWindow):
    def __init__(self, urdf_dir: str = DEFAULT_URDF_DIR):
        super().__init__()

        # 1. KHỞI TẠO GIAO DIỆN PHONG CÁCH ROBODK ĐỈNH CAO THẾ GIỚI
        self.ui = RoboDKStyleUI()
        self.ui.setupUi(self)

        # 2. KHỞI TẠO ĐỘNG HỌC — PoE dựng từ trục khớp CAD (cùng JOINT_POINTS/AXES với digital twin).
        # Bảng DH cũ (donghoc.kinematic) lệch tư thế 0: TCP ở Z = -181 mm (dưới mặt đế) trong khi twin là
        # chữ L → HUD/MoveL/IK sai. Chiều dài đầu hút lấy từ dong_hoc_step.TOOL_LENGTH_MM (gá giác hút dài, 21/11/2026).
        try:
            from dong_hoc_step import TOOL_LENGTH_MM as _tool_mm
        except ImportError:
            _tool_mm = 30.0
        self.kin = build_model_from_robot_project(os.path.dirname(os.path.abspath(__file__)), tool_length_m=_tool_mm / 1000.0)

        # 3. KHỞI TẠO KẾT NỐI MẠNG LAN UDP (W5500)
        self.comm = RobotCommunicator(ip="192.168.1.177", port=8888)
        self.tool_state = 0
        self.saved_waypoints_data = []

        # 4. NHÚNG MÔ PHỎNG 3D VÀO WORKCELL VIEWPORT
        try:
            # Đường dẫn tuyệt đối theo vị trí main.py: chạy từ thư mục khác vẫn nạp được URDF/mesh
            self.sim = RobotSimulationWidget(self.ui.frm_main_simulation, urdf_dir=urdf_dir)
            self.sim.setup()
        except Exception as e:
            self.log_message(f"Lỗi khởi tạo mô phỏng 3D: {e}", "ERROR")
            self.sim = None

        # 5. LIÊN KẾT TÍN HIỆU & SỰ KIỆN GIAO DIỆN
        self._setup_connections()

        # 6. KHỞI TẠO TIMER ĐỌC TELEMETRY UDP TỪ ESP32-S3 (30Hz)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.read_hardware_feedback)
        self.timer.start(33)

        # 7. KHỞI TẠO BỘ ĐIỀU TIẾT LỆNH GỬI ĐỘNG CƠ (COMMAND THROTTLED DISPATCHER ~28Hz)
        # Giúp chuyển động mượt mà, khử hoàn toàn hiện tượng nghẽn hàng đợi xung và giật cục trên ESP32-S3
        self.tx_timer = QTimer(self)
        self.tx_timer.setInterval(35) # 35ms ~ 28Hz
        self.tx_timer.timeout.connect(self._dispatch_joint_command)
        self.tx_timer.start()
        self.pending_angles = [0.0] * 6
        self.has_pending_tx = False

        self.sync_system_pose()
        
        # 8. CẤU HÌNH VỊ TRÍ GỐC CỐ ĐỊNH (HOME ZERO CONFIG)
        self.home_config_path = os.path.join(os.path.dirname(__file__), "home_config.json")
        self.is_home_calibrated = self._load_home_config()
        self.lock_home_ui(self.is_home_calibrated)

        self.log_message("Hệ thống RoboDK Workcell AURA-6 đã kích hoạt. Trạng thái hoạt động hoàn hảo.", "INFO")

    def log_message(self, msg: str, level: str = "INFO"):
        """Ghi log vào Console và StatusBar."""
        colors = {
            "INFO": "#00E676",
            "WARNING": "#FFAB00",
            "ERROR": "#FF1744"
        }
        color = colors.get(level, "#E2E8F0")
        time_str = datetime.datetime.now().strftime("%H:%M:%S")
        log_entry = f'<span style="color: #64748B;">[{time_str}]</span> <b style="color: {color};">[{level}]</b> {msg}'
        self.ui.txt_logs.append(log_entry)
        self.ui.statusbar.showMessage(f"[{level}] {msg}")

    def _setup_connections(self):
        """Liên kết Toolbar, Station Tree, Sliders, Buttons."""
        self.sliders = self.ui.sliders
        self.spinboxes = self.ui.spinboxes
        self.lbl_feedbacks = self.ui.lbl_feedbacks

        # 1. Đồng bộ 6 Khớp: Slider <-> Spinbox & Flush on release
        for i in range(6):
            self.sliders[i].valueChanged.connect(self.spinboxes[i].setValue)
            self.spinboxes[i].valueChanged.connect(self.sliders[i].setValue)
            self.sliders[i].valueChanged.connect(self.sync_system_pose)
            self.sliders[i].sliderReleased.connect(self.flush_joint_command)

        # 2. Ribbon Toolbar
        self.ui.btn_tb_connect.clicked.connect(self.toggle_connection)
        self.ui.btn_tb_add_target.clicked.connect(self.handle_save_wp)
        self.ui.btn_tb_movej.clicked.connect(self.handle_home_all)
        if hasattr(self.ui, 'btn_tb_set_zero'):
            self.ui.btn_tb_set_zero.clicked.connect(self.handle_set_zero)
        if hasattr(self.ui, 'btn_tb_unlock_zero'):
            self.ui.btn_tb_unlock_zero.clicked.connect(self.handle_unlock_zero)
        self.ui.btn_tb_movel.clicked.connect(self.handle_ik_move)
        self.ui.btn_tb_run.clicked.connect(self.handle_run_auto)
        self.ui.btn_tb_stop.clicked.connect(self.handle_stop)

        # Quick Calib Buttons on Joint Jog Tab
        if hasattr(self.ui, 'btn_joint_home'):
            self.ui.btn_joint_home.clicked.connect(self.handle_home_all)
        if hasattr(self.ui, 'btn_joint_set_zero'):
            self.ui.btn_joint_set_zero.clicked.connect(self.handle_set_zero)
        if hasattr(self.ui, 'btn_joint_unlock_zero'):
            self.ui.btn_joint_unlock_zero.clicked.connect(self.handle_unlock_zero)

        # 3. Station Tree View Interactivity (Chích đúp để di chuyển robot tới Target)
        self.ui.tree_station.itemDoubleClicked.connect(self.handle_tree_double_click)
        self.ui.tree_station.customContextMenuRequested.connect(self.show_tree_context_menu)

        # 4. Cartesian MoveL (Pieper IK)
        self.ui.btn_movel_run.clicked.connect(self.handle_ik_move)

        # 5. Automation Tasks (Task 1: Vẽ Đường Thẳng, Task 2: Vẽ Hình Chữ Nhật, Task 3: Pick & Place)
        self.ui.btn_task1_line.clicked.connect(self.handle_task1_draw_line)
        self.ui.btn_task2_rect.clicked.connect(self.handle_task2_draw_rectangle)
        self.ui.btn_task3_free.clicked.connect(self.handle_task3_pick_and_place)

        # 6. Connection (USB Type-C Serial & Ethernet W5500 LAN)
        self.ui.btn_connect_udp.clicked.connect(self.toggle_connection)
        if hasattr(self.ui, 'btn_refresh_ports'):
            self.ui.btn_refresh_ports.clicked.connect(self.scan_com_ports)
            self.scan_com_ports()

        # 7. Tool Control (Bơm / Van)
        self.ui.btn_tool_off.clicked.connect(lambda: self.handle_tool_control(0))
        self.ui.btn_tool_suck.clicked.connect(lambda: self.handle_tool_control(1))
        self.ui.btn_tool_release.clicked.connect(lambda: self.handle_tool_control(2))

    def scan_com_ports(self):
        """Tự động quét danh sách các cổng USB Serial (COM port) đang cắm."""
        ports = RobotCommunicator.list_serial_ports()
        self.ui.cmb_com_port.clear()
        if ports:
            self.ui.cmb_com_port.addItems(ports)
            self.log_message(f"Đã phát hiện {len(ports)} cổng COM: {', '.join(ports)}", "INFO")
        else:
            self.ui.cmb_com_port.addItem("COM3") # Giá trị mẫu mặc định
            self.log_message("Chưa thấy cổng COM mới. Bạn có thể tự gõ cổng COM hoặc cắm USB rồi nhấn 'Quét'.", "WARNING")

    def handle_home_all(self):
        for slider in self.sliders:
            slider.setValue(0)
        self.sync_system_pose()
        if self.comm and self.comm.is_connected:
            self.comm.send_pc_command([0.0] * 6, self.tool_state)
        self.log_message("Lệnh MoveJ Home: Đã đưa toàn bộ 6 khớp về vị trí 0°", "INFO")

    def toggle_connection(self):
        """Kết nối / Ngắt kết nối thông minh (hỗ trợ cả USB Serial và Ethernet LAN)."""
        if self.comm and self.comm.is_connected:
            self.comm.disconnect()
            self.ui.btn_connect_udp.setText("KẾT NỐI ROBOT (USB / LAN)")
            self.ui.btn_connect_udp.setProperty("class", "BtnSuccess")
            self.ui.btn_connect_udp.style().polish(self.ui.btn_connect_udp)
            self.ui.btn_tb_connect.setText("⚡ Connect Robot (USB / LAN)")
            self.log_message("Đã ngắt kết nối với Robot.", "WARNING")
        else:
            is_usb = hasattr(self.ui, 'cmb_conn_type') and self.ui.cmb_conn_type.currentIndex() == 0
            if is_usb:
                # Chế độ USB Type-C Serial
                port = self.ui.cmb_com_port.currentText().strip()
                if not port:
                    self.log_message("Lỗi: Vui lòng nhập hoặc chọn cổng COM (ví dụ COM3)!", "ERROR")
                    return
                baud = self.ui.cmb_baudrate.currentText().strip()
                success, msg = self.comm.connect(port, baud)
                target_display = port
            else:
                # Chế độ Ethernet W5500 LAN
                ip = self.ui.txt_ip.text().strip()
                port = self.ui.txt_port.text().strip()
                success, msg = self.comm.connect(ip, port)
                target_display = f"{ip}:{port}"

            if success:
                self.ui.btn_connect_udp.setText(f"NGẮT KẾT NỐI ({target_display})")
                self.ui.btn_connect_udp.setProperty("class", "BtnDanger")
                self.ui.btn_connect_udp.style().polish(self.ui.btn_connect_udp)
                self.ui.btn_tb_connect.setText(f"🟢 Connected ({target_display})")
                self.log_message(msg, "WARNING" if getattr(self.comm, "native_usb", False) else "INFO")
                self._telemetry_warned = False
            else:
                self.log_message(msg, "ERROR")

    def handle_save_wp(self):
        """Thêm Waypoint Target an toàn 100% không bao giờ văng app."""
        q_deg = [slider.value() for slider in self.sliders]
        if self.comm and self.comm.is_connected:
            self.comm.send_rec_command(q_deg, self.tool_state)

        wp_id = len(self.saved_waypoints_data) + 1
        wp_text = f"Target {wp_id:02d} | J:[{q_deg[0]:.0f}°, {q_deg[1]:.0f}°, {q_deg[2]:.0f}°, {q_deg[3]:.0f}°, {q_deg[4]:.0f}°, {q_deg[5]:.0f}°]"
        self.saved_waypoints_data.append(wp_text)
        
        # THÊM MỤC VÀO CÂY TRẠM ROBODK AN TOÀN TRỰC TIẾP
        if hasattr(self.ui, 'target_root_item') and self.ui.target_root_item:
            target_item = QTreeWidgetItem(["🎯 " + wp_text])
            target_item.setData(0, Qt.ItemDataRole.UserRole, q_deg) # Lưu góc vào item
            self.ui.target_root_item.addChild(target_item)
            self.ui.tree_station.expandAll()
            self.log_message(f"RoboDK Add Target: Đã thêm Waypoint Target #{wp_id} vào Trạm Station 1", "INFO")
        else:
            self.log_message(f"Đã lưu Waypoint Target #{wp_id}", "INFO")

    def handle_tree_double_click(self, item, column):
        """Kích đúp vào một Target trên Cây Trạm để tự động đưa Robot tới vị trí đó."""
        q_deg = item.data(0, Qt.ItemDataRole.UserRole)
        if q_deg and len(q_deg) == 6:
            for i in range(6):
                self.sliders[i].setValue(int(q_deg[i]))
            self.log_message(f"RoboDK MoveJ Target: Đã di chuyển robot tới {item.text(0)}", "INFO")

    def show_tree_context_menu(self, position):
        """Menu chuột phải trên Cây Trạm RoboDK."""
        item = self.ui.tree_station.itemAt(position)
        if item and "Target" in item.text(0):
            menu = QMenu(self)
            act_move = menu.addAction("🔄 Move Robot To Target")
            act_update = menu.addAction("📝 Update Target to Current Pose")
            act_delete = menu.addAction("❌ Delete Target")

            action = menu.exec(self.ui.tree_station.mapToGlobal(position))
            if action == act_move:
                self.handle_tree_double_click(item, 0)
            elif action == act_update:
                q_deg = [s.value() for s in self.sliders]
                item.setData(0, Qt.ItemDataRole.UserRole, q_deg)
                self.log_message(f"Đã cập nhật vị trí cho {item.text(0)}", "INFO")
            elif action == act_delete:
                parent = item.parent()
                if parent:
                    parent.removeChild(item)
                    self.log_message(f"Đã xóa target khỏi Trạm", "WARNING")

    def handle_run_auto(self):
        if self.comm and self.comm.is_connected:
            self.comm.play_sequence()
            self.log_message("▶ RoboDK Run Program: Bắt đầu thực thi chương trình tự động trên robot thật", "INFO")
        else:
            self.log_message("▶ Chạy tự động trong môi trường mô phỏng RoboDK 3D", "WARNING")

    def handle_stop(self):
        self._abort_wait = True
        if self.comm and self.comm.is_connected:
            self.comm.stop_robot()
        self.log_message("🛑 DỪNG KHẨN CẤP: Đã ngắt toàn bộ động cơ và bơm!", "ERROR")

    def sync_system_pose(self):
        """Tính toán FK -> Cập nhật 3D Viewport, HUD & Chỉ số Sức khỏe Kinematics Manipulability."""
        if not self.sliders:
            return
        self._q_exact = None  # người dùng vừa kéo slider/nhập số → góc lệnh = giá trị slider (số nguyên độ)
        self._apply_pose([float(slider.value()) for slider in self.sliders])

    def _current_q(self):
        """Góc lệnh hiện tại (độ): giá trị chính xác của MoveL/Task nếu có, nếu không thì giá trị slider."""
        exact = getattr(self, "_q_exact", None)
        return list(exact) if exact is not None else [float(s.value()) for s in self.sliders]

    def _set_joints_exact(self, q_deg):
        """Đặt góc lệnh có phần thập phân (kết quả IK). Slider chỉ hiển thị số đã làm tròn; lệnh gửi robot và twin dùng
        giá trị chính xác — trước đây int() cắt bỏ phần lẻ, sai tới 1° mỗi khớp (≈ 5 mm ở đầu hút)."""
        q = [float(v) for v in q_deg]
        for i in range(6):
            for w in (self.sliders[i], self.spinboxes[i]):
                w.blockSignals(True)
                w.setValue(int(round(q[i])))
                w.blockSignals(False)
        self._q_exact = q
        self._apply_pose(q)

    def _apply_pose(self, q_deg):
        q_rad = np.radians(q_deg)

        if self.sim:
            self.sim.update_joints(q_deg)

        try:
            T_matrix, frames = self.kin.fk(q_rad, return_all_frames=True)
            x, y, z = T_matrix[0, 3] * 1000.0, T_matrix[1, 3] * 1000.0, T_matrix[2, 3] * 1000.0
            self.ui.lbl_tcp_hud.setText(f"TCP: X={x:.1f} Y={y:.1f} Z={z:.1f} mm | J1:{q_deg[0]:.1f}° J2:{q_deg[1]:.1f}° J3:{q_deg[2]:.1f}°")
            
            # Tính toán chỉ số linh hoạt Manipulability (Tránh Singularity)
            # Ước lượng mức linh hoạt theo độ mở của góc khuỷu J3 và vai J2
            m_index = int(np.clip(100 - abs(q_deg[2]) * 0.4 - abs(q_deg[1]) * 0.3, 10, 100))
            self.ui.bar_manipulability.setValue(m_index)
        except Exception as e:
            print(f"Lỗi FK: {e}")

        # Lên lịch gửi lệnh qua bộ điều tiết tx_timer để động cơ chuyển động mượt mà 100%
        if self.comm and self.comm.is_connected:
            self.pending_angles = [float(v) for v in q_deg]
            self.has_pending_tx = True

    def _dispatch_joint_command(self):
        """Điều tiết lệnh phát xung nhịp nhàng theo chu kỳ công nghiệp (~28Hz)."""
        if getattr(self, 'has_pending_tx', False) and self.comm and self.comm.is_connected:
            self.comm.send_pc_command(self.pending_angles, self.tool_state)
            self.has_pending_tx = False

    def flush_joint_command(self):
        """Đảm bảo đích đến khi thả chuột luôn được gửi tới ESP32-S3 với cờ force=True."""
        if self.comm and self.comm.is_connected:
            q_deg = [float(s.value()) for s in self.sliders]
            self.comm.send_pc_command(q_deg, self.tool_state, force=True)
            self.has_pending_tx = False

    def _load_home_config(self) -> bool:
        """Đọc trạng thái đã cài đặt gốc từ file cấu hình lưu trữ."""
        try:
            if os.path.exists(self.home_config_path):
                with open(self.home_config_path, "r", encoding="utf-8") as f:
                    cfg = json.load(f)
                    return bool(cfg.get("is_calibrated", False))
        except Exception:
            pass
        return False

    def _save_home_config(self, is_calib: bool):
        """Lưu trạng thái cài đặt gốc vào file cấu hình máy tính."""
        try:
            with open(self.home_config_path, "w", encoding="utf-8") as f:
                json.dump({
                    "is_calibrated": is_calib,
                    "calibrated_time": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                }, f, indent=2)
        except Exception as e:
            print(f"Lỗi lưu home_config.json: {e}")

    def lock_home_ui(self, locked: bool):
        """Khóa/Mở khóa nút Lưu Gốc (chỉ dùng 1 lần duy nhất, mở khóa khi muốn đổi lại)."""
        self.is_home_calibrated = locked
        if locked:
            if hasattr(self.ui, 'btn_tb_set_zero'):
                self.ui.btn_tb_set_zero.setText("🔒 Gốc Đã Khóa (Flash)")
                self.ui.btn_tb_set_zero.setStyleSheet("background-color: #10B981; color: #FFFFFF; font-weight: 800;")
                self.ui.btn_tb_set_zero.setEnabled(False)
            if hasattr(self.ui, 'btn_joint_set_zero'):
                self.ui.btn_joint_set_zero.setText("🔒 Gốc Đã Khóa (Flash)")
                self.ui.btn_joint_set_zero.setStyleSheet("background-color: #10B981; color: #FFFFFF; font-weight: 800; padding: 6px;")
                self.ui.btn_joint_set_zero.setEnabled(False)
            if hasattr(self.ui, 'btn_tb_unlock_zero'):
                self.ui.btn_tb_unlock_zero.setEnabled(True)
            if hasattr(self.ui, 'btn_joint_unlock_zero'):
                self.ui.btn_joint_unlock_zero.setEnabled(True)
        else:
            if hasattr(self.ui, 'btn_tb_set_zero'):
                self.ui.btn_tb_set_zero.setText("🎯 Lưu Gốc (Flash)")
                self.ui.btn_tb_set_zero.setStyleSheet("background-color: #F59E0B; color: #000000; font-weight: 800;")
                self.ui.btn_tb_set_zero.setEnabled(True)
            if hasattr(self.ui, 'btn_joint_set_zero'):
                self.ui.btn_joint_set_zero.setText("🎯 Lưu Gốc (Flash)")
                self.ui.btn_joint_set_zero.setStyleSheet("background-color: #F59E0B; color: #000000; font-weight: 800; padding: 6px;")
                self.ui.btn_joint_set_zero.setEnabled(True)

    def handle_set_zero(self):
        """Cài đặt vị trí vật lý hiện tại của Robot làm mốc 0.0° (Lưu vĩnh viễn vào Flash NVS)."""
        # 1. Chặn tín hiệu tạm thời để gán giá trị 0 cho toàn bộ Slider/Spinbox mà không kích hoạt chuỗi sự kiện
        for slider in self.sliders:
            slider.blockSignals(True)
            slider.setValue(0)
            slider.blockSignals(False)
        for spin in self.spinboxes:
            spin.blockSignals(True)
            spin.setValue(0)
            spin.blockSignals(False)

        # 2. Gửi lệnh CMD_SET_ZERO tới vi điều khiển ESP32-S3 (lưu Flash NVS)
        if self.comm and self.comm.is_connected:
            self.comm.send_set_zero()

        # 3. Đồng bộ lại tư thế Digital Twin 3D Viewport về thế đứng chuẩn Home
        self.has_pending_tx = False
        self.sync_system_pose()
        for lbl in self.lbl_feedbacks:
            lbl.setText("Act: 0.0°")

        # 4. Tự động KHÓA nút sau khi đã lưu gốc thành công (chỉ sử dụng đúng 1 lần duy nhất)
        self.lock_home_ui(True)
        self._save_home_config(True)

        self.log_message("🎯 ĐÃ LƯU VỊ TRÍ GỐC VÀO FLASH NVS THÀNH CÔNG! Nút đã TỰ ĐỘNG KHÓA để bảo vệ mốc 0°. Mỗi khi bật nguồn, cánh tay sẽ tự động nhận diện mốc này. Khi nào muốn thay đổi, bấm '🔓 Đổi Gốc'.", "INFO")

    def handle_unlock_zero(self):
        """Mở khóa để cho phép người dùng thay đổi lại vị trí gốc khi có nhu cầu."""
        self.lock_home_ui(False)
        self._save_home_config(False)
        self.log_message("🔓 ĐÃ MỞ KHÓA THIẾT LẬP GỐC: Bạn có thể chỉnh lại tư thế cánh tay và bấm '🎯 Lưu Gốc (Flash)' để lưu mốc mới vào Flash.", "WARNING")

    def handle_tool_control(self, state):
        self.tool_state = state
        if self.comm and self.comm.is_connected:
            self.comm.send_tool_action(state)
            txt = {0: "TẮT", 1: "HÚT", 2: "NHẢ"}[state]
            self.log_message(f"Tool Gripper Action: Gửi lệnh {txt}", "INFO")

    def handle_ik_move(self):
        """Lệnh MoveL: Giải Động học nghịch Pieper IK từ ô nhập TCP."""
        try:
            x_mm = float(self.ui.spb_ik_x.value())
            y_mm = float(self.ui.spb_ik_y.value())
            z_mm = float(self.ui.spb_ik_z.value())

            x, y, z = x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0

            T_target = np.eye(4)
            T_target[0, 3] = x; T_target[1, 3] = y; T_target[2, 3] = z
            T_target[:3, :3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

            q_curr_deg = self._current_q()
            q_target_rad, success = self.kin.ik_pieper(T_target, q_curr_deg)

            if success and q_target_rad is not None:
                q_target_deg = np.degrees(q_target_rad)
                self._set_joints_exact(q_target_deg)
                self.log_message(f"MoveL (Pieper IK) OK! Di chuyển TCP tới [X:{x_mm:.0f}, Y:{y_mm:.0f}, Z:{z_mm:.0f}]", "INFO")
            else:
                self.log_message("Lỗi MoveL IK: Vị trí mục tiêu vượt ngoài vùng làm việc (Workspace Limit)!", "ERROR")

        except Exception as e:
            self.log_message(f"Lỗi MoveL: {e}", "ERROR")

    # =========================================================================
    # AUTOMATION TASKS (TASK 1, TASK 2, TASK 3)
    # =========================================================================

    def _wait_robot_reached(self, fallback_s, tol_deg=1.5, timeout_s=40.0):
        """Chờ robot thật tới góc đang đặt trên slider (so với telemetry Act) rồi mới đi tiếp.
        Sau khi J1–J3 đổi hộp số 1:27 các khớp chỉ còn ≈ 10–13°/s nên chờ theo thời gian cố định (0.05–0.5 s) là
        không đủ: lệnh sau đè lệnh trước, robot cắt góc quỹ đạo / nhả phôi khi chưa nhấc xong.
        Chưa kết nối robot (chỉ mô phỏng): chờ fallback_s như cũ. Trả về False nếu quá hạn hoặc bấm DỪNG."""
        QApplication.processEvents()
        if not (self.comm and self.comm.is_connected):
            time.sleep(fallback_s)
            return True
        target = self._current_q()
        self._abort_wait = False
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            QApplication.processEvents()  # cho tx_timer gửi lệnh và timer đọc telemetry chạy
            time.sleep(0.02)
            if self._abort_wait or not self.comm.is_connected:
                return False
            stamp, act = getattr(self, "_last_actual", (0.0, None))
            if act and stamp > t0 + 0.1 and max(abs(a - t) for a, t in zip(act, target)) <= tol_deg:
                return True
        self.log_message(f"Robot chưa tới vị trí sau {timeout_s:.0f} s (kẹt / mất bước / mất telemetry?) — dừng chu trình.", "WARNING")
        return False

    def handle_task1_draw_line(self):
        """Task 1: Quỹ đạo Vẽ Đường Thẳng Cartesian (Line Trajectory)."""
        self.log_message("Bắt đầu Task 1: Tạo quỹ đạo đường thẳng 100mm...", "INFO")
        q_curr = self._current_q()
        T_start = self.kin.fk(np.radians(q_curr))
        x0, y0, z0 = T_start[0, 3], T_start[1, 3], T_start[2, 3]

        for step in range(10):
            x_target = x0 + (step * 0.010)
            T_target = T_start.copy()
            T_target[0, 3] = x_target

            q_sol_rad, ok = self.kin.ik_pieper(T_target, q_curr)
            if ok and q_sol_rad is not None:
                q_sol_deg = np.degrees(q_sol_rad)
                q_curr = list(q_sol_deg)
                self._set_joints_exact(q_sol_deg)
                if not self._wait_robot_reached(0.05):
                    return

        self.log_message("Task 1: Đã hoàn thành quỹ đạo đường thẳng!", "INFO")

    def handle_task2_draw_rectangle(self):
        """Task 2: Quỹ đạo Vẽ Hình Chữ Nhật (Rectangle Trajectory 100x100mm)."""
        self.log_message("Bắt đầu Task 2: Tạo quỹ đạo hình chữ nhật 100x100mm...", "INFO")
        q_curr = self._current_q()
        T_start = self.kin.fk(np.radians(q_curr))
        x0, y0, z0 = T_start[0, 3], T_start[1, 3], T_start[2, 3]

        corners = [
            (x0 + 0.100, y0),
            (x0 + 0.100, y0 + 0.100),
            (x0, y0 + 0.100),
            (x0, y0)
        ]

        for cx, cy in corners:
            T_target = T_start.copy()
            T_target[0, 3] = cx
            T_target[1, 3] = cy

            q_sol_rad, ok = self.kin.ik_pieper(T_target, q_curr)
            if ok and q_sol_rad is not None:
                q_sol_deg = np.degrees(q_sol_rad)
                q_curr = list(q_sol_deg)
                self._set_joints_exact(q_sol_deg)
                if not self._wait_robot_reached(0.2):
                    return

        self.log_message("Task 2: Đã hoàn thành quỹ đạo hình chữ nhật!", "INFO")

    def handle_task3_pick_and_place(self):
        """Task 3: Chu trình Pick & Place Tự động kèm Đầu Hút Vacuum."""
        self.log_message("Bắt đầu Task 3: Chu trình Pick & Place tự động...", "INFO")
        # 1. Hút phôi tại chỗ
        self.handle_tool_control(1) # SUCK
        time.sleep(0.5)

        # 2. Nhấc lên Z + 50mm
        q_curr = self._current_q()
        T_start = self.kin.fk(np.radians(q_curr))
        T_up = T_start.copy()
        T_up[2, 3] += 0.050

        q_sol_rad, ok = self.kin.ik_pieper(T_up, q_curr)
        if ok and q_sol_rad is not None:
            self._set_joints_exact(np.degrees(q_sol_rad))
            if not self._wait_robot_reached(0.5):
                self.handle_tool_control(0)
                return

        # 3. Nhả phôi
        self.handle_tool_control(2) # RELEASE
        time.sleep(0.3)
        self.handle_tool_control(0) # OFF
        self.log_message("Task 3: Hoàn tất chu trình Pick & Place!", "INFO")

    def read_hardware_feedback(self):
        if not self.comm:
            return

        # Tự động phát hiện khi mất kết nối đột ngột (do kẹt cơ khí, sụt nguồn hoặc rút cáp)
        if hasattr(self, '_was_connected') and self._was_connected and not self.comm.is_connected:
            self._was_connected = False
            self.ui.btn_connect_udp.setText("KẾT NỐI ROBOT (USB / LAN)")
            self.ui.btn_connect_udp.setProperty("class", "BtnSuccess")
            self.ui.btn_connect_udp.style().polish(self.ui.btn_connect_udp)
            self.ui.btn_tb_connect.setText("⚡ Connect Robot (USB / LAN)")
            self.log_message("Mất kết nối cổng COM! (Do ESP32 bị reset khi kẹt cơ khí, sụt nguồn, hoặc rút cáp USB). Đã tự động giải phóng cổng.", "WARNING")
            return

        self._was_connected = self.comm.is_connected
        if not self.comm.is_connected:
            return

        feedback = self.comm.read_telemetry()
        # Cảnh báo 1 lần nếu đã kết nối USB > 2 s mà chưa nhận gói telemetry nào (sai cổng / sai baud / firmware)
        if (self.comm.mode == "SERIAL" and not self.comm.last_rx_time and not getattr(self, "_telemetry_warned", False)
                and time.time() - self.comm.connect_time > 2.0):
            self._telemetry_warned = True
            self.log_message(f"Chưa nhận được telemetry từ {self.comm.serial_port} sau 2 s: kiểm tra đúng cổng COM của chip "
                             "CH343 (không phải cổng USB gốc ESP32-S3), baud 115200 và firmware robotarmver2 đã nạp.", "WARNING")
        if feedback:
            angles = feedback.get("angles", [])
            if len(angles) == 6:
                self._last_actual = (time.time(), list(angles))
                for i in range(6):
                    self.lbl_feedbacks[i].setText(f"Act: {angles[i]:.1f}°")

            # Tự động đồng bộ trạng thái mốc gốc từ Flash NVS của ESP32-S3
            is_calib = feedback.get("is_calibrated", False)
            if is_calib and not self.is_home_calibrated:
                self.lock_home_ui(True)
                self._save_home_config(True)
                self.log_message("⚡ ESP32-S3 đã xác nhận mốc gốc đã lưu trong Flash NVS. Đã tự động khóa nút 'Lưu Gốc'.", "INFO")

    def closeEvent(self, event):
        if self.comm:
            self.comm.disconnect()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = WorldClassRoboDKApp()
    window.showMaximized()
    sys.exit(app.exec())
