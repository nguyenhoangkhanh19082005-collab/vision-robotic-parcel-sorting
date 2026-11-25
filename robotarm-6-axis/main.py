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

# 24/11/2026: PyQt6 GỌI abort() (app tắt ngay, không báo gì) khi một slot/timer ném exception chưa bắt.
# Bộ bắt lỗi toàn cục: ghi traceback vào logs/gui_errors.log + in ra console, app KHÔNG tắt.
GUI_ERROR_LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", "gui_errors.log")


def _gui_excepthook(exc_type, exc, tb):
    import traceback
    text = "".join(traceback.format_exception(exc_type, exc, tb))
    try:
        os.makedirs(os.path.dirname(GUI_ERROR_LOG), exist_ok=True)
        with open(GUI_ERROR_LOG, "a", encoding="utf-8") as fh:
            fh.write(f"\n===== {datetime.datetime.now():%Y-%m-%d %H:%M:%S} =====\n{text}")
    except Exception:
        pass
    try:
        print("[LỖI GUI - đã bắt, app tiếp tục chạy]\n" + text, file=sys.stderr)
    except Exception:
        pass


sys.excepthook = _gui_excepthook

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QTreeWidgetItem, QMenu, QInputDialog, QMessageBox
)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QAction

from uis.robodk_ui import RoboDKStyleUI
from robot_simulation import RobotSimulationWidget
from communication import RobotCommunicator
from donghoc import kinematic, dh_params  # noqa: F401 (giữ tương thích script cũ)
from kinematics_poe import build_model_from_robot_project

DEFAULT_URDF_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets", "ROBOTARM2_description")
# 24/11/2026: chương trình RECORD (dạy tay → phát lại) lưu tại đây, tự nạp lại khi mở app.
RECORD_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "programs", "record_demo.json")
TOOL_TXT = {0: "TẮT", 1: "HÚT", 2: "NHẢ"}


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
        # 24/11/2026: J2 khóa cứng (donghoc.J2_LOCK_DEG, firmware J2_HARD_LOCK) → IK/MoveL chỉ dùng J1, J3–J6;
        # đầu hút được nghiêng tối đa max_tool_tilt_deg khi không giữ được thẳng đứng.
        try:
            import donghoc as _dh
            if getattr(_dh, "J2_LOCK_DEG", None) is not None:
                self.kin.max_tool_tilt_deg = 15.0
                self.kin.lock_joint(1, float(_dh.J2_LOCK_DEG))
        except Exception as _e:
            print(f"[main] Không khóa được J2 trong IK: {_e}")

        # 3. KHỞI TẠO KẾT NỐI MẠNG LAN UDP (W5500)
        self.comm = RobotCommunicator(ip="192.168.1.177", port=8888)
        self.tool_state = 0
        self._rec_tool = 0          # trạng thái giác hút do NGƯỜI DÙNG bấm gần nhất → gắn vào điểm record kế tiếp
        self._playing = False
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
        
        self._load_record_program()
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

    # ================= RECORD / PLAYBACK (24/11/2026) =================
    # Nhiều CHƯƠNG TRÌNH record có tên (mặc định "Sọt 1", "Sọt 2" — mỗi sọt một quỹ đạo gắp → thả riêng).
    # Dạy tay: chọn chương trình đang ghi (chuột phải cây Waypoint Targets), kéo robot tới điểm, bấm HÚT/NHẢ nếu cần,
    # rồi "Add Target". Mỗi điểm lưu 6 góc + trạng thái giác hút áp dụng KHI ĐÃ TỚI điểm đó.
    # "Run Auto": chọn chương trình (hoặc chạy lần lượt tất cả) + số vòng; phát lại từ PC: gửi góc → chờ robot thật tới
    # (telemetry) → bật/tắt giác hút → điểm sau. Twin chạy cùng nhịp (đẹp cho video demo). Lưu ở RECORD_FILE.

    def _record_items(self):
        root = getattr(self.ui, "target_root_item", None)
        if root is None:
            return []
        items = [root.child(i) for i in range(root.childCount())]
        return [it for it in items if isinstance(it.data(0, Qt.ItemDataRole.UserRole), dict)]

    @staticmethod
    def _record_text(data):
        return (f"{data['name']} | {TOOL_TXT.get(int(data['tool']), '?')} | J:["
                + ", ".join(f"{v:.1f}" for v in data["q"]) + "]")

    def _sync_programs_from_tree(self):
        """Danh sách điểm trên cây = chương trình đang chọn."""
        self._programs[self._active_prog] = [dict(it.data(0, Qt.ItemDataRole.UserRole)) for it in self._record_items()]

    def _show_program(self, name):
        """Đổi chương trình đang ghi/hiển thị trên cây Waypoint Targets."""
        if getattr(self, "_programs", None) is None:
            return
        root = getattr(self.ui, "target_root_item", None)
        self._active_prog = name
        self._programs.setdefault(name, [])
        if root is not None:
            for it in self._record_items():
                root.removeChild(it)
            root.setText(0, f"🎯 Waypoint Targets — [{name}]")
            for d in self._programs[name]:
                item = QTreeWidgetItem(["🎯 " + self._record_text(d)])
                item.setData(0, Qt.ItemDataRole.UserRole, dict(d))
                root.addChild(item)
            self.ui.tree_station.expandAll()

    def _save_record_program(self):
        try:
            self._sync_programs_from_tree()
            os.makedirs(os.path.dirname(RECORD_FILE), exist_ok=True)
            with open(RECORD_FILE, "w", encoding="utf-8") as fh:
                json.dump({"version": 2, "active": self._active_prog, "programs": self._programs},
                          fh, indent=2, ensure_ascii=False)
        except Exception as e:
            self.log_message(f"Không lưu được chương trình record: {e}", "ERROR")

    def _load_record_program(self):
        self._programs = {"Sọt 1": [], "Sọt 2": []}
        self._active_prog = "Sọt 1"
        if os.path.isfile(RECORD_FILE):
            try:
                with open(RECORD_FILE, encoding="utf-8") as fh:
                    raw = json.load(fh)
                if "programs" in raw:
                    self._programs = {str(k): list(v) for k, v in raw["programs"].items()} or self._programs
                    self._active_prog = raw.get("active") if raw.get("active") in self._programs else next(iter(self._programs))
                else:                                   # bản 1 chương trình cũ → "Sọt 1"
                    self._programs["Sọt 1"] = list(raw.get("points", []))
                n = sum(len(v) for v in self._programs.values())
                self.log_message(f"Đã nạp {len(self._programs)} chương trình record ({n} điểm) từ {RECORD_FILE}", "INFO")
            except Exception as e:
                self.log_message(f"Không đọc được {RECORD_FILE}: {e}", "ERROR")
        self._show_program(self._active_prog)

    def handle_save_wp(self):
        """RECORD: ghi tư thế lệnh hiện tại + trạng thái giác hút vào chương trình đang chọn."""
        if self._playing:
            return
        n = len(self._record_items()) + 1
        data = {"name": f"P{n:02d}", "q": [float(v) for v in self._current_q()], "tool": int(self._rec_tool)}
        if getattr(self.ui, "target_root_item", None) is not None:
            item = QTreeWidgetItem(["🎯 " + self._record_text(data)])
            item.setData(0, Qt.ItemDataRole.UserRole, data)
            self.ui.target_root_item.addChild(item)
            self.ui.tree_station.expandAll()
        if self._rec_tool == 2:
            self._rec_tool = 0      # NHẢ chỉ áp dụng 1 lần tại điểm thả
        self._save_record_program()
        self.log_message(f"⏺ Record [{self._active_prog}]: {self._record_text(data)}", "INFO")

    def handle_tree_double_click(self, item, column):
        """Kích đúp vào một điểm record trên Cây Trạm để đưa robot tới đó."""
        data = item.data(0, Qt.ItemDataRole.UserRole)
        if isinstance(data, dict) and not self._playing:
            self._set_joints_exact(data["q"])
            self.flush_exact()
            self.log_message(f"MoveJ tới {item.text(0)}", "INFO")

    def flush_exact(self):
        """Gửi ngay góc lệnh chính xác (kể cả phần thập phân) tới ESP32."""
        if self.comm and self.comm.is_connected:
            self.comm.send_pc_command(self._current_q(), self.tool_state, force=True)
            self.has_pending_tx = False

    def show_tree_context_menu(self, position):
        """Menu chuột phải: chọn/tạo/đổi tên chương trình; đi tới / ghi đè / xóa điểm; xóa chương trình."""
        item = self.ui.tree_station.itemAt(position)
        if item is None or self._playing:
            return
        is_point = isinstance(item.data(0, Qt.ItemDataRole.UserRole), dict)
        is_root = item is getattr(self.ui, "target_root_item", None)
        if not (is_point or is_root):
            return
        menu = QMenu(self)
        sub = menu.addMenu(f"📂 Chương trình đang ghi: {self._active_prog}")
        pick_actions = {}
        for name in self._programs:
            act = sub.addAction(("✔ " if name == self._active_prog else "   ") + f"{name} ({len(self._programs[name])} điểm)")
            pick_actions[act] = name
        act_new = menu.addAction("➕ Tạo chương trình mới")
        act_rename = menu.addAction("✏ Đổi tên chương trình này")
        menu.addSeparator()
        act_move = act_update = act_delete = None
        if is_point:
            act_move = menu.addAction("🔄 Đi tới điểm này")
            act_update = menu.addAction("📝 Ghi đè bằng tư thế hiện tại")
            act_delete = menu.addAction("❌ Xóa điểm")
        act_clear = menu.addAction(f"🗑 Xóa toàn bộ điểm của [{self._active_prog}]")
        act_drop = menu.addAction(f"🗑 Xóa hẳn chương trình [{self._active_prog}]")
        action = menu.exec(self.ui.tree_station.mapToGlobal(position))
        if action is None:
            return
        if action in pick_actions:
            self._sync_programs_from_tree()
            self._show_program(pick_actions[action])
            self._save_record_program()
            self.log_message(f"Đang ghi vào chương trình [{self._active_prog}]", "INFO")
        elif action == act_new:
            name, ok = QInputDialog.getText(self, "Chương trình mới", "Tên (ví dụ: Sọt 3):")
            name = name.strip()
            if ok and name and name not in self._programs:
                self._sync_programs_from_tree()
                self._show_program(name)
                self._save_record_program()
        elif action == act_rename:
            name, ok = QInputDialog.getText(self, "Đổi tên", "Tên mới:", text=self._active_prog)
            name = name.strip()
            if ok and name and name not in self._programs:
                self._sync_programs_from_tree()
                self._programs[name] = self._programs.pop(self._active_prog)
                self._show_program(name)
                self._save_record_program()
        elif action == act_move:
            self.handle_tree_double_click(item, 0)
        elif action == act_update:
            data = item.data(0, Qt.ItemDataRole.UserRole)
            data["q"] = self._current_q()
            data["tool"] = int(self._rec_tool)
            item.setData(0, Qt.ItemDataRole.UserRole, data)
            item.setText(0, "🎯 " + self._record_text(data))
            self._save_record_program()
            self.log_message(f"Đã ghi đè {data['name']} [{self._active_prog}]", "INFO")
        elif action == act_delete:
            item.parent().removeChild(item)
            self._save_record_program()
            self.log_message("Đã xóa điểm record", "WARNING")
        elif action == act_clear:
            if QMessageBox.question(self, "Xóa điểm", f"Xóa toàn bộ điểm của [{self._active_prog}]?") == QMessageBox.StandardButton.Yes:
                for it in self._record_items():
                    it.parent().removeChild(it)
                self._save_record_program()
                self.log_message(f"Đã xóa toàn bộ điểm của [{self._active_prog}]", "WARNING")
        elif action == act_drop:
            if len(self._programs) <= 1:
                self.log_message("Phải còn ít nhất 1 chương trình", "WARNING")
            elif QMessageBox.question(self, "Xóa chương trình", f"Xóa hẳn chương trình [{self._active_prog}]?") == QMessageBox.StandardButton.Yes:
                self._programs.pop(self._active_prog)
                self._show_program(next(iter(self._programs)))
                self._save_record_program()

    def handle_run_auto(self):
        """PLAYBACK: chọn chương trình (hoặc lần lượt tất cả) + số vòng; chạy từ PC (robot thật + twin đồng bộ)."""
        if self._playing:
            return
        self._sync_programs_from_tree()
        names = [n for n, pts in self._programs.items() if pts]
        if not names:
            self.log_message("Chưa có điểm record: kéo robot tới điểm rồi bấm 'Add Target'.", "WARNING")
            return
        ALL = "▶ Lần lượt tất cả (" + " → ".join(names) + ")"
        choices = names + ([ALL] if len(names) > 1 else [])
        start = names.index(self._active_prog) if self._active_prog in names else 0
        choice, ok = QInputDialog.getItem(self, "Run Auto", "Chạy chương trình:", choices, start, False)
        if not ok:
            return
        loops, ok = QInputDialog.getInt(self, "Run Auto", "Số vòng:", 1, 1, 100)
        if not ok:
            return
        order = names if choice == ALL else [choice]
        live = bool(self.comm and self.comm.is_connected)
        self.log_message(f"▶ Playback {' → '.join(order)} × {loops} vòng ({'robot thật' if live else 'chỉ mô phỏng'})", "INFO")
        self._playing = True
        self._abort_wait = False
        stopped = False
        try:
            for n in range(loops):
                for prog in order:
                    for p in self._programs[prog]:
                        if self._abort_wait:
                            stopped = True
                            break
                        self.log_message(f"Vòng {n + 1}/{loops} – [{prog}] {p['name']} ({TOOL_TXT.get(p['tool'], '?')})", "INFO")
                        self._set_joints_exact(p["q"])
                        self.flush_exact()
                        if not self._wait_robot_reached(fallback_s=1.2):
                            stopped = True
                            break
                        if int(p["tool"]) != self.tool_state:
                            self.handle_tool_control(int(p["tool"]), user=False)
                            self._sleep_ui(0.8)        # chờ giác hút bám / nhả hẳn
                        else:
                            self._sleep_ui(0.15)
                    if stopped:
                        break
                if stopped:
                    break
        finally:
            self._playing = False
        self.log_message("⏹ Playback dừng giữa chừng" if stopped else "✅ Playback xong",
                         "WARNING" if stopped else "INFO")

    def _sleep_ui(self, seconds):
        t_end = time.time() + seconds
        while time.time() < t_end and not getattr(self, "_abort_wait", False):
            QApplication.processEvents()
            time.sleep(0.02)

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

    def handle_tool_control(self, state, user=True):
        self.tool_state = state
        if user:
            self._rec_tool = state   # điểm record kế tiếp sẽ mang trạng thái này
        txt = {0: "TẮT", 1: "HÚT", 2: "NHẢ"}[state]
        if self.comm and self.comm.is_connected:
            self.comm.send_tool_action(state)
            self.log_message(f"Giác hút: gửi lệnh {txt}", "INFO")
        else:
            self._show_tool_state(state, live=False)
            self.log_message(f"Giác hút: {txt} (chưa kết nối robot — chỉ mô phỏng)", "WARNING")
        if state == 2:
            # 24/11/2026: firmware tự đóng van sau 0.6 s; GUI về TẮT để các lệnh MOVE sau không mang trạng thái NHẢ
            QTimer.singleShot(800, lambda: self.tool_state == 2 and self.handle_tool_control(0, user=False))

    def _show_tool_state(self, state, live=True):
        """Đèn trạng thái giác hút trên GUI: live = theo telemetry robot, ngược lại = theo lệnh."""
        if not hasattr(self.ui, "lbl_tool_status"):
            return
        text, color = {1: ("● ĐANG HÚT", "#22C55E"), 2: ("● ĐANG NHẢ", "#F59E0B")}.get(int(state), ("● TẮT", "#9CA3AF"))
        self.ui.lbl_tool_status.setText(text)
        self.ui.lbl_tool_status.setStyleSheet(f"color: {color}; font-weight: 800;")
        self.ui.lbl_tool_hint.setText("Trạng thái thật từ robot (telemetry)." if live else "Chưa kết nối robot: hiển thị theo lệnh.")
        if getattr(self, "_shown_tool_state", None) != int(state):
            self._shown_tool_state = int(state)
            if getattr(self, "sim", None) is not None and hasattr(self.sim, "set_tool_state"):
                self.sim.set_tool_state(int(state))

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
            if "tool_state" in feedback:
                self._show_tool_state(feedback["tool_state"], live=True)

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
