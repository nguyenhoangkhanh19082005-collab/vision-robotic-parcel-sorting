"""
robodk_ui.py
===================
Giao diện Đỉnh cao Thế giới - RoboDK CAD/CAM Workcell & Industrial Digital Twin Theme
Dành cho Cánh tay Robot 6 DOF.

Bao gồm:
- Thanh Ribbon Toolbar công nghiệp cao cấp (Add Target, MoveJ, MoveL, Connect, Run, Step, Stop)
- Cây Quản lý Trạm (Station Tree) tương tác trực tiếp (Add / Delete / Update / Move to Target)
- 3D Digital Twin Viewport với nút bấm Camera Presets (Iso, Top, Front, Right, Reset)
- Bảng Robot Panel: Joint Jog, Cartesian MoveL, Pieper IK, Tool Vacuum, Tasks (Line, Rectangle, Pick&Place)
- Thanh chỉ số sức khỏe Kinematics: Manipulability Index %, Singularity Alert, UDP Ping Latency ms
- Log Console có màu sắc phân loại sự kiện
"""

from PyQt6.QtWidgets import (
    QWidget, QMainWindow, QVBoxLayout, QHBoxLayout, QLabel, 
    QPushButton, QSlider, QSpinBox, QDoubleSpinBox, QListWidget, 
    QFrame, QTabWidget, QLineEdit, QProgressBar, QTextEdit, 
    QSplitter, QTreeWidget, QTreeWidgetItem, QToolBar, QStatusBar,
    QMenu, QToolButton, QComboBox
)
from PyQt6.QtCore import Qt, QSize
from PyQt6.QtGui import QFont, QColor, QIcon, QAction

WORLDCLASS_ROBODK_QSS = """
/* GLOBAL STYLE - WORLD CLASS ROBODK CAD THEME */
QMainWindow, QWidget#CentralWidget {
    background-color: #121319;
    color: #E2E8F0;
    font-family: 'Segoe UI', 'Inter', 'Roboto', sans-serif;
    font-size: 12px;
}

/* TOP RIBBON TOOLBAR */
QToolBar#RoboDKToolbar {
    background-color: #1A1C24;
    border-bottom: 2px solid #282C3D;
    spacing: 6px;
    padding: 6px;
}

QToolBar QToolButton, QToolBar QPushButton {
    background-color: #232736;
    color: #F1F5F9;
    border: 1px solid #333A52;
    border-radius: 5px;
    padding: 6px 14px;
    font-weight: 700;
}

QToolBar QToolButton:hover, QToolBar QPushButton:hover {
    background-color: #00E5FF;
    color: #0B0E14;
    border-color: #00E5FF;
}

/* STATION TREE VIEW */
QTreeWidget#StationTree {
    background-color: #161821;
    border: 1px solid #282C3D;
    border-radius: 6px;
    color: #E2E8F0;
    font-family: 'Segoe UI', sans-serif;
    font-size: 12px;
}

QTreeWidget#StationTree::item {
    padding: 6px 4px;
}

QTreeWidget#StationTree::item:hover {
    background-color: #232736;
}

QTreeWidget#StationTree::item:selected {
    background-color: rgba(0, 229, 255, 0.2);
    color: #00E5FF;
    font-weight: bold;
    border-left: 3px solid #00E5FF;
}

/* PANELS & CARDS */
QFrame.RoboCard {
    background-color: #1A1C24;
    border: 1px solid #282C3D;
    border-radius: 8px;
}

QLabel.PanelHeader {
    font-size: 12px;
    font-weight: 800;
    color: #00E5FF;
    background-color: #121319;
    padding: 6px 12px;
    border-top-left-radius: 8px;
    border-top-right-radius: 8px;
    border-bottom: 1px solid #282C3D;
    letter-spacing: 0.5px;
}

/* TAB WIDGET */
QTabWidget::pane {
    border: 1px solid #282C3D;
    background-color: #1A1C24;
    border-radius: 6px;
}

QTabBar::tab {
    background-color: #121319;
    color: #94A3B8;
    padding: 7px 14px;
    border: 1px solid #282C3D;
    border-top-left-radius: 5px;
    border-top-right-radius: 5px;
    font-weight: 700;
}

QTabBar::tab:selected {
    background-color: #1A1C24;
    color: #00E5FF;
    border-top: 2px solid #00E5FF;
}

/* BUTTONS */
QPushButton {
    background-color: #232736;
    color: #F1F5F9;
    border: 1px solid #333A52;
    border-radius: 5px;
    padding: 6px 12px;
    font-weight: 700;
}

QPushButton:hover {
    background-color: #2D3347;
    border-color: #00E5FF;
    color: #00E5FF;
}

QPushButton.BtnPrimary {
    background-color: #00E5FF;
    color: #0B0E14;
    border: none;
    font-weight: 800;
}

QPushButton.BtnPrimary:hover {
    background-color: #33ECFF;
}

QPushButton.BtnDanger {
    background-color: #FF1744;
    color: #FFFFFF;
    border: none;
    font-weight: 800;
}

QPushButton.BtnDanger:hover {
    background-color: #FF4569;
}

QPushButton.BtnWarning {
    background-color: #FFAB00;
    color: #0B0E14;
    border: none;
    font-weight: 800;
}

QPushButton.BtnSuccess {
    background-color: #00E676;
    color: #0B0E14;
    border: none;
    font-weight: 800;
}

/* SLIDERS */
QSlider::groove:horizontal {
    height: 6px;
    background: #121319;
    border-radius: 3px;
}

QSlider::sub-page:horizontal {
    background: #00E5FF;
    border-radius: 3px;
}

QSlider::handle:horizontal {
    background: #FFFFFF;
    border: 2px solid #00E5FF;
    width: 16px;
    margin-top: -5px;
    margin-bottom: -5px;
    border-radius: 8px;
}

/* SPINBOX & LINEEDIT */
QSpinBox, QDoubleSpinBox, QLineEdit {
    background-color: #121319;
    border: 1px solid #282C3D;
    border-radius: 4px;
    color: #00E5FF;
    padding: 4px 8px;
    font-weight: 700;
    font-family: 'Consolas', monospace;
}

QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus {
    border-color: #00E5FF;
}

/* PROGRESS BAR */
QProgressBar {
    background-color: #121319;
    border: 1px solid #282C3D;
    border-radius: 4px;
    text-align: center;
    color: #FFFFFF;
    font-weight: 700;
    font-size: 10px;
}

QProgressBar::chunk {
    background-color: #00E5FF;
    border-radius: 3px;
}

/* LOGS & TABLES */
QListWidget, QTextEdit {
    background-color: #121319;
    border: 1px solid #282C3D;
    border-radius: 6px;
    color: #E2E8F0;
    font-family: 'Consolas', monospace;
    font-size: 11px;
}

QStatusBar {
    background-color: #121319;
    color: #94A3B8;
    border-top: 1px solid #282C3D;
}
"""

class RoboDKStyleUI(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.setWindowTitle("AURA-6 World-Class RoboDK Robot Controller & Digital Twin")
        MainWindow.resize(1650, 980)
        MainWindow.setStyleSheet(WORLDCLASS_ROBODK_QSS)

        self.centralwidget = QWidget(parent=MainWindow)
        self.centralwidget.setObjectName("CentralWidget")

        main_layout = QVBoxLayout(self.centralwidget)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(6)

        # =========================================================================
        # 1. TOP RIBBON TOOLBAR (PHONG CÁCH ROBODK CÔNG NGHIỆP)
        # =========================================================================
        self.toolbar = QToolBar("RoboDK Toolbar")
        self.toolbar.setObjectName("RoboDKToolbar")
        MainWindow.addToolBar(Qt.ToolBarArea.TopToolBarArea, self.toolbar)

        self.btn_tb_connect = QPushButton("⚡ Connect Robot (USB / LAN)")
        self.btn_tb_add_target = QPushButton("🎯 Add Target")
        self.btn_tb_add_target.setProperty("class", "BtnPrimary")
        
        self.btn_tb_movej = QPushButton("🔄 MoveJ (Home 0°)")
        self.btn_tb_set_zero = QPushButton("🎯 Lưu Gốc (Flash)")
        self.btn_tb_set_zero.setStyleSheet("background-color: #F59E0B; color: #000000; font-weight: 800;")
        self.btn_tb_unlock_zero = QPushButton("🔓 Đổi Gốc")
        self.btn_tb_unlock_zero.setStyleSheet("background-color: #232736; color: #94A3B8; font-weight: 700;")
        self.btn_tb_movel = QPushButton("📏 MoveL (Pieper IK)")
        
        self.btn_tb_run = QPushButton("▶ Run Auto")
        self.btn_tb_run.setProperty("class", "BtnSuccess")
        
        self.btn_tb_stop = QPushButton("🛑 EMERGENCY STOP")
        self.btn_tb_stop.setProperty("class", "BtnDanger")

        self.toolbar.addWidget(self.btn_tb_connect)
        self.toolbar.addSeparator()
        self.toolbar.addWidget(self.btn_tb_add_target)
        self.toolbar.addWidget(self.btn_tb_movej)
        self.toolbar.addWidget(self.btn_tb_set_zero)
        self.toolbar.addWidget(self.btn_tb_unlock_zero)
        self.toolbar.addWidget(self.btn_tb_movel)
        self.toolbar.addSeparator()
        self.toolbar.addWidget(self.btn_tb_run)
        self.toolbar.addWidget(self.btn_tb_stop)

        # =========================================================================
        # 2. MAIN SPLITTER (STATION TREE | 3D VIEWPORT | ROBOT PANEL)
        # =========================================================================
        body_splitter = QSplitter(Qt.Orientation.Horizontal)

        # -------------------------------------------------------------------------
        # LEFT PANEL: STATION TREE VIEW (CÂY TRẠM WORKCELL AN TOÀN 100%)
        # -------------------------------------------------------------------------
        tree_panel = QFrame()
        tree_panel.setProperty("class", "RoboCard")
        tree_panel.setMinimumWidth(280)
        tree_panel.setMaximumWidth(360)
        tree_layout = QVBoxLayout(tree_panel)
        tree_layout.setContentsMargins(6, 6, 6, 6)

        lbl_tree_head = QLabel("ROBODK STATION TREE")
        lbl_tree_head.setProperty("class", "PanelHeader")
        tree_layout.addWidget(lbl_tree_head)

        self.tree_station = QTreeWidget()
        self.tree_station.setObjectName("StationTree")
        self.tree_station.setHeaderLabel("Workcell Objects & Targets")
        self.tree_station.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)

        # Khởi tạo danh trúc Cây Trạm và Lưu Mọi Biến Gốc An Toàn Tuyệt Đối (Zero Crash)
        self.station_root_item = QTreeWidgetItem(self.tree_station, ["📍 Station 1 (AURA-6 Cell)"])
        self.robot_tree_item = QTreeWidgetItem(self.station_root_item, ["🤖 AURA 6-DOF Robot Arm"])
        
        QTreeWidgetItem(self.robot_tree_item, ["📐 Base Frame (World 0,0,0)"])
        self.tool_tree_item = QTreeWidgetItem(self.robot_tree_item, ["🧲 Tool: Vacuum Gripper"])
        
        self.target_root_item = QTreeWidgetItem(self.robot_tree_item, ["🎯 Waypoint Targets"])
        QTreeWidgetItem(self.target_root_item, ["Target 01 (Home 0,0,0,0,0,0)"])

        self.prog_root_item = QTreeWidgetItem(self.robot_tree_item, ["📜 Programs"])
        QTreeWidgetItem(self.prog_root_item, ["Prog 1: PickAndPlace"])
        QTreeWidgetItem(self.prog_root_item, ["Prog 2: DrawLine (Task 1)"])
        QTreeWidgetItem(self.prog_root_item, ["Prog 3: DrawRectangle (Task 2)"])

        self.tree_station.expandAll()
        tree_layout.addWidget(self.tree_station)

        # Trạng thái Sức khỏe & Chỉ số Kinematics (Manipulability Gauge)
        health_box = QFrame()
        health_box.setStyleSheet("background-color: #121319; border-radius: 6px; padding: 6px;")
        h_lay = QVBoxLayout(health_box)
        
        lbl_m_title = QLabel("ROBOT MANIPULABILITY & SINGULARITY")
        lbl_m_title.setStyleSheet("font-size: 10px; font-weight: 700; color: #64748B;")
        h_lay.addWidget(lbl_m_title)

        self.bar_manipulability = QProgressBar()
        self.bar_manipulability.setRange(0, 100)
        self.bar_manipulability.setValue(92)
        self.bar_manipulability.setFormat("Manipulability: %p%")
        h_lay.addWidget(self.bar_manipulability)

        tree_layout.addWidget(health_box)
        body_splitter.addWidget(tree_panel)

        # -------------------------------------------------------------------------
        # CENTER PANEL: 3D DIGITAL TWIN VIEWPORT & CAMERA PRESETS
        # -------------------------------------------------------------------------
        center_panel = QFrame()
        center_panel.setProperty("class", "RoboCard")
        center_layout = QVBoxLayout(center_panel)
        center_layout.setContentsMargins(6, 6, 6, 6)

        view_bar = QHBoxLayout()
        lbl_view_title = QLabel("3D DIGITAL TWIN WORKCELL VIEWPORT")
        lbl_view_title.setProperty("class", "PanelHeader")
        
        self.lbl_tcp_hud = QLabel("TCP: X=250.0 Y=0.0 Z=200.0 mm | Roll=0° Pitch=-90° Yaw=0°")
        self.lbl_tcp_hud.setStyleSheet("color: #00E5FF; font-family: Consolas; font-weight: 700;")

        view_bar.addWidget(lbl_view_title)
        view_bar.addStretch()
        view_bar.addWidget(self.lbl_tcp_hud)
        center_layout.addLayout(view_bar)

        # Frame chứa PyVista 3D Canvas
        self.frm_main_simulation = QFrame()
        self.frm_main_simulation.setStyleSheet("background-color: #0B0D12; border-radius: 6px;")
        center_layout.addWidget(self.frm_main_simulation)

        body_splitter.addWidget(center_panel)

        # -------------------------------------------------------------------------
        # RIGHT PANEL: ROBOT PANEL (JOINTS / CARTESIAN / TASKS / TOOL)
        # -------------------------------------------------------------------------
        right_panel = QFrame()
        right_panel.setProperty("class", "RoboCard")
        right_panel.setMinimumWidth(380)
        right_panel.setMaximumWidth(460)
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(6, 6, 6, 6)

        lbl_robot_panel_head = QLabel("ROBOT CONTROLLER PANEL")
        lbl_robot_panel_head.setProperty("class", "PanelHeader")
        right_layout.addWidget(lbl_robot_panel_head)

        self.robot_tabs = QTabWidget()

        # TAB 1: JOINT JOGGING (J1-J6)
        tab_joints = QWidget()
        joint_layout = QVBoxLayout(tab_joints)

        self.sliders = []
        self.spinboxes = []
        self.lbl_feedbacks = []

        joint_names = ["Waist (J1)", "Shoulder (J2)", "Elbow (J3)", "Wrist Roll (J4)", "Wrist Pitch (J5)", "Wrist Yaw (J6)"]
        # 24/11/2026: lấy giới hạn từ donghoc (tư thế 0 chữ L đầu hút chúc xuống); J2 khóa cứng → slider J2 khóa ở góc khóa
        try:
            import donghoc as _dh
            joint_ranges = [(int(lo), int(hi)) for lo, hi in _dh.kinematic().joint_limits]
            j2_lock = getattr(_dh, "J2_LOCK_DEG", None)
        except Exception:
            joint_ranges = [(-170, 170), (-90, 110), (-225, 45), (-180, 180), (-20, 200), (-360, 360)]
            j2_lock = 0.0
        if j2_lock is not None:
            joint_ranges[1] = (int(round(j2_lock)), int(round(j2_lock)))
            joint_names[1] = f"Shoulder (J2) 🔒 {j2_lock:g}°"

        for i in range(6):
            j_box = QVBoxLayout()
            row1 = QHBoxLayout()
            lbl_n = QLabel(joint_names[i])
            lbl_n.setStyleSheet("font-weight: 700;")
            
            spin = QSpinBox()
            spin.setRange(joint_ranges[i][0], joint_ranges[i][1])
            spin.setSuffix("°")
            spin.setFixedWidth(75)

            lbl_fb = QLabel("Act: 0.0°")
            lbl_fb.setStyleSheet("color: #00E676; font-family: Consolas; font-weight: 700;")

            row1.addWidget(lbl_n); row1.addStretch(); row1.addWidget(lbl_fb); row1.addWidget(spin)
            j_box.addLayout(row1)

            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(joint_ranges[i][0], joint_ranges[i][1])
            slider.setValue(0)
            if i == 1 and j2_lock is not None:   # J2 khóa cứng: không cho kéo
                slider.setEnabled(False)
                spin.setEnabled(False)
            j_box.addWidget(slider)

            joint_layout.addLayout(j_box)

            self.sliders.append(slider)
            self.spinboxes.append(spin)
            self.lbl_feedbacks.append(lbl_fb)

        row_quick_calib = QHBoxLayout()
        self.btn_joint_home = QPushButton("🔄 Move Home (0°)")
        self.btn_joint_set_zero = QPushButton("🎯 Lưu Gốc (Flash)")
        self.btn_joint_set_zero.setStyleSheet("background-color: #F59E0B; color: #000000; font-weight: 800; padding: 6px;")
        self.btn_joint_unlock_zero = QPushButton("🔓 Đổi Gốc")
        self.btn_joint_unlock_zero.setStyleSheet("background-color: #232736; color: #94A3B8; font-weight: 700; padding: 6px;")
        row_quick_calib.addWidget(self.btn_joint_home)
        row_quick_calib.addWidget(self.btn_joint_set_zero)
        row_quick_calib.addWidget(self.btn_joint_unlock_zero)
        joint_layout.addLayout(row_quick_calib)

        joint_layout.addStretch()
        self.robot_tabs.addTab(tab_joints, "JOINTS JOG")

        # TAB 2: CARTESIAN MOVE L & PIEPER IK
        tab_cart = QWidget()
        cart_layout = QVBoxLayout(tab_cart)

        self.spb_ik_x = QDoubleSpinBox(); self.spb_ik_x.setRange(-800, 800); self.spb_ik_x.setValue(250); self.spb_ik_x.setSuffix(" mm")
        self.spb_ik_y = QDoubleSpinBox(); self.spb_ik_y.setRange(-800, 800); self.spb_ik_y.setValue(0); self.spb_ik_y.setSuffix(" mm")
        self.spb_ik_z = QDoubleSpinBox(); self.spb_ik_z.setRange(-500, 800); self.spb_ik_z.setValue(200); self.spb_ik_z.setSuffix(" mm")

        rx_lay = QHBoxLayout(); rx_lay.addWidget(QLabel("Target TCP X:")); rx_lay.addWidget(self.spb_ik_x)
        ry_lay = QHBoxLayout(); ry_lay.addWidget(QLabel("Target TCP Y:")); ry_lay.addWidget(self.spb_ik_y)
        rz_lay = QHBoxLayout(); rz_lay.addWidget(QLabel("Target TCP Z:")); rz_lay.addWidget(self.spb_ik_z)

        cart_layout.addLayout(rx_lay); cart_layout.addLayout(ry_lay); cart_layout.addLayout(rz_lay)

        self.btn_movel_run = QPushButton("MoveL (Pieper Analytical IK)")
        self.btn_movel_run.setProperty("class", "BtnPrimary")
        self.btn_movel_run.setFixedHeight(36)
        cart_layout.addWidget(self.btn_movel_run)

        cart_layout.addStretch()
        self.robot_tabs.addTab(tab_cart, "CARTESIAN (IK)")

        # TAB 3: CHƯƠNG TRÌNH TỰ ĐỘNG (AUTOMATION TASKS)
        tab_tasks = QWidget()
        task_layout = QVBoxLayout(tab_tasks)

        self.btn_task1_line = QPushButton("Task 1: Vẽ Đường Thẳng (Line Trajectory)")
        self.btn_task2_rect = QPushButton("Task 2: Vẽ Hình Chữ Nhật (Rectangle)")
        self.btn_task3_free = QPushButton("Task 3: Pick & Place Tự Do (Cycle)")

        task_layout.addWidget(self.btn_task1_line)
        task_layout.addWidget(self.btn_task2_rect)
        task_layout.addWidget(self.btn_task3_free)
        task_layout.addStretch()

        self.robot_tabs.addTab(tab_tasks, "AUTOMATION")

        # TAB 4: KẾT NỐI ĐIỀU KHIỂN DUAL-MODE (USB TYPE-C SERIAL & ETHERNET W5500 LAN)
        tab_net = QWidget()
        net_layout = QVBoxLayout(tab_net)
        net_layout.setContentsMargins(10, 10, 10, 10)
        net_layout.setSpacing(10)

        # 1. Chọn phương thức kết nối
        mode_box = QHBoxLayout()
        mode_box.addWidget(QLabel("Giao thức:"))
        self.cmb_conn_type = QComboBox()
        self.cmb_conn_type.addItems(["🔌 USB Type-C (Serial COM)", "🌐 Ethernet W5500 (LAN UDP)"])
        mode_box.addWidget(self.cmb_conn_type)
        net_layout.addLayout(mode_box)

        # 2. Khung cài đặt USB Serial (Mặc định hiển thị)
        self.grp_usb = QFrame()
        self.grp_usb.setStyleSheet("background-color: #121319; border: 1px solid #282C3D; border-radius: 6px; padding: 6px;")
        usb_layout = QVBoxLayout(self.grp_usb)
        
        u1 = QHBoxLayout()
        u1.addWidget(QLabel("Cổng COM:"))
        self.cmb_com_port = QComboBox()
        self.cmb_com_port.setEditable(True)
        self.cmb_com_port.setPlaceholderText("VD: COM3, COM5...")
        u1.addWidget(self.cmb_com_port)
        self.btn_refresh_ports = QPushButton("🔄 Quét")
        self.btn_refresh_ports.setFixedWidth(65)
        u1.addWidget(self.btn_refresh_ports)
        usb_layout.addLayout(u1)

        u2 = QHBoxLayout()
        u2.addWidget(QLabel("Baudrate:"))
        self.cmb_baudrate = QComboBox()
        self.cmb_baudrate.addItems(["115200", "921600", "230400", "57600"])
        u2.addWidget(self.cmb_baudrate)
        usb_layout.addLayout(u2)
        net_layout.addWidget(self.grp_usb)

        # 3. Khung cài đặt Ethernet LAN W5500 (Ẩn khi chọn USB)
        self.grp_net = QFrame()
        self.grp_net.setStyleSheet("background-color: #121319; border: 1px solid #282C3D; border-radius: 6px; padding: 6px;")
        net_box_layout = QVBoxLayout(self.grp_net)
        r1 = QHBoxLayout(); r1.addWidget(QLabel("Robot IP:")); self.txt_ip = QLineEdit("192.168.1.177"); r1.addWidget(self.txt_ip)
        r2 = QHBoxLayout(); r2.addWidget(QLabel("Port UDP:")); self.txt_port = QLineEdit("8888"); r2.addWidget(self.txt_port)
        net_box_layout.addLayout(r1); net_box_layout.addLayout(r2)
        net_layout.addWidget(self.grp_net)
        self.grp_net.hide()

        # Logic chuyển đổi hiển thị giữa USB và Ethernet
        def _on_conn_mode_changed(idx):
            if idx == 0:
                self.grp_usb.show()
                self.grp_net.hide()
            else:
                self.grp_usb.hide()
                self.grp_net.show()
        self.cmb_conn_type.currentIndexChanged.connect(_on_conn_mode_changed)

        # 4. Nút bấm Kết Nối Thông Minh
        self.btn_connect_udp = QPushButton("KẾT NỐI ROBOT (USB / LAN)")
        self.btn_connect_udp.setProperty("class", "BtnSuccess")
        self.btn_connect_udp.setFixedHeight(38)
        net_layout.addWidget(self.btn_connect_udp)

        lbl_hint = QLabel("💡 Dual-Mode: Cắm cáp USB Type-C điều khiển trực tiếp cực nhanh, không cần cấu hình mạng LAN.")
        lbl_hint.setStyleSheet("color: #64748B; font-size: 11px;")
        lbl_hint.setWordWrap(True)
        net_layout.addWidget(lbl_hint)
        net_layout.addStretch()

        self.robot_tabs.addTab(tab_net, "CONNECTION")

        right_layout.addWidget(self.robot_tabs)

        # TOOL CONTROL CARD (VACUUM PUMP)
        tool_card = QFrame()
        tool_card.setStyleSheet("background-color: #121319; border-radius: 6px; padding: 6px;")
        tool_lay = QVBoxLayout(tool_card)
        # 24/11/2026: giác hút = bơm chân không + van điện từ qua L298N (GPIO10 bơm, GPIO11 van — xem firmware).
        t_head = QHBoxLayout()
        lbl_tool_title = QLabel("GIÁC HÚT (BƠM + VAN ĐIỆN TỪ)")
        lbl_tool_title.setStyleSheet("color: #E5E7EB; font-weight: 800;")
        self.lbl_tool_status = QLabel("● TẮT")
        self.lbl_tool_status.setStyleSheet("color: #9CA3AF; font-weight: 800;")
        t_head.addWidget(lbl_tool_title); t_head.addStretch(); t_head.addWidget(self.lbl_tool_status)
        tool_lay.addLayout(t_head)

        t_btn_lay = QHBoxLayout()
        self.btn_tool_suck = QPushButton("🟢 HÚT")
        self.btn_tool_suck.setToolTip("Bật bơm hút, đóng van xả (giữ vật)")
        self.btn_tool_release = QPushButton("🟠 NHẢ")
        self.btn_tool_release.setToolTip("Tắt bơm, mở van xả chân không ~0.6 s rồi tự tắt")
        self.btn_tool_off = QPushButton("⏹ TẮT")
        self.btn_tool_off.setToolTip("Tắt bơm và van")
        for btn, bg in ((self.btn_tool_suck, "#16A34A"), (self.btn_tool_release, "#D97706"), (self.btn_tool_off, "#374151")):
            btn.setMinimumHeight(38)
            btn.setStyleSheet(f"QPushButton {{ background-color: {bg}; color: #FFFFFF; font-weight: 800; "
                              f"border-radius: 5px; padding: 6px; }} QPushButton:hover {{ border: 2px solid #E5E7EB; }}")
            t_btn_lay.addWidget(btn)
        tool_lay.addLayout(t_btn_lay)
        self.lbl_tool_hint = QLabel("Trạng thái lấy từ robot (telemetry) khi đã kết nối.")
        self.lbl_tool_hint.setStyleSheet("color: #6B7280; font-size: 11px;")
        tool_lay.addWidget(self.lbl_tool_hint)
        right_layout.addWidget(tool_card)

        body_splitter.addWidget(right_panel)

        body_splitter.setSizes([300, 850, 420])
        main_layout.addWidget(body_splitter)

        # =========================================================================
        # 3. BOTTOM PANEL: EVENT LOG CONSOLE
        # =========================================================================
        bottom_panel = QFrame()
        bottom_panel.setProperty("class", "RoboCard")
        bottom_panel.setMaximumHeight(110)
        bot_layout = QVBoxLayout(bottom_panel)
        bot_layout.setContentsMargins(6, 4, 6, 4)

        bot_layout.addWidget(QLabel("SYSTEM DIAGNOSTICS & EVENT LOG CONSOLE"))
        self.txt_logs = QTextEdit()
        self.txt_logs.setReadOnly(True)
        bot_layout.addWidget(self.txt_logs)

        main_layout.addWidget(bottom_panel)

        MainWindow.setCentralWidget(self.centralwidget)

        # StatusBar
        self.statusbar = QStatusBar(MainWindow)
        MainWindow.setStatusBar(self.statusbar)
        self.statusbar.showMessage("RoboDK World-Class Workcell Ready | UDP Disconnected")
