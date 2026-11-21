"""
modern_ui.py
===================
Giao diện Chuyên nghiệp chuẩn Công nghiệp (Industrial Dark Cyber Theme) 
cho Cánh tay Robot 6 DOF.

Bao gồm:
- Header Status Bar với đèn báo trạng thái & E-STOP
- Joint Jogging (J1-J6) & Cartesian Jogging (X, Y, Z, R, P, Y)
- Digital Twin 3D Viewport với Camera Presets
- Teach Pendant Program Manager (Lập trình Waypoints & G-code)
- Real-time Diagnostics & Event Log Console
"""

from PyQt6.QtWidgets import (
    QWidget, QMainWindow, QVBoxLayout, QHBoxLayout, QLabel, 
    QPushButton, QSlider, QSpinBox, QDoubleSpinBox, QListWidget, 
    QListWidgetItem, QFrame, QTabWidget, QLineEdit, QProgressBar, 
    QTextEdit, QGroupBox, QSplitter, QHeaderView, QTableWidget, QTableWidgetItem
)
from PyQt6.QtCore import Qt, QSize
from PyQt6.QtGui import QFont, QColor, QIcon

INDUSTRIAL_QSS = """
/* GLOBAL STYLE */
QMainWindow, QWidget#CentralWidget {
    background-color: #111319;
    color: #E2E8F0;
    font-family: 'Segoe UI', 'Inter', 'Roboto', sans-serif;
    font-size: 13px;
}

/* HEADER BAR */
QFrame#HeaderFrame {
    background-color: #181B26;
    border-bottom: 2px solid #232736;
    min-height: 54px;
    max-height: 54px;
    padding: 0 10px;
}

QLabel#AppTitle {
    font-size: 18px;
    font-weight: 800;
    color: #00E5FF;
    letter-spacing: 1px;
}

QLabel#AppSubtitle {
    font-size: 11px;
    color: #64748B;
    font-weight: 600;
}

/* STATUS BADGES */
QLabel.Badge {
    background-color: #1E2333;
    border: 1px solid #2E364F;
    border-radius: 6px;
    padding: 4px 10px;
    font-size: 11px;
    font-weight: 700;
    color: #94A3B8;
}

QLabel.BadgeOnline {
    background-color: rgba(0, 230, 118, 0.12);
    border: 1px solid #00E676;
    color: #00E676;
}

QLabel.BadgeError {
    background-color: rgba(255, 23, 68, 0.12);
    border: 1px solid #FF1744;
    color: #FF1744;
}

/* PANELS & CARDS */
QFrame.CardPanel {
    background-color: #181B26;
    border: 1px solid #232736;
    border-radius: 8px;
}

QLabel.PanelTitle {
    font-size: 13px;
    font-weight: 700;
    color: #00E5FF;
    padding-bottom: 4px;
    border-bottom: 1px solid #232736;
}

/* TAB WIDGET */
QTabWidget::pane {
    border: 1px solid #232736;
    background-color: #181B26;
    border-radius: 8px;
    top: -1px;
}

QTabBar::tab {
    background-color: #111319;
    color: #94A3B8;
    padding: 8px 16px;
    border: 1px solid #232736;
    border-bottom: none;
    border-top-left-radius: 6px;
    border-top-right-radius: 6px;
    font-weight: 600;
    margin-right: 4px;
}

QTabBar::tab:selected {
    background-color: #181B26;
    color: #00E5FF;
    border-top: 2px solid #00E5FF;
}

QTabBar::tab:hover {
    color: #F1F5F9;
}

/* BUTTONS */
QPushButton {
    background-color: #232736;
    color: #F1F5F9;
    border: 1px solid #333A52;
    border-radius: 6px;
    padding: 6px 12px;
    font-weight: 600;
}

QPushButton:hover {
    background-color: #2D3347;
    border-color: #00E5FF;
    color: #00E5FF;
}

QPushButton:pressed {
    background-color: #1A1E2B;
}

QPushButton.BtnPrimary {
    background-color: #00E5FF;
    color: #0B0E14;
    border: none;
    font-weight: 800;
}

QPushButton.BtnPrimary:hover {
    background-color: #33ECFF;
    box-shadow: 0 0 10px rgba(0,229,255,0.5);
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
    background: #232736;
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

QSlider::handle:horizontal:hover {
    background: #00E5FF;
}

/* SPINBOX & LINEEDIT */
QSpinBox, QDoubleSpinBox, QLineEdit {
    background-color: #111319;
    border: 1px solid #232736;
    border-radius: 5px;
    color: #00E5FF;
    padding: 4px 8px;
    font-weight: 700;
    font-family: 'Consolas', monospace;
}

QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus {
    border-color: #00E5FF;
}

/* LIST & LOGS */
QListWidget, QTextEdit, QTableWidget {
    background-color: #111319;
    border: 1px solid #232736;
    border-radius: 6px;
    color: #E2E8F0;
    font-family: 'Consolas', 'Courier New', monospace;
    font-size: 12px;
}

QListWidget::item {
    padding: 6px 10px;
    border-bottom: 1px solid #1E2333;
}

QListWidget::item:selected {
    background-color: rgba(0, 230, 118, 0.15);
    color: #00E676;
    border-left: 3px solid #00E676;
}

/* SCROLLBAR */
QScrollBar:vertical {
    background: #111319;
    width: 8px;
    margin: 0px;
}

QScrollBar::handle:vertical {
    background: #232736;
    min-height: 20px;
    border-radius: 4px;
}

QScrollBar::handle:vertical:hover {
    background: #00E5FF;
}
"""

class ModernRobotUI(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.setWindowTitle("AURA-6 Industrial Robot Controller & Digital Twin")
        MainWindow.resize(1600, 960)
        MainWindow.setStyleSheet(INDUSTRIAL_QSS)

        self.centralwidget = QWidget(parent=MainWindow)
        self.centralwidget.setObjectName("CentralWidget")
        
        main_layout = QVBoxLayout(self.centralwidget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # =========================================================================
        # 1. HEADER BAR
        # =========================================================================
        header_frame = QFrame()
        header_frame.setObjectName("HeaderFrame")
        header_layout = QHBoxLayout(header_frame)
        header_layout.setContentsMargins(10, 0, 10, 0)

        # Brand Title
        title_box = QVBoxLayout()
        lbl_title = QLabel("AURA-6 ROBOTICS")
        lbl_title.setObjectName("AppTitle")
        lbl_sub = QLabel("6-DOF SPHERICAL WRIST CONTROLLER & DIGITAL TWIN")
        lbl_sub.setObjectName("AppSubtitle")
        title_box.addWidget(lbl_title)
        title_box.addWidget(lbl_sub)
        header_layout.addLayout(title_box)

        header_layout.addStretch()

        # Status Badges
        self.badge_net = QLabel("CONNECTING...")
        self.badge_net.setProperty("class", "Badge")
        self.badge_mode = QLabel("MODE: MANUAL")
        self.badge_mode.setProperty("class", "Badge")
        self.badge_safety = QLabel("SAFETY: OK")
        self.badge_safety.setProperty("class", "Badge BadgeOnline")

        header_layout.addWidget(self.badge_net)
        header_layout.addWidget(self.badge_mode)
        header_layout.addWidget(self.badge_safety)

        # Action Header Buttons
        self.btn_estop = QPushButton("EMERGENCY STOP")
        self.btn_estop.setProperty("class", "BtnDanger")
        self.btn_estop.setFixedHeight(36)
        
        self.btn_home_all = QPushButton("HOME ALL (0°)")
        self.btn_home_all.setProperty("class", "BtnWarning")
        self.btn_home_all.setFixedHeight(36)

        header_layout.addWidget(self.btn_home_all)
        header_layout.addWidget(self.btn_estop)

        main_layout.addWidget(header_frame)

        # =========================================================================
        # 2. MAIN BODY (SPLITTER 3 PANELS)
        # =========================================================================
        body_splitter = QSplitter(Qt.Orientation.Horizontal)

        # -------------------------------------------------------------------------
        # LEFT PANEL: CONTROLS & JOGGING (Width: ~420px)
        # -------------------------------------------------------------------------
        left_panel = QFrame()
        left_panel.setProperty("class", "CardPanel")
        left_panel.setMinimumWidth(380)
        left_panel.setMaximumWidth(460)
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(10, 10, 10, 10)

        # Tab Widget cho Joint Jog vs Cartesian Jog vs Connection
        self.control_tabs = QTabWidget()
        
        # TAB 1: JOINT JOGGING (J1-J6)
        tab_joints = QWidget()
        joint_layout = QVBoxLayout(tab_joints)

        self.sliders = []
        self.spinboxes = []
        self.lbl_feedbacks = []

        joint_names = ["Waist (J1)", "Shoulder (J2)", "Elbow (J3)", "Wrist Roll (J4)", "Wrist Pitch (J5)", "Wrist Yaw (J6)"]
        joint_ranges = [(-170, 170), (-90, 110), (-150, 150), (-180, 180), (-110, 110), (-360, 360)]

        for i in range(6):
            j_box = QVBoxLayout()
            row1 = QHBoxLayout()
            
            lbl_name = QLabel(joint_names[i])
            lbl_name.setStyleSheet("font-weight: 700; color: #E2E8F0;")
            
            spin = QSpinBox()
            spin.setRange(joint_ranges[i][0], joint_ranges[i][1])
            spin.setSuffix("°")
            spin.setFixedWidth(80)

            lbl_fb = QLabel("Act: 0.0°")
            lbl_fb.setStyleSheet("color: #00E676; font-family: Consolas; font-weight: 700;")

            row1.addWidget(lbl_name)
            row1.addStretch()
            row1.addWidget(lbl_fb)
            row1.addWidget(spin)
            j_box.addLayout(row1)

            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(joint_ranges[i][0], joint_ranges[i][1])
            slider.setValue(0)
            j_box.addWidget(slider)

            joint_layout.addLayout(j_box)

            self.sliders.append(slider)
            self.spinboxes.append(spin)
            self.lbl_feedbacks.append(lbl_fb)

        joint_layout.addStretch()
        self.control_tabs.addTab(tab_joints, "JOINT JOG")

        # TAB 2: CARTESIAN JOG & IK
        tab_cartesian = QWidget()
        cart_layout = QVBoxLayout(tab_cartesian)

        cart_title = QLabel("CARTESIAN TARGET (TCP POSITION)")
        cart_title.setProperty("class", "PanelTitle")
        cart_layout.addWidget(cart_title)

        # Ô nhập X, Y, Z
        grid_xyz = QVBoxLayout()
        self.spb_ik_x = QDoubleSpinBox()
        self.spb_ik_x.setRange(-800, 800)
        self.spb_ik_x.setValue(250)
        self.spb_ik_x.setSuffix(" mm")

        self.spb_ik_y = QDoubleSpinBox()
        self.spb_ik_y.setRange(-800, 800)
        self.spb_ik_y.setValue(0)
        self.spb_ik_y.setSuffix(" mm")

        self.spb_ik_z = QDoubleSpinBox()
        self.spb_ik_z.setRange(-500, 800)
        self.spb_ik_z.setValue(200)
        self.spb_ik_z.setSuffix(" mm")

        row_x = QHBoxLayout(); row_x.addWidget(QLabel("TCP X:")); row_x.addWidget(self.spb_ik_x)
        row_y = QHBoxLayout(); row_y.addWidget(QLabel("TCP Y:")); row_y.addWidget(self.spb_ik_y)
        row_z = QHBoxLayout(); row_z.addWidget(QLabel("TCP Z:")); row_z.addWidget(self.spb_ik_z)

        grid_xyz.addLayout(row_x); grid_xyz.addLayout(row_y); grid_xyz.addLayout(row_z)
        cart_layout.addLayout(grid_xyz)

        self.btn_ik_run = QPushButton("EXECUTE PIEPER IK")
        self.btn_ik_run.setProperty("class", "BtnPrimary")
        self.btn_ik_run.setFixedHeight(38)
        cart_layout.addWidget(self.btn_ik_run)

        cart_layout.addStretch()
        self.control_tabs.addTab(tab_cartesian, "CARTESIAN (IK)")

        # TAB 3: NETWORK CONFIG
        tab_net = QWidget()
        net_layout = QVBoxLayout(tab_net)

        row_ip = QHBoxLayout()
        row_ip.addWidget(QLabel("Robot IP:"))
        self.txt_ip = QLineEdit("192.168.1.177")
        row_ip.addWidget(self.txt_ip)

        row_port = QHBoxLayout()
        row_port.addWidget(QLabel("UDP Port:"))
        self.txt_port = QLineEdit("8888")
        row_port.addWidget(self.txt_port)

        net_layout.addLayout(row_ip)
        net_layout.addLayout(row_port)

        self.btn_connect_udp = QPushButton("CONNECT W5500 ETHERNET")
        self.btn_connect_udp.setProperty("class", "BtnSuccess")
        self.btn_connect_udp.setFixedHeight(38)
        net_layout.addWidget(self.btn_connect_udp)
        net_layout.addStretch()

        self.control_tabs.addTab(tab_net, "NETWORK")

        left_layout.addWidget(self.control_tabs)

        # TOOL CONTROL CARD AT BOTTOM OF LEFT PANEL
        tool_card = QFrame()
        tool_card.setStyleSheet("background-color: #111319; border-radius: 6px; padding: 8px;")
        tool_layout = QVBoxLayout(tool_card)
        lbl_tool_title = QLabel("END-EFFECTOR TOOL (VACUUM PUMP)")
        lbl_tool_title.setStyleSheet("font-weight: 700; color: #F4A460; font-size: 11px;")
        tool_layout.addWidget(lbl_tool_title)

        tool_btn_box = QHBoxLayout()
        self.btn_tool_off = QPushButton("OFF (0)")
        self.btn_tool_suck = QPushButton("SUCK (1)")
        self.btn_tool_release = QPushButton("RELEASE (2)")

        self.btn_tool_off.setProperty("class", "BtnDefault")
        self.btn_tool_suck.setProperty("class", "BtnPrimary")
        self.btn_tool_release.setProperty("class", "BtnWarning")

        tool_btn_box.addWidget(self.btn_tool_off)
        tool_btn_box.addWidget(self.btn_tool_suck)
        tool_btn_box.addWidget(self.btn_tool_release)
        tool_layout.addLayout(tool_btn_box)

        left_layout.addWidget(tool_card)

        body_splitter.addWidget(left_panel)

        # -------------------------------------------------------------------------
        # CENTER PANEL: 3D DIGITAL TWIN VIEWPORT & HUD OVERLAY
        # -------------------------------------------------------------------------
        center_panel = QFrame()
        center_panel.setProperty("class", "CardPanel")
        center_layout = QVBoxLayout(center_panel)
        center_layout.setContentsMargins(10, 10, 10, 10)

        # Top Bar of 3D Viewport
        view_bar = QHBoxLayout()
        lbl_view_title = QLabel("3D DIGITAL TWIN VIEWPORT")
        lbl_view_title.setProperty("class", "PanelTitle")
        
        self.lbl_tcp_hud = QLabel("TCP: X=250.0 Y=0.0 Z=200.0 mm")
        self.lbl_tcp_hud.setStyleSheet("color: #00E5FF; font-family: Consolas; font-weight: 700;")

        view_bar.addWidget(lbl_view_title)
        view_bar.addStretch()
        view_bar.addWidget(self.lbl_tcp_hud)
        center_layout.addLayout(view_bar)

        # Container Frame cho PyVista Canvas
        self.frm_main_simulation = QFrame()
        self.frm_main_simulation.setStyleSheet("background-color: #0B0D12; border-radius: 6px;")
        center_layout.addWidget(self.frm_main_simulation)

        body_splitter.addWidget(center_panel)

        # -------------------------------------------------------------------------
        # RIGHT PANEL: TEACH PENDANT & PROGRAM MANAGER (Width: ~380px)
        # -------------------------------------------------------------------------
        right_panel = QFrame()
        right_panel.setProperty("class", "CardPanel")
        right_panel.setMinimumWidth(340)
        right_panel.setMaximumWidth(420)
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(10, 10, 10, 10)

        lbl_prog_title = QLabel("TEACH PENDANT & PROGRAMMER")
        lbl_prog_title.setProperty("class", "PanelTitle")
        right_layout.addWidget(lbl_prog_title)

        # List Waypoints
        self.list_waypoints = QListWidget()
        right_layout.addWidget(self.list_waypoints)

        # Speed scaling slider
        speed_box = QHBoxLayout()
        speed_box.addWidget(QLabel("Speed Scale:"))
        self.slider_speed = QSlider(Qt.Orientation.Horizontal)
        self.slider_speed.setRange(1, 100)
        self.slider_speed.setValue(50)
        self.lbl_speed_val = QLabel("50%")
        self.lbl_speed_val.setStyleSheet("color: #00E5FF; font-weight: 700; min-width: 35px;")
        self.slider_speed.valueChanged.connect(lambda v: self.lbl_speed_val.setText(f"{v}%"))

        speed_box.addWidget(self.slider_speed)
        speed_box.addWidget(self.lbl_speed_val)
        right_layout.addLayout(speed_box)

        # Action Buttons cho Waypoints
        grid_prog_btns = QVBoxLayout()
        row_p1 = QHBoxLayout()
        self.btn_rec_wp = QPushButton("RECORD POSE (REC)")
        self.btn_rec_wp.setProperty("class", "BtnPrimary")
        
        self.btn_run_auto = QPushButton("PLAY AUTO")
        self.btn_run_auto.setProperty("class", "BtnSuccess")

        row_p1.addWidget(self.btn_rec_wp)
        row_p1.addWidget(self.btn_run_auto)

        row_p2 = QHBoxLayout()
        self.btn_stop_auto = QPushButton("STOP AUTO")
        self.btn_stop_auto.setProperty("class", "BtnDanger")
        
        self.btn_clear_wp = QPushButton("CLEAR ALL")
        self.btn_clear_wp.setProperty("class", "BtnWarning")

        row_p2.addWidget(self.btn_stop_auto)
        row_p2.addWidget(self.btn_clear_wp)

        grid_prog_btns.addLayout(row_p1)
        grid_prog_btns.addLayout(row_p2)
        right_layout.addLayout(grid_prog_btns)

        body_splitter.addWidget(right_panel)

        # Set ratio cho 3 panel (Left: 25%, Center: 50%, Right: 25%)
        body_splitter.setSizes([380, 800, 380])
        main_layout.addWidget(body_splitter)

        # =========================================================================
        # 3. BOTTOM PANEL: EVENT LOG CONSOLE
        # =========================================================================
        bottom_panel = QFrame()
        bottom_panel.setProperty("class", "CardPanel")
        bottom_panel.setMaximumHeight(120)
        bottom_layout = QVBoxLayout(bottom_panel)
        bottom_layout.setContentsMargins(10, 6, 10, 6)

        lbl_log_title = QLabel("SYSTEM DIAGNOSTICS & EVENT LOGS")
        lbl_log_title.setStyleSheet("font-size: 11px; font-weight: 700; color: #64748B;")
        bottom_layout.addWidget(lbl_log_title)

        self.txt_logs = QTextEdit()
        self.txt_logs.setReadOnly(True)
        bottom_layout.addWidget(self.txt_logs)

        main_layout.addWidget(bottom_panel)

        MainWindow.setCentralWidget(self.centralwidget)
