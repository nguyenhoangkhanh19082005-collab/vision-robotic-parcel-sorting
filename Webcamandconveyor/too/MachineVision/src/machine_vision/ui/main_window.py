from __future__ import annotations

from datetime import datetime
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QColor, QImage, QPixmap
from PyQt6.QtWidgets import (
    QButtonGroup,
    QCheckBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QSlider,
    QSplitter,
    QStackedWidget,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from machine_vision import __version__
from machine_vision.imaging.focus import focus_peaking, is_acceptable
from machine_vision.models import Decision, FocusMetrics, InspectionResult, PackageDetection
from machine_vision.repositories.sorting_logs import SortingLogReporter
from machine_vision.paths import runtime_root
from machine_vision.services.evidence import EvidenceWriter
from machine_vision.services.pipeline import FrameQualityError, VisionPipeline
from machine_vision.ui.auto_cycle import AutoCycleThread
from machine_vision.ui.asset_manifest import SceneManifest, load_scene_manifest
from machine_vision.ui.camera_worker import CameraWorker, FrameBuffer, TaskThread
from machine_vision.ui.conveyor_panel import ConveyorPanel
from machine_vision.ui.digital_twin import DigitalTwinWidget, ProcessStage
from machine_vision.ui.logo_calibration import LogoCalibrationDialog
from machine_vision.ui.station_theme import (
    DECISION_COLORS,
    DECISION_TEXT,
    DISPLAY,
    PALETTE as P,
    STATE_COLORS,
    STATE_TEXT,
    STYLE,
    ProcessRoute,
)



class VideoCanvas(QLabel):
    def __init__(self) -> None:
        super().__init__("Camera chưa kết nối\nBấm Kết nối camera để bắt đầu")
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(480, 260)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setStyleSheet(f"background:#0d1312; border:1px solid {P['line']}; color:{P['faint']};")

    def show_frame(self, frame: np.ndarray, roi: list[float] | None = None) -> None:
        display = frame.copy()
        if roi:
            h, w = display.shape[:2]
            x, y, rw, rh = roi
            p1 = (int(x * w), int(y * h))
            p2 = (int((x + rw) * w), int((y + rh) * h))
            cv2.rectangle(display, p1, p2, (45, 210, 140), 3)
        rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
        h, w, channels = rgb.shape
        image = QImage(rgb.data, w, h, channels * w, QImage.Format.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(image).scaled(
            self.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.setPixmap(pixmap)


class MetricCard(QFrame):
    """One shift figure: label, big number (Bahnschrift), unit. ``accent`` colours the number."""

    def __init__(
        self, title: str, value: str = "—", unit: str = "", accent: str | None = None
    ) -> None:
        super().__init__()
        self.setObjectName("metric")
        self.setStyleSheet("QFrame#metric { background: transparent; border: 0; }")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.setSpacing(0)
        label = QLabel(title)
        label.setObjectName("metricTitle")
        self.value = QLabel(value)
        self.value.setObjectName("metricValue")
        if accent:
            self.value.setStyleSheet(f"color:{accent};")
        self.unit = QLabel(unit)
        self.unit.setObjectName("metricUnit")
        layout.addWidget(label)
        layout.addWidget(self.value)
        layout.addWidget(self.unit)


class StatusPill(QFrame):
    """Link state in the header: dot, name, state (codes stay English, display Vietnamese)."""

    COLORS = STATE_COLORS

    def __init__(self, name: str, state: str) -> None:
        super().__init__()
        self.setObjectName("statusbar")
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        self.dot = QLabel()
        self.dot.setFixedSize(8, 8)
        self.name = QLabel(name)
        self.name.setStyleSheet(f"color:{P['muted']}; font-size:12px; background:transparent;")
        self.state = QLabel()
        layout.addWidget(self.dot)
        layout.addWidget(self.name)
        layout.addWidget(self.state)
        self.code = state
        self.set_state(state)

    def set_state(self, state: str) -> None:
        self.code = state
        color = self.COLORS.get(state, P["muted"])
        self.dot.setStyleSheet(f"background:{color}; border-radius:4px;")
        self.state.setText(STATE_TEXT.get(state, state.lower() if state.isupper() else state))
        self.state.setStyleSheet(f"color:{color}; font-size:12px; font-weight:600; background:transparent;")


class MainWindow(QMainWindow):
    def __init__(
        self,
        config: dict,
        pipeline: VisionPipeline,
        evidence: EvidenceWriter | None = None,
        project_root: Path | None = None,
        reporter: SortingLogReporter | None = None,
    ):
        super().__init__()
        self.config = config
        self.pipeline = pipeline
        self.evidence = evidence
        # Uploads each result to the data station; None in local test mode.
        self.reporter = reporter
        self.local_test_mode = bool(
            config.get("classification", {}).get("local_test_mode", False)
        )
        # Camera reading and focus metrics run in CameraWorker; ``frames`` is the
        # shared, thread-safe burst buffer used by inspection and calibration.
        self.camera: CameraWorker | None = None
        self.frames = FrameBuffer(int(config["acquisition"].get("burst_frames", 15)))
        self.inspection_thread: TaskThread | None = None
        self.auto_thread: AutoCycleThread | None = None
        self.focus_overlay_enabled = False
        self.live_package: PackageDetection | None = None
        self._live_detection_misses = 0
        self.clock_timer = QTimer(self)
        self.clock_timer.timeout.connect(self._update_clock)
        self.clock_timer.start(1000)
        # Background upload (repositories/sorting_logs.py): show its state, not per parcel.
        self.upload_timer = QTimer(self)
        self.upload_timer.timeout.connect(self._refresh_upload_status)
        self.upload_timer.start(2000)
        self._inspection_started = 0.0
        self.total_count = 0
        self.qr_success_count = 0
        self.pass_count = 0
        self.reject_count = 0
        self.review_count = 0
        self.conveyor_state = "UNKNOWN"
        self.conveyor_speed = 0
        self.project_root = project_root or Path(__file__).resolve().parents[3]
        self.scene_manifest: SceneManifest | None = None
        self.scene_manifest_error: str | None = None
        manifest_relative = config.get("digital_twin", {}).get(
            "manifest_path", "assets/digital_twin/scene_manifest.json"
        )
        try:
            self.scene_manifest = load_scene_manifest(
                self.project_root / manifest_relative, self.project_root
            )
        except (OSError, TypeError, ValueError) as exc:
            self.scene_manifest_error = str(exc)
        self.setWindowTitle(f"{config['station']['name']} · Machine Vision v{__version__} · ESP32 UDP")
        self.resize(1440, 860)
        self.setMinimumSize(1100, 700)
        self.setStyleSheet(STYLE)
        self._build_ui()

    def _build_ui(self) -> None:
        """One operator screen for the whole station (2026-11-25 redesign).

        Header: station name, link states, clock, station-wide actions (``header_actions``).
        Route strip: where the current parcel is (``self.route``).
        "Vận hành": camera | automatic run (``operator_control_layout``, the integrated
        station adds the 3D cell and the robot card here) | last parcel + shift figures,
        log below. Engineering tools live in the other tabs.
        """
        root = QWidget()
        main = QVBoxLayout(root)
        main.setContentsMargins(0, 0, 0, 0)
        main.setSpacing(0)

        topbar = QFrame()
        topbar.setObjectName("topbar")
        header = QHBoxLayout(topbar)
        header.setContentsMargins(16, 10, 16, 10)
        header.setSpacing(12)
        brand = QLabel("MV")
        brand.setAlignment(Qt.AlignmentFlag.AlignCenter)
        brand.setFixedSize(38, 38)
        brand.setStyleSheet(
            f"background:{P['kraft']}; color:{P['ink']}; border-radius:3px;"
            f"font-family:'{DISPLAY}'; font-size:15px; font-weight:700;"
        )
        header.addWidget(brand)
        title = QLabel("Trạm phân loại kiện hàng")
        title.setObjectName("title")
        header.addWidget(title)
        header.addSpacing(24)
        self.camera_status = StatusPill("Camera", "OFFLINE")
        self.vision_status = StatusPill("Vision", "READY")
        self.controller_status = StatusPill("Băng tải", "NOT CONFIGURED")
        self.data_status = StatusPill(
            "Dữ liệu", "LOCAL TEST" if self.local_test_mode else "LOCAL"
        )
        if self.reporter is not None:
            remote_state = "READY"
        else:
            remote_state = "BYPASSED" if self.local_test_mode else "NOT CONFIGURED"
        self.remote_status = StatusPill("Web", remote_state)
        self.status_layout = QHBoxLayout()
        self.status_layout.setSpacing(18)
        for status in (
            self.camera_status,
            self.vision_status,
            self.controller_status,
            self.data_status,
            self.remote_status,
        ):
            self.status_layout.addWidget(status)
        header.addLayout(self.status_layout)
        header.addStretch()
        self.mode_label = QLabel()
        self.mode_label.hide()
        header.addWidget(self.mode_label)
        self.clock_label = QLabel()
        self.clock_label.setStyleSheet(
            f"color:{P['muted']}; font-family:'{DISPLAY}'; font-size:15px; background:transparent;"
        )
        header.addWidget(self.clock_label)
        self.header_actions = QHBoxLayout()
        self.header_actions.setSpacing(8)
        header.addLayout(self.header_actions)
        main.addWidget(topbar)

        self.route = ProcessRoute()
        main.addWidget(self.route)

        self.throughput_metric = MetricCard("Đã kiểm tra", "0", "kiện trong phiên")
        self.success_metric = MetricCard("Đọc được QR", "—", "% số kiện", P["info"])
        self.pass_metric = MetricCard("Đạt", "0", "kiện", P["pass"])
        self.reject_metric = MetricCard("Loại", "0", "kiện", P["reject"])
        self.cycle_metric = MetricCard("Thời gian kiểm tra", "—", "ms mỗi kiện", P["kraft"])

        operator_page = QWidget()
        operator_layout = QVBoxLayout(operator_page)
        operator_layout.setContentsMargins(14, 12, 14, 12)
        operator_layout.setSpacing(10)
        body = QHBoxLayout()
        body.setSpacing(10)

        # -- camera ------------------------------------------------------------------
        camera_panel = QFrame()
        camera_panel.setObjectName("panel")
        camera_layout = QVBoxLayout(camera_panel)
        camera_layout.setContentsMargins(12, 10, 12, 12)
        camera_layout.setSpacing(8)
        camera_header = QHBoxLayout()
        self.camera_title = QLabel("Camera trực tiếp")
        self.camera_title.setObjectName("sectionTitle")
        self.frame_state = QLabel("Chờ")
        self.frame_state.setStyleSheet(f"color:{P['muted']}; font-weight:700;")
        camera_header.addWidget(self.camera_title)
        camera_header.addStretch()
        self.camera_view_group = QButtonGroup(self)
        self.camera_view_group.setExclusive(True)
        self.live_view_button = QPushButton("Trực tiếp")
        self.live_view_button.setObjectName("viewPreset")
        self.live_view_button.setCheckable(True)
        self.live_view_button.setChecked(True)
        self.live_view_button.clicked.connect(self._show_live_view)
        self.snapshot_view_button = QPushButton("Ảnh kiểm tra")
        self.snapshot_view_button.setObjectName("viewPreset")
        self.snapshot_view_button.setCheckable(True)
        self.snapshot_view_button.setEnabled(False)
        self.snapshot_view_button.clicked.connect(self._show_snapshot_view)
        self.camera_view_group.addButton(self.live_view_button)
        self.camera_view_group.addButton(self.snapshot_view_button)
        camera_header.addWidget(self.live_view_button)
        camera_header.addWidget(self.snapshot_view_button)
        camera_header.addSpacing(8)
        camera_header.addWidget(self.frame_state)
        camera_layout.addLayout(camera_header)
        self.video_stack = QStackedWidget()
        self.video = VideoCanvas()
        self.snapshot_video = VideoCanvas()
        self.snapshot_video.setText("Chưa có ảnh kiểm tra")
        self.video_stack.addWidget(self.video)
        self.video_stack.addWidget(self.snapshot_video)
        camera_layout.addWidget(self.video_stack, 1)

        self.connect_button = QPushButton("Kết nối camera")
        self.connect_button.setObjectName("secondary")
        self.connect_button.clicked.connect(self.toggle_camera)
        self.inspect_button = QPushButton("Kiểm tra tay")
        self.inspect_button.setObjectName("secondary")
        self.inspect_button.setToolTip("Chụp và kiểm tra kiện đang nằm dưới camera, không chạy băng tải")
        self.inspect_button.setEnabled(False)
        self.inspect_button.clicked.connect(self.inspect)
        self.background_button = QPushButton("Chụp nền trống")
        self.background_button.setObjectName("secondary")
        self.background_button.setEnabled(False)
        self.background_button.setToolTip(
            "Bỏ kiện khỏi khung hình rồi chụp băng tải trống. Camera dùng ảnh này để biết lúc kiện "
            "nằm trọn khung hình; chụp lại mỗi khi camera, đèn hoặc băng tải bị xê dịch."
        )
        self.background_button.clicked.connect(self._capture_bench_background)
        camera_tools = QHBoxLayout()
        camera_tools.setSpacing(8)
        camera_tools.addWidget(self.connect_button)
        camera_tools.addWidget(self.inspect_button)
        camera_tools.addWidget(self.background_button)
        camera_tools.addStretch()
        camera_layout.addLayout(camera_tools)
        body.addWidget(camera_panel, 11)

        # Engineering camera tools (tab "Băng tải & camera").
        self.bench_check = QCheckBox("Chế độ bàn thử 15–30 cm")
        self.bench_check.setToolTip(
            "Chế độ thử tạm khi camera đặt trên bàn/sàn, kiện đứng yên ở 15–30 cm. "
            "Tắt chế độ này khi lắp camera lên băng tải."
        )
        self.bench_check.setChecked(self.pipeline.bench_mode)
        self.bench_check.toggled.connect(self._set_bench_mode)
        self.logo_calibration_button = QPushButton("Lấy mẫu logo")
        self.logo_calibration_button.setObjectName("secondary")
        self.logo_calibration_button.setEnabled(False)
        self.logo_calibration_button.setToolTip(
            "Chụp nhãn thực tế, kéo chọn logo và lưu template nhận diện mới."
        )
        self.logo_calibration_button.clicked.connect(self._open_logo_calibration)
        self.focus_check = QCheckBox("Tô viền vùng nét (focus peaking)")
        self.focus_check.toggled.connect(self._set_focus_overlay)
        self.focus_metric = MetricCard("Độ nét Laplacian", unit="độ nét")
        self.tenengrad_metric = MetricCard("Độ nét Tenengrad", unit="gradient")
        self.motion_metric = MetricCard("Chuyển động", unit="pixel delta", accent=P["review"])
        self.quality_metric = MetricCard("Điều kiện chụp", unit="sẵn sàng / giữ")

        # -- automatic run -------------------------------------------------------------------
        control_panel = QFrame()
        control_panel.setObjectName("panel")
        self.operator_control_layout = QVBoxLayout(control_panel)
        self.operator_control_layout.setContentsMargins(12, 10, 12, 12)
        self.operator_control_layout.setSpacing(8)
        control_title = QLabel("Chạy tự động")
        control_title.setObjectName("sectionTitle")
        self.operator_control_layout.addWidget(control_title)
        self.auto_button = QPushButton()
        self.auto_button.setToolTip(
            "Camera dừng kiện để kiểm tra QR, tem, logo -> kiện đạt chạy tới điểm gắp, "
            "kiện lỗi rơi xuống sọt lỗi -> kết quả lưu lên web. Cần camera đã kết nối."
        )
        self.auto_button.clicked.connect(self._toggle_auto_cycle)
        self._set_auto_button(False)
        self.operator_control_layout.addWidget(self.auto_button)
        # 2026-11-25: without a robot the PASS parcel stops past the pick sensor (delayed
        # AUTO stop), so nothing sees it being lifted: the belt waits for this button.
        self.picked_button = QPushButton("Đã lấy kiện  (phím Space)")
        self.picked_button.setObjectName("confirm")
        self.picked_button.setShortcut("Space")
        self.picked_button.setEnabled(False)
        self.picked_button.setToolTip("Nhấc kiện ở điểm gắp ra rồi bấm để băng tải chạy tiếp")
        self.picked_button.clicked.connect(self._confirm_manual_pick)
        self.operator_control_layout.addWidget(self.picked_button)
        self.auto_phase = "IDLE"
        self.auto_status = QLabel("Chu trình tự động chưa chạy")
        self.auto_status.setWordWrap(True)
        self.auto_status.setStyleSheet(f"color:{P['muted']}; font-size:12px;")
        self.operator_control_layout.addWidget(self.auto_status)
        speed = (self.config.get("station_cycle") or {}).get("speed")
        if speed:
            belt_hint = QLabel(f"Băng tải chạy cố định PWM {speed} khi tự động")
            belt_hint.setObjectName("hint")
            self.operator_control_layout.addWidget(belt_hint)
        # Last item: the integrated station replaces it with the 3D cell + robot card.
        self.operator_control_layout.addStretch(1)
        body.addWidget(control_panel, 5)

        # -- last parcel + shift ----------------------------------------------------------
        result_column = QVBoxLayout()
        result_column.setSpacing(10)
        result_panel = QFrame()
        result_panel.setObjectName("panel")
        result_layout = QVBoxLayout(result_panel)
        result_layout.setContentsMargins(12, 10, 12, 12)
        result_layout.setSpacing(8)
        result_title = QLabel("Kiện vừa kiểm tra")
        result_title.setObjectName("sectionTitle")
        result_layout.addWidget(result_title)
        self.decision_label = QLabel("Chưa có kiện")
        self.decision_label.setStyleSheet(
            f"font-family:'{DISPLAY}'; font-size:26px; font-weight:600; color:{P['faint']};"
            "padding:4px 0;"
        )
        self.waybill_label = QLabel("—")
        self.qr_payload_label = QLabel("—")
        self.logo_label = QLabel("—")
        self.damage_label = QLabel("—")
        self.destination_label = QLabel("—")
        self.background_status_label = QLabel("—")
        self.reason_label = QLabel("Kiện đầu tiên sẽ hiện ở đây sau khi camera kiểm tra.")
        self.reason_label.setWordWrap(True)
        self.reason_label.setObjectName("reasonBox")
        result_layout.addWidget(self.decision_label)
        result_grid = QGridLayout()
        result_grid.setHorizontalSpacing(12)
        result_grid.setVerticalSpacing(6)
        result_fields = (
            ("Mã vận đơn", self.waybill_label),
            ("Nội dung QR", self.qr_payload_label),
            ("Logo", self.logo_label),
            ("Tình trạng kiện", self.damage_label),
            ("Kết quả" if self.local_test_mode else "Sọt đích", self.destination_label),
            ("Ảnh nền camera", self.background_status_label),
        )
        for row, (name, value) in enumerate(result_fields):
            field_name = QLabel(name)
            field_name.setObjectName("fieldName")
            value.setObjectName("fieldValue")
            value.setWordWrap(True)
            result_grid.addWidget(field_name, row, 0)
            result_grid.addWidget(value, row, 1)
        result_grid.setColumnStretch(1, 1)
        result_layout.addLayout(result_grid)
        result_layout.addWidget(self.reason_label)
        result_column.addWidget(result_panel)

        shift_panel = QFrame()
        shift_panel.setObjectName("panel")
        shift_layout = QVBoxLayout(shift_panel)
        shift_layout.setContentsMargins(12, 10, 12, 10)
        shift_title = QLabel("Ca làm việc")
        shift_title.setObjectName("sectionTitle")
        shift_layout.addWidget(shift_title)
        shift_grid = QGridLayout()
        shift_grid.setHorizontalSpacing(16)
        shift_grid.setVerticalSpacing(6)
        for index, metric in enumerate((
            self.throughput_metric, self.pass_metric, self.reject_metric,
            self.success_metric, self.cycle_metric,
        )):
            shift_grid.addWidget(metric, index // 3, index % 3)
        shift_layout.addLayout(shift_grid)
        result_column.addWidget(shift_panel)
        body.addLayout(result_column, 6)
        operator_layout.addLayout(body, 1)
        # The parcel log continues "Kiện vừa kiểm tra": same column, below it (added
        # after it is built, see log_tabs).

        log_tabs = QTabWidget()
        self.history = QTableWidget(0, 6)
        self.history.setHorizontalHeaderLabels(
            [
                "Thời gian",
                "Mã vận đơn",
                "QR",
                "Quyết định",
                "Kết quả" if self.local_test_mode else "Sọt",
                "Lý do",
            ]
        )
        self.history.horizontalHeader().setStretchLastSection(True)
        self.history.verticalHeader().setVisible(False)
        self.history.verticalHeader().setDefaultSectionSize(26)
        self.history.setShowGrid(False)
        self.history.setAlternatingRowColors(True)
        self.alarms = QTableWidget(1, 3)
        self.alarms.setHorizontalHeaderLabels(["Mức độ", "Nguồn", "Thông báo"])
        self.alarms.horizontalHeader().setStretchLastSection(True)
        self.alarms.verticalHeader().setVisible(False)
        self.alarms.verticalHeader().setDefaultSectionSize(26)
        self.alarms.setShowGrid(False)
        self.alarms.setItem(0, 0, QTableWidgetItem("Cảnh báo"))
        self.alarms.setItem(0, 1, QTableWidgetItem("Giám sát từ xa"))
        self.alarms.setItem(0, 2, QTableWidgetItem("Kênh giám sát từ xa chưa được cấu hình"))
        log_tabs.addTab(self.history, "Các kiện trước")
        log_tabs.addTab(self.alarms, "Cảnh báo")
        # Narrow column: time, waybill, decision, bin; QR and reason show as a tooltip.
        self.history.setColumnHidden(2, True)
        self.history.setColumnHidden(5, True)
        header_view = self.history.horizontalHeader()
        header_view.setStretchLastSection(False)
        header_view.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        header_view.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        self.history.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        result_column.addWidget(log_tabs, 1)

        # -- 3D model -------------------------------------------------------------------------
        twin_panel = QFrame()
        twin_panel.setObjectName("panel")
        twin_layout = QVBoxLayout(twin_panel)
        twin_header = QHBoxLayout()
        twin_title = QLabel("Mô hình 3D toàn trạm")
        twin_title.setObjectName("sectionTitle")
        self.twin_sync = QLabel("Mô hình cục bộ")
        self.twin_sync.setStyleSheet(f"color:{P['review']}; font-weight:600; font-size:11px;")
        twin_header.addWidget(twin_title)
        twin_header.addStretch()
        twin_header.addWidget(self.twin_sync)
        twin_layout.addLayout(twin_header)
        self.digital_twin = DigitalTwinWidget()
        twin_layout.addWidget(self.digital_twin)

        twin_page = QWidget()
        twin_page_layout = QVBoxLayout(twin_page)
        twin_page_layout.setContentsMargins(14, 12, 14, 12)
        twin_page_layout.setSpacing(8)
        twin_command_bar = QFrame()
        twin_command_bar.setObjectName("subpanel")
        command_layout = QHBoxLayout(twin_command_bar)
        command_layout.setContentsMargins(10, 7, 10, 7)
        cell_label = QLabel("Ô trạm MV-01")
        cell_label.setObjectName("sectionTitle")
        command_layout.addWidget(cell_label)
        command_layout.addSpacing(14)
        command_layout.addWidget(StatusPill("Mô hình", "READY"))
        command_layout.addWidget(StatusPill("Đồng bộ", "LOCAL"))
        command_layout.addStretch()
        frame_label = QLabel("Hệ tọa độ thế giới, trục Y hướng lên, đơn vị mét")
        frame_label.setObjectName("hint")
        command_layout.addWidget(frame_label)
        twin_page_layout.addWidget(twin_command_bar)
        engineering_split = QSplitter(Qt.Orientation.Horizontal)
        engineering_split.setChildrenCollapsible(False)
        engineering_split.addWidget(self._build_scene_browser())
        engineering_split.addWidget(twin_panel)
        engineering_split.addWidget(self._build_twin_controls())
        engineering_split.setStretchFactor(0, 0)
        engineering_split.setStretchFactor(1, 1)
        engineering_split.setStretchFactor(2, 0)
        engineering_split.setSizes([215, 800, 290])
        twin_page_layout.addWidget(engineering_split, 1)

        # -- conveyor & camera tools ---------------------------------------------------------
        conveyor_page = QWidget()
        conveyor_page_layout = QHBoxLayout(conveyor_page)
        conveyor_page_layout.setContentsMargins(14, 12, 14, 12)
        conveyor_page_layout.setSpacing(10)
        belt_panel = QFrame()
        belt_panel.setObjectName("panel")
        belt_layout = QVBoxLayout(belt_panel)
        belt_layout.setContentsMargins(12, 10, 12, 12)
        belt_title = QLabel("Băng tải chạy tay")
        belt_title.setObjectName("sectionTitle")
        belt_layout.addWidget(belt_title)
        belt_intro = QLabel(
            "Lệnh trực tiếp tới ESP32 qua UDP để thử băng tải. Khi chạy tự động, "
            "dùng nút ở trang Vận hành."
        )
        belt_intro.setObjectName("hint")
        belt_intro.setWordWrap(True)
        belt_layout.addWidget(belt_intro)
        self.conveyor_panel = ConveyorPanel(self.config.get("controller", {}))
        self.conveyor_panel.state_changed.connect(self._on_conveyor_state)
        self.conveyor_panel.connection_changed.connect(self._on_controller_connection)
        # The UDP STOP button also ends the automatic cycle.
        self.conveyor_panel.stop_button.clicked.connect(self._stop_auto_cycle)
        belt_layout.addWidget(self.conveyor_panel)
        safety = QLabel(
            "An toàn: nút Dừng trên phần mềm là gói UDP, không phải dừng khẩn. Luôn có công tắc "
            "ngắt nguồn motor hoặc nút dừng khẩn phần cứng và người giám sát tại chỗ khi chạy thử. "
            "Không nhận được ACK nghĩa là trạng thái thật của băng tải chưa xác định."
        )
        safety.setWordWrap(True)
        safety.setStyleSheet(f"color:{P['review']};")
        belt_layout.addWidget(safety)
        belt_layout.addStretch()
        conveyor_page_layout.addWidget(belt_panel, 1)

        camera_tools_panel = QFrame()
        camera_tools_panel.setObjectName("panel")
        camera_tools_layout = QVBoxLayout(camera_tools_panel)
        camera_tools_layout.setContentsMargins(12, 10, 12, 12)
        camera_tools_layout.setSpacing(10)
        tools_title = QLabel("Chất lượng ảnh và hiệu chỉnh camera")
        tools_title.setObjectName("sectionTitle")
        camera_tools_layout.addWidget(tools_title)
        quality_grid = QGridLayout()
        quality_grid.setHorizontalSpacing(18)
        quality_grid.addWidget(self.focus_metric, 0, 0)
        quality_grid.addWidget(self.tenengrad_metric, 0, 1)
        quality_grid.addWidget(self.motion_metric, 1, 0)
        quality_grid.addWidget(self.quality_metric, 1, 1)
        camera_tools_layout.addLayout(quality_grid)
        camera_tools_layout.addWidget(self.logo_calibration_button)
        camera_tools_layout.addWidget(self.bench_check)
        camera_tools_layout.addWidget(self.focus_check)
        camera_tools_layout.addStretch()
        conveyor_page_layout.addWidget(camera_tools_panel, 1)

        self.workspace_tabs = QTabWidget()
        self.workspace_tabs.setDocumentMode(True)
        self.workspace_tabs.addTab(operator_page, "Vận hành")
        self.workspace_tabs.addTab(twin_page, "Mô hình 3D")
        self.workspace_tabs.addTab(conveyor_page, "Băng tải && camera")
        main.addWidget(self.workspace_tabs, 1)
        self.setCentralWidget(root)
        self._set_bench_mode(self.pipeline.bench_mode)

    def _set_auto_button(self, running: bool) -> None:
        self.auto_button.setText("Dừng chạy tự động" if running else "Bắt đầu chạy tự động")
        self.auto_button.setObjectName("running" if running else "primary")
        self.auto_button.style().unpolish(self.auto_button)
        self.auto_button.style().polish(self.auto_button)

    def _build_scene_browser(self) -> QFrame:
        panel = QFrame()
        panel.setObjectName("panel")
        panel.setMinimumWidth(190)
        panel.setMaximumWidth(245)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 12, 10, 10)
        layout.setSpacing(8)

        title = QLabel("SCENE EXPLORER")
        title.setObjectName("sectionTitle")
        layout.addWidget(title)
        subtitle = QLabel("CẤU TRÚC THIẾT BỊ")
        subtitle.setStyleSheet("color:#657b8e; font-size:9px; font-weight:600;")
        layout.addWidget(subtitle)

        self.scene_tree = QTreeWidget()
        self.scene_tree.setHeaderHidden(True)
        self.scene_tree.setIndentation(15)
        self.scene_tree.setAnimated(True)
        self.scene_tree.setAlternatingRowColors(True)
        cell = QTreeWidgetItem(["▣  CELL MV-01"])
        conveyor = QTreeWidgetItem(["▰  Main conveyor"])
        vision = QTreeWidgetItem(["⌁  Vision gantry"])
        vision.addChildren(
            [QTreeWidgetItem(["◉  Camera 2K"]), QTreeWidgetItem(["◇  Sensor S1"])]
        )
        robot = QTreeWidgetItem(["⌬  Robot 6-DOF"])
        bins = QTreeWidgetItem(["▱  Sorting bins"])
        bins.addChildren(
            [QTreeWidgetItem(["Pass"]), QTreeWidgetItem(["Defect"]), QTreeWidgetItem(["Review"])]
        )
        cell.addChildren([conveyor, vision, robot, bins])
        self.scene_tree.addTopLevelItem(cell)
        cell.setExpanded(True)
        vision.setExpanded(True)
        bins.setExpanded(True)
        self.scene_tree.currentItemChanged.connect(self._scene_item_selected)
        self.scene_tree.setCurrentItem(cell)
        layout.addWidget(self.scene_tree, 1)

        layer_title = QLabel("LAYERS")
        layer_title.setObjectName("sectionTitle")
        layout.addWidget(layer_title)
        self.layer_checks: dict[str, QCheckBox] = {}
        for layer, text in (
            ("equipment", "Thiết bị"),
            ("vision_zones", "Vision zones"),
            ("telemetry_labels", "Telemetry labels"),
        ):
            option = QCheckBox(text)
            option.setChecked(True)
            option.setMinimumHeight(25)
            option.toggled.connect(
                lambda visible, name=layer: self.digital_twin.set_layer_visible(name, visible)
            )
            layout.addWidget(option)
            self.layer_checks[layer] = option
        return panel

    def _scene_item_selected(self, current: QTreeWidgetItem | None, _previous) -> None:
        if current is None:
            return
        text = current.text(0).lower()
        if "robot" in text:
            self._set_twin_view("robot")
            if hasattr(self, "inspector_tabs"):
                self.inspector_tabs.setCurrentIndex(1)
        elif "camera" in text or "vision" in text:
            self._set_twin_view("front")
            if hasattr(self, "inspector_tabs"):
                self.inspector_tabs.setCurrentIndex(0)
        elif "conveyor" in text:
            self._set_twin_view("top")
            if hasattr(self, "inspector_tabs"):
                self.inspector_tabs.setCurrentIndex(0)
        elif "cell" in text or "bin" in text:
            self._set_twin_view("isometric")

    def _set_twin_view(self, preset: str) -> None:
        self.digital_twin.set_view(preset)
        if hasattr(self, "view_buttons") and preset in self.view_buttons:
            self.view_buttons[preset].setChecked(True)

    def _build_twin_controls(self) -> QFrame:
        panel = QFrame()
        panel.setObjectName("panel")
        panel.setMinimumWidth(290)
        panel.setMaximumWidth(360)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        title = QLabel("INSPECTOR / TELEMETRY")
        title.setObjectName("sectionTitle")
        layout.addWidget(title)

        self.inspector_tabs = QTabWidget()
        self.inspector_tabs.setObjectName("inspectorTabs")
        self.inspector_tabs.setDocumentMode(True)
        self.inspector_tabs.tabBar().setExpanding(True)
        self.inspector_tabs.tabBar().setUsesScrollButtons(False)

        overview_page = QWidget()
        overview_layout = QVBoxLayout(overview_page)
        overview_layout.setContentsMargins(8, 10, 8, 8)
        overview_layout.setSpacing(8)

        view_title = QLabel("CAMERA VIEW PRESETS")
        view_title.setObjectName("sectionTitle")
        overview_layout.addWidget(view_title)

        view_grid = QGridLayout()
        view_grid.setHorizontalSpacing(7)
        view_grid.setVerticalSpacing(7)
        self.view_button_group = QButtonGroup(self)
        self.view_button_group.setExclusive(True)
        self.view_buttons: dict[str, QPushButton] = {}
        for index, (text, preset) in enumerate(
            (("ISOMETRIC", "isometric"), ("TOP", "top"), ("FRONT", "front"), ("ROBOT", "robot"))
        ):
            button = QPushButton(text)
            button.setObjectName("viewPreset")
            button.setCheckable(True)
            button.setMinimumHeight(34)
            button.setChecked(preset == "isometric")
            button.clicked.connect(
                lambda _checked=False, name=preset: self._set_twin_view(name)
            )
            self.view_button_group.addButton(button)
            self.view_buttons[preset] = button
            view_grid.addWidget(button, index // 2, index % 2)
        overview_layout.addLayout(view_grid)

        hint = QLabel(
            "Chuột trái: xoay camera\n"
            "Chuột phải: dịch chuyển\n"
            "Con lăn: phóng to / thu nhỏ  •  Double-click: reset"
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color:#7f91a3; font-size:10px;")
        overview_layout.addWidget(hint)

        state_title = QLabel("TRẠNG THÁI HỆ THỐNG")
        state_title.setObjectName("sectionTitle")
        overview_layout.addWidget(state_title)
        self.twin_stage_value = QLabel("Stage: IDLE")
        self.twin_conveyor_value = QLabel("Conveyor: UNKNOWN")
        self.twin_robot_value = QLabel("Robot: READY")
        for label in (
            self.twin_stage_value,
            self.twin_conveyor_value,
            self.twin_robot_value,
        ):
            label.setStyleSheet("background:#0d141d; padding:6px; color:#aebdca;")
            label.setMinimumHeight(30)
            overview_layout.addWidget(label)
        overview_layout.addStretch()

        robot_page = QWidget()
        robot_layout = QVBoxLayout(robot_page)
        robot_layout.setContentsMargins(8, 10, 8, 8)
        robot_layout.setSpacing(8)

        joint_title = QLabel("ROBOT 6 BẬC TỰ DO")
        joint_title.setObjectName("sectionTitle")
        robot_layout.addWidget(joint_title)
        robot_hint = QLabel("Điều khiển mô phỏng góc khớp • đơn vị độ")
        robot_hint.setStyleSheet("color:#7f91a3; font-size:10px;")
        robot_layout.addWidget(robot_hint)
        limits = ((-180, 180), (-90, 90), (-130, 130), (-180, 180), (-120, 120), (-360, 360))
        self.joint_value_labels: list[QLabel] = []
        for joint, (minimum, maximum) in enumerate(limits):
            row = QHBoxLayout()
            row.setSpacing(7)
            name = QLabel(f"J{joint + 1}")
            name.setFixedWidth(22)
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setMinimumHeight(25)
            slider.setRange(minimum, maximum)
            slider.setValue(self.digital_twin.robot_angles[joint])
            value = QLabel(f"{slider.value()}°")
            value.setFixedWidth(42)
            value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            slider.valueChanged.connect(
                lambda angle, index=joint, output=value: self._set_robot_joint(index, angle, output)
            )
            row.addWidget(name)
            row.addWidget(slider, 1)
            row.addWidget(value)
            robot_layout.addLayout(row)
            self.joint_value_labels.append(value)
        robot_layout.addStretch()

        assets_page = QWidget()
        asset_layout = QVBoxLayout(assets_page)
        asset_layout.setContentsMargins(8, 10, 8, 8)
        asset_layout.setSpacing(8)
        asset_title = QLabel("3D ASSET PIPELINE")
        asset_title.setObjectName("sectionTitle")
        asset_layout.addWidget(asset_title)
        asset_help = QLabel(
            "Mô hình phần mềm tiếp tục hoạt động cho tới khi file GLB được commissioning."
        )
        asset_help.setWordWrap(True)
        asset_help.setStyleSheet("color:#7f91a3; font-size:10px;")
        asset_layout.addWidget(asset_help)
        if self.scene_manifest_error:
            error = QLabel("MANIFEST ERROR")
            error.setObjectName("assetWaiting")
            error.setToolTip(self.scene_manifest_error)
            asset_layout.addWidget(error)
        elif self.scene_manifest is not None:
            for asset in self.scene_manifest.assets:
                ready = asset.is_ready(self.project_root)
                row_panel = QFrame()
                row_panel.setObjectName("subpanel")
                row_layout = QHBoxLayout(row_panel)
                row_layout.setContentsMargins(8, 7, 8, 7)
                name = QLabel(asset.asset_id.upper())
                name.setToolTip(asset.label)
                name.setStyleSheet("color:#aebdca; font-size:10px; font-weight:600;")
                state = QLabel("READY" if ready else "AWAITING GLB")
                state.setObjectName("assetReady" if ready else "assetWaiting")
                row_layout.addWidget(name, 1)
                row_layout.addWidget(state)
                asset_layout.addWidget(row_panel)
        asset_layout.addStretch()

        self.inspector_tabs.addTab(overview_page, "OVERVIEW")
        self.inspector_tabs.addTab(robot_page, "ROBOT")
        self.inspector_tabs.addTab(assets_page, "ASSETS")
        layout.addWidget(self.inspector_tabs, 1)
        return panel

    def _set_robot_joint(self, joint: int, angle: int, output: QLabel) -> None:
        self.digital_twin.set_joint_angle(joint, angle)
        output.setText(f"{angle}°")

    def _set_twin_process(
        self,
        stage: ProcessStage,
        decision: str | None = None,
    ) -> None:
        self.digital_twin.set_process_state(
            stage, decision, self.conveyor_state in {"FWD", "REV"}
        )
        self.digital_twin.set_conveyor_command_state(self.conveyor_state)
        self.twin_stage_value.setText(f"Stage: {stage.value}")
        self.twin_conveyor_value.setText(f"Conveyor cmd: {self.conveyor_state} · PWM {self.conveyor_speed}")

    def _on_conveyor_state(self, state: str, speed: int) -> None:
        self.conveyor_state = state
        self.conveyor_speed = speed
        self._set_twin_process(self.digital_twin.stage, self.digital_twin.decision)

    def _on_controller_connection(self, online: bool) -> None:
        self.controller_status.set_state("ONLINE" if online else "FAULT")

    def toggle_camera(self) -> None:
        if self.camera is not None:
            self._stop_camera()
            return
        # Opening + warm-up (~0.5 s) happens in the worker, so the window stays responsive.
        self.frames.clear()
        self.live_package = None
        self._live_detection_misses = 0
        self.camera = CameraWorker(
            self.config["camera"],
            self.config["label"]["roi"],
            self.frames,
            detect_package=self.pipeline.detect_package_fast,
            detection_interval=int(
                self.config.get("package_detection", {}).get("live_interval_frames", 10)
            ),
        )
        self.camera.opened.connect(self._on_camera_opened)
        self.camera.failed.connect(self._on_camera_failed)
        self.camera.frame_ready.connect(self._next_frame)
        self.connect_button.setEnabled(False)
        self.connect_button.setText("ĐANG KẾT NỐI…")
        self.camera.start()

    def _stop_camera(self) -> None:
        # Without a camera the cycle could only log failures: stop the belt instead.
        self._stop_auto_cycle()
        worker, self.camera = self.camera, None
        if worker is not None:
            worker.stop()
            worker.wait(3000)
        self.camera_status.set_state("OFFLINE")
        self.frame_state.setText("Chờ")
        self.frame_state.setStyleSheet("color:#93a39e; font-weight:700;")
        self._set_twin_process(ProcessStage.IDLE)
        self.connect_button.setEnabled(True)
        self.connect_button.setText("KẾT NỐI CAMERA")
        self.inspect_button.setEnabled(False)
        self.background_button.setEnabled(False)
        self.logo_calibration_button.setEnabled(False)
        self.live_package = None

    def _on_camera_failed(self, message: str) -> None:
        if self.sender() is not self.camera:
            return  # a worker that was already stopped
        self._stop_camera()
        QMessageBox.critical(self, "Camera error", message)

    def _on_camera_opened(self, warnings: list) -> None:
        if self.sender() is not self.camera:
            return
        self.connect_button.setEnabled(True)
        self.camera_status.set_state("ONLINE")
        self.frame_state.setText("Đang xem")
        self.frame_state.setStyleSheet("color:#4fb985; font-weight:700;")
        self._set_twin_process(ProcessStage.DETECT)
        self.connect_button.setText("NGẮT CAMERA")
        self.inspect_button.setEnabled(True)
        self.background_button.setEnabled(True)
        self.logo_calibration_button.setEnabled(True)
        if warnings:
            QMessageBox.warning(self, "Cấu hình camera", "\n\n".join(warnings))

    def _set_focus_overlay(self, enabled: bool) -> None:
        self.focus_overlay_enabled = enabled

    def _show_live_view(self) -> None:
        self.video_stack.setCurrentWidget(self.video)
        self.live_view_button.setChecked(True)
        self.camera_title.setText("Camera trực tiếp")

    def _show_snapshot_view(self) -> None:
        if not self.snapshot_view_button.isEnabled():
            self._show_live_view()
            return
        self.video_stack.setCurrentWidget(self.snapshot_video)
        self.snapshot_view_button.setChecked(True)
        self.camera_title.setText("Ảnh kiện lúc kiểm tra")

    def _update_damage_reference_status(self) -> None:
        if not hasattr(self, "background_status_label"):
            return
        ready = self.pipeline.damage_reference_ready()
        if ready:
            self.background_status_label.setText("Sẵn sàng")
            self.background_status_label.setStyleSheet(
                "color:#4fb985; font-weight:700;"
            )
        else:
            self.background_status_label.setText("Chưa chụp nền")
            self.background_status_label.setStyleSheet(
                "color:#e2b347; font-weight:700;"
            )

    def _capture_bench_background(self) -> None:
        if not self.frames:
            QMessageBox.warning(
                self, "Chụp nền trống", "Camera chưa có hình ảnh để hiệu chuẩn."
            )
            return
        answer = QMessageBox.question(
            self,
            "Xác nhận bối cảnh trống",
            "Hãy lấy kiện hàng ra khỏi khung hình, giữ nguyên camera và ánh sáng.\n\n"
            "Khung hình hiện tại không còn kiện hàng?",
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        try:
            path = self.pipeline.capture_bench_background(self.frames.snapshot())
        except Exception as exc:
            QMessageBox.critical(self, "Lỗi chụp nền", str(exc))
            return
        self.frames.clear()
        if self.camera is not None:
            self.camera.reset_motion()
        self.live_package = None
        self._update_damage_reference_status()
        QMessageBox.information(
            self,
            "Đã chụp nền trống",
            "Ảnh nền đã được lưu. Bây giờ hãy đặt kiện vào đúng vị trí, "
            f"chờ hình ổn định rồi nhấn KIỂM TRA KIỆN.\n\n{path}",
        )

    def _open_logo_calibration(self) -> None:
        if self.camera is None or not self.frames:
            QMessageBox.warning(
                self,
                "Hiệu chuẩn logo",
                "Hãy kết nối camera và chờ hình ảnh ổn định trước khi lấy mẫu logo.",
            )
            return
        dialog = LogoCalibrationDialog(
            self.pipeline,
            frame_provider=lambda: list(self.frames),
            parent=self,
        )
        if dialog.exec():
            self.logo_label.setText("Mẫu logo mới đã sẵn sàng")
            self.logo_label.setStyleSheet("color:#4fb985; font-weight:700;")

    def _set_bench_mode(self, enabled: bool) -> None:
        self.pipeline.set_bench_mode(enabled)
        self.frames.clear()
        if self.camera is not None:
            self.camera.reset_motion()
        self.live_package = None
        acquisition = self.pipeline.acquisition_settings()
        self.focus_metric.unit.setText(
            f"độ nét • ngưỡng ≥ {float(acquisition.get('min_laplacian', 0)):.1f}"
        )
        self.tenengrad_metric.unit.setText(
            f"gradient • ngưỡng ≥ {float(acquisition.get('min_tenengrad', 0)):.0f}"
        )
        if enabled:
            self.mode_label.setText("Chế độ bàn thử 15–30 cm")
            self.mode_label.setStyleSheet(
                f"color:{P['review']}; border:1px solid {P['review']}; border-radius:3px;"
                "padding:4px 9px; font-weight:600; background:transparent;"
            )
            self.mode_label.show()
            self.bench_check.setStyleSheet(f"color:{P['review']}; font-weight:600;")
        else:
            self.mode_label.hide()
            self.bench_check.setStyleSheet("")
        if hasattr(self, "background_button"):
            self.background_button.setEnabled(enabled and self.camera is not None)
        self._update_damage_reference_status()

    def _next_frame(
        self, frame: np.ndarray, metrics: FocusMetrics, detection: object
    ) -> None:
        """Paint one frame delivered by CameraWorker (metrics already computed)."""
        worker = self.sender()
        try:
            if worker is not self.camera:
                return
            good = is_acceptable(metrics, self.pipeline.acquisition_settings())
            self.focus_metric.value.setText(f"{metrics.laplacian:.0f}")
            self.tenengrad_metric.value.setText(f"{metrics.tenengrad:.0f}")
            self.motion_metric.value.setText(f"{metrics.motion:.1f}")
            self.quality_metric.value.setText("READY" if good else "HOLD")
            self.quality_metric.value.setStyleSheet(
                "color:#4fb985;" if good else "color:#e2b347;"
            )
            # None: no search this frame; False: searched, nothing found.
            if isinstance(detection, PackageDetection):
                self.live_package = detection
                self._live_detection_misses = 0
            elif detection is False:
                self._live_detection_misses += 1
                if self._live_detection_misses >= 3:
                    self.live_package = None

            # Focus feedback is useful on the printed label, not on carton creases/floor edges.
            focus_roi = (
                self.live_package.label_bbox if self.live_package is not None else None
            )
            if focus_roi is None:
                fx, fy, fw, fh = self.config["label"]["roi"]
                focus_roi = (
                    int(fx * frame.shape[1]),
                    int(fy * frame.shape[0]),
                    int(fw * frame.shape[1]),
                    int(fh * frame.shape[0]),
                )
            display = (
                focus_peaking(frame, roi=focus_roi)
                if self.focus_overlay_enabled
                else frame.copy()
            )
            if self.live_package is not None:
                display = self.pipeline.draw_package_detection(display, self.live_package)
            self.video.show_frame(display)
        finally:
            if isinstance(worker, CameraWorker):
                worker.frame_consumed()

    def _set_inspecting(self, running: bool) -> None:
        # Calibration shares the QR reader/pipeline with inspection, so lock it out.
        live = self.camera is not None and not running and self.auto_thread is None
        self.inspect_button.setEnabled(live)
        self.logo_calibration_button.setEnabled(live)
        # Both modes: the conveyor camera trigger needs a fresh empty-belt image too.
        self.background_button.setEnabled(live)
        self.bench_check.setEnabled(not running)

    def inspect(self) -> None:
        if not self.frames or self.inspection_thread is not None or self.auto_thread is not None:
            return
        self._inspection_started = perf_counter()
        self.frame_state.setText("Đang kiểm tra")
        self.frame_state.setStyleSheet("color:#d49b4e; font-weight:700;")
        self._set_twin_process(ProcessStage.VISION)
        frames = self.frames.snapshot()

        def run() -> tuple[InspectionResult, str | None, str | None]:
            result = self.pipeline.inspect(frames)
            return (result, *self._record_result(result))

        # pipeline.inspect takes 0.5-3 s on 2K frames; keep the live view running.
        self.inspection_thread = TaskThread(run)
        self.inspection_thread.succeeded.connect(self._on_inspection_done)
        self.inspection_thread.errored.connect(self._on_inspection_error)
        self.inspection_thread.finished.connect(self._on_inspection_finished)
        self._set_inspecting(True)
        self.inspection_thread.start()

    def _record_result(self, result: InspectionResult) -> tuple[str | None, str | None]:
        """Save evidence and upload the sorting log; runs in a worker thread."""
        storage_error = None
        evidence_folder = None
        if self.evidence is not None:
            try:
                evidence_folder = self.evidence.save(result)
            except Exception as exc:  # audit failure must not hide the result
                storage_error = str(exc)
        upload_error = None
        if self.reporter is not None:
            try:
                upload_error = self.reporter.report(result, evidence_folder)
            except Exception as exc:  # upload failure must not hide the result
                upload_error = str(exc)
        return storage_error, upload_error

    # -- automatic cycle: sensor -> stop -> inspect -> record -> release --------
    def _stop_auto_cycle(self) -> None:
        if self.auto_thread is not None:
            self.auto_thread.request_stop()

    def _toggle_auto_cycle(self) -> None:
        if self.auto_thread is not None:
            self.auto_thread.request_stop()
            self.auto_button.setEnabled(False)
            self.auto_status.setText("Đang dừng chu trình…")
            return
        if self.camera is None:
            QMessageBox.warning(self, "Chu trình tự động", "Hãy kết nối camera trước.")
            return
        if self.inspection_thread is not None:
            return
        cycle_cfg = dict(self.config.get("station_cycle", {}))
        cycle_cfg.setdefault("speed", self.conveyor_panel.speed.value())
        self.auto_thread = AutoCycleThread(
            self.conveyor_panel.host.text(),
            self.conveyor_panel.port.value(),
            cycle_cfg,
            self.frames,
            int(self.config["acquisition"].get("burst_frames", 15)),
            inspect=self.pipeline.inspect,
            record=self._record_result,
            background=getattr(self.pipeline.damage_detector, "reference", None),
            external_pick=self._robot_takes_parcels,
            log_dir=runtime_root() / "logs",
            parent=self,
        )
        self.auto_thread.parcel_at_pick.connect(self._on_parcel_at_pick)
        self.auto_thread.phase_changed.connect(self._on_auto_phase)
        self.auto_thread.result_ready.connect(self._on_auto_result)
        self.auto_thread.finished.connect(self._on_auto_finished)
        self._set_auto_button(True)
        self.inspect_button.setEnabled(False)
        self.logo_calibration_button.setEnabled(False)
        self.auto_thread.start()

    def _robot_takes_parcels(self) -> bool:
        """Overridden by the integrated station: True when the robot sorts automatically.

        Called from the cycle thread; must only read plain attributes.
        """
        return False

    def _on_parcel_at_pick(self, result: InspectionResult) -> None:
        """Parcel stopped at the pick sensor. Standalone: wait for the operator to lift it."""
        self.picked_button.setEnabled(True)
        self.auto_status.setText(
            f"PICKING · Kiện ({result.classification.decision.value}) ở điểm gắp: nhấc kiện ra "
            "rồi bấm ĐÃ LẤY KIỆN (phím Space) để băng tải chạy tiếp"
        )

    def _confirm_manual_pick(self) -> None:
        self.picked_button.setEnabled(False)
        if self.auto_thread is not None:
            self.auto_thread.notify_manual_pick()

    # Cycle phase -> (route step, tone).
    ROUTE_STEPS = {
        "RUNNING": (0, "active"),
        "SETTLING": (0, "active"),
        "INSPECTING": (0, "active"),
        "RELEASING": (1, "active"),
        "EJECTING": (1, "reject"),
        "PICKING": (2, "active"),
    }

    def _on_auto_phase(self, phase: str, text: str) -> None:
        if phase != "PICKING":
            self.picked_button.setEnabled(False)
        previous = self.auto_phase
        self.auto_phase = phase
        self.auto_status.setText(text)
        color = {"FAULT": P["reject"], "IDLE": P["muted"]}.get(phase, P["text"])
        self.auto_status.setStyleSheet(f"color:{color}; font-size:12px; font-weight:600;")
        if phase == "FAULT":
            step = self.ROUTE_STEPS.get(previous, (0, "active"))[0]
            self.route.set_active(step, text, "fault")
        elif phase == "IDLE":
            self.route.set_active(None)
        elif phase in self.ROUTE_STEPS:
            step, tone = self.ROUTE_STEPS[phase]
            if phase == "RUNNING":
                text = "Chờ kiện tới camera"
            elif phase == "EJECTING":
                text = "Kiện lỗi chạy ra sọt lỗi"
            self.route.set_active(step, text, tone)
        if phase in {"SETTLING", "INSPECTING"}:
            self._inspection_started = perf_counter()
            self.frame_state.setText("Đang kiểm tra")
            self._set_twin_process(ProcessStage.VISION)
        if phase == "FAULT":
            self.vision_status.set_state("FAULT")

    def _on_auto_result(self, payload: tuple[InspectionResult, str | None, str | None]) -> None:
        result, storage_error, upload_error = payload
        self._refresh_upload_status()
        if storage_error:
            # No modal dialog in automatic mode: the line keeps running.
            self.auto_status.setText(f"Cảnh báo lưu bằng chứng: {storage_error}")
        self._show_result(result)

    def _refresh_upload_status(self) -> None:
        """SUPABASE pill: ONLINE / OFFLINE / READY, rows waiting, last error (tooltip)."""
        if not hasattr(self, "remote_status"):
            return
        if self.reporter is None:
            reason = self.config.get("supabase", {}).get("unavailable_reason")
            if reason:
                self.remote_status.setToolTip(f"Không gửi lên web: {reason}")
            self.route.set_web("pending", "Chưa nối web (thử cục bộ)")
            return
        status = self.reporter.status()
        self.remote_status.set_state(status.state)
        if status.pending:
            self.remote_status.state.setText(
                f"{STATE_TEXT.get(status.state, status.state)}, {status.pending} chờ gửi"
            )
        if status.state == "OFFLINE" or status.pending:
            self.route.set_web("fault", f"Mất mạng, {status.pending} kết quả chờ gửi")
        elif status.state == "ONLINE":
            self.route.set_web("done", f"Đã lưu {status.sent} kết quả")
        if status.state == "OFFLINE":
            tip = f"Mất kết nối Supabase, {status.pending} kết quả đang chờ gửi lại: {status.last_error}"
        elif status.state == "ONLINE":
            tip = f"Đã gửi {status.sent} kết quả lên web trong phiên này"
        else:
            tip = "Đã kết nối cấu hình Supabase, chưa gửi kết quả nào"
        self.remote_status.setToolTip(tip)

    def _on_auto_finished(self) -> None:
        self.auto_thread = None
        self.picked_button.setEnabled(False)
        self._set_auto_button(False)
        self.auto_phase = "IDLE"
        self.route.set_active(None)
        self.auto_button.setEnabled(True)
        self._set_inspecting(False)

    def _on_inspection_finished(self) -> None:
        self.inspection_thread = None
        self._set_inspecting(False)

    def _on_inspection_done(
        self, payload: tuple[InspectionResult, str | None, str | None]
    ) -> None:
        result, storage_error, upload_error = payload
        if storage_error:
            QMessageBox.warning(self, "Audit storage warning", storage_error)
        # No dialog on upload failure: the row is queued and the belt keeps running.
        self._refresh_upload_status()
        self._show_result(result)

    def _on_inspection_error(self, exc: Exception) -> None:
        if isinstance(exc, FrameQualityError):
            self.frame_state.setText("Ảnh chưa đạt, chờ chụp lại")
            self.frame_state.setStyleSheet("color:#e2b347; font-weight:700;")
            self._set_twin_process(ProcessStage.VISION)
            QMessageBox.warning(self, "Quality gate", str(exc))
            return
        # show adapter/runtime errors to the operator
        self.vision_status.set_state("FAULT")
        self.frame_state.setText("Lỗi")
        self.frame_state.setStyleSheet("color:#e35d4f; font-weight:700;")
        QMessageBox.critical(self, "Inspection error", str(exc))

    def _show_result(self, result: InspectionResult) -> None:
        classification = result.classification
        colors = {decision: DECISION_COLORS[decision.value] for decision in Decision}
        decision_text = DECISION_TEXT[classification.decision.value]
        if classification.decision == Decision.PASS and not self.local_test_mode:
            decision_text += f", sọt {classification.destination_bin}"
        if self.local_test_mode:
            decision_text += " (thử cục bộ)"
        self.decision_label.setText(decision_text)
        self.decision_label.setStyleSheet(
            f"font-family:'{DISPLAY}'; font-size:26px; font-weight:600;"
            f"color:{colors[classification.decision]}; padding:4px 0;"
        )
        self.waybill_label.setText(classification.parcel.waybill_code or "—")
        self.qr_payload_label.setText(
            classification.parcel.qr_text or "Không đọc được"
        )
        logo = classification.parcel.logo
        logo_text = "Chưa hiệu chuẩn"
        if logo and logo.calibrated:
            logo_text = "Đúng logo" if logo.present else "Không khớp"
            logo_text += f" ({logo.score:.2f})"
        self.logo_label.setText(logo_text)
        damage = classification.parcel.damage
        damage_text = "Chưa hiệu chuẩn"
        if damage and damage.calibrated:
            if damage.damaged is None:
                damage_text = "Chưa kết luận"
            elif damage.damaged:
                names = {
                    "label_tear": "rách tem",
                    "deformation": "móp, méo",
                    "dent_or_edge_tear": "móp hoặc rách cạnh",
                    "irregular_outline": "biến dạng",
                    "hole_or_puncture": "thủng",
                }
                translated = [names.get(item, item) for item in damage.damage_types]
                damage_text = "Hư hỏng: " + ", ".join(translated)
            else:
                damage_text = "Bình thường"
        self.damage_label.setText(damage_text)
        self.destination_label.setText(classification.destination_bin)
        self.reason_label.setText(classification.reason)
        snapshot = result.debug_images.get("damage_overlay", result.original_frame)
        self.snapshot_video.show_frame(snapshot)
        self.snapshot_view_button.setEnabled(True)
        self._show_snapshot_view()

        elapsed_ms = max(0.0, (perf_counter() - self._inspection_started) * 1000)
        self.total_count += 1
        if classification.decision == Decision.PASS:
            self.pass_count += 1
        elif classification.decision == Decision.REJECT:
            self.reject_count += 1
        else:
            self.review_count += 1
        qr_ok = bool(classification.parcel.qr_text)
        if qr_ok:
            self.qr_success_count += 1
        self.throughput_metric.value.setText(str(self.total_count))
        self.success_metric.value.setText(f"{self.qr_success_count / self.total_count * 100:.1f}")
        self.pass_metric.value.setText(str(self.pass_count))
        self.reject_metric.value.setText(str(self.reject_count))
        self.cycle_metric.value.setText(f"{elapsed_ms:.0f}")
        self.frame_state.setText(classification.decision.value)
        self.frame_state.setStyleSheet(
            f"color:{colors[classification.decision]}; font-weight:700;"
        )
        self._set_twin_process(
            ProcessStage.COMPLETE,
            decision=classification.decision.value,
        )

        self.history.insertRow(0)
        values = (
            datetime.now().strftime("%H:%M:%S"),
            classification.parcel.waybill_code or "—",
            "OK" if qr_ok else "FAIL",
            classification.decision.value,
            classification.destination_bin,
            classification.reason,
        )
        tooltip = f"QR: {values[2]}. {classification.reason}"
        for column, value in enumerate(values):
            item = QTableWidgetItem(value)
            item.setToolTip(tooltip)
            if column == 3:
                item.setForeground(QColor(colors[classification.decision]))
            self.history.setItem(0, column, item)

    def _update_clock(self) -> None:
        self.clock_label.setText(datetime.now().strftime("%H:%M:%S  %d/%m/%Y"))

    def closeEvent(self, event) -> None:
        if self.auto_thread is not None:
            # The cycle sends STOP and AUTO:0 itself when it ends.
            self.auto_thread.request_stop()
            self.auto_thread.wait(15000)
        stop_error = self.conveyor_panel.shutdown()
        if stop_error:
            response = QMessageBox.warning(
                self, "Chưa xác nhận dừng băng tải",
                "ESP32 chưa ACK lệnh STOP. Hãy ngắt nguồn motor bằng phần cứng "
                "và kiểm tra tại chỗ. Chỉ chọn Close khi đã xác minh motor dừng "
                f"và nguồn động lực bị ngắt.\n\n{stop_error}",
                QMessageBox.StandardButton.Close | QMessageBox.StandardButton.Cancel,
                QMessageBox.StandardButton.Cancel,
            )
            if response != QMessageBox.StandardButton.Close:
                event.ignore()
                return
        if self.inspection_thread is not None:
            self.inspection_thread.wait(10000)
        if self.camera is not None:
            self.camera.stop()
            self.camera.wait(3000)
        event.accept()
