"""Cửa sổ chính hợp nhất: GUI MachineVision (camera, Vision, băng tải) + GUI robot 6 trục + twin chung.

Kế thừa ``machine_vision.ui.main_window.MainWindow`` và chỉ ghép thêm; KHÔNG sửa mã nguồn MachineVision:
- Twin phần mềm cũ (robot hình que, không đúng động học) được thay bằng CellTwinView (mesh CAD).
- Tab VẬN HÀNH: thêm twin thu gọn + thẻ phân loại cạnh camera → một màn hình thấy cả hai.
- Tab ROBOT 6 TRỤC: nhúng nguyên GUI robot (jog, MoveL, kết nối USB/LAN, lưu gốc...).
- Tab PHÂN LOẠI: bộ tuần tự gắp–thả theo kết quả Vision, dạy tư thế, kiểm tra tầm với.
- Nút DỪNG TOÀN TRẠM luôn hiển thị (góc phải thanh tab).
"""

from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QFrame, QLabel, QPushButton, QVBoxLayout, QWidget

from machine_vision.ui.digital_twin import ProcessStage
from machine_vision.ui.main_window import MainWindow, StatusPill
from sorting_station import __version__
from sorting_station.cell_state import CellLayout, CellState
from sorting_station.cell_twin import CellTwinView
from sorting_station.robot_bridge import create_robot_window
from sorting_station.robot_model import build_model_from_robot_project
from sorting_station.sorting import MachineVisionConveyor, PoseLibrary, SortingSequencer, SortingSettings
from sorting_station.sorting_panel import SortingCard, SortingController, SortingPanel
from sorting_station.station_config import StationPaths


def _layout_containing(root_layout, widget):
    """Tìm layout con (đệ quy) đang chứa `widget` — dùng để chèn thêm widget vào GUI MV."""

    if root_layout is None:
        return None
    if root_layout.indexOf(widget) >= 0:
        return root_layout
    for index in range(root_layout.count()):
        child = root_layout.itemAt(index).layout()
        found = _layout_containing(child, widget)
        if found is not None:
            return found
    return None


class IntegratedMainWindow(MainWindow):
    def __init__(self, mv_config: dict, pipeline, evidence, project_root: Path,
                 station_config: dict, paths: StationPaths) -> None:
        super().__init__(mv_config, pipeline, evidence, project_root=project_root)
        self.station_config = station_config
        self.paths = paths
        robot_cfg = station_config.get("robot", {})
        self.model = build_model_from_robot_project(
            paths.robot_root,
            tool_length_m=float(robot_cfg.get("tool_length_mm", 30.0)) / 1000.0,
            scale=float(robot_cfg.get("scale", 1.0)),
        )
        self.cell = CellState(
            CellLayout.from_config(station_config.get("layout", {})),
            telemetry_timeout_s=float(robot_cfg.get("telemetry_timeout_s", 0.5)),
            parent=self,
        )
        mesh_dir = paths.robot_root / "assets" / "ROBOTARM2_description" / "meshes"

        self._install_cell_twins(mesh_dir)
        self.robot_window = create_robot_window(paths.robot_root, self.model, robot_cfg)
        self._install_sorting()
        self._install_tabs_and_estop()
        self._connect_signals()

        self.setWindowTitle(
            f"UDMT · Trạm phân loại kiện hàng v{__version__} — Vision + Băng tải ESP32 + Robot 6 trục"
        )
        self.resize(1600, 960)
        self._mirror_timer = QTimer(self)
        self._mirror_timer.timeout.connect(self._refresh_robot_mirror)
        self._mirror_timer.start(200)

    # ------------------------------------------------------------ dựng giao diện
    def _install_cell_twins(self, mesh_dir: Path) -> None:
        old = self.digital_twin
        holder = old.parentWidget().layout()
        self.cell_twin = CellTwinView(self.cell, self.model, mesh_dir)
        holder.replaceWidget(old, self.cell_twin)
        old.hide()
        old.deleteLater()
        self.digital_twin = self.cell_twin  # mọi lời gọi của MV giờ đi vào CellState
        self.twin_sync.setText("CAD + TELEMETRY")
        self.twin_sync.setStyleSheet("color:#26a269; font-weight:700; font-size:10px;")

        # Tab VẬN HÀNH: twin thu gọn thành cột riêng giữa camera và cột kết quả
        right_panel = self.focus_metric.parentWidget()
        frame = QFrame()
        frame.setObjectName("panel")
        frame_layout = QVBoxLayout(frame)
        frame_layout.setContentsMargins(8, 8, 8, 8)
        title = QLabel("DIGITAL TWIN TOÀN TRẠM · THỜI GIAN THỰC")
        title.setObjectName("sectionTitle")
        frame_layout.addWidget(title)
        self.cell_twin_compact = CellTwinView(self.cell, self.model, mesh_dir, compact=True)
        frame_layout.addWidget(self.cell_twin_compact, 1)
        self._twin_column = frame_layout  # thẻ phân loại được thêm vào dưới twin
        body_layout = right_panel.parentWidget().layout()
        body_layout.insertWidget(body_layout.indexOf(right_panel), frame, 5)
        body_layout.setStretch(0, 6)  # camera
        body_layout.setStretch(body_layout.indexOf(right_panel), 4)

    def _install_sorting(self) -> None:
        settings = SortingSettings.from_config(self.station_config.get("sorting", {}))
        taught = self.paths.station_root / self.station_config.get("sorting", {}).get(
            "taught_poses_file", "config/taught_poses.json"
        )
        self.poses = PoseLibrary(self.model, settings, taught)
        self.conveyor_port = MachineVisionConveyor(self.conveyor_panel)
        self.sequencer = SortingSequencer(self.robot_window, self.conveyor_port, self.poses, self.cell,
                                          settings, parent=self)
        self.sorting_controller = SortingController(self.sequencer, settings, parent=self)
        self.sorting_card = SortingCard(self.sorting_controller)
        self._twin_column.addWidget(self.sorting_card)
        self.sorting_panel = SortingPanel(self.sorting_controller, self.poses, self.robot_window)

    def _install_tabs_and_estop(self) -> None:
        robot_page = QWidget()
        robot_layout = QVBoxLayout(robot_page)
        robot_layout.setContentsMargins(0, 0, 0, 0)
        robot_layout.addWidget(self.robot_window)
        self.robot_tab_index = self.workspace_tabs.addTab(robot_page, "ROBOT 6 TRỤC")
        self.workspace_tabs.addTab(self.sorting_panel, "PHÂN LOẠI TỰ ĐỘNG")

        self.estop_button = QPushButton("⛔ DỪNG TOÀN TRẠM")
        self.estop_button.setToolTip(
            "Gửi STOP tới robot (dừng động cơ, tắt bơm) và băng tải, hủy chu trình. "
            "Đây là lệnh phần mềm — KHÔNG thay thế nút dừng khẩn phần cứng."
        )
        self.estop_button.setStyleSheet(
            "background:#c62828; border:1px solid #ff8a80; color:white; font-weight:800; padding:6px 14px;"
        )
        self.estop_button.clicked.connect(self.emergency_stop_all)
        self.workspace_tabs.setCornerWidget(self.estop_button)

        # Thay trang ROBOT (slider mô phỏng độc lập) trong Inspector bằng bảng giám sát chỉ đọc
        mirror = QWidget()
        mirror_layout = QVBoxLayout(mirror)
        mirror_layout.setContentsMargins(8, 10, 8, 8)
        heading = QLabel("ROBOT 6 TRỤC · GIÁM SÁT")
        heading.setObjectName("sectionTitle")
        mirror_layout.addWidget(heading)
        self.robot_mirror_labels = []
        for joint in range(6):
            label = QLabel(f"J{joint + 1}: —")
            label.setStyleSheet("background:#0d141d; padding:5px; color:#aebdca; font-family:Consolas;")
            mirror_layout.addWidget(label)
            self.robot_mirror_labels.append(label)
        self.robot_mirror_tcp = QLabel("TCP: —")
        self.robot_mirror_tcp.setStyleSheet("color:#8fc1ff; font-family:Consolas;")
        mirror_layout.addWidget(self.robot_mirror_tcp)
        open_robot = QPushButton("MỞ TAB ROBOT 6 TRỤC")
        open_robot.setObjectName("secondary")
        open_robot.clicked.connect(lambda: self.workspace_tabs.setCurrentIndex(self.robot_tab_index))
        mirror_layout.addWidget(open_robot)
        mirror_layout.addStretch()
        self.inspector_tabs.removeTab(1)
        self.inspector_tabs.insertTab(1, mirror, "ROBOT")

        # Trạng thái robot cạnh các trạng thái của MV
        self.robot_status = StatusPill("ROBOT", "MÔ PHỎNG")
        statuses = _layout_containing(self.centralWidget().layout(), self.camera_status)
        if statuses is not None:
            statuses.insertWidget(statuses.indexOf(self.controller_status) + 1, self.robot_status)
        self.controller_status.name.setText("BĂNG TẢI")

    def _connect_signals(self) -> None:
        robot = self.robot_window
        robot.command_changed.connect(self.cell.set_robot_command)
        robot.telemetry_changed.connect(self.cell.set_robot_actual)
        robot.tool_changed.connect(self.cell.set_tool)
        robot.link_changed.connect(self._on_robot_link)
        self.cell.set_robot_command(robot.current_command())
        self.conveyor_panel.state_changed.connect(lambda state, speed: self.cell.set_conveyor(state, speed))
        self.sequencer.state_changed.connect(self._on_sequence_state)
        self.workspace_tabs.currentChanged.connect(self._on_tab_changed)

    # ------------------------------------------------------------ hành vi
    def _show_result(self, result) -> None:
        super()._show_result(result)
        classification = result.classification
        self.sorting_controller.on_inspection(
            classification.decision.value,
            classification.destination_bin,
            classification.parcel.waybill_code,
        )

    def _on_sequence_state(self, state: str) -> None:
        running = state == "RUNNING"
        if running:
            self.cell.set_process(ProcessStage.ROUTING, self.cell.decision)
        elif state == "DONE":
            self.cell.set_process(ProcessStage.COMPLETE, self.cell.decision)
        self.inspect_button.setEnabled(not running and self.camera is not None)
        self.twin_stage_value.setText(f"Stage: {self.cell.stage.value}")

    def _on_robot_link(self, online: bool) -> None:
        self.cell.set_robot_link("ONLINE" if online else "OFFLINE")
        self.robot_status.set_state("ONLINE" if online else "MÔ PHỎNG")
        if not online and self.sequencer.running:
            self.sequencer.abort("Mất kết nối robot trong khi chạy chu trình")
        for card in (self.sorting_card, self.sorting_panel.card):
            card.refresh_mode()

    def _on_tab_changed(self, index: int) -> None:
        if index == getattr(self, "robot_tab_index", -1):
            self.robot_window.refresh_view()

    def _refresh_robot_mirror(self) -> None:
        command = self.cell.robot_command_deg
        actual = self.cell.robot_actual_deg if self.cell.actual_is_fresh() else None
        for joint, label in enumerate(self.robot_mirror_labels):
            act = f"{actual[joint]:+7.1f}" if actual else "    —  "
            label.setText(f"J{joint + 1}  CMD {command[joint]:+7.1f}°   ACT {act}°")
        x, y, z = self.model.fk_deg(command)[:3, 3] * 1000.0
        self.robot_mirror_tcp.setText(f"TCP  X={x:.0f}  Y={y:.0f}  Z={z:.0f} mm")
        link = "ONLINE" if self.cell.robot_link == "ONLINE" else "MÔ PHỎNG"
        tool = {0: "OFF", 1: "HÚT", 2: "NHẢ"}.get(self.cell.tool_state, "?")
        self.twin_robot_value.setText(f"Robot: {link} · đầu hút {tool}")

    def emergency_stop_all(self) -> None:
        self.sequencer.abort("DỪNG TOÀN TRẠM (người vận hành)")
        self.conveyor_panel._send("STOP")  # STOP luôn được phép trong ConveyorPanel
        self.robot_window.log_message("DỪNG TOÀN TRẠM: đã gửi STOP robot + băng tải.", "ERROR")

    def closeEvent(self, event) -> None:
        if self.sequencer.running:
            self.sequencer.abort("Đóng ứng dụng")
        super().closeEvent(event)  # MV: xác nhận STOP băng tải, có thể hủy đóng
        if not event.isAccepted():
            return
        self._mirror_timer.stop()
        self.robot_window.shutdown()
        for view in (self.cell_twin, self.cell_twin_compact):
            view.close_view()
