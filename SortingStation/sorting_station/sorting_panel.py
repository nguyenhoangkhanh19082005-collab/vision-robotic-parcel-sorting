"""Giao diện phân loại: thẻ gọn trên tab VẬN HÀNH + tab cấu hình/dạy tư thế đầy đủ."""

from __future__ import annotations

from datetime import datetime

import numpy as np
from PyQt6.QtCore import QObject, Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from sorting_station.sorting import PlanningError, PoseLibrary, SortingSequencer, SortingSettings


class SortingController(QObject):
    """Giữ trạng thái 'tự động phân loại' và kết quả Vision gần nhất (dùng chung cho thẻ và tab)."""

    armed_changed = pyqtSignal(bool)
    last_result_changed = pyqtSignal(str)

    def __init__(self, sequencer: SortingSequencer, settings: SortingSettings, parent=None) -> None:
        super().__init__(parent)
        self.sequencer = sequencer
        self.settings = settings
        self.armed = False
        self.last_bin: str | None = None
        self.last_text = "Chưa có kết quả Vision"
        sequencer.state_changed.connect(self._on_state)

    def set_armed(self, armed: bool) -> None:
        if armed != self.armed:
            self.armed = armed
            self.armed_changed.emit(armed)

    def on_inspection(
        self, decision: str, destination: str | None, waybill: str | None, start: bool = True
    ) -> None:
        """``start=False``: the camera cycle will call ``on_parcel_at_pick`` later."""
        self.last_bin = self.settings.resolve_bin(decision, destination)
        stamp = datetime.now().strftime("%H:%M:%S")
        self.last_text = f"{stamp} · {decision} ({destination or '—'}) · {waybill or 'không mã'} → thùng {self.last_bin}"
        self.last_result_changed.emit(self.last_text)
        if self.armed and start:
            self.sequencer.start_cycle(self.last_bin)

    def on_parcel_at_pick(self, decision: str, destination: str | None) -> bool:
        """Parcel stopped on the pick sensor: sort it with the bin of ITS inspection."""
        bin_key = self.settings.resolve_bin(decision, destination)
        return self.sequencer.start_cycle(bin_key, at_pick=True)

    def trigger_last(self) -> bool:
        if self.last_bin is None:
            self.sequencer.message.emit("Chưa có kết quả KIỂM TRA KIỆN để phân loại")
            return False
        return self.sequencer.start_cycle(self.last_bin)

    def _on_state(self, state: str) -> None:
        if state == "FAULT" and self.armed:
            self.set_armed(False)  # lỗi → tự hủy chế độ tự động, cần người vận hành xác nhận lại


def _confirm_real_motion(parent: QWidget, sequencer: SortingSequencer, action: str) -> bool:
    if sequencer.simulation_mode():
        return True
    answer = QMessageBox.warning(
        parent,
        "Robot thật sẽ chuyển động",
        f"{action}\n\nRobot đang KẾT NỐI THẬT. Kiểm tra: không có người/vật cản trong vùng làm việc, "
        "đã lưu gốc đúng tư thế, có công tắc ngắt nguồn động lực trong tầm tay.\n\n"
        "STOP trên phần mềm KHÔNG thay thế E-stop phần cứng. Tiếp tục?",
        QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
        QMessageBox.StandardButton.Cancel,
    )
    return answer == QMessageBox.StandardButton.Yes


class SortingCard(QFrame):
    """Thẻ gọn đặt cạnh camera trên tab VẬN HÀNH."""

    def __init__(self, controller: SortingController, parent=None) -> None:
        super().__init__(parent)
        self.controller = controller
        sequencer = controller.sequencer
        self.setObjectName("subpanel")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(5)
        header = QHBoxLayout()
        title = QLabel("Robot 6 trục")
        title.setObjectName("sectionTitle")
        self.mode = QLabel()
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.mode)
        layout.addLayout(header)
        self.last = QLabel(controller.last_text)
        self.last.setWordWrap(True)
        self.last.setStyleSheet("color:#93a39e; font-size:11px;")
        layout.addWidget(self.last)
        self.status = QLabel("Sẵn sàng")
        self.status.setWordWrap(True)
        self.status.setStyleSheet("color:#e7edea; font-weight:600;")
        layout.addWidget(self.status)
        row = QHBoxLayout()
        self.armed = QCheckBox("Robot tự gắp sau mỗi lần kiểm tra")
        self.armed.setToolTip("Mỗi lần KIỂM TRA KIỆN xong, robot tự gắp kiện vào thùng theo kết quả")
        self.run_button = QPushButton("Gắp kiện vừa kiểm tra")
        self.run_button.setToolTip("Chạy chu trình gắp–thả cho kết quả Vision gần nhất")
        self.stop_button = QPushButton("Dừng robot")
        self.stop_button.setStyleSheet("background:#c8372d; border-color:#ef7a70; color:white;")
        row.addWidget(self.armed, 1)
        row.addWidget(self.run_button)
        row.addWidget(self.stop_button)
        layout.addLayout(row)

        self.armed.toggled.connect(self._toggle_armed)
        self.run_button.clicked.connect(self._run)
        self.stop_button.clicked.connect(lambda: sequencer.abort("Người vận hành nhấn DỪNG"))
        controller.armed_changed.connect(self._sync_armed)
        controller.last_result_changed.connect(self.last.setText)
        sequencer.message.connect(self.status.setText)
        sequencer.progress.connect(lambda i, n, text: self.status.setText(f"[{i}/{n}] {text}"))
        sequencer.state_changed.connect(lambda _s: self.refresh_mode())
        self.refresh_mode()

    def refresh_mode(self) -> None:
        sim = self.controller.sequencer.simulation_mode()
        self.mode.setText("Mô phỏng" if sim else "Robot thật")
        color = "#e2b347" if sim else "#e35d4f"
        self.mode.setStyleSheet(f"color:{color}; font-size:12px; font-weight:600;")
        self.run_button.setEnabled(not self.controller.sequencer.running)

    def _toggle_armed(self, checked: bool) -> None:
        if checked and not _confirm_real_motion(self, self.controller.sequencer,
                                                "Bật phân loại TỰ ĐỘNG sau mỗi lần kiểm tra."):
            self.armed.blockSignals(True)
            self.armed.setChecked(False)
            self.armed.blockSignals(False)
            return
        self.controller.set_armed(checked)

    def _sync_armed(self, armed: bool) -> None:
        self.armed.blockSignals(True)
        self.armed.setChecked(armed)
        self.armed.blockSignals(False)

    def _run(self) -> None:
        if _confirm_real_motion(self, self.controller.sequencer, "Chạy chu trình gắp–thả cho kiện gần nhất."):
            self.controller.trigger_last()


class SortingPanel(QWidget):
    """Tab đầy đủ: trạng thái chu trình, bảng tư thế, dạy tư thế, kiểm tra tầm với, tham số."""

    def __init__(self, controller: SortingController, poses: PoseLibrary, robot, parent=None) -> None:
        super().__init__(parent)
        self.controller = controller
        self.sequencer = controller.sequencer
        self.poses = poses
        self.robot = robot
        settings = controller.settings
        root = QHBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        # --- cột trái: trạng thái + nhật ký
        left = QFrame()
        left.setObjectName("panel")
        left_layout = QVBoxLayout(left)
        self.card = SortingCard(controller)
        left_layout.addWidget(self.card)
        self.progress = QProgressBar()
        self.progress.setRange(0, 1)
        self.progress.setValue(0)
        self.progress.setFormat("%v / %m bước")
        left_layout.addWidget(self.progress)
        log_title = QLabel("NHẬT KÝ CHU TRÌNH")
        log_title.setObjectName("sectionTitle")
        left_layout.addWidget(log_title)
        self.log = QListWidget()
        left_layout.addWidget(self.log, 1)
        safety = QLabel(
            "AN TOÀN: DỪNG/STOP trên phần mềm là gói tin, không phải dừng khẩn phần cứng. "
            "Robot dùng động cơ bước KHÔNG có encoder — telemetry là số bước firmware đã phát, "
            "không phát hiện được trượt bước. Luôn có E-stop/ngắt nguồn động lực khi chạy thật."
        )
        safety.setWordWrap(True)
        safety.setStyleSheet("color:#e2b347; font-size:11px;")
        left_layout.addWidget(safety)
        root.addWidget(left, 4)

        # --- cột phải: tư thế + tham số
        right = QFrame()
        right.setObjectName("panel")
        right_layout = QVBoxLayout(right)
        pose_title = QLabel("TƯ THẾ PHÂN LOẠI (hệ base robot, mm)")
        pose_title.setObjectName("sectionTitle")
        right_layout.addWidget(pose_title)
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["Tư thế", "Nguồn", "Góc khớp [°]", "TCP X/Y/Z [mm]"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        right_layout.addWidget(self.table, 1)
        buttons = QGridLayout()
        self.teach_button = QPushButton("GHI TƯ THẾ ROBOT HIỆN TẠI")
        self.forget_button = QPushButton("XÓA TƯ THẾ ĐÃ DẠY")
        self.forget_button.setObjectName("secondary")
        self.goto_button = QPushButton("ĐƯA ROBOT TỚI TƯ THẾ")
        self.check_button = QPushButton("KIỂM TRA TẦM VỚI (IK)")
        self.check_button.setObjectName("secondary")
        buttons.addWidget(self.teach_button, 0, 0)
        buttons.addWidget(self.forget_button, 0, 1)
        buttons.addWidget(self.goto_button, 1, 0)
        buttons.addWidget(self.check_button, 1, 1)
        right_layout.addLayout(buttons)

        params = QGridLayout()
        params.addWidget(QLabel("Tốc độ khớp tối đa"), 0, 0)
        self.speed = QDoubleSpinBox()
        self.speed.setRange(5.0, 90.0)
        self.speed.setSuffix(" °/s")
        self.speed.setValue(settings.joint_speed_deg_s)
        params.addWidget(self.speed, 0, 1)
        params.addWidget(QLabel("Thời gian băng tải chuyển kiện"), 1, 0)
        self.transfer = QDoubleSpinBox()
        self.transfer.setRange(0.0, 20.0)
        self.transfer.setSingleStep(0.1)
        self.transfer.setSuffix(" s")
        self.transfer.setToolTip("0 = không dùng băng tải, robot gắp tại điểm gắp. Chưa có cảm biến → "
                                 "thời gian phải hiệu chuẩn bằng thực nghiệm.")
        self.transfer.setValue(settings.conveyor_transfer_s)
        params.addWidget(self.transfer, 1, 1)
        right_layout.addLayout(params)
        hint = QLabel(
            "Tọa độ mặc định lấy từ config/station.yaml (ước lượng). Nên đưa robot tới đúng vị trí "
            "thực tế bằng tab ROBOT rồi nhấn GHI TƯ THẾ cho 'pick' và từng thùng."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color:#93a39e; font-size:11px;")
        right_layout.addWidget(hint)
        root.addWidget(right, 5)

        self.teach_button.clicked.connect(self._teach)
        self.forget_button.clicked.connect(self._forget)
        self.goto_button.clicked.connect(self._goto)
        self.check_button.clicked.connect(self._check)
        self.speed.valueChanged.connect(lambda v: setattr(settings, "joint_speed_deg_s", float(v)))
        self.transfer.valueChanged.connect(lambda v: setattr(settings, "conveyor_transfer_s", float(v)))
        self.sequencer.message.connect(self._log)
        self.sequencer.progress.connect(self._on_progress)
        self.sequencer.state_changed.connect(self._on_state)
        self.refresh_table()

    # ------------------------------------------------------------------
    def _log(self, text: str) -> None:
        self.log.insertItem(0, f"{datetime.now().strftime('%H:%M:%S')}  {text}")

    def _on_progress(self, index: int, total: int, text: str) -> None:
        self.progress.setRange(0, total)
        self.progress.setValue(index)
        self._log(f"[{index}/{total}] {text}")

    def _on_state(self, state: str) -> None:
        running = state == "RUNNING"
        for button in (self.teach_button, self.forget_button, self.goto_button, self.check_button):
            button.setEnabled(not running)
        if state == "DONE":
            self.progress.setValue(self.progress.maximum())

    def _selected_name(self) -> str | None:
        row = self.table.currentRow()
        if row < 0:
            QMessageBox.information(self, "Tư thế", "Chọn một dòng trong bảng tư thế trước.")
            return None
        return self.table.item(row, 0).data(Qt.ItemDataRole.UserRole)

    def refresh_table(self) -> None:
        names = self.poses.base_names()
        self.table.setRowCount(len(names))
        seed = self.robot.current_command()
        for row, name in enumerate(names):
            try:
                joints = self.poses.resolve(name, seed)
                tcp = self.poses.model.fk_deg(joints)[:3, 3] * 1000.0
                joint_text = " ".join(f"{v:+.1f}" for v in joints)
                xyz_text = " / ".join(f"{v:.0f}" for v in tcp)
            except PlanningError as exc:
                joint_text, xyz_text = "KHÔNG VỚI TỚI", str(exc)
            label = QTableWidgetItem(name)
            label.setData(Qt.ItemDataRole.UserRole, name)
            self.table.setItem(row, 0, label)
            self.table.setItem(row, 1, QTableWidgetItem(self.poses.source(name)))
            self.table.setItem(row, 2, QTableWidgetItem(joint_text))
            self.table.setItem(row, 3, QTableWidgetItem(xyz_text))
        self.table.resizeColumnsToContents()

    def _teach(self) -> None:
        name = self._selected_name()
        if name is None:
            return
        joints = self.robot.actual_angles() or self.robot.current_command()
        tcp = self.poses.model.fk_deg(joints)
        if name != "home" and abs(tcp[2, 2] + 1.0) > 0.1:
            answer = QMessageBox.question(
                self, "Hướng đầu hút",
                "Đầu hút chưa hướng thẳng xuống (lệch > ~25°). Vẫn ghi tư thế này?",
            )
            if answer != QMessageBox.StandardButton.Yes:
                return
        try:
            self.poses.teach(name, joints)
        except PlanningError as exc:
            QMessageBox.warning(self, "Không ghi được", str(exc))
            return
        self._log(f"Đã dạy '{name}': {np.round(joints, 1).tolist()}")
        self.refresh_table()

    def _forget(self) -> None:
        name = self._selected_name()
        if name is not None:
            self.poses.forget(name)
            self._log(f"Đã xóa tư thế dạy '{name}' — dùng lại giá trị cấu hình")
            self.refresh_table()

    def _goto(self) -> None:
        name = self._selected_name()
        if name is not None and _confirm_real_motion(self, self.sequencer, f"Đưa robot tới tư thế '{name}'."):
            self.sequencer.start_move_to(name)

    def _check(self) -> None:
        report = self.poses.validate_all(self.robot.current_command())
        text = "\n".join(f"Thùng {key}: {status}" for key, status in report)
        self._log("Kiểm tra tầm với: " + "; ".join(f"{k}={s}" for k, s in report))
        QMessageBox.information(self, "Kiểm tra tầm với (IK)", text)
        self.refresh_table()
