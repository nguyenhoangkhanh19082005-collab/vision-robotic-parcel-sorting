"""Nhúng GUI robot (robotarm-6-axis/main.py) vào app chung và mở API điều khiển cho bộ tuần tự.

Lớp con ``IntegratedRobotWindow`` giữ nguyên toàn bộ giao diện/logic gốc, chỉ:
1. Thay động học DH (lệch tư thế 0, xem robot_model.py) bằng RobotModel PoE khớp với twin.
2. Đọc telemetry không chặn GUI và luôn lấy gói MỚI NHẤT (bản gốc đọc 1 gói/chu kỳ, socket UDP
   timeout 20 ms chặn luồng GUI ~60% thời gian khi robot không trả lời).
3. Phát tín hiệu góc lệnh / góc thực / đầu hút cho CellState.
4. Cung cấp các hàm ``command_angles``, ``set_tool``, ``emergency_stop``, ``set_manual_lock``
   cho bộ tuần tự phân loại.
"""

from __future__ import annotations

import importlib.util
import select
import sys
import time
from pathlib import Path

import numpy as np
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import QFrame, QScrollArea

from sorting_station.robot_model import RobotModel


def load_robot_main(robot_root: Path):
    """Nạp robotarm-6-axis/main.py dưới tên module riêng (tránh trùng tên 'main')."""

    if str(robot_root) not in sys.path:
        sys.path.insert(0, str(robot_root))
    module_name = "aura6_robot_main"
    if module_name in sys.modules:
        return sys.modules[module_name]
    spec = importlib.util.spec_from_file_location(module_name, robot_root / "main.py")
    if spec is None or spec.loader is None:
        raise ImportError(f"Không nạp được {robot_root / 'main.py'}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def create_robot_window(robot_root: Path, model: RobotModel, robot_config: dict, parent=None):
    robot_main = load_robot_main(robot_root)

    class IntegratedRobotWindow(robot_main.WorldClassRoboDKApp):
        command_changed = pyqtSignal(list)
        telemetry_changed = pyqtSignal(list)
        tool_changed = pyqtSignal(int)
        link_changed = pyqtSignal(bool)

        def __init__(self) -> None:
            urdf_dir = robot_root / "assets" / "ROBOTARM2_description"
            super().__init__(urdf_dir=str(urdf_dir))
            self.setWindowFlags(Qt.WindowType.Widget)  # nhúng như widget con trong tab
            self.kin = model
            self.robot_model = model
            self.last_feedback: dict | None = None
            self.last_feedback_time = 0.0
            self._auto_locked = False
            self._wrap_telemetry_reader()
            self._make_panels_scrollable()
            self._apply_connection_defaults(robot_config)
            self._ready = True
            self.log_message(
                "Đã thay động học DH bằng mô hình PoE từ trục khớp CAD (khớp với digital twin). "
                f"Chiều dài đầu hút: {model.tool_length_m * 1000:.0f} mm.",
                "WARNING",
            )
            self.sync_system_pose()

        # ---------------------------------------------------- khởi tạo phụ
        def _make_panels_scrollable(self) -> None:
            """Bọc từng trang của ROBOT CONTROLLER PANEL trong QScrollArea.

            Khi nhúng dưới header/KPI của MachineVision, chiều cao còn lại nhỏ hơn bản độc lập;
            không có thanh cuộn thì layout ép 6 slider chồng lên nhau.
            """

            tabs = self.ui.robot_tabs
            current = tabs.currentIndex()
            for index in range(tabs.count()):
                page, text = tabs.widget(index), tabs.tabText(index)
                tabs.removeTab(index)
                scroll = QScrollArea()
                scroll.setWidgetResizable(True)
                scroll.setFrameShape(QFrame.Shape.NoFrame)
                scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
                page.setMinimumHeight(page.sizeHint().height())
                scroll.setWidget(page)
                tabs.insertTab(index, scroll, text)
            tabs.setCurrentIndex(current)

        def _apply_connection_defaults(self, cfg: dict) -> None:
            if cfg.get("connection", "usb").lower() == "lan":
                self.ui.cmb_conn_type.setCurrentIndex(1)
            self.ui.txt_ip.setText(str(cfg.get("lan_ip", self.ui.txt_ip.text())))
            self.ui.txt_port.setText(str(cfg.get("lan_port", self.ui.txt_port.text())))
            port = str(cfg.get("serial_port", "") or "").strip()
            if port:
                self.ui.cmb_com_port.setCurrentText(port)
            baud = str(cfg.get("baudrate", "") or "")
            if baud:
                self.ui.cmb_baudrate.setCurrentText(baud)

        def _wrap_telemetry_reader(self) -> None:
            comm = self.comm
            original = comm.read_telemetry

            def read_latest():
                latest = None
                for _ in range(16):  # xả hàng đợi, giữ gói mới nhất
                    if comm.mode == "UDP":
                        if comm.sock is None:
                            break
                        try:
                            readable, _, _ = select.select([comm.sock], [], [], 0)
                        except (OSError, ValueError):
                            break
                        if not readable:
                            break
                    packet = original()
                    if packet is None:
                        break
                    latest = packet
                if latest is not None:
                    angles = latest.get("angles", [])
                    if len(angles) == 6 and all(np.isfinite(angles)):
                        self.last_feedback = latest
                        self.last_feedback_time = time.monotonic()
                        self.telemetry_changed.emit([float(a) for a in angles])
                return latest

            comm.read_telemetry = read_latest

        # ------------------------------------------- ghi đè để phát tín hiệu
        def sync_system_pose(self):
            super().sync_system_pose()
            if getattr(self, "_ready", False) and not getattr(self, "_auto_locked", False):
                self.command_changed.emit([float(s.value()) for s in self.sliders])

        def handle_tool_control(self, state):
            super().handle_tool_control(state)
            self.tool_changed.emit(int(state))

        def read_hardware_feedback(self):
            was_connected = self.is_connected()
            super().read_hardware_feedback()
            if was_connected != self.is_connected():
                self.link_changed.emit(self.is_connected())

        def toggle_connection(self):
            super().toggle_connection()
            self.link_changed.emit(self.is_connected())

        # ------------------------------------------------ API bộ tuần tự
        def is_connected(self) -> bool:
            return bool(self.comm and self.comm.is_connected)

        def home_calibrated(self) -> bool:
            # Lớp gốc dùng thuộc tính bool `is_home_calibrated` (không phải hàm)
            return bool(getattr(self, "is_home_calibrated", False))

        def current_command(self) -> list[float]:
            sliders = [float(s.value()) for s in self.sliders]
            last = getattr(self, "_auto_angles", None)
            if self._auto_locked or (
                last is not None and all(int(round(a)) == s for a, s in zip(last, sliders))
            ):
                return list(last)  # giữ góc thực số của chu trình, slider chỉ hiển thị làm tròn
            return sliders

        def actual_angles(self, max_age_s: float = 0.5) -> list[float] | None:
            if not self.is_connected() or self.last_feedback is None:
                return None
            if time.monotonic() - self.last_feedback_time > max_age_s:
                return None
            return [float(a) for a in self.last_feedback["angles"]]

        def command_angles(self, angles_deg) -> None:
            """Lệnh góc dạng số thực (không làm tròn về độ nguyên như slider)."""

            angles = [float(a) for a in angles_deg]
            if not self.robot_model.within_limits_deg(angles, margin_deg=0.05):
                raise ValueError(f"Góc vượt giới hạn khớp: {np.round(angles, 1).tolist()}")
            self._auto_angles = angles
            for widgets in (self.sliders, self.spinboxes):
                for widget, value in zip(widgets, angles):
                    widget.blockSignals(True)
                    widget.setValue(int(round(value)))
                    widget.blockSignals(False)
            if self.sim:
                self.sim.update_joints(angles)
            tcp = self.robot_model.fk(np.radians(angles))
            x, y, z = tcp[:3, 3] * 1000.0
            self.ui.lbl_tcp_hud.setText(f"TCP: X={x:.1f} Y={y:.1f} Z={z:.1f} mm | AUTO")
            self.has_pending_tx = False
            if self.is_connected():
                self.comm.send_pc_command(angles, self.tool_state)
            self.command_changed.emit(angles)

        def set_tool(self, state: int) -> None:
            self.tool_state = int(state)
            if self.is_connected():
                self.comm.send_tool_action(int(state))
            self.tool_changed.emit(int(state))

        def emergency_stop(self) -> None:
            """Gửi CMD_STOP (dừng động cơ + tắt bơm). KHÔNG thay thế E-stop phần cứng."""

            self.has_pending_tx = False
            self.tool_state = 0
            if self.is_connected():
                self.comm.stop_robot()
            self.tool_changed.emit(0)
            self.log_message("🛑 STOP từ trạm phân loại: đã gửi CMD_STOP (dừng động cơ, tắt bơm).", "ERROR")

        def set_manual_lock(self, locked: bool) -> None:
            """Khóa thao tác tay khi bộ tuần tự đang chạy (nút STOP vẫn dùng được)."""

            if locked and not self._auto_locked:
                self._auto_angles = self.current_command()
            self._auto_locked = locked
            for widget in (
                self.ui.robot_tabs,
                self.ui.btn_tb_add_target,
                self.ui.btn_tb_movej,
                self.ui.btn_tb_movel,
                self.ui.btn_tb_run,
                self.ui.btn_tb_connect,
                self.ui.btn_tool_off,
                self.ui.btn_tool_suck,
                self.ui.btn_tool_release,
                self.ui.tree_station,
            ):
                widget.setEnabled(not locked)
            if not locked and getattr(self, "_auto_angles", None) is not None:
                # Không gọi sync_system_pose: tránh gửi lại góc slider đã làm tròn (lệch ≤0.5°)
                self.command_changed.emit(list(self._auto_angles))

        def refresh_view(self) -> None:
            if self.sim:
                self.sim.update_joints(self.current_command())

        def shutdown(self) -> None:
            self.timer.stop()
            self.tx_timer.stop()
            if self.is_connected():
                self.comm.stop_robot()
                self.comm.disconnect()
            if self.sim:
                self.sim.close()

    window = IntegratedRobotWindow()
    if parent is not None:
        window.setParent(parent, Qt.WindowType.Widget)
    return window

