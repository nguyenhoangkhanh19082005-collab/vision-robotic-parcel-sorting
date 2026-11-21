"""Trạng thái dùng chung của cả trạm — nguồn dữ liệu duy nhất cho mọi view digital twin.

Các nguồn ghi vào:
- MachineVision: giai đoạn xử lý (stage), quyết định, lệnh băng tải đã được ACK.
- Robot: góc lệnh (slider / bộ tuần tự) và góc thực từ telemetry ESP32-S3, trạng thái đầu hút.
- Bộ tuần tự phân loại: vị trí kiện, gắn/thả kiện, bộ đếm từng thùng.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
from PyQt6.QtCore import QObject, pyqtSignal

from machine_vision.ui.digital_twin import ProcessStage
from sorting_station.conveyor_model import ConveyorSpec

CONVEYOR_STATES = {"UNKNOWN", "STOP", "FWD", "REV"}


@dataclass(frozen=True)
class BinLayout:
    key: str
    label: str
    color: str
    center: tuple[float, float, float]  # tâm đáy thùng (m, hệ base robot)
    size: tuple[float, float, float]


@dataclass(frozen=True)
class CellLayout:
    """Bố trí trạm theo hệ base robot (m). Giá trị mặc định là ước lượng — cần đo lại tại hiện trường."""

    conveyor_center_x: float = 0.25
    conveyor_start_y: float = -0.65
    conveyor_end_y: float = 0.25
    conveyor_width: float = 0.20
    belt_top_z: float = 0.06
    camera_y: float = -0.40
    camera_height_above_belt: float = 0.40
    pick_y: float = 0.0
    parcel_size: tuple[float, float, float] = (0.15, 0.10, 0.06)
    bins: tuple[BinLayout, ...] = field(default_factory=tuple)
    conveyor_base_z: float = 0.0
    camera_show_mount: bool = False  # giá camera chưa in → chỉ vẽ khối camera + vùng nhìn
    conveyor_spec: ConveyorSpec = field(default_factory=ConveyorSpec)

    @staticmethod
    def from_config(section: dict) -> "CellLayout":
        conveyor = section.get("conveyor", {})
        camera = section.get("camera", {})
        if "model" in conveyor:
            # Kích thước băng tải lấy từ mô hình thật; chỉ cần cấu hình vị trí đặt (tâm, cao độ mặt đặt)
            conveyor_spec = ConveyorSpec.from_config(conveyor["model"])
            center_x, center_y = (float(v) for v in conveyor.get("center", (0.25, -0.10)))
            base_z = float(conveyor.get("base_z", 0.0))
            conveyor = {
                "center_x": center_x,
                "start_y": center_y - conveyor_spec.length_total / 2000.0,
                "end_y": center_y + conveyor_spec.length_total / 2000.0,
                "width": conveyor_spec.belt_width / 1000.0,
                "belt_top_z": base_z + conveyor_spec.belt_top_height / 1000.0,
            }
        else:
            conveyor_spec, base_z = ConveyorSpec(), float(conveyor.get("belt_top_z", 0.06)) - ConveyorSpec().belt_top_height / 1000.0
        bins = []
        for key, spec in section.get("bins", {}).items():
            bins.append(
                BinLayout(
                    key=str(key),
                    label=str(spec.get("label", key)),
                    color=str(spec.get("color", "#8b9aab")),
                    center=tuple(float(v) for v in spec.get("center", (0.0, 0.0, 0.0))),
                    size=tuple(float(v) for v in spec.get("size", (0.16, 0.14, 0.08))),
                )
            )
        return CellLayout(
            conveyor_center_x=float(conveyor.get("center_x", 0.25)),
            conveyor_start_y=float(conveyor.get("start_y", -0.65)),
            conveyor_end_y=float(conveyor.get("end_y", 0.25)),
            conveyor_width=float(conveyor.get("width", 0.20)),
            belt_top_z=float(conveyor.get("belt_top_z", 0.06)),
            camera_y=float(camera.get("y", -0.40)),
            camera_height_above_belt=float(camera.get("height_above_belt", 0.40)),
            pick_y=float(section.get("pick_y", 0.0)),
            parcel_size=tuple(float(v) for v in section.get("parcel_size", (0.15, 0.10, 0.06))),
            bins=tuple(bins),
            conveyor_base_z=base_z,
            camera_show_mount=bool(camera.get("show_mount", False)),
            conveyor_spec=conveyor_spec,
        )

    def parcel_on_belt(self, y: float) -> tuple[float, float, float]:
        """Tâm kiện khi nằm trên băng tại tọa độ y."""

        return (self.conveyor_center_x, y, self.belt_top_z + self.parcel_size[2] / 2.0)

    def bin_by_key(self, key: str) -> BinLayout | None:
        for spec in self.bins:
            if spec.key == key:
                return spec
        return None


class CellState(QObject):
    changed = pyqtSignal()

    def __init__(self, layout: CellLayout, telemetry_timeout_s: float = 0.5, parent=None) -> None:
        super().__init__(parent)
        self.layout = layout
        self.telemetry_timeout_s = float(telemetry_timeout_s)
        self.stage = ProcessStage.IDLE
        self.decision: str | None = None
        self.conveyor_state = "UNKNOWN"
        self.conveyor_speed = 0
        self.robot_command_deg = [0.0] * 6
        self.robot_actual_deg: list[float] | None = None
        self._actual_time = 0.0
        self.robot_link = "OFFLINE"  # OFFLINE | ONLINE (đang nối phần cứng)
        self.tool_state = 0  # 0 OFF, 1 SUCK, 2 RELEASE
        self.sequence_text = "SẴN SÀNG"
        self.parcel_visible = False
        self.parcel_position = layout.parcel_on_belt(layout.camera_y)
        self.parcel_attached = False
        self.parcel_bin: str | None = None
        self.bin_counts: dict[str, int] = {spec.key: 0 for spec in layout.bins}

    # -- Vision / băng tải ----------------------------------------------------
    def set_process(self, stage: ProcessStage, decision: str | None = None) -> None:
        self.stage = stage
        self.decision = decision
        if stage in (ProcessStage.DETECT, ProcessStage.VISION) and not self.parcel_attached:
            self.parcel_visible = True
            self.parcel_bin = None
            self.parcel_position = self.layout.parcel_on_belt(self.layout.camera_y)
        elif stage == ProcessStage.IDLE and not self.parcel_attached:
            self.parcel_visible = False
        self.changed.emit()

    def set_conveyor(self, state: str, speed: int | None = None) -> None:
        if state not in CONVEYOR_STATES:
            raise ValueError(f"Trạng thái băng tải không hợp lệ: {state}")
        self.conveyor_state = state
        if speed is not None:
            self.conveyor_speed = int(speed)
        self.changed.emit()

    # -- Robot ------------------------------------------------------------------
    def set_robot_command(self, angles_deg) -> None:
        values = [float(v) for v in angles_deg]
        if len(values) != 6:
            raise ValueError("Cần đúng 6 góc khớp")
        self.robot_command_deg = values
        self.changed.emit()

    def set_robot_actual(self, angles_deg, timestamp: float | None = None) -> None:
        values = [float(v) for v in angles_deg]
        if len(values) != 6 or not all(np.isfinite(values)):
            return
        self.robot_actual_deg = values
        self._actual_time = time.monotonic() if timestamp is None else timestamp
        self.changed.emit()

    def set_robot_link(self, link: str) -> None:
        if link != self.robot_link:
            self.robot_link = link
            if link != "ONLINE":
                self.robot_actual_deg = None
            self.changed.emit()

    def actual_is_fresh(self, now: float | None = None) -> bool:
        if self.robot_actual_deg is None or self.robot_link != "ONLINE":
            return False
        now = time.monotonic() if now is None else now
        return now - self._actual_time <= self.telemetry_timeout_s

    def robot_display_deg(self) -> list[float]:
        """Twin hiển thị góc THỰC khi telemetry còn mới, ngược lại hiển thị góc LỆNH."""

        if self.actual_is_fresh():
            return list(self.robot_actual_deg or self.robot_command_deg)
        return list(self.robot_command_deg)

    def set_tool(self, state: int) -> None:
        self.tool_state = int(state)
        self.changed.emit()

    # -- Kiện hàng / bộ tuần tự --------------------------------------------------
    def set_sequence_text(self, text: str) -> None:
        self.sequence_text = text
        self.changed.emit()

    def place_parcel(self, position, visible: bool = True) -> None:
        self.parcel_position = tuple(float(v) for v in position)
        self.parcel_visible = visible
        self.changed.emit()

    def attach_parcel(self, attached: bool) -> None:
        self.parcel_attached = attached
        self.changed.emit()

    def drop_parcel_in_bin(self, key: str) -> None:
        spec = self.layout.bin_by_key(key)
        self.parcel_attached = False
        self.parcel_bin = key
        if spec is not None:
            cx, cy, cz = spec.center
            self.parcel_position = (cx, cy, cz + self.layout.parcel_size[2] / 2.0 + 0.01)
        self.bin_counts[key] = self.bin_counts.get(key, 0) + 1
        self.changed.emit()
