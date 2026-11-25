"""Lập kế hoạch tư thế và bộ tuần tự phân loại (pick & place) — không chặn luồng GUI.

Chu trình cho một kiện (sau khi Vision ra quyết định):
  [băng tải FWD trong `conveyor_transfer_s` → STOP]  (đưa kiện từ camera tới điểm gắp)
  → trên điểm gắp → hạ xuống → HÚT → chờ → nhấc lên → trên thùng → hạ → NHẢ → chờ → TẮT
  → nhấc lên → về HOME.

An toàn (phần mềm — KHÔNG thay thế E-stop phần cứng):
- Mọi tư thế được giải IK và kiểm tra giới hạn khớp TRƯỚC khi robot di chuyển.
- Nối robot thật: bắt buộc đã "Lưu Gốc"; góc thực (telemetry) phải bám góc lệnh trong
  `settle_tolerance_deg` sau mỗi đoạn, quá `settle_timeout_s` → dừng khẩn và báo lỗi.
- Băng tải chỉ chạy khi đã ONLINE và có ACK; chờ ACK quá hạn → dừng khẩn.
- Lỗi/abort: gửi CMD_STOP robot (tắt bơm), STOP băng tải nếu chu trình đã cho chạy.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Protocol

import numpy as np
from PyQt6.QtCore import QObject, QTimer, pyqtSignal

from sorting_station.cell_state import CellState
from sorting_station.robot_model import RobotModel

DOWN = (0.0, 0.0, -1.0)


class PlanningError(RuntimeError):
    pass


# ---------------------------------------------------------------- cổng phần cứng
class RobotPort(Protocol):
    def is_connected(self) -> bool: ...
    def home_calibrated(self) -> bool: ...
    def current_command(self) -> list[float]: ...
    def actual_angles(self, max_age_s: float = 0.5) -> list[float] | None: ...
    def command_angles(self, angles_deg) -> None: ...
    def set_tool(self, state: int) -> None: ...
    def emergency_stop(self) -> None: ...
    def set_manual_lock(self, locked: bool) -> None: ...


class ConveyorPort(Protocol):
    @property
    def online(self) -> bool: ...
    @property
    def state(self) -> str: ...
    @property
    def busy(self) -> bool: ...
    def command(self, action: str) -> None: ...


class MachineVisionConveyor:
    """Bọc ConveyorPanel của MachineVision (không sửa mã gốc; dùng hàm gửi nội bộ `_send`)."""

    def __init__(self, panel) -> None:
        self.panel = panel

    @property
    def online(self) -> bool:
        return bool(self.panel._online)

    @property
    def state(self) -> str:
        return str(self.panel.state)

    @property
    def busy(self) -> bool:
        return bool(self.panel._workers)

    def command(self, action: str) -> None:
        self.panel._send(action)


# ---------------------------------------------------------------- cấu hình
@dataclass
class SortingSettings:
    joint_speed_deg_s: float = 30.0
    min_move_s: float = 0.4
    settle_tolerance_deg: float = 1.5
    settle_timeout_s: float = 4.0
    approach_height_mm: float = 60.0
    suck_dwell_s: float = 0.6
    release_dwell_s: float = 0.5
    conveyor_transfer_s: float = 2.5
    conveyor_ack_timeout_s: float = 2.0
    tick_ms: int = 40
    home_joints_deg: list[float] = field(default_factory=lambda: [0.0] * 6)
    pick_xyz_mm: list[float] = field(default_factory=lambda: [250.0, 0.0, 118.0])
    bins: dict[str, list[float]] = field(default_factory=dict)  # key -> drop_xyz_mm
    destination_map: dict[str, str] = field(default_factory=dict)
    decision_map: dict[str, str] = field(default_factory=dict)
    fallback_bin: str = "LOI"

    @staticmethod
    def from_config(section: dict) -> "SortingSettings":
        settings = SortingSettings()
        for name in (
            "joint_speed_deg_s", "min_move_s", "settle_tolerance_deg", "settle_timeout_s",
            "approach_height_mm", "suck_dwell_s", "release_dwell_s", "conveyor_transfer_s",
            "conveyor_ack_timeout_s",
        ):
            if name in section:
                setattr(settings, name, float(section[name]))
        if "tick_ms" in section:
            settings.tick_ms = int(section["tick_ms"])
        if "home_joints_deg" in section:
            settings.home_joints_deg = [float(v) for v in section["home_joints_deg"]]
        if "pick_xyz_mm" in section:
            settings.pick_xyz_mm = [float(v) for v in section["pick_xyz_mm"]]
        settings.bins = {
            str(key): [float(v) for v in spec["drop_xyz_mm"]]
            for key, spec in section.get("bins", {}).items()
        }
        settings.destination_map = {str(k): str(v) for k, v in section.get("destination_map", {}).items()}
        settings.decision_map = {str(k): str(v) for k, v in section.get("decision_map", {}).items()}
        settings.fallback_bin = str(section.get("fallback_bin", "LOI"))
        if settings.joint_speed_deg_s <= 0 or settings.joint_speed_deg_s > 120:
            raise ValueError("joint_speed_deg_s phải trong (0, 120] °/s")
        if settings.fallback_bin not in settings.bins:
            raise ValueError(f"fallback_bin '{settings.fallback_bin}' không có trong sorting.bins")
        return settings

    def resolve_bin(self, decision: str | None, destination: str | None) -> str:
        """Chọn thùng cho một kiện.

        - PASS: theo rổ đích Vision trả về (hub của đơn) qua `destination_map`.
        - Mọi quyết định khác: theo `decision_map`, KHÔNG xét rổ đích — kiện hỏng hay cần
          xem lại không được rơi vào thùng giao hàng dù đơn có hub hợp lệ.
        - Không khớp → `fallback_bin`.
        """

        if decision == "PASS":
            candidates = (self.destination_map.get(destination or ""), self.decision_map.get("PASS"))
        else:
            candidates = (self.decision_map.get(decision or ""),)
        for key in candidates:
            if key and key in self.bins:
                return key
        return self.fallback_bin


# ---------------------------------------------------------------- tư thế
class PoseLibrary:
    """Tư thế dạng tên → góc khớp. Nguồn: tư thế đã DẠY (ưu tiên) hoặc IK từ tọa độ cấu hình."""

    def __init__(self, model: RobotModel, settings: SortingSettings, taught_path: Path | None) -> None:
        self.model = model
        self.settings = settings
        self.taught_path = taught_path
        self.taught: dict[str, list[float]] = {}
        if taught_path is not None and taught_path.is_file():
            data = json.loads(taught_path.read_text(encoding="utf-8"))
            self.taught = {str(k): [float(v) for v in vals] for k, vals in data.items()}

    def base_names(self) -> list[str]:
        return ["home", "pick"] + [f"bin:{key}" for key in self.settings.bins]

    def teach(self, name: str, joints_deg) -> None:
        values = [float(v) for v in joints_deg]
        for index, angle in self.model.locked.items():   # J2 khóa: telemetry có thể lệch vài phần độ
            values[index] = angle
        if not self.model.within_limits_deg(values):
            raise PlanningError("Tư thế dạy vượt giới hạn khớp")
        self.taught[name] = values
        self._save()

    def forget(self, name: str) -> None:
        self.taught.pop(name, None)
        self._save()

    def _save(self) -> None:
        if self.taught_path is None:
            return
        self.taught_path.parent.mkdir(parents=True, exist_ok=True)
        self.taught_path.write_text(json.dumps(self.taught, indent=2, ensure_ascii=False), encoding="utf-8")

    def source(self, name: str) -> str:
        return "DẠY" if name in self.taught else ("CỐ ĐỊNH" if name == "home" else "IK")

    def target_xyz_mm(self, name: str) -> list[float] | None:
        if name == "pick":
            return list(self.settings.pick_xyz_mm)
        if name.startswith("bin:"):
            return list(self.settings.bins[name[4:]])
        return None

    def resolve(self, name: str, seed_deg) -> list[float]:
        if name in self.taught:
            return list(self.taught[name])
        if name == "home":
            home = list(self.settings.home_joints_deg)
            for index, angle in self.model.locked.items():   # khớp khóa: HOME giữ đúng góc khóa
                home[index] = angle
            return home
        xyz = self.target_xyz_mm(name)
        if xyz is None:
            raise PlanningError(f"Không có tư thế '{name}'")
        q, info = self.model.solve_ik(np.asarray(xyz) / 1000.0, approach=DOWN, seed_deg=seed_deg)
        if q is None:
            raise PlanningError(f"'{name}' {xyz} mm: {info.get('reason', 'IK thất bại')}")
        return [float(v) for v in q]

    def above(self, name: str, base_joints, seed_deg) -> list[float]:
        """Tư thế cao hơn `approach_height_mm` theo phương Z, giữ hướng dụng cụ của tư thế gốc."""

        tcp = self.model.fk_deg(base_joints)
        target = tcp[:3, 3] + np.array([0.0, 0.0, self.settings.approach_height_mm / 1000.0])
        q, info = self.model.solve_ik(target, approach=tcp[:3, 2], seed_deg=seed_deg)
        if q is None:
            raise PlanningError(f"Điểm tiếp cận trên '{name}': {info.get('reason', 'IK thất bại')}")
        return [float(v) for v in q]

    def plan_cycle(self, bin_key: str, start_deg) -> list[tuple[str, list[float]]]:
        if bin_key not in self.settings.bins:
            raise PlanningError(f"Không có thùng '{bin_key}'")
        pick = self.resolve("pick", start_deg)
        pick_above = self.above("pick", pick, pick)
        drop = self.resolve(f"bin:{bin_key}", pick_above)
        # 24/11/2026: J2 khóa → điểm thả DẠY TAY ở rìa tầm với có thể không lên thêm được approach_height_mm.
        # Khi đó thả luôn tại tư thế đã dạy (bỏ bước hạ/nhấc ở thùng) thay vì hủy cả chu trình.
        try:
            drop_above = self.above(f"bin:{bin_key}", drop, drop)
        except PlanningError:
            if f"bin:{bin_key}" not in self.taught:
                raise
            drop_above = list(drop)
        home = self.resolve("home", drop_above)
        return [
            ("trên điểm gắp", pick_above),
            ("hạ xuống gắp", pick),
            ("nhấc kiện", pick_above),
            (f"trên thùng {bin_key}", drop_above),
            (f"hạ vào thùng {bin_key}", drop),
            (f"rời thùng {bin_key}", drop_above),
            ("về HOME", home),
        ]

    def validate_all(self, start_deg) -> list[tuple[str, str]]:
        """Kiểm tra tầm với cho mọi thùng — dùng cho nút 'Kiểm tra IK' trên GUI."""

        report = []
        for key in self.settings.bins:
            try:
                self.plan_cycle(key, start_deg)
                report.append((key, "OK"))
            except PlanningError as exc:
                report.append((key, f"LỖI: {exc}"))
        return report


# ---------------------------------------------------------------- bộ tuần tự
@dataclass
class _Step:
    kind: str  # move | tool | wait | conveyor | attach | drop | parcel_at_pick
    text: str
    joints: list[float] | None = None
    value: object = None
    duration: float = 0.0


class SortingSequencer(QObject):
    state_changed = pyqtSignal(str)  # IDLE | RUNNING | DONE | FAULT
    message = pyqtSignal(str)
    progress = pyqtSignal(int, int, str)

    def __init__(
        self,
        robot: RobotPort,
        conveyor: ConveyorPort | None,
        poses: PoseLibrary,
        cell: CellState,
        settings: SortingSettings,
        clock: Callable[[], float] = time.monotonic,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.robot = robot
        self.conveyor = conveyor
        self.poses = poses
        self.cell = cell
        self.settings = settings
        self.clock = clock
        self.state = "IDLE"
        self._steps: list[_Step] = []
        self._index = 0
        self._step_started = 0.0
        self._move_from: np.ndarray | None = None
        self._move_duration = 0.0
        self._sent = False
        self._conveyor_commanded = False
        self._bin_key: str | None = None
        self._timer = QTimer(self)
        self._timer.setInterval(settings.tick_ms)
        self._timer.timeout.connect(self.tick)

    @property
    def running(self) -> bool:
        return self.state == "RUNNING"

    def simulation_mode(self) -> bool:
        return not self.robot.is_connected()

    # -------------------------------------------------------- khởi động
    def start_cycle(self, bin_key: str, at_pick: bool = False) -> bool:
        """``at_pick``: the parcel already waits at the pick point (MachineVision camera
        cycle stopped it on the pick sensor), so no timed conveyor transfer."""

        def build(start):
            plan = self.poses.plan_cycle(bin_key, start)
            steps: list[_Step] = []
            if self.settings.conveyor_transfer_s > 0 and not at_pick:
                steps += self._conveyor_steps()
            else:
                steps.append(_Step("parcel_at_pick", "kiện tại điểm gắp"))
            (above_pick, q_above), (pick_txt, q_pick), (lift_txt, q_lift) = plan[0], plan[1], plan[2]
            steps += [
                _Step("move", above_pick, q_above),
                _Step("move", pick_txt, q_pick),
                _Step("tool", "bật HÚT", value=1),
                _Step("wait", "chờ giữ chân không", duration=self.settings.suck_dwell_s),
                _Step("attach", "kiện bám đầu hút"),
                _Step("move", lift_txt, q_lift),
            ]
            for text, joints in plan[3:5]:
                steps.append(_Step("move", text, joints))
            steps += [
                _Step("tool", "NHẢ kiện", value=2),
                _Step("wait", "chờ nhả", duration=self.settings.release_dwell_s),
                _Step("drop", f"kiện vào thùng {bin_key}", value=bin_key),
                _Step("tool", "tắt bơm/van", value=0),
            ]
            for text, joints in plan[5:]:
                steps.append(_Step("move", text, joints))
            return steps

        self._bin_key = bin_key
        self._at_pick = at_pick
        try:
            return self._start(build, f"PHÂN LOẠI → {bin_key}")
        finally:
            self._at_pick = False

    def start_move_to(self, pose_name: str) -> bool:
        def build(start):
            joints = self.poses.resolve(pose_name, start)
            return [_Step("move", f"tới '{pose_name}'", joints)]

        self._bin_key = None
        return self._start(build, f"DI CHUYỂN → {pose_name}")

    def _conveyor_steps(self) -> list[_Step]:
        travel = _Step("wait", "băng tải đưa kiện tới điểm gắp", duration=self.settings.conveyor_transfer_s,
                       value="transfer")
        if self.conveyor is not None and self.conveyor.online:
            steps = []
            if self.conveyor.state != "STOP":
                steps.append(_Step("conveyor", "xác nhận băng tải DỪNG", value="STOP"))
            return steps + [
                _Step("conveyor", "băng tải CHẠY TIẾN", value="FWD"),
                travel,
                _Step("conveyor", "băng tải DỪNG", value="STOP"),
            ]
        return [travel]  # chỉ mô phỏng (đã kiểm tra điều kiện ở _start)

    def _start(self, build, title: str) -> bool:
        if self.running:
            self.message.emit("Chu trình đang chạy — bỏ qua yêu cầu mới")
            return False
        real = self.robot.is_connected()
        if real and not self.robot.home_calibrated():
            return self._refuse("Robot chưa LƯU GỐC — không cho chạy tự động")
        if (
            real
            and self.settings.conveyor_transfer_s > 0
            and not getattr(self, "_at_pick", False)
            and (self.conveyor is None or not self.conveyor.online)
        ):
            return self._refuse("Băng tải chưa ONLINE (PING/ACK) — không thể chuyển kiện tới điểm gắp")
        start = list(self.robot.current_command())
        if real:
            actual = self.robot.actual_angles()
            if actual is None:
                return self._refuse("Chưa có telemetry robot mới — kiểm tra kết nối")
            if np.max(np.abs(np.asarray(actual) - np.asarray(start))) > self.settings.settle_tolerance_deg * 2:
                return self._refuse("Góc thực của robot chưa khớp góc lệnh — đợi robot dừng hẳn")
        try:
            self._steps = build(start)
        except Exception as exc:  # PlanningError, ValueError
            return self._refuse(f"Lập kế hoạch thất bại: {exc}")
        self._index = 0
        self._conveyor_commanded = False
        self._enter_step()
        self.state = "RUNNING"
        self.robot.set_manual_lock(True)
        mode = "MÔ PHỎNG" if not real else "ROBOT THẬT"
        self.cell.set_sequence_text(f"{title} ({mode})")
        self.state_changed.emit(self.state)
        self.message.emit(f"▶ {title} — {mode}, {len(self._steps)} bước")
        self._timer.start()
        return True

    def _refuse(self, reason: str) -> bool:
        self.message.emit(f"⛔ {reason}")
        self.cell.set_sequence_text(f"TỪ CHỐI: {reason}")
        self.state_changed.emit("FAULT")
        return False

    # -------------------------------------------------------- vòng lặp
    def _enter_step(self) -> None:
        self._step_started = self.clock()
        self._sent = False
        if self._index >= len(self._steps):
            return
        step = self._steps[self._index]
        if step.kind == "move":
            self._move_from = np.asarray(self.robot.current_command(), dtype=float)
            delta = np.abs(np.asarray(step.joints) - self._move_from)
            self._move_duration = max(float(delta.max()) / self.settings.joint_speed_deg_s,
                                      self.settings.min_move_s)
        self.progress.emit(self._index + 1, len(self._steps), step.text)
        self.cell.set_sequence_text(f"[{self._index + 1}/{len(self._steps)}] {step.text}")

    def _next(self) -> None:
        self._index += 1
        if self._index >= len(self._steps):
            self._finish()
            return
        self._enter_step()

    def tick(self) -> None:
        if not self.running:
            return
        try:
            done = self._run_step(self._steps[self._index], self.clock() - self._step_started)
        except Exception as exc:  # lỗi gửi lệnh / vượt giới hạn → dừng an toàn
            self.abort(f"Lỗi khi thực thi: {exc}")
            return
        if done:
            self._next()

    def _run_step(self, step: _Step, elapsed: float) -> bool:
        if step.kind == "move":
            return self._run_move(step, elapsed)
        if step.kind == "tool":
            self.robot.set_tool(int(step.value))
            self.cell.set_tool(int(step.value))
            return True
        if step.kind == "wait":
            if step.value == "transfer":
                lay = self.cell.layout
                ratio = min(elapsed / max(step.duration, 1e-6), 1.0)
                y = lay.camera_y + (lay.pick_y - lay.camera_y) * ratio
                self.cell.place_parcel(lay.parcel_on_belt(y))
            return elapsed >= step.duration
        if step.kind == "parcel_at_pick":
            lay = self.cell.layout
            self.cell.place_parcel(lay.parcel_on_belt(lay.pick_y))
            return True
        if step.kind == "attach":
            self.cell.attach_parcel(True)
            return True
        if step.kind == "drop":
            self.cell.drop_parcel_in_bin(str(step.value))
            return True
        if step.kind == "conveyor":
            return self._run_conveyor(str(step.value), elapsed)
        raise ValueError(f"Bước không hợp lệ: {step.kind}")

    def _run_move(self, step: _Step, elapsed: float) -> bool:
        target = np.asarray(step.joints, dtype=float)
        ratio = min(elapsed / self._move_duration, 1.0)
        smooth = ratio * ratio * (3.0 - 2.0 * ratio)  # vận tốc = 0 ở hai đầu đoạn
        if not self._sent:
            self.robot.command_angles((self._move_from + (target - self._move_from) * smooth).tolist())
            if ratio >= 1.0:
                self._sent = True  # đã gửi điểm cuối
        if ratio < 1.0:
            return False
        if self.simulation_mode():
            return True
        actual = self.robot.actual_angles()
        if actual is not None and np.max(np.abs(np.asarray(actual) - target)) <= self.settings.settle_tolerance_deg:
            return True
        if elapsed > self._move_duration + self.settings.settle_timeout_s:
            raise RuntimeError(
                f"Robot không đạt '{step.text}' sau {self.settings.settle_timeout_s:.1f}s "
                f"(telemetry: {None if actual is None else np.round(actual, 1).tolist()})"
            )
        return False

    def _run_conveyor(self, action: str, elapsed: float) -> bool:
        if self.conveyor is None or not self.conveyor.online:
            raise RuntimeError("Mất kết nối băng tải (không có ACK/PING)")
        if not self._sent:
            if self.conveyor.busy:  # đang có PING heartbeat — gửi ở tick sau
                if elapsed > self.settings.conveyor_ack_timeout_s:
                    raise RuntimeError("Băng tải bận quá lâu, không gửi được lệnh")
                return False
            self.conveyor.command(action)
            self._sent = True
            self._step_started = self.clock()
            if action == "FWD":
                self._conveyor_commanded = True
            return False
        if self.conveyor.state == action:
            if action == "STOP":
                self._conveyor_commanded = False
            return True
        if elapsed > self.settings.conveyor_ack_timeout_s:
            raise RuntimeError(f"Không nhận ACK {action} từ băng tải sau {self.settings.conveyor_ack_timeout_s:.1f}s")
        return False

    # -------------------------------------------------------- kết thúc
    def _finish(self) -> None:
        self._timer.stop()
        self.state = "DONE"
        self.robot.set_manual_lock(False)
        text = f"HOÀN TẤT → thùng {self._bin_key}" if self._bin_key else "HOÀN TẤT di chuyển"
        self.cell.set_sequence_text(text)
        self.message.emit(f"✔ {text}")
        self.state_changed.emit(self.state)

    def abort(self, reason: str = "Người vận hành dừng") -> None:
        was_running = self.running
        self._timer.stop()
        self.robot.emergency_stop()
        self.cell.set_tool(0)
        if self._conveyor_commanded and self.conveyor is not None:
            try:
                self.conveyor.command("STOP")
            except Exception:
                pass
            self._conveyor_commanded = False
        if self.cell.parcel_attached:
            self.cell.attach_parcel(False)
            tcp = self.poses.model.fk_deg(self.robot.current_command())
            self.cell.place_parcel(tcp[:3, 3])
        self.state = "FAULT"
        if was_running:
            self.robot.set_manual_lock(False)
        self.cell.set_sequence_text(f"DỪNG: {reason}")
        self.message.emit(f"🛑 {reason}")
        self.state_changed.emit(self.state)
