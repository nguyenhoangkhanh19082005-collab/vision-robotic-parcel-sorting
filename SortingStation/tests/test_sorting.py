import numpy as np
import pytest


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeRobot:
    def __init__(self, connected=False, calibrated=True, lag=False):
        self.connected = connected
        self.calibrated = calibrated
        self.lag = lag
        self.command = [0.0] * 6
        self.tool_history = []
        self.stops = 0
        self.locked = False
        self.sent = []

    def is_connected(self):
        return self.connected

    def home_calibrated(self):
        return self.calibrated

    def current_command(self):
        return list(self.command)

    def actual_angles(self, max_age_s=0.5):
        if not self.connected:
            return None
        return [0.0] * 6 if self.lag else list(self.command)

    def command_angles(self, angles):
        self.command = [float(a) for a in angles]
        self.sent.append(self.command)

    def set_tool(self, state):
        self.tool_history.append(state)

    def emergency_stop(self):
        self.stops += 1

    def set_manual_lock(self, locked):
        self.locked = locked


class FakeConveyor:
    def __init__(self, online=True):
        self.online = online
        self.state = "STOP"
        self.busy = False
        self.commands = []
        self.ack = True

    def command(self, action):
        self.commands.append(action)
        if self.ack:
            self.state = action


def _build(qt_app, model, station_config, robot, conveyor, transfer=None, tmp_path=None):
    from sorting_station.cell_state import CellLayout, CellState
    from sorting_station.sorting import PoseLibrary, SortingSequencer, SortingSettings

    settings = SortingSettings.from_config(station_config["sorting"])
    if transfer is not None:
        settings.conveyor_transfer_s = transfer
    cell = CellState(CellLayout.from_config(station_config["layout"]))
    poses = PoseLibrary(model, settings, None if tmp_path is None else tmp_path / "taught.json")
    clock = FakeClock()
    seq = SortingSequencer(robot, conveyor, poses, cell, settings, clock=clock)
    return seq, cell, clock, settings


def _run(seq, clock, limit=5000):
    for _ in range(limit):
        if not seq.running:
            return
        clock.now += 0.04
        seq.tick()
    raise AssertionError("Chu trình không kết thúc")


def test_all_configured_bins_reachable(qt_app, model, station_config):
    seq, *_ = _build(qt_app, model, station_config, FakeRobot(), FakeConveyor())
    report = seq.poses.validate_all([0.0] * 6)
    assert all(status == "OK" for _key, status in report), report


def test_resolve_bin_mapping(station_config):
    from sorting_station.sorting import SortingSettings

    settings = SortingSettings.from_config(station_config["sorting"])
    # Kiện đạt: theo hub của đơn (orders.destination_chute_id)
    for hub in ("1", "3", "5"):
        assert settings.resolve_bin("PASS", hub) == "DONG_BANG"
    for hub in ("2", "4"):
        assert settings.resolve_bin("PASS", hub) == "TRUNG_DU"
    assert settings.resolve_bin("PASS", "0") == "LOI"
    assert settings.resolve_bin("PASS", "LOCAL_TEST") == "DONG_BANG"
    assert settings.resolve_bin("PASS", "KHO_HN_99") == "LOI"  # hub lạ → không đoán vùng
    assert settings.resolve_bin("PASS", None) == "LOI"
    # Kiện hỏng / cần xem lại: luôn vào thùng lỗi, kể cả khi đơn có hub hợp lệ
    assert settings.resolve_bin("REJECT", "DEFECT") == "LOI"
    assert settings.resolve_bin("REJECT", "1") == "LOI"
    assert settings.resolve_bin("MANUAL_REVIEW", "MANUAL_REVIEW") == "LOI"
    assert settings.resolve_bin("MANUAL_REVIEW", "4") == "LOI"
    assert settings.resolve_bin(None, None) == "LOI"


def test_simulated_cycle_completes_and_counts_bin(qt_app, model, station_config):
    robot, conveyor = FakeRobot(), FakeConveyor(online=False)
    seq, cell, clock, _ = _build(qt_app, model, station_config, robot, conveyor)
    assert seq.start_cycle("LOI")
    assert robot.locked
    _run(seq, clock)
    assert seq.state == "DONE"
    assert robot.tool_history == [1, 2, 0]
    assert cell.bin_counts["LOI"] == 1
    assert not robot.locked
    assert np.allclose(robot.command, [0.0] * 6, atol=1e-6)  # về HOME
    # Nội suy không nhảy cóc: mỗi tick ≤ tốc độ * chu kỳ (+ sai số)
    steps = np.abs(np.diff(np.asarray(robot.sent), axis=0)).max()
    assert steps <= 30.0 * 0.04 * 1.6


def test_real_cycle_drives_conveyor_with_ack(qt_app, model, station_config):
    robot, conveyor = FakeRobot(connected=True), FakeConveyor(online=True)
    seq, cell, clock, _ = _build(qt_app, model, station_config, robot, conveyor)
    assert seq.start_cycle("DONG_BANG")
    _run(seq, clock)
    assert seq.state == "DONE"
    assert conveyor.commands == ["FWD", "STOP"]
    assert cell.bin_counts["DONG_BANG"] == 1


def test_real_robot_requires_home_calibration(qt_app, model, station_config):
    robot = FakeRobot(connected=True, calibrated=False)
    seq, *_ = _build(qt_app, model, station_config, robot, FakeConveyor())
    assert not seq.start_cycle("DONG_BANG")
    assert robot.sent == []


def test_real_robot_requires_online_conveyor_when_transfer_enabled(qt_app, model, station_config):
    robot = FakeRobot(connected=True)
    seq, *_ = _build(qt_app, model, station_config, robot, FakeConveyor(online=False))
    assert not seq.start_cycle("DONG_BANG")


def test_missing_conveyor_ack_aborts_and_stops(qt_app, model, station_config):
    robot, conveyor = FakeRobot(connected=True), FakeConveyor(online=True)
    conveyor.ack = False
    seq, _cell, clock, _ = _build(qt_app, model, station_config, robot, conveyor)
    assert seq.start_cycle("DONG_BANG")
    _run(seq, clock)
    assert seq.state == "FAULT"
    assert robot.stops == 1


def test_robot_not_following_telemetry_triggers_fault(qt_app, model, station_config):
    robot = FakeRobot(connected=True, lag=True)
    seq, _cell, clock, _ = _build(qt_app, model, station_config, robot, FakeConveyor(), transfer=0.0)
    robot.lag = False  # điều kiện khởi động: góc thực khớp góc lệnh
    assert seq.start_cycle("DONG_BANG")
    robot.lag = True
    _run(seq, clock)
    assert seq.state == "FAULT"
    assert robot.stops == 1
    assert not robot.locked


def test_abort_releases_lock_and_stops(qt_app, model, station_config):
    robot = FakeRobot()
    seq, _cell, clock, _ = _build(qt_app, model, station_config, robot, FakeConveyor(online=False))
    assert seq.start_cycle("DONG_BANG")
    clock.now += 0.04
    seq.tick()
    seq.abort("test")
    assert seq.state == "FAULT"
    assert robot.stops == 1
    assert not robot.locked


def test_taught_pose_overrides_ik_and_persists(qt_app, model, station_config, tmp_path):
    from sorting_station.sorting import PoseLibrary

    seq, *_ = _build(qt_app, model, station_config, FakeRobot(), FakeConveyor(), tmp_path=tmp_path)
    taught = [5.0, 0.0, 20.0, -10.0, 15.0, 0.0]   # 24/11/2026: trong giới hạn tư thế 0 chữ L (J5 ≥ -20°)
    seq.poses.teach("pick", taught)
    assert seq.poses.resolve("pick", [0] * 6) == taught
    again = PoseLibrary(model, seq.settings, tmp_path / "taught.json")
    assert again.resolve("pick", [0] * 6) == taught
    with pytest.raises(Exception):
        seq.poses.teach("pick", [500, 0, 0, 0, 0, 0])
