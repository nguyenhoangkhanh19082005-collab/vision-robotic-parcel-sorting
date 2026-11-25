import numpy as np
import pytest


def test_model_matches_robot_simulation_transforms(paths):
    """Twin trong GUI robot (RobotSimulationWidget) và RobotModel (tỉ lệ CAD) cho cùng ma trận khâu."""
    import robot_simulation as rs

    from sorting_station.robot_model import build_model_from_robot_project

    model = build_model_from_robot_project(paths.robot_root, scale=1.0)

    widget = rs.RobotSimulationWidget.__new__(rs.RobotSimulationWidget)
    widget._parser = None
    q = np.radians([20, -30, 40, 10, -25, 60])
    widget._joint_angles = list(q)
    sim = widget._compute_link_transforms()
    mine = model.link_transforms_cad(q)
    names = ["link_1_1", "link_2_1", "link_3_1", "link_4_1", "link_5_1", "link_6_1"]
    for index, name in enumerate(names):
        assert np.allclose(sim[name], mine[index], atol=1e-12)


def test_home_pose_is_real_home(model):
    """24/11/2026 (lần 3): tư thế 0 = HOME THẬT: J2 cúi trước 12.5°, cẳng tay chúc 25°, đầu hút gần thẳng xuống (≈3°)."""
    tcp = model.fk(np.zeros(6))
    assert 0.23 < tcp[0, 3] < 0.27    # ≈ 252 mm × scale phía trước (băng tải)
    assert abs(tcp[1, 3]) < 0.01
    assert 0.05 < tcp[2, 3] < 0.10    # ≈ 77 mm × scale, cao hơn mặt băng (39 mm)
    assert tcp[2, 2] < -0.99          # đầu hút chỉ gần thẳng xuống
    # Tư thế dựng thẳng đứng = chữ L (J2 −12.5, J3 −12.5, J5 −22, J6 +124) rồi J3 −90°, J5 +90°
    up = model.fk(np.radians([0, -12.5, -102.5, 0, 68, 124]))
    assert up[2, 3] > 0.6
    assert np.hypot(up[0, 3], up[1, 3]) < 0.01
    assert up[2, 2] > 0.99


def test_jacobian_matches_finite_difference(model):
    q = np.radians([15, 20, -35, 30, 40, -60])
    jac = model.jacobian(q)
    eps = 1e-6
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = eps
        numeric = (model.fk(q + dq)[:3, 3] - model.fk(q - dq)[:3, 3]) / (2 * eps)
        assert np.allclose(jac[:3, i], numeric, atol=1e-7)


def test_full_pose_ik_roundtrip(model):
    rng = np.random.default_rng(7)
    for _ in range(25):
        q = np.array([rng.uniform(lo * 0.8, hi * 0.8) for lo, hi in model.joint_limits])
        q[5] = rng.uniform(-170, 170)
        target = model.fk_deg(q)
        sol, info = model.solve_ik(target[:3, 3], rotation=target[:3, :3], seed_deg=q + rng.normal(0, 10, 6))
        assert sol is not None, info
        reached = model.fk_deg(sol)
        assert np.linalg.norm(reached[:3, 3] - target[:3, 3]) < 1e-3
        assert model.within_limits_deg(sol)


def test_ik_pieper_compatibility(model):
    target = np.eye(4)
    target[:3, 3] = [0.25, 0.0, 0.2]
    target[:3, :3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    q_rad, ok = model.ik_pieper(target, [0] * 6)
    assert ok
    assert np.allclose(model.fk(q_rad)[:3, 3], target[:3, 3], atol=1e-3)


def test_unreachable_target_is_rejected(model):
    sol, info = model.solve_ik([0.9, 0.0, 0.2], approach=[0, 0, -1])
    assert sol is None
    assert info["ok"] is False


def test_robot_scale_scales_geometry_consistently(paths):
    """robot.scale nhân đều mọi kích thước: TCP ở cùng tư thế nhân đúng hệ số (dụng cụ = 0)."""
    from sorting_station.robot_model import build_model_from_robot_project

    base = build_model_from_robot_project(paths.robot_root, scale=1.0)
    small = build_model_from_robot_project(paths.robot_root, scale=0.8)
    assert small.mesh_scale == 0.8
    for q in ([0, 0, 0, 0, 0, 0], [30, 20, -40, 15, 35, -60]):
        assert np.allclose(small.fk_deg(q)[:3, 3], 0.8 * base.fk_deg(q)[:3, 3], atol=1e-9)
        assert np.allclose(small.fk_deg(q)[:3, :3], base.fk_deg(q)[:3, :3], atol=1e-9)
    with pytest.raises(ValueError):
        build_model_from_robot_project(paths.robot_root, scale=0.0)
