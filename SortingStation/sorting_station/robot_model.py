"""Mô hình động học robot 6 trục dựng trực tiếp từ trục khớp đo trên CAD.

Vì sao không dùng thẳng bảng DH trong ``robotarm-6-axis/donghoc.py``:
ở q = 0 bảng DH (mọi theta_offset = 0) cho cánh tay trên nằm ngang và TCP ở
Z ≈ -181 mm (dưới mặt đế), trong khi digital twin (robot_simulation.py) và robot
thật sau khi "Lưu Gốc" ở q = 0 là tư thế chữ L (cánh tay trên dựng đứng, cẳng tay
hướng ra trước). HUD TCP và MoveL/IK vì thế không khớp với robot thật.

Mô hình ở đây dùng Product of Exponentials (PoE) với đúng các điểm/trục khớp
``JOINT_POINTS``/``JOINT_AXES`` mà digital twin đang dùng, nên:
- FK hiển thị trên HUD == tư thế mesh trong twin == (giả định) robot thật;
- IK là bộ giải số Damped Least Squares, có ràng buộc giới hạn khớp.

Quy ước:
- Đơn vị: mét, radian bên trong; API tương thích donghoc nhận/trả độ ở các chỗ cũ.
- Hệ cơ sở (base) của robot: gốc tại chân đế trên trục J1, Z dọc trục J1 hướng lên,
  X hướng về phía cổ tay ở tư thế 0, Y = Z × X. Cảnh digital twin dùng chung hệ này
  làm hệ thế giới (world), nên tọa độ bố trí băng tải/thùng chính là tọa độ IK.
- Hệ dụng cụ (tool): Z dọc trục J6 hướng ra khỏi mặt bích (hướng hút),
  X ở tư thế 0 hướng lên.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

# Giới hạn khớp đồng bộ với donghoc.kinematic.joint_limits và slider của GUI robot (độ)
# 24/11/2026: tư thế 0 = CHỮ L, ĐẦU HÚT CHÚC XUỐNG (dong_hoc_step.py). So với tư thế 0 dựng đứng (22/11):
#   q_mới(J3) = q_cũ − 90°, q_mới(J5) = q_cũ + 90° → giới hạn J3, J5 dời theo để giữ đúng dải cơ khí cũ.
#   J3: dải cũ [-150, 150] → [-240, 60], rồi THU HẸP 15° mỗi phía theo yêu cầu an toàn → [-225, 45]
#       (+ = cẳng tay chúc xuống; +45° là cẳng tay chúc 45° dưới phương ngang).
#   J5: dải cũ [-110, 110] → [-20, 200] (0 = đầu hút thẳng xuống khi cẳng tay ngang; +90 = cổ tay thẳng).
DEFAULT_JOINT_LIMITS_DEG: tuple[tuple[float, float], ...] = (
    (-170.0, 170.0),
    (-102.5, 97.5),
    (-220.0, 20.0),
    (-180.0, 180.0),
    (-42.0, 178.0),
    (-360.0, 360.0),
)

# Đo trên mesh: mặt bích khâu 6 nhô 7.4 mm khỏi JOINT_POINTS[5] dọc trục J6;
# đáy base_link nằm 78.7 mm dưới JOINT_POINTS[0] dọc trục J1.
DEFAULT_FLANGE_OFFSET_M = 0.0074
DEFAULT_FOOT_OFFSET_M = 0.0787


# ---------------------------------------------------------------------------
# Tiện ích hình học
# ---------------------------------------------------------------------------
def _unit(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        raise ValueError("Vector trục khớp bằng 0")
    return np.asarray(vector, dtype=float) / norm


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Công thức Rodrigues: R = I + sinθ[u]x + (1-cosθ)[u]x²."""

    k = _skew(_unit(axis))
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def screw_transform(point: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    """Phép quay góc `angle` quanh đường thẳng đi qua `point` theo hướng `axis` (4x4)."""

    rot = axis_angle_matrix(axis, angle)
    transform = np.eye(4)
    transform[:3, :3] = rot
    transform[:3, 3] = point - rot @ point
    return transform


def rotation_log(rot: np.ndarray) -> np.ndarray:
    """Vector quay (axis * angle) của ma trận quay — dùng làm sai số hướng trong IK."""

    cos_angle = float(np.clip((np.trace(rot) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cos_angle)
    if angle < 1e-9:
        return np.zeros(3)
    if math.pi - angle < 1e-6:
        # Gần 180°: lấy trục từ phần đối xứng (R + I)/2 = u uᵀ
        sym = (rot + np.eye(3)) / 2.0
        axis = np.sqrt(np.clip(np.diag(sym), 0.0, None))
        index = int(np.argmax(axis))
        axis = sym[:, index] / max(axis[index], 1e-12)
        return _unit(axis) * angle
    vee = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]])
    return vee * (angle / (2.0 * math.sin(angle)))


def _closest_point_on_first(p1, d1, p2, d2) -> np.ndarray:
    """Điểm trên đường thẳng 1 gần đường thẳng 2 nhất (hai trục khớp không giao hẳn do sai số CAD)."""

    w = p1 - p2
    a, b, c = d1 @ d1, d1 @ d2, d2 @ d2
    d, e = d1 @ w, d2 @ w
    den = a * c - b * b
    if abs(den) < 1e-12:
        return np.asarray(p1, dtype=float)
    s = (b * e - c * d) / den
    return p1 + s * d1


def read_binary_stl_vertices(path: str | Path, scale: float = 0.001) -> np.ndarray:
    """Đọc đỉnh của file STL nhị phân (không cần PyVista). Mặc định mm → m."""

    data = Path(path).read_bytes()
    if len(data) < 84:
        raise ValueError(f"STL quá ngắn: {path}")
    count = int(np.frombuffer(data, dtype="<u4", count=1, offset=80)[0])
    if 84 + count * 50 != len(data):
        raise ValueError(f"Không phải STL nhị phân hợp lệ: {path}")
    records = np.frombuffer(
        data,
        dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")]),
        count=count,
        offset=84,
    )
    return records["v"].reshape(-1, 3).astype(float) * scale


# ---------------------------------------------------------------------------
# Mô hình robot
# ---------------------------------------------------------------------------
class RobotModel:
    """FK/IK theo PoE trên hình học CAD; API ``fk``/``ik_pieper`` tương thích donghoc.kinematic."""

    def __init__(
        self,
        joint_points_m: Sequence[Sequence[float]],
        joint_axes: Sequence[Sequence[float]],
        joint_limits_deg: Iterable[tuple[float, float]] = DEFAULT_JOINT_LIMITS_DEG,
        tool_length_m: float = 0.0,
        flange_offset_m: float = DEFAULT_FLANGE_OFFSET_M,
        foot_offset_m: float = DEFAULT_FOOT_OFFSET_M,
    ) -> None:
        self.points = [np.asarray(p, dtype=float) for p in joint_points_m]
        self.axes = [_unit(np.asarray(a, dtype=float)) for a in joint_axes]
        if len(self.points) != 6 or len(self.axes) != 6:
            raise ValueError("Cần đúng 6 điểm và 6 trục khớp")
        self.joint_limits = [tuple(map(float, limit)) for limit in joint_limits_deg]
        if len(self.joint_limits) != 6:
            raise ValueError("Cần đúng 6 cặp giới hạn khớp")
        self.tool_length_m = float(tool_length_m)
        self.flange_offset_m = float(flange_offset_m)
        self.foot_offset_m = float(foot_offset_m)
        self._limits_rad = np.radians(np.array(self.joint_limits, dtype=float))
        self._original_limits = list(self.joint_limits)
        self.mesh_scale = 1.0
        # Ma trận đặt lại mesh STL của base_link + 6 khâu trước khi áp biến đổi khớp (xem dong_hoc_step.py)
        self.mesh_pretransforms = [np.eye(4) for _ in range(7)]
        # Khớp bị KHÓA (22/11/2026: J2 cố định vì đai J2 yếu): {chỉ số 0..5: góc độ}. IK không dùng khớp này.
        self.locked: dict[int, float] = {}
        # Khi khóa khớp chỉ còn 4 bậc định vị đầu hút (J6 chỉ xoay quanh trục hút) → không ép được cả vị trí
        # lẫn hướng thẳng đứng. Vị trí được ưu tiên tuyệt đối; trục hút được phép NGHIÊNG tối đa góc này.
        self.max_tool_tilt_deg = 15.0
        self._build_frames()

    # -- dựng hệ base và tư thế TCP ở q = 0 (trong hệ CAD) --------------------
    def _build_frames(self) -> None:
        z_axis = self.axes[0]
        origin = self.points[0] - self.foot_offset_m * z_axis
        wrist_center = _closest_point_on_first(
            self.points[3], self.axes[3], self.points[4], self.axes[4]
        )
        forward = wrist_center - origin
        x_axis = forward - (forward @ z_axis) * z_axis
        if np.linalg.norm(x_axis) < 0.05:
            # 22/11/2026: tư thế 0 = DỰNG THẲNG ĐỨNG → tâm cổ tay nằm ngay trên trục J1, không xác định được hướng
            # "trước". Lấy hướng cẳng tay ở tư thế chữ L (J3 = +90°) để X base giữ đúng như trước (băng tải ở +X).
            elbow = screw_transform(self.points[2], self.axes[2], math.radians(90.0))
            forward = (elbow @ np.append(wrist_center, 1.0))[:3] - origin
            x_axis = forward - (forward @ z_axis) * z_axis
        x_axis = _unit(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        cad_from_base = np.eye(4)
        cad_from_base[:3, :3] = np.column_stack([x_axis, y_axis, z_axis])
        cad_from_base[:3, 3] = origin
        self.cad_from_base = cad_from_base
        self.base_from_cad = np.linalg.inv(cad_from_base)
        self.wrist_center_home_cad = wrist_center

        tool_z = self.axes[5]
        tool_x = z_axis - (z_axis @ tool_z) * tool_z
        if np.linalg.norm(tool_x) < 1e-6:  # trục J6 dựng đứng ở q=0 → chọn X ngang
            tool_x = x_axis - (x_axis @ tool_z) * tool_z
        tool_x = _unit(tool_x)
        tool_y = np.cross(tool_z, tool_x)
        self._update_tool_home(tool_x, tool_y, tool_z)

    def _update_tool_home(self, tool_x, tool_y, tool_z) -> None:
        home = np.eye(4)
        home[:3, :3] = np.column_stack([tool_x, tool_y, tool_z])
        home[:3, 3] = self.points[5] + (self.flange_offset_m + self.tool_length_m) * tool_z
        self.tool_home_cad = home

    def set_tool_length(self, tool_length_m: float) -> None:
        self.tool_length_m = float(tool_length_m)
        rot = self.tool_home_cad[:3, :3]
        self._update_tool_home(rot[:, 0], rot[:, 1], rot[:, 2])

    # -- động học thuận -------------------------------------------------------
    def link_transforms_cad(self, q_rad: Sequence[float]) -> list[np.ndarray]:
        """6 ma trận tích lũy (khâu 1..6) — trùng công thức RobotSimulationWidget."""

        transforms = []
        current = np.eye(4)
        for point, axis, angle in zip(self.points, self.axes, q_rad):
            current = current @ screw_transform(point, axis, float(angle))
            transforms.append(current)
        return transforms

    def link_transforms_base(self, q_rad: Sequence[float]) -> list[np.ndarray]:
        """Ma trận đặt mesh (tọa độ CAD) vào hệ base: base_link + 6 khâu (7 phần tử)."""

        cad_links = [np.eye(4)] + self.link_transforms_cad(q_rad)
        return [self.base_from_cad @ link @ pre for link, pre in zip(cad_links, self.mesh_pretransforms)]

    def fk(self, q_rad: Sequence[float], return_all_frames: bool = False):
        links = self.link_transforms_cad(q_rad)
        tcp = self.base_from_cad @ links[-1] @ self.tool_home_cad
        if return_all_frames:
            frames = [np.eye(4)] + [self.base_from_cad @ link for link in links]
            return tcp, frames
        return tcp

    def fk_deg(self, q_deg: Sequence[float]) -> np.ndarray:
        return self.fk(np.radians(np.asarray(q_deg, dtype=float)))

    def jacobian(self, q_rad: Sequence[float]) -> np.ndarray:
        """Jacobian hình học 6x6 tại TCP trong hệ base: hàng 0..2 vận tốc dài, 3..5 vận tốc góc."""

        rot_bc = self.base_from_cad[:3, :3]
        links = self.link_transforms_cad(q_rad)
        tcp_cad = (links[-1] @ self.tool_home_cad)[:3, 3]
        jac = np.zeros((6, 6))
        previous = np.eye(4)
        for index in range(6):
            omega = previous[:3, :3] @ self.axes[index]
            point = previous[:3, :3] @ self.points[index] + previous[:3, 3]
            jac[:3, index] = rot_bc @ np.cross(omega, tcp_cad - point)
            jac[3:, index] = rot_bc @ omega
            previous = links[index]
        return jac

    # -- giới hạn khớp ---------------------------------------------------------
    def within_limits_deg(self, q_deg: Sequence[float], margin_deg: float = 1e-6) -> bool:
        for value, (low, high) in zip(q_deg, self.joint_limits):
            if value < low - margin_deg or value > high + margin_deg:
                return False
        return True

    def _check_limits_rad(self, q_rad) -> bool:  # tương thích donghoc
        return self.within_limits_deg(np.degrees(q_rad))

    def clamp_rad(self, q_rad: np.ndarray) -> np.ndarray:
        return np.clip(q_rad, self._limits_rad[:, 0], self._limits_rad[:, 1])

    def lock_joint(self, index: int, angle_deg: float | None) -> None:
        """Khóa khớp `index` (0..5) ở `angle_deg`: giới hạn khớp đó thành [góc, góc]. None = mở khóa."""

        if angle_deg is None:
            self.locked.pop(index, None)
            low, high = self._original_limits[index]
        else:
            lo0, hi0 = self._original_limits[index]
            if not lo0 <= angle_deg <= hi0:
                raise ValueError(f"Góc khóa J{index + 1} = {angle_deg}° ngoài giới hạn [{lo0}, {hi0}]")
            self.locked[index] = float(angle_deg)
            low = high = float(angle_deg)
        self.joint_limits[index] = (float(low), float(high))
        self._limits_rad = np.radians(np.array(self.joint_limits, dtype=float))

    # -- động học nghịch -------------------------------------------------------
    def solve_ik(
        self,
        position_m: Sequence[float],
        rotation: np.ndarray | None = None,
        approach: Sequence[float] | None = None,
        seed_deg: Sequence[float] | None = None,
        position_tol_m: float = 5e-4,
        angle_tol_rad: float = math.radians(0.3),
        max_iterations: int = 300,
    ) -> tuple[np.ndarray | None, dict]:
        """IK số Damped Least Squares.

        - ``rotation`` (3x3): ép đủ 6 bậc (vị trí + hướng).
        - ``approach`` (vector): chỉ ép trục Z dụng cụ (5 bậc) — phù hợp đầu hút đối xứng,
          để J6 tự do.
        - Không truyền cả hai: chỉ giải vị trí (3 bậc).
        Trả về (q_deg hoặc None, info). Seed hiện tại được thử trước (nghiệm liên tục,
        gần tư thế đang đứng); chỉ khi không hội tụ mới thử các seed dự phòng.
        """

        target_p = np.asarray(position_m, dtype=float)
        target_z = _unit(np.asarray(approach, dtype=float)) if approach is not None else None
        seed = np.radians(np.asarray(seed_deg if seed_deg is not None else [0.0] * 6, dtype=float))
        if self.locked and rotation is None:
            best, tilt_min, best_dist = None, math.inf, math.inf
            for start in self._seeds(seed, target_p):
                q, info = self._ik_locked(start, target_p, target_z, position_tol_m, max_iterations)
                if q is None:
                    continue
                tilt_min = min(tilt_min, info["tilt_deg"])
                if info["tilt_deg"] > self.max_tool_tilt_deg:
                    continue
                # Trong các nghiệm đủ thẳng, chọn nghiệm GẦN tư thế hiện tại nhất (không lật cổ tay J4/J5)
                dist = float(np.max(np.abs(q - seed)))
                if dist < best_dist:
                    best, best_dist = (q, info), dist
            if best is not None:
                return np.degrees(best[0]), best[1]
            if tilt_min < math.inf:
                return None, {"ok": False, "tilt_deg": tilt_min,
                              "reason": f"J2 khóa ở {self.locked.get(1, 0):.1f}°: đầu hút phải nghiêng "
                                        f"{tilt_min:.1f}° > {self.max_tool_tilt_deg:.0f}° cho phép"}
            return None, {"ok": False, "reason": f"Ngoài vùng với khi khóa khớp {[i + 1 for i in self.locked]}"}
        for start in self._seeds(seed, target_p):
            q, info = self._dls(
                start, target_p, rotation, target_z, position_tol_m, angle_tol_rad, max_iterations
            )
            if q is not None:
                return np.degrees(q), info
        return None, {"ok": False, "reason": "Ngoài vùng với hoặc vượt giới hạn khớp"}

    def _ik_locked(self, start, target_p, target_z, pos_tol, max_iterations):
        """IK ưu tiên 2 cấp khi có khớp khóa.

        Cấp 1: vị trí TCP (bắt buộc đạt sai số pos_tol).
        Cấp 2: trục hút gần `target_z` nhất có thể, chỉ dùng chuyển động trong KHÔNG GIAN NULL của cấp 1:
            dq = J_p⁺ e_p + N (J_o N)⁺ (e_o − J_o J_p⁺ e_p),  N = I − J_p⁺ J_p
        Cột Jacobian của khớp khóa = 0 nên khớp đó không bao giờ được yêu cầu chuyển động.
        Trả (q_rad, info có 'tilt_deg') hoặc (None, {}).
        """

        q = self.clamp_rad(start.copy())
        locked = list(self.locked)
        damping = 0.02
        best_tilt, last_gain = math.inf, 0
        for iteration in range(max_iterations):
            tcp = self.fk(q)
            e_pos = target_p - tcp[:3, 3]
            current_z = tcp[:3, 2]
            jac = self.jacobian(q)
            jac[:, locked] = 0.0
            jp = jac[:3]
            jp_pinv = jp.T @ np.linalg.inv(jp @ jp.T + damping**2 * np.eye(3))
            # Phép chiếu null CHÍNH XÁC (pinv SVD, không damping) để cấp 2 không làm lệch vị trí
            null = np.eye(6) - np.linalg.pinv(jp, rcond=1e-4) @ jp
            if target_z is not None:
                cross = np.cross(current_z, target_z)
                tilt = math.atan2(float(np.linalg.norm(cross)), float(current_z @ target_z))
                e_ang = _unit(cross) * tilt if np.linalg.norm(cross) > 1e-12 else np.zeros(3)
                projector = np.eye(3) - np.outer(current_z, current_z)   # bỏ xoay quanh trục hút (J6 tự do)
                jo = projector @ jac[3:]
                jon = jo @ null
                # damping lớn hơn cho cấp 2: chỉ còn ≈ 1 bậc tự do dư → tránh khuếch đại bước khi gần suy biến
                jon_pinv = jon.T @ np.linalg.inv(jon @ jon.T + 0.1**2 * np.eye(3))
                step = jp_pinv @ e_pos + 0.5 * null @ (jon_pinv @ e_ang)
            else:
                tilt = 0.0
                step = jp_pinv @ e_pos
            if tilt < best_tilt - math.radians(0.02):
                best_tilt, last_gain = tilt, iteration
            pos_ok = float(np.linalg.norm(e_pos)) < pos_tol
            if pos_ok and (tilt < math.radians(0.3) or iteration - last_gain > 30):
                if not self.within_limits_deg(np.degrees(q)):
                    return None, {}
                return q, {
                    "ok": True,
                    "iterations": iteration,
                    "position_error_mm": float(np.linalg.norm(e_pos) * 1000),
                    "angle_error_deg": math.degrees(tilt),
                    "tilt_deg": math.degrees(tilt),
                }
            norm = float(np.linalg.norm(step))
            if norm > 0.25:
                step *= 0.25 / norm
            q = self.clamp_rad(q + step)
            for index in locked:
                q[index] = math.radians(self.locked[index])
        return None, {}

    def _seeds(self, seed: np.ndarray, target_p: np.ndarray) -> list[np.ndarray]:
        yaw = math.atan2(target_p[1], target_p[0])
        seeds = [self.clamp_rad(seed.copy())]
        for shoulder, elbow in ((0.0, 0.0), (0.4, -0.4), (-0.3, 0.6), (0.6, 0.3), (-0.5, -0.5)):
            for wrist in (0.0, 1.2, -1.2):
                guess = np.array([yaw, shoulder, elbow, 0.0, wrist, 0.0])
                seeds.append(self.clamp_rad(guess))
        return seeds

    def _dls(self, start, target_p, target_rot, target_z, pos_tol, ang_tol, max_iterations):
        q = start.copy()
        length_scale = 0.15  # m/rad: cân bằng sai số vị trí và hướng
        damping = 0.02
        best_error, last_improvement = math.inf, 0
        for iteration in range(max_iterations):
            tcp = self.fk(q)
            pos = tcp[:3, 3]
            rot = tcp[:3, :3]
            e_pos = target_p - pos
            jac = self.jacobian(q)
            if target_rot is not None:
                e_ang = rotation_log(target_rot @ rot.T)
                jac_task = np.vstack([jac[:3], length_scale * jac[3:]])
            elif target_z is not None:
                current_z = rot[:, 2]
                cross = np.cross(current_z, target_z)
                angle = math.atan2(np.linalg.norm(cross), float(current_z @ target_z))
                e_ang = _unit(cross) * angle if np.linalg.norm(cross) > 1e-12 else (
                    np.zeros(3) if angle < 1e-6 else _unit(np.cross(current_z, rot[:, 0])) * angle
                )
                # Bỏ thành phần quay quanh trục dụng cụ (J6 tự do)
                projector = np.eye(3) - np.outer(current_z, current_z)
                e_ang = projector @ e_ang
                jac_task = np.vstack([jac[:3], length_scale * (projector @ jac[3:])])
            else:
                e_ang = np.zeros(3)
                jac_task = jac[:3]
            if np.linalg.norm(e_pos) < pos_tol and np.linalg.norm(e_ang) < ang_tol:
                if self.within_limits_deg(np.degrees(q)):
                    return q, {
                        "ok": True,
                        "iterations": iteration,
                        "position_error_mm": float(np.linalg.norm(e_pos) * 1000),
                        "angle_error_deg": float(np.degrees(np.linalg.norm(e_ang))),
                    }
                return None, {}
            error = np.concatenate([e_pos, length_scale * e_ang]) if jac_task.shape[0] == 6 else e_pos
            error_norm = float(np.linalg.norm(error))
            if error_norm < best_error * 0.999:
                best_error, last_improvement = error_norm, iteration
            elif iteration - last_improvement > 40:
                return None, {}  # kẹt ở biên khớp / ngoài vùng với → thử seed khác
            jjt = jac_task @ jac_task.T
            step = jac_task.T @ np.linalg.solve(jjt + (damping**2) * np.eye(jjt.shape[0]), error)
            norm = float(np.linalg.norm(step))
            if norm > 0.25:
                step *= 0.25 / norm
            q = self.clamp_rad(q + step)
        return None, {}

    def ik_pieper(self, T_target: np.ndarray, q_current_deg=None):  # noqa: N803 (API cũ)
        """Tương thích donghoc.kinematic.ik_pieper: trả (q_rad, success)."""

        q_deg, info = self.solve_ik(
            T_target[:3, 3], rotation=np.asarray(T_target[:3, :3]), seed_deg=q_current_deg
        )
        if q_deg is None:
            return None, False
        return np.radians(q_deg), True

    def ik(self, T_target, q_current_deg=None):  # noqa: N803
        return self.ik_pieper(T_target, q_current_deg)


def build_model_from_robot_project(
    robot_root: str | Path,
    tool_length_m: float = 0.0,
    flange_offset_m: float | None = None,
    foot_offset_m: float | None = None,
    scale: float = 1.0,
) -> RobotModel:
    """Dựng RobotModel từ hằng số trong robot_simulation.py và giới hạn khớp trong donghoc.py.

    ``scale``: tỉ lệ robot in thật / CAD (Roboter.STEP). 1.0 = đúng CAD. Hệ số nhân cho mọi
    kích thước hình học (điểm khớp, chân đế, mặt bích) và cho mesh trên twin — IK vì thế dùng
    đúng kích thước thật. Chiều dài đầu hút (đo thật) KHÔNG nhân hệ số.
    Yêu cầu robot_root đã nằm trong sys.path (xem station_config.install_import_paths).
    """

    if not 0.2 <= scale <= 5.0:
        raise ValueError(f"robot.scale = {scale} không hợp lý (0.2–5.0)")

    import donghoc  # noqa: PLC0415 - module của dự án robot
    import robot_simulation  # noqa: PLC0415

    limits = donghoc.kinematic().joint_limits
    if foot_offset_m is None:
        foot_offset_m = DEFAULT_FOOT_OFFSET_M
        mesh = Path(robot_root) / "assets" / "ROBOTARM2_description" / "meshes" / "base_link.stl"
        if mesh.is_file():
            vertices = read_binary_stl_vertices(mesh)
            axis = _unit(np.asarray(robot_simulation.JOINT_AXES[0], dtype=float))
            heights = (vertices - np.asarray(robot_simulation.JOINT_POINTS[0])) @ axis
            foot_offset_m = float(-heights.min())
    flange = DEFAULT_FLANGE_OFFSET_M if flange_offset_m is None else flange_offset_m
    model = RobotModel(
        [np.asarray(p, dtype=float) * scale for p in robot_simulation.JOINT_POINTS],
        robot_simulation.JOINT_AXES,
        limits,
        tool_length_m=tool_length_m,
        flange_offset_m=flange * scale,
        foot_offset_m=foot_offset_m * scale,
    )
    model.mesh_scale = scale  # twin nhân hệ số này cho mesh STL (tọa độ CAD)
    pres = list(getattr(robot_simulation, "LINK_MESH_TRANSFORMS", {}).values())
    if len(pres) == 7:  # thứ tự: base_link, link_1_1 … link_6_1
        model.mesh_pretransforms = []
        for pre in pres:
            pre = np.array(pre, dtype=float)
            pre[:3, 3] *= scale
            model.mesh_pretransforms.append(pre)
    return model
