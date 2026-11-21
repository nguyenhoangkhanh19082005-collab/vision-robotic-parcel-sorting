import numpy as np
import math

# ================== THÔNG SỐ CƠ KHÍ ROBOT 6 DOF (CALIBRATED FROM Roboter.STEP) ==================
# Kích thước cơ khí chuẩn trích xuất trực tiếp từ file CAD Roboter.STEP (Đơn vị: mét)
d1 = 0.1260   # Khoảng cách từ Đế đến tâm trục khớp 2 (Vai) - đo từ vòng bi Lager 78x60x10 đến Lager 65x50x7
a2 = 0.1545   # Chiều dài Cánh tay trên (Link 2) - đo từ tâm trục Vai đến tâm trục Khuỷu
d3 = 0.1480   # Chiều dài Khuỷu tay (Link 3a)
d4 = 0.1190   # Chiều dài đến Tâm cổ tay (Link 3b) -> Tổng d3+d4 = 0.2670m (đo từ Khuỷu đến Pw)
d6 = 0.0400   # Chiều dài từ Tâm cổ tay đến Đầu công tác hút phôi (TCP)

# Bảng tham số DH chuẩn [a, alpha, d, theta_offset]
dh_params = np.array([
    [0.0,   -np.pi/2, d1,      0.0],
    [a2,     0.0,     0.0,     0.0],
    [0.0,   -np.pi/2, 0.0,     0.0],
    [0.0,    np.pi/2, d3 + d4, 0.0],
    [0.0,   -np.pi/2, 0.0,     0.0],
    [0.0,    0.0,     d6,      0.0]
])

def calc_crc16(data: bytes) -> int:
    """Tính mã kiểm tra lỗi CRC16 MODBUS cho mảng byte."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def dh_matrix(a, alpha, d, theta):
    """Ma trận biến đổi đồng nhất Denavit-Hartenberg (Standard DH)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0, sa,      ca,      d],
        [0.0, 0.0,     0.0,     1.0]
    ])

def euler_to_matrix(roll, pitch, yaw, degrees=False):
    """
    Chuyển đổi góc Euler (Roll, Pitch, Yaw) sang Ma trận xoay 3x3 (ZYX).
    Mặc định nhận đơn vị Radian. Nếu degrees=True, nhận đơn vị Độ.
    """
    if degrees:
        roll, pitch, yaw = np.radians([roll, pitch, yaw])
    Rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, np.cos(roll), -np.sin(roll)],
        [0.0, np.sin(roll), np.cos(roll)]
    ])
    Ry = np.array([
        [np.cos(pitch), 0.0, np.sin(pitch)],
        [0.0, 1.0, 0.0],
        [-np.sin(pitch), 0.0, np.cos(pitch)]
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0.0],
        [np.sin(yaw), np.cos(yaw), 0.0],
        [0.0, 0.0, 1.0]
    ])
    return Rz @ Ry @ Rx

def matrix_to_euler(R, degrees=True):
    """
    Chuyển đổi Ma trận xoay 3x3 sang góc Euler (Roll, Pitch, Yaw).
    Mặc định trả về Độ (degrees=True). Nếu degrees=False, trả về Radian.
    """
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0

    res = np.array([roll, pitch, yaw])
    return np.degrees(res) if degrees else res

class kinematic:
    def __init__(self, dh_params_custom=None):
        if dh_params_custom is None:
            self.dh = np.array(dh_params, copy=True, dtype=float)
        else:
            self.dh = np.array(dh_params_custom, copy=True, dtype=float)

        self.joint_limits = [
            (-170.0, 170.0), # Joint 1: Waist
            (-90.0,  110.0), # Joint 2: Shoulder
            (-150.0, 150.0), # Joint 3: Elbow
            (-180.0, 180.0), # Joint 4: Wrist Roll
            (-110.0, 110.0), # Joint 5: Wrist Pitch
            (-360.0, 360.0)  # Joint 6: Wrist Yaw
        ]

    def fk(self, q_rad, return_all_frames=False):
        """
        Động học thuận (Forward Kinematics).
        Input: q_rad - Mảng 6 góc khớp (Radian)
        Output: Ma trận 4x4 T_06 biểu diễn vị trí & hướng của TCP
        """
        T = np.eye(4)
        frames = [T.copy()]

        for i in range(6):
            a, alpha, d, offset = self.dh[i]
            theta = q_rad[i] + offset
            T_i = dh_matrix(a, alpha, d, theta)
            T = T @ T_i
            frames.append(T.copy())

        if return_all_frames:
            return T, frames
        return T

    def ik_pieper(self, T_target, q_current_deg=None):
        """
        Động học nghịch kín Pieper Analytical IK dành cho Robot 6-DOF Cổ tay cầu (Spherical Wrist).
        Đã chuẩn hóa đọc từ self.dh, bù theta_offset, xử lý sai số biên và chọn nghiệm góc vòng.
        """
        R_target = T_target[:3, :3]
        P_target = T_target[:3, 3]

        # Đọc trực tiếp kích thước hình học từ self.dh (chống lỗi biến toàn cục)
        d1_val = self.dh[0, 2]
        a2_val = self.dh[1, 0]
        d_arm4 = self.dh[3, 2]
        d6_val = self.dh[5, 2]

        off1 = self.dh[0, 3]
        off2 = self.dh[1, 3]
        off3 = self.dh[2, 3]
        off4 = self.dh[3, 3]
        off5 = self.dh[4, 3]
        off6 = self.dh[5, 3]

        # 1. Tính vị trí Tâm cổ tay (Wrist Center P_w)
        z_end = R_target[:, 2]
        P_w = P_target - d6_val * z_end
        xw, yw, zw = P_w[0], P_w[1], P_w[2]

        solutions = []

        # 2. Giải q1 (2 nghiệm: Thuận và Nghịch 180 độ)
        q1_opts = []
        if abs(xw) < 1e-6 and abs(yw) < 1e-6:
            # Singularity vai (tâm cổ tay nằm thẳng trên trục Z0)
            if q_current_deg is not None:
                q1_seed = np.radians(q_current_deg[0])
                q1_opts = [q1_seed]
            else:
                q1_opts = [0.0]
        else:
            q1_base = math.atan2(yw, xw) - off1
            q1_flip = (q1_base + np.pi) if q1_base <= 0 else (q1_base - np.pi)
            q1_opts = [q1_base, q1_flip]

        for q1 in q1_opts:
            # Tọa độ chiếu r trong mặt phẳng cánh tay (có cộng offset theta1)
            theta1_actual = q1 + off1
            R_xy = xw * math.cos(theta1_actual) + yw * math.sin(theta1_actual)
            Z_eff = d1_val - zw

            # Tính sin(q3) chuẩn xác
            denom_q3 = 2.0 * a2_val * d_arm4
            if denom_q3 < 1e-9:
                continue

            sin_q3_val = (a2_val**2 + d_arm4**2 - (R_xy**2 + Z_eff**2)) / denom_q3

            # Ràng buộc điều kiện không gian làm việc (kèm epsilon dung sai số thực)
            if abs(sin_q3_val) > 1.0 + 1e-7:
                continue
            sin_q3 = float(np.clip(sin_q3_val, -1.0, 1.0))

            # 3. Giải q3 (2 nghiệm: Elbow Up / Elbow Down)
            cos_q3_opts = [math.sqrt(max(0.0, 1.0 - sin_q3**2)), -math.sqrt(max(0.0, 1.0 - sin_q3**2))]

            for cos_q3 in cos_q3_opts:
                theta3_val = math.atan2(sin_q3, cos_q3)
                q3 = theta3_val - off3

                # 4. Giải q2 tương ứng với q3
                A = a2_val - d_arm4 * sin_q3
                B = d_arm4 * cos_q3
                denom = A**2 + B**2

                if denom < 1e-9:
                    continue

                c2 = (A * R_xy + B * Z_eff) / denom
                s2 = (-B * R_xy + A * Z_eff) / denom
                theta2_val = math.atan2(s2, c2)
                q2 = theta2_val - off2

                # Tính ma trận quay R_03 từ Base đến Frame 3 CÓ BÙ THETA_OFFSET
                T01 = dh_matrix(self.dh[0][0], self.dh[0][1], self.dh[0][2], q1 + off1)
                T12 = dh_matrix(self.dh[1][0], self.dh[1][1], self.dh[1][2], q2 + off2)
                T23 = dh_matrix(self.dh[2][0], self.dh[2][1], self.dh[2][2], q3 + off3)
                R03 = (T01 @ T12 @ T23)[:3, :3]

                # 5. Ma trận quay Cổ tay: R_36 = R_03^T * R_target
                R36 = R03.T @ R_target

                # 6. Trích xuất q4, q5, q6 từ R_36
                cos_q5 = float(np.clip(R36[2, 2], -1.0, 1.0))
                q5_opts = [math.acos(cos_q5), -math.acos(cos_q5)]

                for q5_val in q5_opts:
                    sin_q5 = math.sin(q5_val)

                    if abs(sin_q5) > 1e-4:
                        if sin_q5 > 0:
                            theta4 = math.atan2(-R36[1, 2], -R36[0, 2])
                            theta6 = math.atan2(-R36[2, 1], R36[2, 0])
                        else:
                            theta4 = math.atan2(R36[1, 2], R36[0, 2])
                            theta6 = math.atan2(R36[2, 1], -R36[2, 0])
                    else:
                        # Wrist Singularity (q5 = 0, Gimbal Lock)
                        theta4 = 0.0
                        theta6 = math.atan2(-R36[0, 1], R36[0, 0])

                    q4 = theta4 - off4
                    q5 = q5_val - off5
                    q6 = theta6 - off6

                    # Chuẩn hóa về [-pi, pi]
                    sol = np.array([q1, q2, q3, q4, q5, q6])
                    sol = np.arctan2(np.sin(sol), np.cos(sol))

                    if self._check_limits_rad(sol):
                        solutions.append(sol)

        if not solutions:
            return None, False

        # Chọn nghiệm tối ưu: Dùng khoảng cách góc vòng (Angular Geodesic Distance)
        if q_current_deg is not None:
            q_curr_rad = np.radians(q_current_deg)
            def angular_distance(s, curr):
                # Hiệu góc ngắn nhất trên vòng tròn [-pi, pi]
                diff = np.arctan2(np.sin(s - curr), np.cos(s - curr))
                return np.linalg.norm(diff)

            best_sol = min(solutions, key=lambda s: angular_distance(s, q_curr_rad))
            return best_sol, True

        return solutions[0], True

    def _check_limits_rad(self, q_rad):
        q_deg = np.degrees(q_rad)
        for i in range(6):
            min_deg, max_deg = self.joint_limits[i]
            if q_deg[i] < min_deg - 1e-5 or q_deg[i] > max_deg + 1e-5:
                return False
        return True

    def ik(self, T_target, q_current_deg=None):
        return self.ik_pieper(T_target, q_current_deg)

    def compensate_gravity_compliance(self, q_rad, k_flex2=0.015, k_flex3=0.012):
        """
        Bù trừ độ võng đàn hồi tĩnh (Elastic Compliance Compensation) cho cánh tay nhựa in 3D.
        Tự động tính góc võng của Link 2 và Link 3 do trọng lượng cụm cổ tay gây ra khi vươn ngang,
        và cộng thêm góc bù vi mô vào q2 và q3 để đầu công tác TCP đến đúng độ cao Z thực tế.
        - k_flex2: Hệ số bù góc vai J2 (mặc định ~ 0.015 rad ~ 0.86 độ khi vươn ngang)
        - k_flex3: Hệ số bù góc khuỷu J3 (mặc định ~ 0.012 rad ~ 0.69 độ khi vươn ngang)
        """
        q_comp = np.array(q_rad, dtype=float).copy()
        q2 = q_comp[1]
        q3 = q_comp[2]
        delta_q2 = k_flex2 * math.cos(q2)
        delta_q3 = k_flex3 * math.cos(q2 + q3)
        q_comp[1] += delta_q2
        q_comp[2] += delta_q3
        return q_comp
