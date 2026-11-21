import sys
import numpy as np
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')

# ==============================================================================
# BÀI TOÁN TÍNH TOÁN ĐỘNG LỰC HỌC TĨNH (STATIC TORQUE) & ĐỘ CỨNG VỮNG (RIGIDITY)
# CÁNH TAY ROBOT 6 TRỤC AURA-6: PHÂN TÍCH TẢI TRỌNG CỔ TAY & SO SÁNH VẬT LIỆU
# ==============================================================================

# 1. THÔNG SỐ HÌNH HỌC DỰA TRÊN CAD ROBOTER.STEP (Đơn vị: mét)
L1 = 0.133  # Chiều cao bệ quay J1 -> J2
L2 = 0.1545 # Chiều dài khâu vai Link 2 (J2 -> J3)
L3 = 0.1478 # Chiều dài khâu khuỷu Link 3 (J3 -> J4)
L4 = 0.1055 # Chiều dài khâu cổ tay Link 4 (J4 -> J5)
L5 = 0.0350 # Chiều dài từ J5 -> J6 (Flange)
g = 9.81    # Gia tốc trọng trường (m/s^2)

# 2. KHỐI LƯỢNG CÁC THÀNH PHẦN (kg)
# Trường hợp 1: Hiện tại (Dùng 6 động cơ NEMA 17 tiêu chuẩn ~ 0.30 kg/con)
m_motor_standard = 0.300 # Động cơ NEMA 17 tiêu chuẩn (42mm)
m_link2_plastic = 0.250  # Vỏ nhựa Link 2 (in 3D infill 40%)
m_link3_plastic = 0.165  # Vỏ nhựa Link 3
m_link4_plastic = 0.150  # Vỏ nhựa Link 4
m_link5_plastic = 0.080  # Vỏ nhựa Link 5
m_flange = 0.050         # Mặt bích + đầu hút
m_bearings_hardware = 0.150 # Ốc vít, ổ bi các loại

# Tổng khối lượng dồn tại cụm cổ tay (tính từ sau J3 trở đi: Link 4, 5, 6 và 3 motor J4, J5, J6)
m_wrist_current = (3 * m_motor_standard) + m_link4_plastic + m_link5_plastic + m_flange + m_bearings_hardware
# Khoảng cách từ J3 tới trọng tâm cụm cổ tay ~ 0.16 m
L_wrist_com = 0.160

# Tổng khối lượng Link 2 và Link 3
m_arm_current = m_link2_plastic + m_motor_standard + m_link3_plastic # Motor J3 nằm tại khớp 3

print("=" * 70)
print("1. PHÂN TÍCH KHỐI LƯỢNG HIỆN TẠI (BASELINE - 6 ĐỘNG CƠ GIỐNG NHAU)")
print("=" * 70)
print(f"Khối lượng 3 động cơ ở cụm cổ tay (J4, J5, J6): {3 * m_motor_standard * 1000:.0f} g")
print(f"Khối lượng phần cứng, vỏ nhựa cổ tay: {(m_link4_plastic + m_link5_plastic + m_flange + m_bearings_hardware)*1000:.0f} g")
print(f"-> TỔNG KHỐI LƯỢNG CỤM CỔ TAY DỒN SAU J3: {m_wrist_current:.3f} kg ({m_wrist_current*1000:.0f} g)")

# 3. TÍNH TOÁN MÔ-MEN UỐN TRỌNG TRƯỜNG TẠI CÁC TƯ THẾ
def calculate_torques(m_wrist, m_motor_wrist, name=""):
    print(f"\n--- {name} ---")
    # Tư thế vươn ngang tối đa (Worst-case Horizontal Stretch: q2 = 0 deg, q3 = 0 deg)
    # Khoảng cách từ J2 tới cổ tay:
    d_wrist_from_j2 = L2 + L_wrist_com # 0.1545 + 0.160 = 0.3145 m
    d_end_from_j2 = L2 + L3 + L4 + L5   # 0.4428 m
    
    # Mô-men tác dụng lên J3 (Khuỷu tay):
    tau_j3 = m_wrist * g * L_wrist_com
    
    # Mô-men tác dụng lên J2 (Vai):
    tau_j2 = (m_wrist * g * d_wrist_from_j2) + (m_link2_plastic * g * (L2/2)) + (m_motor_standard * g * L2)
    
    # Mô-men lật (Overturning moment) tác dụng lên ổ bi đế J1:
    M_tilt_j1 = tau_j2
    
    print(f"Mô-men trọng trường đè lên Khớp Khuỷu (J3): {tau_j3:.2f} N.m")
    print(f"Mô-men trọng trường đè lên Khớp Vai    (J2): {tau_j2:.2f} N.m")
    print(f"Mô-men lật (Overturning Moment) đè lên J1: {M_tilt_j1:.2f} N.m")
    
    return tau_j3, tau_j2, M_tilt_j1

tau3_base, tau2_base, tilt1_base = calculate_torques(m_wrist_current, 3*m_motor_standard, "TRƯỜNG HỢP HIỆN TẠI (NHỰA IN 3D + 6 MOTOR TIÊU CHUẨN)")

# Trường hợp 2: Nếu làm bằng KIM LOẠI (Nhôm 6061) như nhóm trưởng đề xuất (vẫn giữ nguyên thiết kế treo 3 motor)
# Mật độ nhôm 2.7 g/cm3 vs PLA 1.24 g/cm3 (infill 40% thực tế ~ 0.55 g/cm3) -> Vỏ nhôm nặng hơn ít nhất 2.5 - 3 lần!
m_wrist_metal = (3 * m_motor_standard) + (m_link4_plastic + m_link5_plastic + m_flange)*2.8 + m_bearings_hardware
tau3_metal, tau2_metal, tilt1_metal = calculate_torques(m_wrist_metal, 3*m_motor_standard, "NẾU LÀM BẰNG KIM LOẠI NHÔM CNC (GIỮ NGUYÊN KIẾN TRÚC HIỆN TẠI)")

# Trường hợp 3: Nâng cấp Động cơ Pancake mỏng (NEMA 17 Pancake 0.13 kg cho J5, J6)
m_wrist_pancake = m_motor_standard + (2 * 0.130) + m_link4_plastic + m_link5_plastic + m_flange + m_bearings_hardware
tau3_pancake, tau2_pancake, tilt1_pancake = calculate_torques(m_wrist_pancake, m_motor_standard + 0.260, "GIẢI PHÁP 1: THAY MOTOR J5, J6 BẰNG NEMA 17 PANCAKE (GIẢM CÂN)")

# Trường hợp 4: Tích hợp Lò Xo Trợ Lực (Gravity Spring Compensation 75%)
tau3_spring = tau3_base * 0.25 # Lò xo triệt tiêu 75% tải tĩnh
tau2_spring = tau2_base * 0.20 # Lò xo triệt tiêu 80% tải tĩnh
print(f"\n--- GIẢI PHÁP 2: LẮP LÒ XO / TI HƠI TRỢ LỰC TRỌNG LỰC CHO J2, J3 ---")
print(f"Mô-men thực tế sau khi lò xo trợ lực đè lên J3: {tau3_spring:.2f} N.m (GIẢM 75%)")
print(f"Mô-men thực tế sau khi lò xo trợ lực đè lên J2: {tau2_spring:.2f} N.m (GIẢM 80%)")

# 4. SO SÁNH KHẢ NĂNG SINH MÔ-MEN CỦA ĐỘNG CƠ HIỆN TẠI
# Động cơ NEMA 17 tiêu chuẩn: T_hold ~ 0.40 - 0.45 N.m ở dòng 1.5A
# Hiện tại TMC2209 cài: J2 = 650mA, J3 = 500mA
# Tỉ số truyền J2, J3 là 5.18:1 (nếu có hộp số hành tinh) hoặc 1:1 (nếu động cơ trơn)
t_motor_nominal = 0.42 # N.m ở 1.2A
# Với dòng 650mA (J2): T_motor_j2 ~ 0.42 * (0.65 / 1.2) ~ 0.228 N.m
# Nếu có hộp số 5.18:1, hiệu suất 75%: T_avail_j2 = 0.228 * 5.18 * 0.75 = 0.885 N.m
# Nếu KHÔNG có hộp số (tỉ số 1:1): T_avail_j2 = 0.228 N.m!

print("\n" + "=" * 70)
print("2. SO SÁNH NĂNG LỰC ĐỘNG CƠ VS TẢI TRỌNG THỰC TẾ")
print("=" * 70)
print("Mô-men yêu cầu để giữ cánh tay nằm ngang tại J2:")
print(f"  - Yêu cầu thực tế (Baseline): {tau2_base:.2f} N.m")
print(f"  - Năng lực động cơ J2 (dòng 650mA + hộp số 5.18:1): ~ 0.89 N.m  -> THIẾU {tau2_base / 0.89:.1f} LẦN!")
print(f"  - Năng lực động cơ J2 nếu không có hộp số (1:1):     ~ 0.23 N.m  -> THIẾU {tau2_base / 0.23:.1f} LẦN!")
print(f"  - Yêu cầu sau khi gắn LÒ XO TRỢ LỰC:                  {tau2_spring:.2f} N.m  -> VỪA VẶN 100% ĐỘNG CƠ HOẠT ĐỘNG ÊM ÁI!")

# 5. SO SÁNH ĐỘ CỨNG VỮNG VÀ ĐỘ VÕNG (DEFLECTION) GIỮA NHỰA VÀ KIM LOẠI
# Dầm console chịu tải đầu mút: delta = F * L^3 / (3 * E * I)
# E_pla = 3.5 GPa = 3.5e9 Pa
# E_alu = 69.0 GPa = 69.0e9 Pa
# I_beam (ước tính cho tiết diện rỗng 40x40 dày 3mm): ~ 8.0e-8 m^4
I_beam = 8.0e-8
F_load_wrist = m_wrist_current * g # ~ 1.48 * 9.81 = 14.5 N
L_arm = 0.40 # m

delta_pla = (F_load_wrist * (L_arm**3)) / (3 * 3.5e9 * I_beam)
# Với nhôm, tải trọng cổ tay tăng lên (F = m_wrist_metal * g)
delta_alu = (m_wrist_metal * g * (L_arm**3)) / (3 * 69.0e9 * I_beam)

print("\n" + "=" * 70)
print("3. PHÂN TÍCH ĐỘ VÕNG ĐÀN HỒI (DEFLECTION) & SỰ THẬT VỀ KIM LOẠI")
print("=" * 70)
print(f"Độ võng đàn hồi cánh tay bằng Nhựa PLA in 3D: {delta_pla * 1000:.2f} mm")
print(f"Độ võng đàn hồi nếu làm bằng Nhôm CNC:        {delta_alu * 1000:.2f} mm")
print("-> NHẬN ĐỊNH: Nhôm cứng vững hơn (võng ít hơn), NHƯNG khối lượng tăng làm mô-men đè lên J2 tăng từ")
print(f"   {tau2_base:.2f} N.m lên {tau2_metal:.2f} N.m (TĂNG {(tau2_metal - tau2_base)/tau2_base*100:.1f}%)!")
print("   Do đó, nếu đổi sang kim loại mà không đổi động cơ và không có lò xo cân bằng,")
print("   cánh tay kim loại sẽ KHÔNG THỂ NHẤC NỔI BẢN THÂN NÓ!")
print("=" * 70)
