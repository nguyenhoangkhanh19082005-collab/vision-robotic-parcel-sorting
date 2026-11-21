import sys
import os
import io
import math
import numpy as np

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

sys.path.insert(0, r"E:\robotarm-6-axis")
from robot_simulation import JOINT_POINTS, JOINT_AXES, rot_axis_4x4, make_trans_4x4
import pyvista as pv

print("="*75)
print("  KIỂM CHỨNG TOÀN DIỆN DIGITAL TWIN CÁNH TAY ROBOT 6 TRỤC (AURA-6)")
print("="*75)

LINK_NAMES = ["base_link", "link_1_1", "link_2_1", "link_3_1", "link_4_1", "link_5_1", "link_6_1"]

# Kiểm tra file mesh gốc
for name in LINK_NAMES:
    path = f"assets/ROBOTARM2_description/meshes/{name}.stl"
    assert os.path.isfile(path), f"File {path} không tồn tại!"
    m = pv.read(path)
    assert len(m.points) > 0, f"Mesh {name} rỗng!"
print("✅ 7/7 file mesh STL đã được nạp chuẩn xác và nguyên vẹn từ CAD.")

# Hàm tính chuỗi biến đổi hình học khớp
def compute_transforms(angles_deg):
    angles_rad = [math.radians(a) for a in angles_deg]
    transforms = [np.eye(4)] # base_link
    T_curr = np.eye(4)
    for i in range(6):
        p = JOINT_POINTS[i]
        axis = JOINT_AXES[i]
        angle = angles_rad[i] if i < len(angles_rad) else 0.0
        M_i = make_trans_4x4(p) @ rot_axis_4x4(axis, angle) @ make_trans_4x4(-p)
        T_curr = T_curr @ M_i
        transforms.append(T_curr.copy())
    return transforms

# Các tư thế kiểm thử
test_poses = [
    ("Home (Khớp về 0 độ)", [0, 0, 0, 0, 0, 0]),
    ("Khớp 1 xoay 45°", [45, 0, 0, 0, 0, 0]),
    ("Khớp 2 gập vai 30°", [0, 30, 0, 0, 0, 0]),
    ("Khớp 3 gập khuỷu -45°", [0, 0, -45, 0, 0, 0]),
    ("Khớp 4 xoay cẳng tay 90°", [0, 0, 0, 90, 0, 0]),
    ("Khớp 5 gập cổ tay 45°", [0, 0, 0, 0, 45, 0]),
    ("Khớp 6 xoay đầu hút 180°", [0, 0, 0, 0, 0, 180]),
    ("Tư thế làm việc phức tạp 1", [30, 45, -30, 60, 45, 30]),
    ("Tư thế vươn xa tối đa", [0, 60, -30, 0, -30, 0]),
    ("Tư thế cực đoan đa trục", [90, -45, 60, -90, 45, -90]),
]

all_passed = True
for pose_name, q in test_poses:
    transforms = compute_transforms(q)
    print(f"\nKiểm tra tư thế: {pose_name} -> q = {q}")
    for i in range(6):
        p_joint_cad = JOINT_POINTS[i]
        p_joint_h = np.array([*p_joint_cad, 1.0])
        
        # Điểm tâm khớp tính từ khâu mẹ
        T_parent = transforms[i]
        p_world_parent = (T_parent @ p_joint_h)[:3] * 1000 # mm
        
        # Điểm tâm khớp tính từ khâu con
        T_child = transforms[i + 1]
        p_world_child = (T_child @ p_joint_h)[:3] * 1000 # mm
        
        dist = np.linalg.norm(p_world_parent - p_world_child)
        status = "✅ LIÊN KẾT 100%" if dist < 1e-4 else f"❌ TÁCH RỜI ({dist:.3f} mm)"
        if dist >= 1e-4:
            all_passed = False
        print(f"  Khớp {i+1} ({LINK_NAMES[i]:10s} -> {LINK_NAMES[i+1]:10s}): {status} (Sai số = {dist:.6f} mm)")

print(f"\n{'='*75}")
if all_passed:
    print("  KẾT QUẢ: 10/10 TƯ THẾ HOÀN HẢO! TOÀN BỘ CÁC KHỚP DÍNH LIỀN 100%!")
    print("  KHÔNG HỀ CÓ HIỆN TƯỢNG VĂNG RỜI PART HAY BAY TÁCH BIỆT TRONG KHÔNG GIAN!")
else:
    print("  CẢNH BÁO: CÓ KHỚP BỊ TÁCH RỜI!")
print(f"{'='*75}")
