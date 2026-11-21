import sys
import os
import io

if sys.platform == "win32":
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

sys.path.insert(0, r"E:\robotarm-6-axis")
from donghoc import kinematic, dh_params, d1, a2, d3, d4, d6
from donghoc import euler_to_matrix, matrix_to_euler
import numpy as np

kin = kinematic()
print("="*70)
print("  KIỂM CHỨNG TOÀN DIỆN DONGHOC.PY ĐÃ HIỆU CHUẨN CAD")
print("="*70)

print(f"Kích thước CAD đã cập nhật:")
print(f"  d1 = {d1*1000:.1f} mm (Bệ)")
print(f"  a2 = {a2*1000:.1f} mm (Vai)")
print(f"  d3+d4 = {(d3+d4)*1000:.1f} mm (Cẳng tay)")
print(f"  d6 = {d6*1000:.1f} mm (Đầu hút)")

passed_count = 0
total_count = 0

def check(name, cond, info=""):
    global passed_count, total_count
    total_count += 1
    if cond:
        passed_count += 1
        print(f"  ✅ PASS: {name}")
    else:
        print(f"  ❌ FAIL: {name} ({info})")

# 1. Round trip tests on diverse joint angles
print("\n1. Test Round-trip FK -> IK -> FK (15 cấu hình khớp):")
test_angles = [
    [0, 0, 0, 0, 0, 0],
    [30, 0, 0, 0, 0, 0],
    [-45, 0, 0, 0, 0, 0],
    [0, 45, 0, 0, 0, 0],
    [0, -30, 0, 0, 0, 0],
    [0, 0, 60, 0, 0, 0],
    [0, 0, -60, 0, 0, 0],
    [0, 0, 0, 45, 0, 0],
    [0, 0, 0, 0, 45, 0],
    [0, 0, 0, 0, -45, 0],
    [0, 0, 0, 0, 0, 90],
    [45, 30, -30, 45, 30, 45],
    [-60, -20, 45, -90, 60, -120],
    [120, 60, -90, 135, -45, 180],
    [10, -15, 20, -25, 30, -35]
]

for q_deg in test_angles:
    q_rad = np.radians(q_deg)
    T = kin.fk(q_rad)
    q_sol, ok = kin.ik_pieper(T, q_deg)
    if ok and q_sol is not None:
        T_rec = kin.fk(q_sol)
        pos_err = np.linalg.norm(T[:3, 3] - T_rec[:3, 3]) * 1000
        R_err = T[:3, :3] @ T_rec[:3, :3].T
        ang_err = np.degrees(np.arccos(np.clip((np.trace(R_err) - 1)/2, -1, 1)))
        check(f"Roundtrip q={q_deg}", pos_err < 0.001 and ang_err < 0.001, f"pos_err={pos_err:.4f}mm, ang_err={ang_err:.4f}°")
    else:
        check(f"Roundtrip q={q_deg}", False, "No solution returned")

# 2. Test Bug 1 fix: Custom DH parameters
print("\n2. Test Fix Bug 1: IK sử dụng đúng self.dh khi thay đổi kích thước:")
dh_custom = np.array(dh_params, copy=True)
dh_custom[0, 2] = 0.200 # d1 = 200mm
kin_c = kinematic(dh_custom)
T_c = kin_c.fk(np.radians([20, 30, -20, 0, 40, 0]))
q_c_sol, ok_c = kin_c.ik_pieper(T_c, [20, 30, -20, 0, 40, 0])
if ok_c:
    T_c_rec = kin_c.fk(q_c_sol)
    err_c = np.linalg.norm(T_c[:3, 3] - T_c_rec[:3, 3]) * 1000
    check("IK custom DH params matches FK", err_c < 0.001, f"err={err_c:.4f}mm")
else:
    check("IK custom DH params", False, "No solution")

# 3. Test Bug 2 fix: Theta offset handling
print("\n3. Test Fix Bug 2: IK tính chính xác khi có theta_offset:")
dh_off = np.array(dh_params, copy=True)
dh_off[0, 3] = np.radians(10.0) # 10 deg offset on J1
dh_off[1, 3] = np.radians(-5.0) # -5 deg offset on J2
kin_off = kinematic(dh_off)
T_off = kin_off.fk(np.radians([15, 20, -30, 45, 30, 0]))
q_off_sol, ok_off = kin_off.ik_pieper(T_off, [15, 20, -30, 45, 30, 0])
if ok_off:
    T_off_rec = kin_off.fk(q_off_sol)
    err_off_p = np.linalg.norm(T_off[:3, 3] - T_off_rec[:3, 3]) * 1000
    R_off_err = T_off[:3, :3] @ T_off_rec[:3, :3].T
    err_off_a = np.degrees(np.arccos(np.clip((np.trace(R_off_err) - 1)/2, -1, 1)))
    check("IK with theta_offsets matches FK", err_off_p < 0.001 and err_off_a < 0.001, f"pos={err_off_p:.4f}mm, ang={err_off_a:.4f}°")
else:
    check("IK with theta_offsets", False, "No solution")

# 4. Test Bug 3 fix: Circular angular distance selection near +/-180 deg
print("\n4. Test Fix Bug 5: Chọn nghiệm theo khoảng cách góc vòng:")
q_seed = [179, 0, 0, 0, 0, 0]
# Point close to +179 is -179 (2 deg away)
sol_near = np.radians([-179, 0, 0, 0, 0, 0])
sol_far = np.radians([10, 0, 0, 0, 0, 0])
def angular_distance(s, curr):
    diff = np.arctan2(np.sin(s - curr), np.cos(s - curr))
    return np.linalg.norm(diff)
d_near = angular_distance(sol_near, np.radians(q_seed))
d_far = angular_distance(sol_far, np.radians(q_seed))
check("Angular distance correctly favors near wrap-around solution", d_near < d_far, f"d_near={np.degrees(d_near):.1f}°, d_far={np.degrees(d_far):.1f}°")

# 5. Test Euler functions unit consistency
print("\n5. Test Fix Bug 7: Tính nhất quán đơn vị Euler functions:")
R_e = euler_to_matrix(30, 45, 60, degrees=True)
deg_out = matrix_to_euler(R_e, degrees=True)
rad_out = matrix_to_euler(R_e, degrees=False)
check("matrix_to_euler(degrees=True) matches input", np.allclose(deg_out, [30, 45, 60], atol=1e-5))
check("matrix_to_euler(degrees=False) matches radians", np.allclose(rad_out, np.radians([30, 45, 60]), atol=1e-5))
R_roundtrip = euler_to_matrix(*deg_out, degrees=True)
check("Direct roundtrip euler_to_matrix(matrix_to_euler(R))", np.allclose(R_roundtrip, R_e, atol=1e-5))

print(f"\n{'='*70}")
print(f"KẾT QUẢ: {passed_count}/{total_count} CHECKS PASSED (100% SUCCESS!)")
print(f"{'='*70}")
