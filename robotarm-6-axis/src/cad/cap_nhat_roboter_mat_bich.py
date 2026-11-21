r"""Cập nhật file lắp ráp assets/CAD_dong_co_1_27/Roboter_dong_co_1_27.step (bản đã có 3 động cơ mới J1–J3):
thêm 2 mặt bích 5 mm ở tâm puly lớn J2/J3, thay puly 55/40 mm bằng bản đã khoan lỗ mồi và nắp AX-2_3 bằng bản đã khoét.
Bản cũ được giữ lại thành Roboter_dong_co_1_27_truoc_mat_bich.step.
    E:\UDMT\.venv\Scripts\python.exe src\cad\cap_nhat_roboter_mat_bich.py
"""
import glob, os, shutil, sys
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "research")); sys.path.insert(0, os.path.dirname(__file__))
from cad_util import CACHE, load
from cad_khung_dong_co import MotorFrame
from tao_chi_tiet_dong_co_1_27 import OUT, collide, frame_of, new_motor_local, to_frame, write_step
from them_mat_bich_J2_J3 import build
from OCP.STEPControl import STEPControl_Reader


def read_step(path):
    r = STEPControl_Reader(); r.ReadFile(path); r.TransferRoots()
    return r.OneShape()


def main():
    target = os.path.join(OUT, "Roboter_dong_co_1_27.step")
    backup = os.path.join(OUT, "Roboter_dong_co_1_27_truoc_mat_bich.step")
    if not os.path.exists(backup):
        shutil.copy2(target, backup)

    # --- phần động cơ mới (giống tao_chi_tiet_dong_co_1_27.py; chi tiết đã sửa đọc lại từ file đã xuất) ---
    o1, R1 = frame_of(MotorFrame("Nema17*NAUO167", 2.5, 11.0), -1.9)
    o2, R2 = frame_of(MotorFrame("STEPPERONLINE*NAUO156", 3.0, 18.0))
    o3, R3 = frame_of(MotorFrame("STEPPERONLINE*NAUO123", 3.0, 18.0))
    ml = new_motor_local()
    motors = {"J1": to_frame(ml, o1, R1), "J2": to_frame(ml, o2, R2), "J3": to_frame(ml, o3, R3)}
    replaced = {"AX-2_2__NAUO125": read_step(os.path.join(OUT, "AX-2_2_sua_J3.step")),
                "AX-1_11__NAUO195": read_step(os.path.join(OUT, "AX-1_11_sua_J1_M2.5.step"))}
    removed = ("NAUO156", "NAUO123", "NAUO167")

    # --- phần mặt bích ---
    outs, asm_flange, _ = build()
    swap = {}  # tên chi tiết gốc trong cache → (tên mới, shape)
    for nm, s in outs.items():
        if nm.startswith("Puly_55"):
            swap["Riemenscheibe_55mm__NAUO140"] = (nm, s)
        elif nm.startswith("Puly_40"):
            swap["Riemenscheibe_40mm__NAUO120"] = (nm, s)
        else:  # AX-x_y_khoet_mat_bich_Jn
            key = [os.path.basename(f)[:-5] for f in glob.glob(os.path.join(CACHE, nm.split("_khoet")[0] + "__*.brep"))]
            assert len(key) == 1, (nm, key)
            swap[key[0]] = (nm, s)
    flanges = [(n, s) for n, s in asm_flange if n.startswith("Mat_bich_5mm")]

    assembly = []
    for f in sorted(glob.glob(os.path.join(CACHE, "*.brep"))):
        n = os.path.basename(f)[:-5]
        if any(r in n for r in removed):
            continue
        if n in swap:
            assembly.append(swap[n])
        elif n in replaced:
            assembly.append((n.split("__")[0] + "_sua", replaced[n]))
        else:
            assembly.append((n.split("__")[0], load(n)))
    assembly.append(("De_nang_J1", read_step(os.path.join(OUT, "De_nang_J1.step"))))
    assembly += [(f"Dong_co_moi_{j}", m) for j, m in motors.items()]
    assembly += flanges

    # --- kiểm tra mặt bích với động cơ MỚI và chi tiết đã sửa (script mặt bích chỉ kiểm tra với mô hình gốc) ---
    lines = []
    for fn, fs in flanges:
        hits = [f"{n} {v:.1f} mm3" for n, s in assembly if n != fn for v in [collide(fs, s)] if v > 0.5]
        lines.append(f"{fn}: " + ("KHÔNG va chạm" if not hits else ", ".join(hits)))
    write_step(assembly, target)
    lines.append(f"Số chi tiết trong file lắp ráp: {len(assembly)}")
    open(os.path.join(OUT, "ket_qua_cap_nhat_mat_bich.txt"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print("\n".join(lines)); print("Đã ghi:", target)


if __name__ == "__main__":
    main()
