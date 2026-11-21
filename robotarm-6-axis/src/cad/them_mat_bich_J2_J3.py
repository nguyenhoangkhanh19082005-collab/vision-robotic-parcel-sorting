r"""Thêm mặt bích 5 mm (vành Ø22 dày 2.1, 4 lỗ M3 trên vòng Ø16, moay-ơ Ø10 cao 11.95, lỗ Ø5) vào tâm puly lớn
J2 (Riemenscheibe 55mm) và J3 (Riemenscheibe 40mm): vành áp MẶT NGOÀI puly (phía đầu bu lông M5), moay-ơ hướng
ra ngoài; puly được khoan 4 lỗ mồi Ø2.5 sâu 8 mm trên vòng Ø16 để vít M3 (qua lỗ ren mặt bích) ăn vào nhựa.
Phía khâu (AX-2_5 / AX-3_2) không đủ chiều dày cho moay-ơ 12 mm nên không đặt ở mặt trong.
Đầu ra: assets/CAD_mat_bich/ (STEP + STL, hệ tọa độ lắp ráp Roboter.STEP) và kết quả kiểm tra va chạm.
    E:\UDMT\.venv\Scripts\python.exe src\cad\them_mat_bich_J2_J3.py
"""
import glob, os, sys
import numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "research")); sys.path.insert(0, os.path.dirname(__file__))
from cad_util import CACHE, cylinders, load, local_frame
from cad_khung_dong_co import verts
from tao_chi_tiet_dong_co_1_27 import collide, cyl, to_frame, vol, write_step, write_stl
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.STEPControl import STEPControl_Reader

OUT = os.path.join(ROOT, "assets", "CAD_mat_bich")
PCD, PILOT_D, PILOT_DEPTH = 16.0, 2.5, 8.0


def build():
    """Trả về (outs, asm, report): chi tiết đã sửa, danh sách (tên, shape) gồm cả 2 mặt bích đã đặt, báo cáo."""
    r = STEPControl_Reader(); r.ReadFile(os.path.join(OUT, "mat_bich_5mm_goc.step")); r.TransferRoots()
    flange = r.OneShape()  # cục bộ: trục +Z, mặt vành tại z = 0, moay-ơ tới z = 11.95, lỗ M3 tại (±8,0),(0,±8)
    report, outs, asm = [], {}, []
    for joint, pul, motor in [("J2", "Riemenscheibe_55mm__NAUO140", "NAUO156"), ("J3", "Riemenscheibe_40mm__NAUO120", "NAUO123")]:
        p = load(pul)
        c = max(cylinders(p), key=lambda c: c[0]); ax = c[2] / np.linalg.norm(c[2])
        R, o = local_frame(c[1], ax)
        w = (verts(p) - o) @ R[2]
        # mặt ngoài = phía xa động cơ (phía đầu bu lông M5, có lỗ khoét Ø9)
        if ((verts(load("STEPPERONLINE*" + motor)).mean(0) - o) @ R[2]) > w.mean():
            R = np.array([R[0], -R[1], -R[2]]); w = -w
        origin = o + R[2] * w.max()
        placed = to_frame(flange, origin, R)
        holes = None
        for k in range(4):
            a = np.radians(90 * k)
            h = cyl(PILOT_D / 2, -PILOT_DEPTH, 0.01, PCD / 2 * np.cos(a), PCD / 2 * np.sin(a))
            holes = h if holes is None else BRepAlgoAPI_Fuse(holes, h).Shape()
        p_new = BRepAlgoAPI_Cut(p, to_frame(holes, origin, R)).Shape()
        report.append(f"{joint}: {pul.split('__')[0]} khoan 4 lỗ Ø{PILOT_D} sâu {PILOT_DEPTH} trên vòng Ø{PCD:.0f} (bớt {vol(p)-vol(p_new):.0f} mm3)")
        # khối bao mặt bích + khe hở 1 mm (vành Ø24 × 3.2, moay-ơ Ø12.4 × 13.5) để khoét nắp/vỏ nếu chạm
        env = to_frame(BRepAlgoAPI_Fuse(cyl(12.0, -0.05, 3.2), cyl(6.2, 0, 13.5)).Shape(), origin, R)
        hits = []
        for f in sorted(glob.glob(os.path.join(CACHE, "*.brep"))):
            n = os.path.basename(f)[:-5]
            if n == pul:
                continue
            part = load(n)
            v = collide(placed, part)
            if v > 0.5 and n.startswith("AX-"):
                cut = BRepAlgoAPI_Cut(part, env).Shape()
                nm = n.split("__")[0] + f"_khoet_mat_bich_{joint}"
                outs[nm] = cut; asm.append((nm, cut))
                report.append(f"{joint}: {n.split('__')[0]} chạm mặt bích {v:.0f} mm3 → ĐÃ KHOÉT (bớt {vol(part)-vol(cut):.0f} mm3); "
                              f"còn chạm sau khoét: {collide(placed, cut):.1f} mm3")
            elif v > 0.5:
                hits.append(f"{n} {v:.0f} mm3")
        report.append(f"{joint}: mặt bích va chạm: " + (", ".join(hits) if hits else "KHÔNG"))
        name = f"Puly_{'55' if joint == 'J2' else '40'}mm_{joint}_khoan_mat_bich"
        outs[name] = p_new; asm += [(name, p_new), (f"Mat_bich_5mm_{joint}", placed)]
    return outs, asm, report


def main():
    outs, asm, report = build()
    for n, s in outs.items():
        write_step([(n, s)], os.path.join(OUT, n + ".step")); write_stl(s, os.path.join(OUT, n + ".stl"))
    write_step(asm, os.path.join(OUT, "Mat_bich_va_puly_J2_J3.step"))
    open(os.path.join(OUT, "ket_qua_kiem_tra.txt"), "w", encoding="utf-8").write("\n".join(report) + "\n")
    print("\n".join(report))


if __name__ == "__main__":
    main()
