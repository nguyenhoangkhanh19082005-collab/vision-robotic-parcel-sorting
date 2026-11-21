r"""LẮP RÁP v3 (21/11/2026) — đúng cấu hình robot thật theo người dùng:

  J1–J3 : động cơ mới 42STH38 + hộp số 1:27, mặt bích 5 mm phía khâu, puly nhỏ +3 mm (lấy từ lắp ráp v2,
          assets/CAD_v2_dai_that_mat_bich/Roboter_1_27_giac_hut_cang_dai_v2_mat_bich.step).
          Vỏ J2 = AX-1_3_cang_dai_giua_nhanh_J2 (trụ căng giữa nhánh, 20/11).
          Vỏ J3 = AX-2_1_cang_dai_J3 (trụ căng bản 18/11 đang dùng với rãnh trượt chữ U); nắp J3 AX-2_3 bản gốc.
  J4    : thiết kế cũ CHƯA rút ngắn (AX-3_1, AX-3_2 gốc) + động cơ cũ 17HS13-0404S-PG5 ở vị trí gốc.
  Bầu   : AX-4_4_rut_ngan (đoạn bầu 32.5 → 16.5 mm) → mọi chi tiết sau bầu (khâu 4 phía cổ tay, J5, J6) lùi 16 mm
          về phía khuỷu. Trục J3 → tâm cổ tay 267.0 → 251.0 mm.
  J5    : giữ nguyên.
  J6    : gá giác hút bản DÀI (tao_ga_giac_hut_J6_dai.py) thay gá cũ.
Đầu ra: assets/CAD_lap_rap_v3/ (STEP lắp ráp có tên chi tiết, ket_qua_kiem_tra.txt, ảnh).
    E:\UDMT\.venv\Scripts\python.exe -X utf8 src\cad\cap_nhat_lap_rap_v3.py
"""
import os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from rut_ngan_lib import parts, loadi, vol, com, shifted, E, F, P2, P3, AXES
from cad_util import load, bbox
from tao_chi_tiet_dong_co_1_27 import write_step, write_stl, collide, cyl, to_frame
import tao_ga_giac_hut_J6_dai as GD
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.STEPControl import STEPControl_Reader
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDataStd import TDataStd_Name
from OCP.collections import Sequence_TDF_Label
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf

ROOT = GD.ROOT; A = os.path.join(ROOT, "assets")
OUT = os.path.join(A, "CAD_lap_rap_v3")
V2 = os.path.join(A, "CAD_v2_dai_that_mat_bich", "Roboter_1_27_giac_hut_cang_dai_v2_mat_bich.step")
FUN_S0, FUN_B, FUN_NEW, SHIFT_2_RN = 154.5, 187.0, 16.5, 15.5     # thông số bầu rút ngắn (rut_ngan_cang_tay.py)
D_FUN = (FUN_B - FUN_S0) - FUN_NEW                                  # 16.0 mm
LOG = []


def P(*a):
    s = " ".join(str(x) for x in a); print(s, flush=True); LOG.append(s)


def rstep(p):
    r = STEPControl_Reader(); r.ReadFile(p); r.TransferRoots(); return r.OneShape()


def read_named(p):
    doc = TDocStd_Document(TCollection_ExtendedString("x"))
    rd = STEPCAFControl_Reader(); rd.SetNameMode(True); rd.ReadFile(p); rd.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()); labs = Sequence_TDF_Label(); st.GetFreeShapes(labs)
    out = []
    for i in range(1, labs.Length() + 1):
        lab = labs.Value(i); nm = TDataStd_Name()
        out.append((nm.Get().ToExtString() if lab.FindAttribute(TDataStd_Name.GetID_s(), nm) else f"x{i}", st.GetShape_s(lab)))
    return out


def s_of(c):
    """(s dọc cẳng tay tính từ khuỷu, thuộc cẳng tay?) theo tâm khối c (giống chain_shift của bản rút ngắn)."""
    v = c - E; s = v @ F; r = np.linalg.norm(v - s * F)
    return s, ((c - P2) @ (P3 - P2) / np.linalg.norm(P3 - P2) > 100 and r < 90)


def rotated(shape, point, axis, deg):
    t = gp_Trsf(); t.SetRotation(gp_Ax1(gp_Pnt(*point), gp_Dir(*axis)), np.radians(deg))
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def main():
    t0 = time.time(); os.makedirs(OUT, exist_ok=True)
    v2 = read_named(V2)
    P(f"Lắp ráp v2: {len(v2)} chi tiết ({time.time() - t0:.0f} s)")
    pcom = np.array([q["com"] for q in parts])
    asm, pre = [], {}          # asm: [name, shape, tag]; pre: tên → hình ở vị trí v2 (để so chồng lấn có sẵn)
    for name, s in v2:
        c = com(s)
        k = int(np.argmin(np.linalg.norm(pcom - c, axis=1))); idx = k if np.linalg.norm(pcom[k] - c) < 0.05 else None
        if name in ("Ga_giac_hut_J6", "Giac_hut_L60-M12_HSP-40_mo_phong"):
            P(f"  bỏ {name} (gá cũ)"); continue
        if name == "AX-1_3_cang_dai_J2_v2":
            asm.append(["AX-1_3_cang_dai_giua_nhanh_J2", rstep(os.path.join(A, "CAD_vo_J2_cang_dai_giua_nhanh", "AX-1_3_cang_dai_giua_nhanh_J2.step")), "moi"])
        elif name == "AX-2_1_cang_dai_J3_v2":
            asm.append(["AX-2_1_cang_dai_J3", rstep(os.path.join(A, "CAD_cang_dai_J2_J3", "AX-2_1_cang_dai_J3.step")), "moi"])
        elif name == "AX-2_3_ranh_cang_dai_J3":
            asm.append(["AX-2_3", load("AX-2_3__NAUO127"), "moi"])
        elif name == "AX-4_4":
            s_new = shifted(rstep(os.path.join(A, "CAD_rut_ngan", "AX-4_4_rut_ngan.step")), -SHIFT_2_RN)   # đầu gần về đúng vị trí gốc
            asm.append(["AX-4_4_rut_ngan", s_new, "bau"]); pre["AX-4_4_rut_ngan"] = s
        else:
            sc, arm = s_of(c)
            if arm and sc > FUN_S0 + 0.5:
                assert sc > FUN_B, (name, sc)
                asm.append([name, shifted(s, D_FUN), "lui"]); pre[name] = s
            else:
                asm.append([name, s, "giu"])
            if sc > 140 and arm: pass
        if asm[-1][2] == "moi": P(f"  {name} → {asm[-1][0]} (thể tích {vol(asm[-1][1]) / 1000:.2f} cm3)")
        if asm[-1][2] != "moi" and asm[-1][0] != "AX-4_4_rut_ngan" and idx is None and name.startswith(("AX-1_3", "AX-2_1", "AX-2_3")):
            P(f"  ? {name} không khớp chi tiết gốc nào")
    lui = [a for a in asm if a[2] == "lui"]
    P(f"  AX-4_4 → AX-4_4_rut_ngan (bầu 32.5 → 16.5 mm); {len(lui)} chi tiết phía cổ tay lùi {D_FUN:g} mm về phía khuỷu: "
      + ", ".join(sorted({a[0].split('__')[0] for a in lui})))

    # ---- kiểm tra bầu ngắn khớp 2 đầu: đầu gần trùng AX-4_4 gốc, đầu xa trùng AX-4_4 gốc đã lùi 16 mm ----
    from rut_ngan_lib import keep, R4, common
    old44, new44 = pre["AX-4_4_rut_ngan"], next(a[1] for a in asm if a[0] == "AX-4_4_rut_ngan")
    for tag, a_, b_ in (("đầu gần (s < 150)", keep(old44, -400, 150, R4), keep(new44, -400, 150, R4)),
                        ("đầu xa (sau bầu)", keep(shifted(old44, D_FUN), FUN_B - D_FUN + 1, 400, R4), keep(new44, FUN_B - D_FUN + 1, 400, R4))):
        P(f"  AX-4_4 {tag}: trùng gốc {vol(common(a_, b_)) / max(vol(a_), 1e-9) * 100:.1f} % thể tích")
    L = np.linalg.norm(AXES[4][0] - E)
    from rut_ngan_lib import WRIST
    P(f"Trục J3 → tâm cổ tay: {np.linalg.norm(WRIST - E):.1f} → {np.linalg.norm(WRIST - E) - D_FUN:.1f} mm")

    # ---- gá giác hút dài ----
    bracket, cup, z_barb, z_tip = GD.build(); face, R = GD.flange_frame()
    asm.append(["Ga_giac_hut_J6_dai", shifted(to_frame(bracket, face, R), D_FUN), "moi"])
    asm.append(["Giac_hut_L60-M12_HSP-40_mo_phong", shifted(to_frame(cup, face, R), D_FUN), "moi"])
    P(f"  thêm Ga_giac_hut_J6_dai (46 × 45 × {GD.H:g} mm) + giác hút mô phỏng; miệng giác cách mặt bích J6 {z_tip:.0f} mm")
    P(f"\nLắp ráp v3: {len(asm)} chi tiết ({time.time() - t0:.0f} s)")

    # ---- chồng lấn ----
    P("\nChồng lấn (> 0.5 mm3) của chi tiết mới / đổi chỗ:")
    boxes = [bbox(a[1]) for a in asm]
    def near(i, j, m=0.5):
        (a0, a1), (b0, b1) = boxes[i], boxes[j]; return np.all(a0 - m < b1) and np.all(b0 - m < a1)
    ALLOW = {frozenset(("Ga_giac_hut_J6_dai", "Giac_hut_L60-M12_HSP-40_mo_phong"))}
    for i, (n, s, tag) in enumerate(asm):
        if tag not in ("moi", "bau"): continue
        hits = []
        for j, (m, t, tg) in enumerate(asm):
            if j == i or not near(i, j) or frozenset((n, m)) in ALLOW or (tg in ("moi", "bau") and j < i): continue
            v = collide(s, t)
            if v > 0.5:
                p0 = collide(pre.get(n, s), pre.get(m, t)) if n in pre or m in pre else None
                hits.append(f"{m} {v:.1f} mm3" + ("" if p0 is None else (f" (bản gốc {p0:.1f})" if p0 > 0.5 else " (MỚI)")))
        P(f"  {n}: {'KHÔNG' if not hits else ', '.join(hits)}")
    # nhóm lùi 16 mm với nhóm giữ nguyên: chỉ báo cặp chồng lấn MỚI so với v2
    new_pairs = []
    for i, (n, s, tag) in enumerate(asm):
        if tag != "lui": continue
        for j, (m, t, tg) in enumerate(asm):
            if tg != "giu" or not near(i, j): continue
            v = collide(s, t)
            if v > 0.5:
                p0 = collide(pre[n], t)
                if v > p0 + 0.5: new_pairs.append(f"{n} × {m} {v:.1f} mm3 (v2 {p0:.1f})")
    P("  Nhóm lùi 16 mm × nhóm giữ nguyên: " + ("không có chồng lấn mới" if not new_pairs else "; ".join(new_pairs)))

    # ---- quét J5: gá + giác hút (J6 quay đủ vòng) so với khâu 4 / cẳng tay ----
    p5 = AXES[4][0] - F * D_FUN; d5 = AXES[4][1]
    fixed = [(n, s) for n, s, tag in asm if n.startswith(("AX-4", "AX-3", "STEPPERONLINE", "Dong_co"))]
    j5m = [(n, s) for n, s, tag in asm if n.startswith("Nema17") and np.linalg.norm(com(s) - (pcom[182] - F * D_FUN)) < 1]
    fixed += j5m
    P(f"\nQuét J5 (so với tư thế CAD, bước 5°), khối bao gá khi J6 quay đủ vòng + giác hút; so với {len(fixed)} chi tiết khâu 4/cẳng tay:")
    for tag, Hb, zt in (("gá cũ (cao 45, miệng giác 99)", 45.0, z_tip - 30.0), (f"gá dài (cao {GD.H:g}, miệng giác {z_tip:.0f})", GD.H, z_tip)):
        env = BRepAlgoAPI_Fuse(cyl(np.hypot(GD.HALF_X, GD.HALF_Y) + 0.5, 0.2, Hb), cyl(21.0, Hb, zt)).Shape()
        env = shifted(to_frame(env, face, R), D_FUN)
        bad = []
        for a in range(-150, 151, 5):
            e = rotated(env, p5, d5, a)
            if any(collide(e, s) > 1.0 for _, s in fixed): bad.append(a)
        free = [a for a in range(-150, 151, 5) if a not in bad]
        lo = max([a for a in bad if a < 0], default=None); hi = min([a for a in bad if a > 0], default=None)
        P(f"  {tag}: không chạm từ {'' if lo is None else lo + 5}{'−150' if lo is None else ''}° tới {'' if hi is None else hi - 5}{'+150' if hi is None else ''}°"
          f" (chạm: {bad[:3]}…{bad[-3:] if bad else ''})")

    # ---- xuất ----
    path = os.path.join(OUT, "Roboter_v3_J4_cu_bau_ngan_giac_hut_dai.step")
    write_step([(n, s) for n, s, _ in asm], path)
    P(f"\nĐã ghi {path} ({time.time() - t0:.0f} s)")
    open(os.path.join(OUT, "ket_qua_kiem_tra.txt"), "w", encoding="utf-8").write("\n".join(LOG) + "\n")

    # ---- ảnh tổng thể ----
    import pyvista as pv
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.StlAPI import StlAPI_Writer
    tmp = os.path.join(ROOT, ".tmp", "v3", "mesh"); os.makedirs(tmp, exist_ok=True)
    col = {"moi": "#f0883e", "bau": "#d2a8ff", "lui": "#9ecbff", "giu": "#c9d1d9"}
    pl = pv.Plotter(off_screen=True, window_size=(1600, 1100))
    for k, (n, s, tag) in enumerate(asm):
        f = os.path.join(tmp, f"{k:03d}.stl"); BRepMesh_IncrementalMesh(s, 0.4, False, 0.5, True)
        w = StlAPI_Writer(); w.ASCIIMode = False; w.Write(s, f)
        try: pl.add_mesh(pv.read(f), color=col[tag])
        except Exception: pass
    pl.add_text("CAM = chi tiet moi (vo J2, vo J3, nap J3 goc, ga giac hut dai)   TIM = bau AX-4_4 rut ngan\n"
                "XANH = cum co tay lui 16 mm (J5, J6)   XAM = giu nguyen (J1-J3 dong co 1:27, J4 cu)", font_size=10, color="black")
    pl.set_background("white"); pl.camera_position = "xz"; pl.camera.azimuth = 35; pl.camera.elevation = 20; pl.reset_camera(); pl.camera.zoom(1.3)
    pl.screenshot(os.path.join(OUT, "anh_lap_rap_v3.png")); pl.close()
    P(f"Ảnh: {os.path.join(OUT, 'anh_lap_rap_v3.png')} ({time.time() - t0:.0f} s)")
    open(os.path.join(OUT, "ket_qua_kiem_tra.txt"), "w", encoding="utf-8").write("\n".join(LOG) + "\n")


if __name__ == "__main__":
    main()
