r"""Dựng lại hình học động học của digital twin từ file lắp ráp STEP (trục khớp chính xác) → dong_hoc_step.py.

Vì sao: (1) mesh STL của twin (assets/ROBOTARM2_description) trùng từng chi tiết với STEP nhưng mỗi khâu bị tịnh tiến
lệch nhau 5–13 mm (bản xuất URDF chưa ghép khít), nên JOINT_POINTS cũ lệch trục thật 10–18 mm; (2) tư thế 0 của twin là
tư thế vẽ CAD (cánh tay trên nghiêng 5.9°, cẳng tay chúc lên 17.5°, cổ tay gập 15.5°) chứ không phải chữ L mà người
dùng đặt gốc trên robot thật.

Kết quả (hệ tọa độ twin = hệ mesh base_link, mét):
- JOINT_POINTS / JOINT_AXES ở TƯ THẾ 0 MỚI = chữ L chuẩn: cánh tay trên song song trục J1 (dựng đứng), trục J4 (cẳng
  tay) vuông góc trục J1 (nằm ngang), trục J5 song song trục J2, trục J6 trùng phương trục J4 (cổ tay thẳng).
- LINK_MESH_TRANSFORMS: ma trận đặt lại mesh từng khâu (ghép khít + xoay về tư thế 0 mới).
Chiều dương các khớp giữ nguyên như twin cũ.

21/11/2026 — theo lắp ráp v3 (assets/CAD_lap_rap_v3/Roboter_v3_J4_cu_bau_ngan_giac_hut_dai.step): J4 bản gốc chưa rút
ngắn, bầu AX-4_4 rút ngắn (đoạn s 154.5..187 → 16.5 mm) → trục J5, J6 và mesh khâu 5, 6 lùi 16 mm về phía khuỷu; mesh
khâu 4 được co lại đúng đoạn bầu (mesh gốc giữ ở meshes/goc_truoc_v3/). In thêm vị trí mặt bích J6 và chiều dài đầu hút
tương ứng gá giác hút dài (miệng giác cách mặt ngoài mặt bích AX-6 129 mm).
    E:\UDMT\.venv\Scripts\python.exe research\cad_doc_step_lap_rap.py      (tạo .tmp/step_lap_rap.json)
    E:\UDMT\.venv\Scripts\python.exe src\cad\tinh_dong_hoc_tu_step.py
"""
import json, os, struct, sys
from collections import Counter
import numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "research"))
from cad_util import load
from cad_khung_dong_co import verts

M = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)  # twin (x,y,z) → STEP (z, y, −x): mesh twin = STEP + t (mm)
MESH_DIR = os.path.join(ROOT, "assets", "ROBOTARM2_description", "meshes")
MESH_ORIG = os.path.join(MESH_DIR, "goc_truoc_v3")      # mesh gốc (bầu dài) — nguồn đo độ lệch và để co khâu 4
BAU_S0, BAU_B, BAU_NEW = 154.5, 187.0, 16.5             # đoạn bầu AX-4_4: s 154.5..187 (dọc cẳng tay từ khuỷu) → 16.5 mm
D_FUN = (BAU_B - BAU_S0) - BAU_NEW                      # 16.0 mm
CUP_FROM_FACE = 129.0                                   # miệng giác (chưa nén) cách mặt ngoài mặt bích AX-6, gá dài
LINKS = ["base_link", "link_1_1", "link_2_1", "link_3_1", "link_4_1", "link_5_1", "link_6_1"]
REF_PARTS = [["AX-1_10__NAUO194", "AX-1_11__NAUO195"], ["AX-1_7__NAUO168", "AX-1_2__NAUO164"], ["AX-2_1__NAUO124", "AX-2_4__NAUO126"],
             ["AX-3_1__NAUO95", "AX-3_3__NAUO97"], ["AX-4_4__NAUO47", "AX-4_1__NAUO43"], ["AX-5_1__NAUO22", "AX-5_2__NAUO23"], ["AX-6__NAUO1"]]
# Điểm/trục khớp cũ của twin (robot_simulation.py trước 15/11/2026) — chỉ dùng để giữ chiều dương và vị trí dọc trục
OLD_P = np.array([[-0.1544785, 0.1454204, -0.0507776], [-0.1538267, 0.1394184, 0.0824138], [-0.1373280, 0.1339924, 0.2348735],
                  [-0.2154871, 0.0110950, 0.2594626], [-0.2646521, -0.0790322, 0.2838017], [-0.2774943, -0.1114704, 0.2813030]])
OLD_A = np.array([[-0.000797, -0.083636, 0.996496], [0.872438, -0.487070, -0.040182], [0.872438, -0.487070, -0.040182],
                  [0.466352, 0.854273, -0.229639], [-0.815770, 0.515717, 0.261830], [-0.540858, -0.840599, -0.029428]])


def stl_vertices(path):
    b = open(path, "rb").read(); n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b, dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]), count=n, offset=84)
    return np.unique(np.round((M @ a["v"].reshape(-1, 3).astype(float).T).T, 3), axis=0)


def write_deformed_stl(src, dst, fn):
    """Ghi lại STL nhị phân với đỉnh v (mm, hệ file) → fn(v) (mảng N×3)."""
    b = bytearray(open(src, "rb").read()); n = struct.unpack("<I", b[80:84])[0]
    dt = np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")])
    a = np.frombuffer(bytes(b[84:84 + 50 * n]), dtype=dt).copy()
    v = fn(a["v"].reshape(-1, 3).astype(float)).reshape(-1, 9)
    a["v"] = v.astype(np.float32)
    tri = v.reshape(-1, 3, 3); nn = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    a["n"] = (nn / np.maximum(np.linalg.norm(nn, axis=1, keepdims=True), 1e-12)).astype(np.float32)
    open(dst, "wb").write(bytes(b[:84]) + a.tobytes())


def mesh_offset(mesh, part):
    """t sao cho mesh ≈ đỉnh chi tiết STEP + t (bỏ phiếu hiệu tọa độ, rồi lấy trung bình các đỉnh trùng)."""
    v = np.unique(np.round(verts(load(part)), 4), axis=0)
    pick = v[np.random.default_rng(0).choice(len(v), min(60, len(v)), replace=False)]
    c = Counter()
    for p in pick:
        d = mesh - p
        d = d[(abs(d[:, 0] - 70) < 30) & (abs(d[:, 1]) < 30) & (abs(d[:, 2] - 262) < 30)]
        c.update(map(tuple, np.round(d, 1)))
    (t, votes), = c.most_common(1)
    t = np.array(t); dd = []
    for p in v[:600]:
        d = mesh - p - t; k = np.argmin((d ** 2).sum(1))
        if np.linalg.norm(d[k]) < 0.3: dd.append(d[k])
    return t + np.mean(dd, 0), len(dd) / min(600, len(v))


def rot(axis, ang):
    a = axis / np.linalg.norm(axis); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K


def screw(p, a, ang):
    T = np.eye(4); R = rot(a, ang); T[:3, :3] = R; T[:3, 3] = p - R @ p
    return T


def signed_angle(v, w, axis):
    """Góc quay quanh `axis` đưa v về w (cả hai chiếu lên mặt phẳng vuông góc trục)."""
    a = axis / np.linalg.norm(axis)
    v = v - (v @ a) * a; w = w - (w @ a) * a
    return np.arctan2(a @ np.cross(v, w), v @ w)


def main():
    parts = json.load(open(os.path.join(ROOT, ".tmp", "step_lap_rap.json"), encoding="utf-8"))

    def axis_of(pred):
        p = next(q for q in parts if pred(q)); c = max(p["cyl"], key=lambda c: c["area"]); d = np.array(c["d"])
        return np.array(c["p"]), d / np.linalg.norm(d)

    S = [axis_of(lambda q: q["name"].startswith("Lager_78")), axis_of(lambda q: q["name"].startswith("Puly_55")),
         axis_of(lambda q: q["name"].startswith("Puly_40")), axis_of(lambda q: q["name"].startswith("STEPPERONLINE")),
         axis_of(lambda q: q["name"].startswith("Riemenscheibe_40mm") and q["com"][0] > 0),
         axis_of(lambda q: q["name"].startswith("Nema17") and q["com"][1] < -70)]
    log = []; P = log.append
    import shutil
    if not os.path.isdir(MESH_ORIG):                        # lần đầu: cất mesh gốc trước khi co khâu 4
        os.makedirs(MESH_ORIG)
        for l in LINKS: shutil.copy(os.path.join(MESH_DIR, l + ".stl"), MESH_ORIG)
    sys.path.insert(0, os.path.dirname(__file__))
    from rut_ngan_lib import E as E_, F as F_              # khuỷu và hướng cẳng tay (hệ STEP, mm)

    # ---- 1. độ lệch tịnh tiến của từng mesh so với STEP ----
    t = []
    for link, refs in zip(LINKS, REF_PARTS):
        mesh = stl_vertices(os.path.join(MESH_ORIG, link + ".stl"))
        r = [mesh_offset(mesh, part) for part in refs]
        t.append(np.mean([x[0] for x in r], 0))
        P(f"{link:9s}: mesh = STEP + {np.round(t[-1], 2)} mm  (" + ", ".join(f"{p.split('__')[0]} {np.round(x[0], 2)} trùng {x[1]*100:.0f}%" for p, x in zip(refs, r)) + ")")
    off = [-(M.T @ (tk - t[0])) / 1000 for tk in t]  # hệ twin (m): dời mesh khâu k để ghép khít như STEP
    # ---- 1b. bầu ngắn: khâu 5, 6 + trục J5, J6 lùi D_FUN về phía khuỷu; co đoạn bầu trong mesh khâu 4 ----
    lui = -F_ * D_FUN
    for k in (5, 6): off[k] = off[k] + M.T @ lui / 1000
    S[4] = (S[4][0] + lui, S[4][1]); S[5] = (S[5][0] + lui, S[5][1])

    def co_bau(v):                                    # v: đỉnh mesh khâu 4 (mm, hệ file twin)
        p = (M @ v.T).T - t[4]; sv = (p - E_) @ F_      # về hệ STEP, s dọc cẳng tay
        ds = np.where(sv >= BAU_B, D_FUN, np.where(sv > BAU_S0, (sv - BAU_S0) * D_FUN / (BAU_B - BAU_S0), 0.0))
        return (M.T @ (p - np.outer(ds, F_) + t[4]).T).T
    write_deformed_stl(os.path.join(MESH_ORIG, "link_4_1.stl"), os.path.join(MESH_DIR, "link_4_1.stl"), co_bau)
    P(f"Bầu ngắn: trục J5, J6 + mesh khâu 5, 6 lùi {D_FUN:g} mm; mesh khâu 4 co đoạn s {BAU_S0}..{BAU_B} → {BAU_NEW} mm "
      f"(mesh gốc: {os.path.relpath(MESH_ORIG, ROOT)})")
    P("Dời mesh từng khâu để ghép khít (mm, hệ twin): " + "; ".join(f"{l} {np.round(o*1000, 1)}" for l, o in zip(LINKS, off)))

    # ---- 2. trục khớp STEP trong hệ twin (tư thế CAD) ----
    pts, axs = [], []
    for i in range(6):
        o = M.T @ (S[i][0] + t[0]) / 1000; d = M.T @ S[i][1]
        d = d * np.sign(d @ OLD_A[i])                         # giữ chiều dương cũ
        old_moved = OLD_P[i] + off[i + 1]                     # điểm cũ đi theo mesh khâu i
        r = OLD_P[i] - o; res_old = np.linalg.norm(r - (r @ d) * d) * 1000
        r = old_moved - o; res_new = np.linalg.norm(r - (r @ d) * d) * 1000
        pts.append(o + ((old_moved - o) @ d) * d); axs.append(d)
        P(f"J{i+1}: điểm twin cũ cách trục STEP {res_old:5.1f} mm (sau khi dời theo mesh khâu {i+1}: {res_new:4.1f} mm); lệch hướng {np.degrees(np.arccos(min(1, abs(d @ OLD_A[i]/np.linalg.norm(OLD_A[i]))))):.2f}°")

    # ---- 3. góc đưa tư thế CAD về chữ L chuẩn ----
    up = axs[0]
    arm = pts[2] - pts[1]; arm = arm - (arm @ axs[1]) * axs[1]
    q0 = np.zeros(6)
    q0[1] = signed_angle(arm, up, axs[1])
    R = rot(axs[1], q0[1])
    fore = axs[3] * np.sign(axs[3] @ (pts[4] - pts[2]))       # hướng cẳng tay (khuỷu → cổ tay)
    h = np.cross(axs[1], up); h = h * np.sign(h @ (R @ fore))  # phương ngang trong mặt phẳng tay, phía cẳng tay đang chỉ
    q0[2] = signed_angle(fore, R.T @ h, axs[2])
    R = R @ rot(axs[2], q0[2])
    tgt = R.T @ axs[1]; tgt = tgt * np.sign(tgt @ axs[4])
    q0[3] = signed_angle(axs[4], tgt, axs[3])
    tool = axs[5] * np.sign(axs[5] @ fore)
    q0[4] = signed_angle(tool, fore, axs[4])
    P("Góc đưa tư thế CAD về chữ L chuẩn (độ, theo chiều dương twin): " + ", ".join(f"J{i+1} {np.degrees(q0[i]):+.2f}" for i in range(6)))

    # ---- 4. nướng tư thế 0 mới vào điểm/trục khớp và ma trận mesh ----
    T = np.eye(4); new_p, new_a, mesh_T = [], [], [np.eye(4)]
    for i in range(6):
        T = T @ screw(pts[i], axs[i], q0[i])
        new_p.append((T @ np.append(pts[i], 1))[:3]); new_a.append(T[:3, :3] @ axs[i])
        D = np.eye(4); D[:3, 3] = off[i + 1]
        mesh_T.append(T @ D)

    # ---- 5. kiểm tra tư thế 0 mới ----
    up = new_a[0]
    arm = new_p[2] - new_p[1]; arm -= (arm @ new_a[1]) * new_a[1]
    wrist = new_p[4]  # nằm trên trục J5; tâm cổ tay = giao J4 ∩ J5
    r = new_p[4] - new_p[3]; wc = new_p[3] + (r @ new_a[3]) * new_a[3]
    L2 = np.linalg.norm(arm) * 1000
    rr = wc - new_p[2]; L3 = np.linalg.norm(rr - (rr @ new_a[2]) * new_a[2]) * 1000
    P(f"Tư thế 0 mới: cánh tay trên lệch phương đứng {np.degrees(np.arccos(np.clip(arm @ up / np.linalg.norm(arm), -1, 1))):.3f}°; "
      f"trục J4 so với phương ngang {np.degrees(np.arcsin(abs(new_a[3] @ up))):.3f}°; J5∥J2 lệch {np.degrees(np.arccos(min(1, abs(new_a[4] @ new_a[1])))):.3f}°; "
      f"J6 so với J4 {np.degrees(np.arccos(min(1, abs(new_a[5] @ new_a[3])))):.3f}°")
    P(f"Kích thước: J2–J3 = {L2:.2f} mm; J3 → tâm cổ tay = {L3:.2f} mm; tâm cổ tay → điểm J6 = {np.linalg.norm(new_p[5] - wc)*1000:.2f} mm; "
      f"J2 cao hơn điểm J1 {((new_p[1] - new_p[0]) @ up)*1000:.2f} mm; lệch trục J4–J3 {abs((new_p[3]-new_p[2]) @ np.cross(new_a[2], new_a[3]))*1000:.3f} mm")

    # ---- 6. mặt bích J6 và chiều dài đầu hút (kinematics_poe: TCP = điểm J6 + (7.4 mm + tool) dọc trục J6) ----
    v6 = verts(load("AX-6__NAUO1")); d6 = S[5][1] * np.sign((v6.mean(0) - S[5][0]) @ S[5][1])
    face = ((v6 + lui - S[5][0]) @ d6).max()                               # mặt ngoài mặt bích, tính từ điểm trên trục J6 (STEP)
    p6 = M @ (pts[5] * 1000) - t[0]                                        # điểm J6 của twin (tư thế CAD) về hệ STEP
    face_from_p6 = face - (p6 - S[5][0]) @ d6
    tool = CUP_FROM_FACE + face_from_p6 - 7.4
    P(f"Mặt ngoài mặt bích AX-6 cách điểm J6 {face_from_p6:.2f} mm dọc trục; miệng giác cách mặt bích {CUP_FROM_FACE:g} mm "
      f"→ tool_length = {tool:.1f} mm (với flange_offset 7.4 mm của kinematics_poe)")
    TOOL_MM = round(tool, 1)

    out = os.path.join(ROOT, "dong_hoc_step.py")
    with open(out, "w", encoding="utf-8") as fh:
        fh.write('"""TỰ SINH bởi src/cad/tinh_dong_hoc_tu_step.py — KHÔNG sửa tay.\n'
                 "Hình học động học lấy từ trục khớp trong assets/CAD_lap_rap_v3/Roboter_v3_J4_cu_bau_ngan_giac_hut_dai.step (hệ mesh base_link, mét):\n"
                 "J1–J3 động cơ 1:27, J4 bản gốc, bầu AX-4_4 rút ngắn (J5, J6 lùi 16 mm), gá giác hút dài.\n"
                 "Tư thế 0 = chữ L chuẩn: cánh tay trên dựng đứng (song song trục J1), cẳng tay nằm ngang, trục J5 song song trục J2,\n"
                 'cổ tay thẳng (trục J6 trùng phương cẳng tay). Đặt robot thật đúng tư thế này rồi mới "Lưu Gốc".\n"""\n'
                 "import numpy as np\n\n")
        fh.write("JOINT_POINTS = [\n" + "".join(f"    np.array([{p[0]:.7f}, {p[1]:.7f}, {p[2]:.7f}]),  # J{i+1}\n" for i, p in enumerate(new_p)) + "]\n\n")
        fh.write("JOINT_AXES = [\n" + "".join(f"    np.array([{a[0]:.8f}, {a[1]:.8f}, {a[2]:.8f}]),\n" for a in new_a) + "]\n\n")
        fh.write("# Ma trận đặt mesh STL (tọa độ gốc của file, mét) về tư thế 0 mới: ghép khít các khâu + xoay từ tư thế vẽ CAD.\n")
        fh.write("LINK_MESH_TRANSFORMS = {\n" + "".join(
            f'    "{l}": np.array({np.array2string(np.round(Tm, 9), separator=", ", max_line_width=200).replace(chr(10), "")}),\n' for l, Tm in zip(LINKS, mesh_T)) + "}\n\n")
        fh.write(f"# Góc (độ) từ tư thế vẽ CAD tới tư thế 0 mới — để tham khảo/chuyển đổi điểm dạy cũ: q_mới = q_cũ − CAD_TO_ZERO_DEG\n"
                 f"CAD_TO_ZERO_DEG = {[round(float(np.degrees(x)), 3) for x in q0]}\n\n"
                 "# Chiều dài đầu hút (mm) theo quy ước kinematics_poe: TCP = điểm J6 + (7.4 + TOOL_LENGTH_MM) dọc trục J6.\n"
                 "# Gá giác hút dài: miệng giác (chưa nén) cách mặt ngoài mặt bích AX-6 129 mm. Đo lại trên hàng thật nếu khác.\n"
                 f"TOOL_LENGTH_MM = {TOOL_MM}\n")
    txt = "\n".join(log); print(txt); print("Đã ghi:", out)
    os.makedirs(os.path.join(ROOT, "reports", "dong_hoc"), exist_ok=True)
    open(os.path.join(ROOT, "reports", "dong_hoc", "dong_hoc_tu_step.txt"), "w", encoding="utf-8").write(txt + "\n")


if __name__ == "__main__":
    main()
