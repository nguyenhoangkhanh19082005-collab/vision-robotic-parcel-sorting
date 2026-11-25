r"""Bước 2/3 xuất STEP toàn trạm: gán từng chi tiết STEP robot v3 vào khâu (base, 1..6) và tính ma trận đặt chi tiết
về hệ BASE robot ở tư thế q (mặc định HOME = 0, dựng thẳng đứng) → .tmp/tram_step/robot_dat.json (mm).
Cách làm: mesh khâu k của twin (file STL, mm) thỏa M·v_file = p_STEP + t_k → tìm t_k bằng bỏ phiếu hiệu tọa độ trên các
chi tiết tham chiếu; chi tiết trùng mesh khâu nào (sai lệch < 0.5 mm) thì thuộc khâu đó; chi tiết không có trong mesh
(động cơ mới, đế nâng, gá giác hút, ốc, vòng bi) lấy khâu của chi tiết đã gán gần nhất.
Đặt: X_base(m) = RobotModel.link_transforms_base(q)[k] · (Mᵀ(p + t_k)/1000)  — đúng cách twin đặt mesh STL.
    E:\UDMT\robotarm-6-axis\.venv\Scripts\python.exe src\cad\tram_step_2_gan_khau.py [q1 .. q6 độ]
"""
import json, os, struct, sys
from collections import Counter
import numpy as np
from scipy.spatial import cKDTree

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
OUT = os.path.join(ROOT, ".tmp", "tram_step")
M = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)
LINKS = ["base_link", "link_1_1", "link_2_1", "link_3_1", "link_4_1", "link_5_1", "link_6_1"]
REFS = [["AX-1_10", "AX-1_11"], ["AX-1_7", "AX-1_2"], ["AX-2_4", "AX-2_3"], ["AX-3_1", "AX-3_3"],
        ["AX-4_1", "AX-4_2"], ["AX-5_1", "AX-5_2"], ["AX-6"]]
T0 = np.array([72.0, -4.0, 264.0])
FORCE = {"De_nang_J1": 0, "Ga_giac_hut_J6_dai": 6, "Giac_hut_L60-M12_HSP-40_mo_phong": 6}


def stl_points(path):
    b = open(path, "rb").read(); n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b, dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]), count=n, offset=84)
    return np.unique(np.round(a["v"].reshape(-1, 3).astype(float), 3), axis=0)


def find_t(tree, mesh, pts):
    """t: mesh ≈ pts + t (hệ hướng STEP, mm). Bỏ phiếu hiệu tọa độ trong bán kính 40 mm, tinh chỉnh bằng NN."""
    pick = pts[np.random.default_rng(0).choice(len(pts), min(80, len(pts)), replace=False)]
    c = Counter()
    for p in pick:
        for j in tree.query_ball_point(p + T0, 15.0):  # độ lệch STEP→mesh ≈ (70, 0, 262) mm
            c[tuple(np.round(mesh[j] - p, 0))] += 1
    t = np.array(c.most_common(1)[0][0])
    for _ in range(5):
        d, j = tree.query(pts + t); ok = d < 1.5
        t = t + np.mean(mesh[j[ok]] - (pts[ok] + t), 0)
    d, _ = tree.query(pts + t)
    return t, float(np.median(d))


def main():
    sys.path.insert(0, r"E:\UDMT\SortingStation")
    from sorting_station.station_config import install_import_paths, load_station_config, resolve_paths
    cfg = load_station_config(); paths = resolve_paths(cfg); install_import_paths(paths)
    from sorting_station.robot_model import build_model_from_robot_project
    model = build_model_from_robot_project(paths.robot_root, tool_length_m=0.126)
    q = [float(x) for x in sys.argv[1:7]] if len(sys.argv) >= 7 else [0.0] * 6

    d = np.load(os.path.join(OUT, "robot_parts.npz"))
    names = [str(x) for x in d["names"]]; P = [d[f"p{i}"] for i in range(len(names))]
    mesh_dir = os.path.join(ROOT, "assets", "ROBOTARM2_description", "meshes")
    # khâu 4: mesh gốc trước khi co bầu (đoạn gần khuỷu trùng mesh đang dùng → cùng t_4; bầu trong STEP v3 đã ngắn)
    src = lambda l: os.path.join(mesh_dir, "goc_truoc_v3", l + ".stl") if l == "link_4_1" else os.path.join(mesh_dir, l + ".stl")
    meshes = [(M @ stl_points(src(l)).T).T for l in LINKS]
    trees = [cKDTree(m) for m in meshes]

    log = []
    t = []
    for k, refs in enumerate(REFS):
        rp = np.vstack([P[i] for i, n in enumerate(names) if n in refs])
        tk, err = find_t(trees[k], meshes[k], rp); t.append(tk)
        log.append(f"{LINKS[k]}: t = {np.round(tk, 2)} mm, sai lệch trung vị {err:.3f} mm")

    link = [-1] * len(names)
    for i, (n, p) in enumerate(zip(names, P)):
        if n in FORCE:
            link[i] = FORCE[n]; continue
        if n.startswith(("AX-2", "AX-3", "AX-4", "AX-5", "AX-6")):   # tên nhóm AX-k = khâu k (AX-1 chia đế / khâu 1)
            link[i] = int(n[3]); continue
        if len(p) == 0:
            continue
        scores = [np.median(trees[k].query(p + t[k])[0]) for k in range(7)]
        k = int(np.argmin(scores))
        if scores[k] < 0.5:
            link[i] = k
    # chi tiết chưa gán → khâu của chi tiết đã gán gần nhất (khoảng cách tập điểm, hệ STEP)
    for _ in range(3):
        done = [i for i in range(len(names)) if link[i] >= 0]
        dtrees = {i: cKDTree(P[i]) for i in done if len(P[i])}
        for i in range(len(names)):
            if link[i] >= 0 or len(P[i]) == 0:
                continue
            best = min(dtrees, key=lambda j: dtrees[j].query(P[i])[0].min())
            link[i] = link[best]
    unmatched = [n for n, l in zip(names, link) if l < 0]
    assert not unmatched, unmatched

    T = model.link_transforms_base(np.radians(q))
    place = []
    for i, n in enumerate(names):
        k = link[i]
        A = np.eye(4); A[:3, :3] = M.T / 1000.0; A[:3, 3] = M.T @ t[k] / 1000.0   # STEP(mm) → mesh file (m)
        G = T[k] @ A                                                               # STEP(mm) → base (m)
        G[:3, 3] *= 1000.0; G[:3, :3] *= 1000.0                                    # → base (mm), quay thuần
        place.append({"name": n, "link": k, "T": G.tolist()})
    json.dump({"q_deg": q, "parts": place}, open(os.path.join(OUT, "robot_dat.json"), "w", encoding="utf-8"))
    cnt = Counter(link)
    log.append("Số chi tiết mỗi khâu: " + ", ".join(f"{LINKS[k]} {cnt[k]}" for k in range(7)))
    for n, l in zip(names, link):
        if not n.startswith(("ISK", "Mutter", "DIN", "ISO", "Lager")):
            log.append(f"  {n:38s} → {LINKS[l]}")
    print("\n".join(log))


if __name__ == "__main__":
    main()
