r"""Kiểm tra va chạm robot ↔ thiết bị trạm (gá camera, webcam, đèn ring, cảm biến) trên đường đi chu trình gắp – thả.
Robot: mesh twin đặt theo RobotModel; thiết bị: SortingStation/assets/station_equipment (mm, hệ base).
Đường đi mỗi thùng: HOME → trên điểm gắp → điểm gắp → trên điểm gắp → trên thùng → điểm thả → trên thùng → HOME
(nội suy khớp như bộ tuần tự). Khoảng cách = từ đỉnh mesh robot (thưa ≈ 4 mm) tới mặt thiết bị (implicit distance).
    E:\UDMT\.venv\Scripts\python.exe -X utf8 src\cad\kiem_tra_va_cham_tram.py [hệ_số_đèn ...]
"""
import json, os, sys
import numpy as np
import pyvista as pv
sys.path.insert(0, r"E:\UDMT\SortingStation")
from sorting_station.station_config import install_import_paths, load_station_config, resolve_paths
cfg = load_station_config(); paths = resolve_paths(cfg); install_import_paths(paths)
from sorting_station.robot_model import build_model_from_robot_project

EQ = r"E:\UDMT\SortingStation\assets\station_equipment"
MESH = os.path.join(paths.robot_root, "assets", "ROBOTARM2_description", "meshes")
LINKS = ["base_link", "link_1_1", "link_2_1", "link_3_1", "link_4_1", "link_5_1", "link_6_1"]


def main():
    model = build_model_from_robot_project(paths.robot_root, tool_length_m=0.126)
    srt = cfg["sorting"]; app = float(srt.get("approach_height_mm", 60))
    pick = np.array(srt["pick_xyz_mm"], float)
    home = np.array(srt.get("home_joints_deg", [0] * 6), float)
    scales = [float(x) for x in sys.argv[1:]] or [1.0]
    man = json.load(open(os.path.join(EQ, "manifest.json"), encoding="utf-8"))["parts"]
    eq = [(p["file"], pv.read(os.path.join(EQ, p["file"])).extract_surface(algorithm="dataset_surface").triangulate())
          for p in man]
    robot = []
    for l in LINKS:
        v = np.asarray(pv.read(os.path.join(MESH, l + ".stl")).points, float) * 1e-3
        robot.append(v[np.unique(np.round(v * 1000 / 4.0), axis=0, return_index=True)[1]])
    # công cụ: trụ giác hút dọc trục Z khâu 6 không có trong mesh → bỏ qua (gá giác hút có trong link_6 mesh)

    def ik(p):
        sol, info = model.solve_ik(np.asarray(p) / 1000.0, approach=[0, 0, -1])
        assert sol is not None, (p, info)
        return np.asarray(sol)

    for scale in scales:
        parts = []
        for f, m in eq:
            if f.startswith("Den_ring"):
                c = np.asarray(m.center); m = m.copy(); m.points = (np.asarray(m.points) - c) * scale + c
            parts.append((f, m, np.array(m.bounds).reshape(3, 2)))
        worst = {}
        for key, b in srt["bins"].items():
            drop = np.array(b["drop_xyz_mm"], float)
            way = [home, ik(pick + [0, 0, app]), ik(pick), ik(pick + [0, 0, app]), ik(drop + [0, 0, app]), ik(drop),
                   ik(drop + [0, 0, app]), home]
            for qa, qb in zip(way, way[1:]):
                for t in np.linspace(0, 1, 12):
                    q = qa + (qb - qa) * t
                    T = model.link_transforms_base(np.radians(q))
                    for k in range(1, 7):
                        v = ((T[k][:3, :3] @ robot[k].T).T + T[k][:3, 3]) * 1000.0
                        for f, surf, bb in parts:
                            near = np.all((v >= bb[:, 0] - 25) & (v <= bb[:, 1] + 25), axis=1)
                            if not near.any():
                                continue
                            d = np.abs(pv.PolyData(v[near]).compute_implicit_distance(surf)["implicit_distance"])
                            dm = float(d.min()); k2 = (LINKS[k], f)
                            if dm < worst.get(k2, (1e9,))[0]:
                                worst[k2] = (dm, key, np.round(q, 1).tolist())
        print(f"\n== Hệ số đèn {scale:.2f} so với đèn đã xuất (station_equipment) — chỉ liệt kê cặp cách nhau < 25 mm")
        for (link, part), (d, b, q) in sorted(worst.items(), key=lambda x: x[1][0]):
            flag = "VA CHẠM" if d < 3 else ("sát" if d < 15 else "")
            print(f"  {d:6.1f} mm  {link:9s} ↔ {part:34s} thùng {b:9s} q={q} {flag}")
        if not worst:
            print("  không có cặp nào gần hơn 25 mm")


if __name__ == "__main__":
    main()
