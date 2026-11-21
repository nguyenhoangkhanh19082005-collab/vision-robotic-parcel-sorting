r"""Kiểm tra bản rút ngắn cẳng tay (chạy sau rut_ngan_cang_tay.py):
  1. chồng lấn giữa chi tiết mới/đã dời với chi tiết bên cạnh, so với bản gốc (chỉ báo chỗ TĂNG);
  2. khe hở quanh động cơ J4 (đã xoay thẳng, lùi vào hốc trục J3) và quanh đoạn bầu mới;
  3. quét khớp J3 (gập/duỗi) tìm góc bắt đầu va chạm, bản gốc so với bản mới; quay J4 đủ vòng;
  4. mô-men trọng lực J2, J3 trước/sau (cùng giả định với research/cad_kiem_tra_mo_men.py);
  5. ảnh so sánh trước/sau.
Kết quả: assets/CAD_rut_ngan/ket_qua_kiem_tra.txt và reports/rut_ngan/*.png
    E:\UDMT\.venv\Scripts\python.exe src\cad\rut_ngan_kiem_tra.py [buoc...]     (buoc: giao khe quet momen anh; mặc định tất cả)
"""
import glob, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from rut_ngan_lib import *  # noqa: F401,F403
from rut_ngan_cang_tay import NEW_NAMES, OUT, SHIFT_1, SHIFT_2, SHIFT_3
from OCP.BRep import BRep_Builder
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepTools import BRepTools
from OCP.StlAPI import StlAPI_Writer
from OCP.TopoDS import TopoDS_Shape

TMP = os.path.join(ROOT, ".tmp"); IMG = os.path.join(ROOT, "reports", "rut_ngan")
shift = np.load(os.path.join(TMP, "rn_shift.npy"))      # mm dời về phía J3; −1 = chi tiết dựng mới
log = []


def P(*a):
    s = " ".join(str(x) for x in a); log.append(s); print(s, flush=True)


_cache = {}


def new_shape(i):
    if i not in _cache:
        f = os.path.join(TMP, f"rn_new_{i:03d}.brep")
        if os.path.exists(f):
            s = TopoDS_Shape(); BRepTools.Read_s(s, f, BRep_Builder()); _cache[i] = s
        else:
            _cache[i] = loadi(i)
    return _cache[i]


def group(i):
    return 1000 + i if shift[i] < 0 else float(shift[i])


def nm(i):
    return f"{NEW_NAMES.get(i, parts[i]['name'])}#{i}"


# ---------------------------------------------------------------- 1. chồng lấn
def check_interference():
    P("\n== 1. Chồng lấn (mm3) giữa các chi tiết có vị trí tương đối thay đổi: bản gốc → bản mới ==")
    changed = [i for i in range(len(parts)) if shift[i] != 0]
    boxes = {i: bbox(new_shape(i)) for i in range(len(parts))}
    pairs = set()
    for a in changed:
        for b in range(len(parts)):
            if a == b or group(a) == group(b): continue
            (la, ha), (lb, hb) = boxes[a], boxes[b]
            if np.any(la > hb + 0.2) or np.any(lb > ha + 0.2): continue
            pairs.add((min(a, b), max(a, b)))
    P(f"  {len(pairs)} cặp có hộp bao chạm nhau")
    # AX-4_4 mới = phần gốc gần (dời SHIFT_2) + đoạn bầu mới + phần gốc xa (dời SHIFT_3). Hai phần gốc giữ nguyên vị trí so với
    # các chi tiết lắp vào chúng, nên chỉ kiểm: đoạn bầu mới với mọi chi tiết; phần gần với nhóm dời SHIFT_3; phần xa với nhóm
    # dời SHIFT_2. (Phép giao cả khối AX-4_4 với AX-4_1 lồng khít trong nó bị treo do dung sai của file gốc.)
    def piece(n):
        sh = TopoDS_Shape(); BRepTools.Read_s(sh, os.path.join(TMP, f"rn_{n}.brep"), BRep_Builder()); return limit_tol(shifted(sh, SHIFT_2))
    pieces = {"bầu mới": piece("funnel"), "phần gần": piece("prox"), "phần xa": piece("dist")}
    bad = 0
    for a, b in sorted(pairs):
        print(f"    ... {nm(a)} × {nm(b)}", flush=True)                  # tiến độ (không ghi vào báo cáo)
        if 24 in (a, b):
            o = b if a == 24 else a; sb = limit_tol(new_shape(o)); vo = 0.0
            use = ["bầu mới"] + (["phần gần"] if shift[o] == SHIFT_3 else []) + (["phần xa"] if shift[o] == SHIFT_2 else [])
            vn = sum(interference(pieces[k], sb) for k in use)
        else:
            # siết dung sai trước: chi tiết gốc có dung sai đỉnh tới vài mm làm phép giao sai hoặc treo
            vn = interference(limit_tol(new_shape(a)), limit_tol(new_shape(b))); vo = interference(limit_tol(loadi(a)), limit_tol(loadi(b)))
        flag = "" if not (vn > vo + 0.5) else "   <== TĂNG"
        if vn > 0.05 or vo > 0.05 or flag:
            P(f"  {nm(a):34s} × {nm(b):34s}: {vo:8.1f} → {vn:8.1f}{flag}")
        bad += bool(flag)
    P(f"  Số cặp chồng lấn tăng: {bad}")


# ---------------------------------------------------------------- 2. khe hở
def check_clearance():
    P("\n== 2. Khe hở nhỏ nhất (mm): bản gốc → bản mới ==")
    for a, others in ((191, (16, 15, 17, 120, 121, 11, 10, 111, 177, 56, 57, 58, 59, 126, 127, 128, 129)),
                      (18, (15, 17)), (19, (15, 17)), (182, (24,)), (24, (15, 20, 21, 118, 25)), (20, (24,)), (21, (24,))):
        for b in others:
            do = dist_min(loadi(a), loadi(b)); dn = dist_min(new_shape(a), new_shape(b))
            P(f"  {nm(a):34s} ↔ {nm(b):34s}: {do:6.2f} → {dn:6.2f}")


# ---------------------------------------------------------------- 3. quét khớp
def link_of(i):
    """Khâu mang chi tiết i (0 đế … 6 mặt bích); ốc vít gán theo chi tiết lớn gần nhất."""
    p = parts[i]; n, (x, y, z) = p["name"], p["com"]
    if n.startswith(("AX-1_10", "AX-1_11", "De_nang", "Dong_co_moi_J1", "Lager_78")): return 0
    if n.startswith("Riemenscheibe_16mm"): return 0 if x < -120 else 1 if x < 0 else 2 if x < 120 else 4
    if n.startswith("Riemenscheibe_40mm"): return 1 if x < 0 else 5
    if n.startswith(("AX-1", "Dong_co_moi_J2")): return 1
    if n.startswith("Lager_65"): return 1 if x < 50 else 2
    if n.startswith(("AX-2", "Puly_55", "Mat_bich_5mm_J2", "Dong_co_moi_J3")): return 2
    if n.startswith(("AX-3", "Puly_40", "Mat_bich_5mm_J3", "STEPPERONLINE", "Lager_52")): return 3
    if n.startswith(("AX-4", "Lager_32")): return 4
    if n.startswith("Nema17"): return 4 if y > -70 else 5
    if n.startswith(("AX-5", "Lager_19")): return 5
    if n.startswith("AX-6"): return 6
    return None


_anchors = [(np.array(parts[i]["com"]), link_of(i)) for i in range(len(parts)) if link_of(i) is not None]
LINK = [link_of(i) if link_of(i) is not None else min(_anchors, key=lambda a: np.linalg.norm(a[0] - np.array(parts[i]["com"])))[1]
        for i in range(len(parts))]


def stl_of(i, new):
    d = os.path.join(TMP, "stl_new" if new else "stl_1_27"); os.makedirs(d, exist_ok=True)
    if not new or shift[i] == 0:
        return glob.glob(os.path.join(TMP, "stl_1_27", f"{i:03d}_*.stl"))[0]
    f = os.path.join(d, f"{i:03d}.stl")
    if not os.path.exists(f) or os.path.getmtime(f) < os.path.getmtime(os.path.join(TMP, "rn_shift.npy")):
        s = new_shape(i); BRepMesh_IncrementalMesh(s, 0.15, False, 0.4, True)
        w = StlAPI_Writer(); w.ASCIIMode = False; w.Write(s, f)
    return f


def merged(idx, new):
    import pyvista as pv
    return pv.merge([pv.read(stl_of(i, new)) for i in idx]).triangulate().clean()


def rot4(p, a, deg):
    a = a / np.linalg.norm(a); t = np.radians(deg); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K; T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p - R @ p
    return T


def check_sweep():
    """Gập/duỗi J3: phần cẳng tay nằm ngoài hộp trục J3 (s > 45 mm) so với các chi tiết in + động cơ của cánh tay trên.
    Bỏ qua cụm quanh trục J3 (hộp, puly, ổ bi) vì chúng luôn tiếp xúc nhau và không đổi giữa hai bản."""
    import pyvista as pv
    P("\n== 3. Quét khớp J3 (lưới tam giác đã giảm mật độ): số tam giác chạm cánh tay trên theo góc ==")
    sta_idx = [i for i in range(len(parts)) if parts[i]["name"].startswith(("AX-2", "Dong_co_moi_J3"))]
    out = {}
    for new in (False, True):
        mov_idx = [i for i in range(len(parts)) if LINK[i] >= 3 and not parts[i]["name"].startswith(("ISK", "Mutter", "Lager"))
                   and (np.array(parts[i]["com"]) - E) @ F - (shift[i] if new and shift[i] > 0 else 0) > 45 and i != 16]
        mov = merged(mov_idx, new).decimate(0.85); sta = merged(sta_idx, new).decimate(0.85)
        res = []
        for deg in range(-170, 171, 10):
            _, n = mov.transform(rot4(P3, D3, deg), inplace=False).collision(sta, contact_mode=0); res.append((deg, int(n)))
        free = [d for d, n in res if n == 0]
        lo = max([d for d in range(-170, 1, 10) if d not in free] or [-999]); hi = min([d for d in range(0, 171, 10) if d not in free] or [999])
        out[new] = (lo, hi)
        P(f"  {'bản mới' if new else 'bản gốc'}: không chạm trong khoảng {lo + 10 if lo > -999 else '≤ -170'}° … {hi - 10 if hi < 999 else '≥ 170'}° quanh trục J3 (0° = tư thế trong file CAD, bước 10°)")
        P("     " + " ".join(f"{d}:{n}" for d, n in res))
    P("  J4: mặt ghép AX-4_4 ↔ AX-3_1 tròn xoay, khe hở dọc trục 1,00 mm không đổi (mục 2) → quay J4 không bị ảnh hưởng.")


# ---------------------------------------------------------------- 4. mô-men
def check_torque():
    P("\n== 4. Mô-men trọng lực xấu nhất (tay duỗi ngang), giả định như research/cad_kiem_tra_mo_men.py ==")
    G = 9.81

    def mass(i, fill, v):
        n = parts[i]["name"]
        if n.startswith("Dong_co_moi"): return 0.50
        if n.startswith("STEPPERONLINE"): return 0.40
        if n.startswith("Nema17"): return 0.35 if parts[i]["com"][1] > -70 else 0.22
        if n.startswith(("AX-", "De_nang", "Puly_", "Riemenscheibe_40", "Riemenscheibe_55")): return v * 1.24 * fill / 1000
        if n.startswith(("Riemenscheibe_16", "Mat_bich")): return v * 2.7 / 1000
        if n.startswith("Lager"): return v * 7.85 * 0.8 / 1000
        return v * 7.85 / 1000

    def dist_ax(pt, ax):
        o, d = ax; r = np.asarray(pt) - o; return np.linalg.norm(r - (r @ d) * d)

    ax2, ax3 = AXES[1], AXES[2]; ax6 = AXES[5]
    flange = np.array(next(p["com"] for p in parts if p["name"].startswith("AX-6")))
    out6 = ax6[1] if (flange - ax6[0]) @ ax6[1] > 0 else -ax6[1]
    L2 = dist_ax(ax3[0], ax2)
    rows = {}
    for new in (False, True):
        data = []
        for i, p in enumerate(parts):
            if new and shift[i] < 0:
                s = new_shape(i); c, v = com(s), vol(s) / 1000
            else:
                c, v = np.array(p["com"]) - (F * shift[i] if new else 0), p["vol_cm3"]
            data.append((i, c, v))
        tcp = flange + 40.0 * out6 - (F * SHIFT_3 if new else 0)
        for fill in (0.6, 1.0):
            for pay in (0.0, 0.2, 0.5):
                ms3 = [(mass(i, fill, v), c) for i, c, v in data if LINK[i] >= 3] + [(pay, tcp)]
                m3 = sum(a for a, _ in ms3); c3 = sum(a * c for a, c in ms3) / m3
                ms2 = [(mass(i, fill, v), c) for i, c, v in data if LINK[i] == 2]
                m2 = sum(a for a, _ in ms2); c2 = sum(a * c for a, c in ms2) / m2
                t3 = G * m3 * dist_ax(c3, ax3) / 1000; t2 = G * (m2 * dist_ax(c2, ax2) + m3 * (L2 + dist_ax(c3, ax3))) / 1000
                rows[(new, fill, pay)] = (t2, t3, m3, dist_ax(c3, ax3))
    for fill in (0.6, 1.0):
        for pay in (0.0, 0.2, 0.5):
            (a2, a3, m, r), (b2, b3, mn, rn) = rows[(False, fill, pay)], rows[(True, fill, pay)]
            P(f"  nhựa đặc {fill*100:.0f} %, tải {pay*1000:.0f} g: J3 {a3:.2f} → {b3:.2f} N·m ({(b3/a3-1)*100:+.1f} %), "
              f"J2 {a2:.2f} → {b2:.2f} N·m ({(b2/a2-1)*100:+.1f} %); sau J3 {m:.2f} → {mn:.2f} kg, trọng tâm cách J3 {r:.0f} → {rn:.0f} mm")


# ---------------------------------------------------------------- 5. ảnh
def images():
    import pyvista as pv
    os.makedirs(IMG, exist_ok=True)
    PAL = {"AX-3_1": "#f0883e", "AX-3_2": "#a371f7", "AX-3_3": "#3fb950", "AX-3_4": "#58a6ff", "AX-4_1": "#db61a2", "AX-4_2": "#e3b341",
           "AX-4_3": "#79c0ff", "AX-4_4": "#56d364", "AX-4_5": "#d2a8ff", "AX-4_6": "#ffc680", "AX-5_1": "#ffa198", "AX-5_2": "#7ee787", "AX-6": "#8957e5"}

    def color(n):
        for k, c in PAL.items():
            if n.startswith(k): return c
        if n.startswith(("Dong_co", "Nema", "STEPPER")): return "#d1242f"
        if n.startswith("Lager"): return "#1f6feb"
        if n.startswith(("Riemen", "Puly")): return "#1a7f37"
        if n.startswith("Mat_bich"): return "#bf8700"
        return "#57606a" if n.startswith(("ISK", "Mutter")) else "#c9d1d9"

    fore = [i for i, q in enumerate(parts) if (np.array(q["com"]) - E) @ F > -45 and np.linalg.norm(np.cross(np.array(q["com"]) - E, F)) < 80
            and not q["name"].startswith("AX-2")]
    n1, up1 = X3, np.cross(F, X3)

    def draw(p, new, normal, clip):
        for i in fore:
            m = pv.read(stl_of(i, new))
            if clip: m = m.clip(normal=normal, origin=E)
            if m.n_points: p.add_mesh(m, color=color(parts[i]["name"]), smooth_shading=False)
        L = np.linalg.norm(WRIST - E) - (SHIFT_3 if new else 0)
        for s in range(0, 301, 10):
            pt = E + F * s
            for up in (up1, -n1):
                p.add_mesh(pv.Line(pt + np.array(up) * 75, pt + np.array(up) * (85 if s % 50 == 0 else 80)), color="black", line_width=3)
        p.add_point_labels(np.array([E + F * s + up1 * 92 for s in range(0, 301, 50)] + [E + F * s - n1 * 92 for s in range(0, 301, 50)]),
                           [str(s) for s in range(0, 301, 50)] * 2, font_size=18, shape=None, always_visible=True, show_points=False)
        p.add_text(("BAN MOI" if new else "BAN GOC") + f": truc J3 -> tam co tay {L:.1f} mm", font_size=14, color="black", position="upper_left")
        p.add_mesh(pv.Line(E + F * L - up1 * 70, E + F * L + up1 * 70), color="red", line_width=2)
        p.add_mesh(pv.Line(E - up1 * 70, E + up1 * 70), color="red", line_width=2)

    for tag, normal, view, up, clip in (("cat_doc_mat_phang_tay", n1, n1, up1, True), ("cat_doc_vuong_goc", up1, up1, -n1, True), ("ben_ngoai", n1, -n1, up1, False)):
        p = pv.Plotter(off_screen=True, shape=(2, 1), window_size=(2000, 1700))
        for row, new in enumerate((False, True)):
            p.subplot(row, 0); draw(p, new, normal, clip); p.enable_parallel_projection()
            c = E + F * 130
            p.camera_position = [tuple(c + np.array(view) * 900), tuple(c), tuple(up)]; p.reset_camera(); p.camera.zoom(1.55)
        p.link_views(); p.set_background("white"); p.screenshot(os.path.join(IMG, f"so_sanh_{tag}.png")); p.close()

    # 3 chi tiết in lại
    p = pv.Plotter(off_screen=True, shape=(2, 3), window_size=(2400, 1500))
    for col, i in enumerate((16, 15, 24)):
        for row, new in enumerate((False, True)):
            p.subplot(row, col); m = pv.read(stl_of(i, new)); p.add_mesh(m, color=color(parts[i]["name"]), smooth_shading=False)
            b = np.array(m.bounds).reshape(3, 2); c = b.mean(1)
            view = X3 * 0.8 + up1 * 0.6 - F * 0.5
            p.camera_position = [tuple(c + view * 400), tuple(c), tuple(up1)]; p.enable_parallel_projection(); p.reset_camera(); p.camera.zoom(1.2)
            p.add_text(f"{NEW_NAMES[i] if new else parts[i]['name']}", font_size=11, color="black", position="upper_left")
    p.set_background("white"); p.screenshot(os.path.join(IMG, "ba_chi_tiet_in_lai.png")); p.close()
    P(f"\nẢnh: {IMG}\\so_sanh_*.png, ba_chi_tiet_in_lai.png")


STEPS = {"giao": check_interference, "khe": check_clearance, "quet": check_sweep, "momen": check_torque, "anh": images}
if __name__ == "__main__":
    todo = sys.argv[1:] or list(STEPS)
    for k in todo: STEPS[k]()
    path = os.path.join(OUT, "ket_qua_kiem_tra.txt" if len(todo) == len(STEPS) else f"ket_qua_kiem_tra_{'_'.join(todo)}.txt")
    open(path, "w", encoding="utf-8").write("\n".join(log) + "\n")
