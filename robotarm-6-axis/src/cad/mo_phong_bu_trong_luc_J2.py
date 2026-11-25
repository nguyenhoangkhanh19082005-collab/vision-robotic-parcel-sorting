"""Mô phỏng ý tưởng BÙ TRỌNG LỰC J2 bằng lò xo kéo + dây cáp (22/11/2026).

Cơ cấu (nhìn ngang dọc trục J2):
  - Cột gá in 3D bắt trên đế xoay J1 (link_1), đặt BÊN CẠNH cánh tay (lệch dọc trục J2) để không vướng tay.
  - Ròng rọc A trên đỉnh cột, cao a phía trên trục J2.
  - Chốt B trên cánh tay J2 (link_2), cách trục J2 đoạn b.
  - Dây cáp B → qua ròng rọc A → chạy dọc cột xuống lò xo kéo móc vào chân cột.
  - Lực lò xo F ≈ k·|AB| (lò xo "chiều dài 0" nhờ căng sẵn) → mô-men lò xo = k·a·b·sin θ,
    cùng dạng với mô-men trọng lực M·g·L·sin θ → bù được gần đúng ở mọi góc.

Toán (mô-men quanh trục J2, đơn vị N·m):
  τ_g  = Σ m_i · ((c_i − P2) × (−g·up)) · u2        (c_i: trọng tâm khâu i, u2: trục J2)
  τ_lx = ((B − P2) × F·(A − B)/|A − B|) · u2,  F = k·|AB|
  Đai chịu τ_dai = τ_g + τ_lx (phần động cơ còn phải gánh); lực nhánh đai ≈ |τ_dai| / 4 / r_puly20T.

GIẢ ĐỊNH (chưa đo): khối lượng = thể tích mesh twin × 1.2 g/cm³ (trộn nhựa in + động cơ), tải đầu hút 0.15 kg.
→ Hãy đo mô-men thật bằng cân treo (xem báo cáo) rồi chạy lại với TAU_DO_NM.

Chạy: E:\\UDMT\\.venv\\Scripts\\python.exe -X utf8 src\\cad\\mo_phong_bu_trong_luc_J2.py
Kết quả: assets/Mo_phong_bu_trong_luc_J2/ (GIF động, ảnh tổng hợp, ket_qua.txt)
"""
import os, sys
import numpy as np
import pyvista as pv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
import dong_hoc_step as D

OUT = os.path.join(ROOT, "assets", "Mo_phong_bu_trong_luc_J2")
TMP = os.path.join(ROOT, ".tmp", "bu_trong_luc")
os.makedirs(OUT, exist_ok=True); os.makedirs(TMP, exist_ok=True)

MESH = os.path.join(ROOT, "assets", "ROBOTARM2_description", "meshes")
LINKS = ["base_link", "link_1_1", "link_2_1", "link_3_1", "link_4_1", "link_5_1", "link_6_1"]
RHO = 1.2e-3          # kg/cm³ hiệu dụng (giả định)
PAYLOAD = 0.15        # kg tại đầu hút
TAU_DO_NM = None      # điền mô-men đo được (tay duỗi ngang) để hiệu chỉnh khối lượng
G = 9.81
R_PULY = 0.002 * 20 / np.pi / 2   # bán kính bước puly 20T 2GT (m)
BELT_RATIO = 4.0
A_POST = 0.090        # chiều cao ròng rọc A trên trục J2 (m)
B_ARM = 0.120         # chốt B trên cánh tay cách trục J2 (m)
COMP = 0.8            # bù 80 % mô-men trọng lực lớn nhất khi duỗi thẳng

P = [np.asarray(p, float) for p in D.JOINT_POINTS]
U = [np.asarray(u, float) / np.linalg.norm(u) for u in D.JOINT_AXES]
up = U[0] if U[0][2] > 0 else -U[0]
u2 = U[1]


def rot(axis, ang):
    """Ma trận quay Rodrigues quanh trục đơn vị axis góc ang (rad)."""
    k = axis / np.linalg.norm(axis); K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K


def H(R, p):
    """Phép biến đổi 4×4: quay R quanh điểm p."""
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p - R @ p; return T


meshes = []
for n in LINKS:
    m = pv.read(os.path.join(MESH, n + ".stl")).scale(1e-3, inplace=False)   # mesh mm → m
    meshes.append(m.transform(D.LINK_MESH_TRANSFORMS[n], inplace=False))
vol = [m.volume * 1e6 for m in meshes]          # cm³
com0 = [np.array(m.center_of_mass()) for m in meshes]
mass = [v * RHO for v in vol]

# Chiều "về phía trước" (đất) = hình chiếu cẳng tay ở tư thế 0 lên mặt ngang
fwd = P[3] - P[2]; fwd -= up * (fwd @ up); fwd /= np.linalg.norm(fwd)
s2 = 1.0 if (rot(u2, 0.3) @ (P[2] - P[1])) @ fwd > 0 else -1.0     # θ > 0 = cúi về phía trước
# Góc J3 làm cẳng tay duỗi thẳng theo cánh tay trên
s3 = max((1.0, -1.0), key=lambda s: (rot(U[2], s * np.pi / 2) @ (P[3] - P[2])) @ up)
TCP0 = P[5] + (P[5] - P[4]) / np.linalg.norm(P[5] - P[4]) * (0.0074 + D.TOOL_LENGTH_MM / 1000)


def pose(theta_deg, straight=True):
    """Ma trận 4×4 cho từng khâu: J2 quay θ, J3 duỗi thẳng (straight) hoặc giữ chữ L."""
    T2 = H(rot(u2, s2 * np.radians(theta_deg)), P[1])
    T3 = T2 @ (H(rot(U[2], s3 * np.pi / 2), P[2]) if straight else np.eye(4))
    return [np.eye(4), np.eye(4), T2, T3, T3, T3, T3]


def apply(T, p):
    return T[:3, :3] @ p + T[:3, 3]


def tau_gravity(theta, straight=True, scale=1.0):
    Ts = pose(theta, straight); t = 0.0
    for i in range(2, 7):
        r = apply(Ts[i], com0[i]) - P[1]
        t += (np.cross(r, -G * mass[i] * scale * up)) @ u2
    r = apply(Ts[6], TCP0) - P[1]
    t += np.cross(r, -G * PAYLOAD * up) @ u2
    return t * s2                     # dương = kéo tay cúi về phía trước


# Lệch bên: cột gá đặt ngoài mặt bên cánh tay (dọc u2)
side = max((meshes[2].points - P[1]) @ u2) + 0.015
A = P[1] + A_POST * up + side * u2
B0 = P[1] + B_ARM * (P[2] - P[1]) / np.linalg.norm(P[2] - P[1]) + side * u2


def spring(theta, k):
    T2 = pose(theta)[2]; B = apply(T2, B0); d = A - B; L = np.linalg.norm(d)
    F = k * L
    t = np.cross(B - P[1], F * d / L) @ u2 * s2
    return t, F, B, L


# Hiệu chỉnh khối lượng nếu đã đo
scale = 1.0
if TAU_DO_NM:
    scale = TAU_DO_NM / tau_gravity(90, True)
th = np.arange(-30, 106, 1.0)
tg_s = np.array([tau_gravity(t, True, scale) for t in th])
tg_l = np.array([tau_gravity(t, False, scale) for t in th])
tmax = tau_gravity(90, True, scale)
K = COMP * tmax / (A_POST * B_ARM)          # N/m
tl = np.array([spring(t, K)[0] for t in th])
Fs = np.array([spring(t, K)[1] for t in th])
belt = lambda tau: np.abs(tau) / BELT_RATIO / R_PULY


# ---------------- Báo cáo số ----------------
lines = [
    "MÔ PHỎNG BÙ TRỌNG LỰC J2 — lò xo kéo + cáp qua ròng rọc (22/11/2026)",
    f"Khối lượng giả định (thể tích mesh × {RHO*1e3:.1f} g/cm³, hệ số hiệu chỉnh {scale:.2f}):",
    *[f"  {n}: {v:6.0f} cm³ → {m*scale:5.2f} kg" for n, v, m in zip(LINKS[2:], vol[2:], mass[2:])],
    f"  tải đầu hút: {PAYLOAD} kg",
    f"Mô-men trọng lực J2 lớn nhất (duỗi thẳng, tay ngang θ = 90°): {tmax:.2f} N·m",
    f"  → KHÔNG lò xo: lực nhánh đai ≈ {belt(tmax):.0f} N",
    f"Ròng rọc A cao {A_POST*1e3:.0f} mm trên trục J2, chốt B cách trục {B_ARM*1e3:.0f} mm, lệch bên {side*1e3:.0f} mm",
    f"Lò xo: k = {K/1000:.2f} N/mm, lực tối đa {Fs.max():.0f} N, hành trình {1e3*(np.ptp([spring(t, K)[3] for t in th])):.0f} mm (θ −30…105°)",
    "θ (°)   τ trọng lực duỗi   τ lò xo   τ còn lại → lực đai   |  chữ L: τ trọng lực, còn lại",
]
for t in (0, 30, 60, 90, 105):
    i = int(np.where(th == t)[0][0])
    lines.append(f"{t:4d}    {tg_s[i]:7.2f}          {tl[i]:7.2f}   {tg_s[i]+tl[i]:6.2f} → {belt(tg_s[i]+tl[i]):5.0f} N"
                 f"   |  {tg_l[i]:6.2f}, {tg_l[i]+tl[i]:6.2f} → {belt(tg_l[i]+tl[i]):4.0f} N")
lines.append(f"Lực đai lớn nhất (duỗi) không lò xo {belt(tg_s).max():.0f} N → có lò xo {belt(tg_s+tl).max():.0f} N")
lines.append(f"Lực đai lớn nhất (chữ L) không lò xo {belt(tg_l).max():.0f} N → có lò xo {belt(tg_l+tl).max():.0f} N")
txt = "\n".join(lines); print(txt)
open(os.path.join(OUT, "ket_qua.txt"), "w", encoding="utf-8").write(txt + "\n")


# ---------------- Hình 3D ----------------
def helix(p0, p1, r=0.006, turns=14, n=400):
    """Lò xo xoắn từ p0 tới p1 (đường cong để vẽ)."""
    ax = p1 - p0; L = np.linalg.norm(ax); ax /= L
    e1 = np.cross(ax, u2); e1 = e1 / np.linalg.norm(e1) if np.linalg.norm(e1) > 1e-6 else np.cross(ax, up)
    e2 = np.cross(ax, e1); s = np.linspace(0, 1, n)
    pts = p0 + np.outer(s * L, ax) + r * (np.outer(np.cos(2 * np.pi * turns * s), e1) + np.outer(np.sin(2 * np.pi * turns * s), e2))
    return pv.Spline(pts, n).tube(radius=0.0012)


COL = ["#8b949e", "#c9d1d9", "#58a6ff", "#79c0ff", "#a5d6ff", "#a5d6ff", "#a5d6ff"]
center = P[1] + 0.13 * up + 0.20 * fwd
foot = P[1] - 0.045 * up + side * u2       # chân cột (móc lò xo)


def render(theta, path, straight=True, size=(900, 900)):
    pl = pv.Plotter(off_screen=True, window_size=size)
    Ts = pose(theta, straight)
    for i, m in enumerate(meshes):
        pl.add_mesh(m.transform(Ts[i], inplace=False), color=COL[i], smooth_shading=True)
    t_l, F, B, L = spring(theta, K)
    # cột gá + ròng rọc + chốt
    pl.add_mesh(pv.Cylinder(center=(A + foot) / 2, direction=up, radius=0.006, height=np.linalg.norm(A - foot) + 0.01), color="#f0883e")
    pl.add_mesh(pv.Cylinder(center=A, direction=u2, radius=0.010, height=0.008), color="#d29922")
    pl.add_mesh(pv.Cylinder(center=B, direction=u2, radius=0.005, height=0.012), color="#d29922")
    # cáp B → A, rồi xuống lò xo dọc cột (lệch ngoài cột 12 mm)
    off = 0.012 * np.cross(u2, up)
    off = off if off @ fwd < 0 else -off
    pl.add_mesh(pv.Line(B, A).tube(radius=0.0012), color="black")
    s_top = A + off - 0.012 * up
    sp_len = 0.055 + 0.25 * (L - A_POST + B_ARM) / (A_POST + B_ARM)   # chỉ để minh họa độ giãn
    s_bot = foot + off
    s_mid = s_bot + up * min(sp_len, np.linalg.norm(s_top - s_bot) - 0.005)
    pl.add_mesh(pv.Line(A + off, s_mid).tube(radius=0.0012), color="black")
    pl.add_mesh(helix(s_bot, s_mid), color="#f85149")
    cam_dir = u2 if (u2 @ np.array([1.0, 0, 0])) > 0 else -u2
    pl.camera.focal_point = center; pl.camera.position = center + 1.2 * cam_dir * (1 if True else -1) + 0.0 * up
    pl.camera.up = up; pl.enable_parallel_projection(); pl.camera.parallel_scale = 0.37
    pl.set_background("white")
    pl.screenshot(path); pl.close()
    return t_l, F


def chart(theta, path, size=(900, 900)):
    fig, ax = plt.subplots(2, 1, figsize=(size[0] / 100, size[1] / 100), dpi=100, gridspec_kw={"height_ratios": [3, 2]})
    a = ax[0]
    a.plot(th, tg_s, color="#1f6feb", lw=2.5, label="Trọng lực kéo tay xuống (duỗi thẳng)")
    a.plot(th, -tl, color="#f85149", lw=2.5, ls="--", label="Lò xo kéo tay lên")
    a.plot(th, tg_s + tl, color="#2da44e", lw=3, label="Phần còn lại động cơ + đai phải gánh")
    a.axvline(theta, color="gray", lw=1); a.axhline(0, color="black", lw=0.6)
    i = int(np.argmin(np.abs(th - theta)))
    a.plot([theta] * 3, [tg_s[i], -tl[i], tg_s[i] + tl[i]], "o", color="black")
    a.set_xlabel("Góc J2 θ (°): 0 = cánh tay dựng đứng, 90 = nằm ngang sát đất"); a.set_ylabel("Mô-men quanh J2 (N·m)")
    a.legend(loc="upper left", fontsize=9); a.grid(alpha=0.3); a.set_title(f"θ = {theta:.0f}°", fontsize=13)
    b = ax[1]
    b.bar(["Không lò xo", "Có lò xo"], [belt(tg_s[i]), belt(tg_s[i] + tl[i])], color=["#1f6feb", "#2da44e"])
    b.set_ylim(0, belt(tg_s).max() * 1.15); b.set_ylabel("Lực trên đai J2 (N)")
    for x, v in enumerate([belt(tg_s[i]), belt(tg_s[i] + tl[i])]):
        b.text(x, v + 5, f"{v:.0f} N", ha="center", fontsize=12, fontweight="bold")
    b.set_title("Lực đai do trọng lực (chưa tính tăng tốc)", fontsize=10)
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


frames = []
seq = list(range(0, 96, 5)) + list(range(90, -1, -10))
for t in seq:
    r = os.path.join(TMP, f"r_{t:03d}.png"); c = os.path.join(TMP, f"c_{t:03d}.png")
    if not os.path.exists(r): render(t, r)
    chart(t, c)
    im = Image.new("RGB", (1800, 900), "white"); im.paste(Image.open(r).convert("RGB"), (0, 0)); im.paste(Image.open(c).convert("RGB"), (900, 0))
    frames.append(im.resize((1200, 600), Image.LANCZOS))
frames[0].save(os.path.join(OUT, "mo_phong_bu_trong_luc_J2.gif"), save_all=True, append_images=frames[1:], duration=220, loop=0)

# Ảnh tĩnh: 3 tư thế + đồ thị cuối (θ = 90)
pics = []
for t in (0, 45, 90):
    r = os.path.join(TMP, f"r_{t:03d}.png")
    if not os.path.exists(r): render(t, r)
    pics.append(Image.open(r).convert("RGB").resize((600, 600)))
chart(90, os.path.join(TMP, "c_static.png"))
im = Image.new("RGB", (2400, 600), "white")
for k, p in enumerate(pics): im.paste(p, (600 * k, 0))
im.paste(Image.open(os.path.join(TMP, "c_static.png")).convert("RGB").resize((600, 600)), (1800, 0))
im.save(os.path.join(OUT, "tong_hop_bu_trong_luc_J2.png"))
print("Đã lưu:", OUT)
