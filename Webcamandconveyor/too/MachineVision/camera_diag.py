"""camera_diag.py - đo FPS thật và quét focus cho webcam (Windows / DirectShow).

Chạy độc lập, KHÔNG dùng GUI của dự án:
    python camera_diag.py modes                 # so sánh YUY2 / MJPG ở các độ phân giải
    python camera_diag.py modes --backend msmf  # thử backend khác
    python camera_diag.py modes --after         # đổi thứ tự set FOURCC
    python camera_diag.py focus --fourcc MJPG --width 1920 --height 1080
"""
import argparse
import sys
import time

import cv2
import numpy as np

BACKENDS = {"dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF}
OPTS = {"backend": "dshow", "after": False}  # chỉnh bằng --backend / --after


def open_cam(index, fourcc, w, h, fps=30):
    cap = cv2.VideoCapture(index, BACKENDS[OPTS["backend"]])

    def set_fourcc():
        if fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))

    if not OPTS["after"]:  # mặc định: set FOURCC trước width/height
        set_fourcc()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    if OPTS["after"]:  # biến thể: set FOURCC sau cùng
        set_fourcc()
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def fourcc_str(v):
    v = int(v)
    return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))


def sharpness(frame):
    """Phương sai Laplacian ở vùng giữa ảnh - càng cao càng nét."""
    g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = g.shape
    return cv2.Laplacian(g[h // 4:3 * h // 4, w // 4:3 * w // 4], cv2.CV_64F).var()


def test_modes(index):
    modes = [("YUY2", 2560, 1440), ("MJPG", 2560, 1440),
             ("MJPG", 1920, 1080), ("MJPG", 1280, 720)]
    for fcc, w, h in modes:
        cap = open_cam(index, fcc, w, h)
        if not cap.isOpened():
            print(f"{fcc} {w}x{h}: không mở được camera")
            continue
        for _ in range(10):  # warm-up
            cap.read()
        n, worst, t0 = 0, 0.0, time.perf_counter()
        while time.perf_counter() - t0 < 5.0:
            t = time.perf_counter()
            ok, _ = cap.read()
            if not ok:
                break
            worst = max(worst, time.perf_counter() - t)
            n += 1
        dt = time.perf_counter() - t0
        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"{fcc} yêu cầu {w}x{h} -> thực {aw}x{ah} | {n / dt:5.1f} FPS thật | "
              f"read() chậm nhất {worst * 1000:4.0f} ms | "
              f"driver báo {fourcc_str(cap.get(cv2.CAP_PROP_FOURCC))}")
        cap.release()
        time.sleep(0.5)


def focus_sweep(index, fcc, w, h, step):
    cap = open_cam(index, fcc, w, h)
    if not cap.isOpened():
        print("Không mở được camera")
        return
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    print("Đặt kiện (có chữ/QR/logo) đúng khoảng cách làm việc, giữ yên rồi chờ...")
    best_val, best_score = None, -1.0
    for val in range(0, 256, step):
        cap.set(cv2.CAP_PROP_FOCUS, val)
        time.sleep(0.5)
        for _ in range(6):  # xả frame cũ trong buffer
            cap.read()
        scores = []
        for _ in range(3):
            ok, f = cap.read()
            if ok:
                scores.append(sharpness(f))
        if not scores:
            continue
        s = float(np.mean(scores))
        print(f"focus={val:3d}  độ nét={s:8.1f}  {'#' * int(s / 10)}")
        if s > best_score:
            best_val, best_score = val, s
    print(f"\n=> focus tốt nhất: {best_val} (độ nét {best_score:.1f})")
    print("Đặt vào config: autofocus: false, focus:", best_val)
    cap.release()


if __name__ == "__main__":
    # Console Windows mặc định cp1252 không in được tiếng Việt -> UnicodeEncodeError.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["modes", "focus"])
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--fourcc", default="MJPG")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--step", type=int, default=10)
    ap.add_argument("--backend", choices=list(BACKENDS), default="dshow")
    ap.add_argument("--after", action="store_true",
                    help="set FOURCC sau width/height (một số driver cần thứ tự này)")
    a = ap.parse_args()
    OPTS["backend"], OPTS["after"] = a.backend, a.after
    print(f"[backend={a.backend}, fourcc_after={a.after}]")
    if a.cmd == "modes":
        test_modes(a.index)
    else:
        focus_sweep(a.index, a.fourcc, a.width, a.height, a.step)
