"""Live focus assist for adjusting the lens, a close-up lens or the camera height.

The Stafor 2K is fixed-focus: CAP_PROP_FOCUS has no effect, so sharpness can
only change optically (rotate a threaded lens, add a +diopter close-up lens,
or move the camera). This tool shows, in real time:

* Laplacian variance of a fixed centre window, plus the best value seen (peak hold).
  Compare values only at the same height and lighting: brightness changes it too.
* a 1:1 zoom of the centre so blur can be judged by eye,
* whether the QR code decodes right now (zxing, a few ms per frame).

Controls: R reset peak, S save the full frame to tmp/focus_assist, Q/ESC quit.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import zxingcpp

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))

from machine_vision.camera.opencv_camera import OpenCVCamera  # noqa: E402
from machine_vision.config import load_config  # noqa: E402
from machine_vision.imaging.focus import highlight_ratio, laplacian_variance  # noqa: E402


def centre_window(frame: np.ndarray, fraction: float) -> np.ndarray:
    height, width = frame.shape[:2]
    h, w = int(height * fraction), int(width * fraction)
    y, x = (height - h) // 2, (width - w) // 2
    return frame[y:y + h, x:x + w]


def decode_qr(frame: np.ndarray) -> str | None:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    for code in zxingcpp.read_barcodes(gray):
        if code.format == zxingcpp.BarcodeFormat.QRCode:
            return code.text
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", default=str(PROJECT / "config" / "default.yaml"))
    parser.add_argument("--window", type=float, default=0.4, help="centre window fraction")
    args = parser.parse_args()
    camera_settings = load_config(args.config).raw["camera"]
    output = PROJECT / "tmp" / "focus_assist"
    peak = 0.0

    with OpenCVCamera(camera_settings) as camera:
        for warning in camera.warnings:
            print("WARNING:", warning)
        while True:
            frame = camera.read()
            centre = centre_window(frame, args.window)
            sharpness = laplacian_variance(centre)
            peak = max(peak, sharpness)
            started = time.perf_counter()
            qr = decode_qr(frame)
            qr_ms = (time.perf_counter() - started) * 1000

            # Left: whole frame with the measured window; right: 1:1 centre zoom.
            preview = cv2.resize(frame, (960, 540), interpolation=cv2.INTER_AREA)
            height, width = frame.shape[:2]
            scale = 960 / width
            ch, cw = centre.shape[:2]
            x1, y1 = int((width - cw) / 2 * scale), int((height - ch) / 2 * scale)
            cv2.rectangle(preview, (x1, y1), (x1 + int(cw * scale), y1 + int(ch * scale)),
                          (40, 220, 150), 2)
            zoom = frame[height // 2 - 270:height // 2 + 270, width // 2 - 270:width // 2 + 270]
            canvas = np.hstack([preview, zoom])

            ratio = sharpness / peak if peak else 0.0
            bar = int(500 * min(1.0, ratio))
            cv2.rectangle(canvas, (20, 470), (520, 500), (60, 60, 60), -1)
            cv2.rectangle(canvas, (20, 470), (20 + bar, 500),
                          (40, 220, 150) if ratio > 0.9 else (0, 180, 255), -1)
            lines = [
                f"Laplacian {sharpness:6.1f}   peak {peak:6.1f}   ({ratio:4.0%} of peak)",
                f"glare {highlight_ratio(frame):.3%}",
                f"QR: {qr[:40] if qr else 'NOT DECODED'}  ({qr_ms:.0f} ms)",
            ]
            for i, text in enumerate(lines):
                colour = (40, 220, 150) if (i < 2 or qr) else (0, 0, 255)
                cv2.putText(canvas, text, (20, 35 + 34 * i), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(canvas, text, (20, 35 + 34 * i), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, colour, 2, cv2.LINE_AA)
            cv2.imshow("Focus assist (R reset peak, S save, Q quit)", canvas)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                peak = 0.0
            if key == ord("s"):
                output.mkdir(parents=True, exist_ok=True)
                path = output / f"focus_{time.strftime('%Y%m%d_%H%M%S')}_L{sharpness:.0f}.png"
                cv2.imwrite(str(path), frame)
                print("Saved", path)
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
