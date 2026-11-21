"""Capture a burst of raw station frames (good parcel / empty belt) as PNG.

Usage:
    python scripts/capture_station_samples.py good_parcel --label size_m
    python scripts/capture_station_samples.py empty_belt

Frames come straight from OpenCVCamera with config/default.yaml (fixed exposure),
no processing. A meta.json with per-frame sharpness, glare and QR text is saved.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml
import zxingcpp

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))

from machine_vision.camera.opencv_camera import OpenCVCamera


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("kind", choices=("good_parcel", "empty_belt", "bad_parcel"))
    parser.add_argument("--label", default="")
    parser.add_argument("--frames", type=int, default=12)
    parser.add_argument("--interval", type=float, default=0.25)
    parser.add_argument("--warmup", type=int, default=15)
    parser.add_argument("--output", type=Path, default=PROJECT / "dataset" / "station")
    args = parser.parse_args()
    config = yaml.safe_load((PROJECT / "config" / "default.yaml").read_text(encoding="utf-8"))

    stamp = datetime.now().strftime("%Y%m%dT%H%M%S")
    name = "_".join(part for part in (args.kind, args.label, stamp) if part)
    folder = args.output / name
    folder.mkdir(parents=True, exist_ok=True)
    records = []
    with OpenCVCamera(config["camera"]) as camera:
        for _ in range(args.warmup):
            camera.read()
        for index in range(args.frames):
            frame = camera.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            codes = zxingcpp.read_barcodes(frame, formats=zxingcpp.BarcodeFormat.QRCode)
            path = folder / f"frame_{index:02d}.png"
            cv2.imwrite(str(path), frame)
            records.append(
                {
                    "file": path.name,
                    "shape": list(frame.shape),
                    "mean": round(float(gray.mean()), 2),
                    "laplacian_var": round(float(cv2.Laplacian(gray, cv2.CV_64F).var()), 2),
                    "glare_ratio": round(float((gray >= 250).mean()), 5),
                    "qr": [code.text for code in codes],
                }
            )
            print(records[-1])
            time.sleep(args.interval)
    meta = {"kind": args.kind, "label": args.label, "captured": stamp,
            "camera": config["camera"], "frames": records}
    (folder / "meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"Saved {len(records)} frames to {folder}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
