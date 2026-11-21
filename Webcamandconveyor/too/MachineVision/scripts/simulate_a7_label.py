"""Simulate A7 shipping labels from the A6 station captures and run the pipeline.

No A7 label was available (2026-11-21). An A7 label (74 x 105 mm) is the A6
(105 x 148 mm) scaled by 0.705, so on the same parcel and camera height:

* the label (and its QR, logo, text) appears 0.705x smaller;
* the part of the parcel top that the A6 label covered now shows carton.

Each composite keeps the real station background and lighting, replaces the A6
label footprint with cardboard texture (cut from the 2026-09-22 dataset box,
brightness matched to the station lamp) and pastes the real label back, scaled
0.705x about its centre. Defects are then painted on the exposed carton:

* ``hole``: dark puncture, 0.15-0.30 QR side;
* ``edge_dent``: notch cut into the parcel outline (belt/background shows);
* ``torn_label``: the 2026-11-04 torn-label capture scaled the same way.

    .venv\\Scripts\\python.exe scripts\\simulate_a7_label.py --make-template
    .venv\\Scripts\\python.exe scripts\\simulate_a7_label.py

Writes tmp/a7_sim/summary.json and contact sheets. Limits: the A7 layout is
assumed to be a scaled A6 (true for SPX templates printed with "fit to page"),
cardboard texture comes from another box, and defects are synthetic.
"""

from __future__ import annotations

import argparse
import copy
import glob
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))

from machine_vision.app import build_pipeline
from machine_vision.config import resolve_label_format
from machine_vision.readers.qr_reader import QrReader
from machine_vision.services.label_integrity import assess_label_integrity

SCALE = 74.0 / 105.0
OUT = PROJECT / "tmp" / "a7_sim"
RNG = np.random.default_rng(7)
LAST_DEFECT: dict = {}


def cardboard_texture(shape: tuple[int, int], paper_lab: np.ndarray) -> np.ndarray:
    """Mirror-tiled cardboard from the dataset box, colour mapped to the station lamp.

    Carton lightness and chroma are taken *relative to the label paper* of the
    same source photo, then re-applied to the station paper colour, so the
    white balance of the old photo does not leak in.
    """
    source = cv2.imread(sorted(glob.glob(str(PROJECT / "dataset/raw/30cm/*.png")))[0])
    patch = source[440:560, 760:920]      # plain carton above the tape, left of the label
    paper = source[830:920, 1020:1200]    # blank label paper of the same photo
    lab_patch = cv2.cvtColor(patch, cv2.COLOR_BGR2LAB).astype(np.float32)
    source_paper = cv2.cvtColor(paper, cv2.COLOR_BGR2LAB).reshape(-1, 3).astype(np.float32).mean(0)
    lab_patch[:, :, 0] *= paper_lab[0] / source_paper[0]
    lab_patch[:, :, 1:] += paper_lab[1:] - source_paper[1:]
    patch = cv2.cvtColor(np.clip(lab_patch, 0, 255).astype(np.uint8), cv2.COLOR_LAB2BGR)
    tile = np.vstack([np.hstack([patch, patch[:, ::-1]]), np.hstack([patch[::-1], patch[::-1, ::-1]])])
    reps = (shape[0] // tile.shape[0] + 1, shape[1] // tile.shape[1] + 1, 1)
    return np.tile(tile, reps)[: shape[0], : shape[1]]


def make_a7(frame: np.ndarray, reader: QrReader, settings: dict) -> dict | None:
    read = reader.read(frame)
    if read is None or read.corners is None:
        return None
    label = assess_label_integrity(frame, read.corners, settings)
    if label.mask is None or label.contour is None:
        return None
    side = label.metrics["label_qr_side_px"]
    footprint = np.zeros(frame.shape[:2], np.uint8)
    cv2.fillPoly(footprint, [np.int32(cv2.boxPoints(cv2.minAreaRect(label.contour)))], 255)
    footprint = cv2.dilate(footprint, np.ones((9, 9), np.uint8))
    paper_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)[label.mask > 0].astype(np.float32).mean(0)
    carton = cardboard_texture(frame.shape[:2], paper_lab)
    out = frame.copy()
    out[footprint > 0] = carton[footprint > 0]
    cx, cy = cv2.minAreaRect(label.contour)[0]
    matrix = cv2.getRotationMatrix2D((cx, cy), 0.0, SCALE)
    small = cv2.warpAffine(frame, matrix, frame.shape[1::-1], flags=cv2.INTER_AREA)
    small_mask = cv2.warpAffine(label.mask, matrix, frame.shape[1::-1], flags=cv2.INTER_NEAREST)
    out[small_mask > 0] = small[small_mask > 0]
    return {"frame": out, "parcel": footprint, "label": small_mask, "qr_side": side * SCALE}


def add_hole(sample: dict) -> np.ndarray:
    frame = sample["frame"].copy()
    carton = cv2.erode(sample["parcel"], np.ones((61, 61), np.uint8))
    carton[cv2.dilate(sample["label"], np.ones((61, 61), np.uint8)) > 0] = 0
    ys, xs = np.nonzero(carton)
    index = RNG.integers(len(xs))
    radius = int(RNG.uniform(0.15, 0.30) * sample["qr_side"] / 2) * 2
    hole = np.zeros(frame.shape[:2], np.float32)
    cv2.ellipse(hole, (int(xs[index]), int(ys[index])), (radius, int(radius * 0.8)),
                float(RNG.integers(0, 180)), 0, 360, 1.0, -1)
    LAST_DEFECT.update(x=int(xs[index]), y=int(ys[index]), radius_px=radius,
                       radius_qr=round(radius / sample["qr_side"], 3))
    hole = cv2.GaussianBlur(hole, (0, 0), 2)[:, :, None]
    return (frame * (1 - hole) + np.array([18, 18, 20]) * hole).astype(np.uint8)


def add_edge_dent(sample: dict, background: np.ndarray) -> np.ndarray:
    frame = sample["frame"].copy()
    contours, _ = cv2.findContours(sample["parcel"], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    points = contours[0][:, 0]
    centre = points.mean(axis=0)
    # Choose an edge point away from the label so the label stays intact.
    label_centre = np.argwhere(sample["label"] > 0).mean(axis=0)[::-1]
    far = points[np.linalg.norm(points - label_centre, axis=1) > 2.2 * sample["qr_side"]]
    point = far[RNG.integers(len(far))].astype(np.float32)
    inward = (centre - point) / np.linalg.norm(centre - point)
    normal = np.array([-inward[1], inward[0]])
    depth, half = 0.8 * sample["qr_side"], 0.6 * sample["qr_side"]
    notch = np.array([point + normal * half - inward * 10, point + inward * depth,
                      point - normal * half - inward * 10], np.int32)
    mask = np.zeros(frame.shape[:2], np.uint8)
    cv2.fillPoly(mask, [notch], 255)
    frame[mask > 0] = background[mask > 0]
    return frame


def make_template(config: dict) -> None:
    """Provisional A7 logo template: A6 template at A7 resolution (0.705x, then back)."""
    a6 = PROJECT / config["label_formats"]["A6"]["logo"]["template_path"]
    a7 = PROJECT / config["label_formats"]["A7"]["logo"]["template_path"]
    template = cv2.imread(str(a6))
    small = cv2.resize(template, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_AREA)
    cv2.imwrite(str(a7), cv2.resize(small, template.shape[1::-1], interpolation=cv2.INTER_CUBIC))
    a6_cal = PROJECT / config["label_formats"]["A6"]["logo"]["calibration_path"]
    a7_cal = PROJECT / config["label_formats"]["A7"]["logo"]["calibration_path"]
    payload = json.loads(a6_cal.read_text(encoding="utf-8"))
    payload.update(template_path=str(a7), note="Mẫu tạm sinh từ A6 (simulate_a7_label.py); "
                   "thay bằng LẤY MẪU LOGO trên tem A7 thật")
    a7_cal.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print("A7 template:", a7)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--make-template", action="store_true")
    parser.add_argument("--frames", type=int, default=10)
    args = parser.parse_args()
    raw = yaml.safe_load((PROJECT / "config" / "default.yaml").read_text(encoding="utf-8"))
    if args.make_template:
        make_template(raw)
    raw = copy.deepcopy(raw)
    raw["label_format"] = "A7"
    raw["acquisition"]["bench_mode_default"] = False
    config = resolve_label_format(raw)
    pipeline, warning = build_pipeline(config, PROJECT, PROJECT)
    if warning:
        print("warning:", warning)
    reader = QrReader(config["qr"])
    background = cv2.imread(str(PROJECT / config["damage"]["background_path"]))

    goods = sorted(glob.glob(str(PROJECT / "dataset/station/good_parcel_verify_*/frame_*.png")))
    goods = goods[:: max(1, len(goods) // args.frames)][: args.frames]
    # Station GUI captures labelled torn by hand (2026-10-31 / 11-03).
    torn = [p for key in ("170725", "171712", "110146", "110234", "112809")
            for p in glob.glob(str(PROJECT / f"captures/*/*T{key}*/frame.jpg"))]
    OUT.mkdir(parents=True, exist_ok=True)

    cases: list[tuple[str, np.ndarray, str, dict]] = []
    for path in goods:
        sample = make_a7(cv2.imread(path), reader, config["damage"])
        if sample is None:
            print("skip (no QR in source):", path)
            continue
        cases.append(("good", sample["frame"], "PASS", {}))
        cases.append(("hole", add_hole(sample), "hole_or_puncture", dict(LAST_DEFECT)))
        cases.append(("edge_dent", add_edge_dent(sample, background), "dent_or_edge_tear", {}))
    for path in torn:
        sample = make_a7(cv2.imread(path), reader, config["damage"])
        if sample is not None:
            cases.append((f"torn_label:{Path(path).parent.name[-12:]}", sample["frame"], "label_tear", {}))

    rows, sheet = [], {}
    for name, frame, expect, defect in cases:
        try:
            result = pipeline.inspect([frame] * 6)
        except Exception as exc:  # noqa: BLE001 - report, keep simulating
            rows.append({"case": name, "expect": expect, "decision": "ERROR", "ok": False,
                         "reason": str(exc)[:200]})
            continue
        classification = result.classification
        parcel = classification.parcel
        damage = parcel.damage
        types = list(damage.damage_types) if damage else []
        kind = name.split(":")[0]
        if kind == "good":
            # Local test mode: PASS needs QR + logo + clean damage check.
            ok = classification.decision.value == "PASS"
        elif kind == "torn_label":
            ok = "label_tear" in types
        else:
            ok = expect in types and classification.decision.value == "REJECT"
        rows.append({
            "case": name, "expect": expect, "decision": classification.decision.value,
            "damage_types": types, "qr": bool(parcel.qr_text),
            "logo": None if parcel.logo is None else parcel.logo.present,
            "logo_score": None if parcel.logo is None else round(parcel.logo.score, 3),
            "notch_qr": round(damage.metrics.get("parcel_notch_qr", -1), 3) if damage else None,
            "holes": damage.metrics.get("surface_holes") if damage else None,
            "ok": ok, "reason": classification.reason[:200], "defect": defect,
        })
        if kind not in sheet:
            overlay = result.debug_images.get("damage_overlay", frame)
            sheet[kind] = cv2.resize(overlay, (960, 540))

    by_kind: dict[str, list[bool]] = {}
    for row in rows:
        by_kind.setdefault(row["case"].split(":")[0], []).append(row["ok"])
    for kind, oks in by_kind.items():
        print(f"{kind:12s} {sum(oks)}/{len(oks)}")
    for row in rows:
        if not row["ok"]:
            print("  MISS", row["case"], row["decision"], row.get("damage_types"),
                  "logo", row.get("logo"), row.get("logo_score"), row.get("defect"), "|", row["reason"][:90])
    (OUT / "summary.json").write_text(
        json.dumps({"by_kind": {k: [sum(v), len(v)] for k, v in by_kind.items()}, "rows": rows},
                   ensure_ascii=False, indent=2), encoding="utf-8")
    tiles = list(sheet.values())
    if tiles:
        while len(tiles) % 2:
            tiles.append(np.zeros_like(tiles[0]))
        grid = np.vstack([np.hstack(tiles[i:i + 2]) for i in range(0, len(tiles), 2)])
        cv2.imwrite(str(OUT / "a7_cases.jpg"), grid, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return 0 if all(all(v) for v in by_kind.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
