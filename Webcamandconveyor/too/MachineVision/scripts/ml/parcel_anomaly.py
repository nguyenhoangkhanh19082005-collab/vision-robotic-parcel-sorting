"""Few-shot parcel anomaly detection from good samples only (AnomalyDINO-style).

Pretrained models from Hugging Face, no training, no damaged boxes needed:
  * facebook/sam2.1-hiera-tiny : parcel mask, prompted with the decoded QR centre
  * facebook/dinov2-small      : patch features; nearest-neighbour distance to a
                                 memory bank of GOOD parcels = anomaly score
  * IDEA-Research/grounding-dino-tiny (--box-detector): whole-carton box prompt

Runs in the separate ML environment (.venv-ml: torch CPU + transformers). For real
cartons use --exclude-label --box-detector for BOTH build and score (label print
differs on every parcel; a QR point prompt alone makes SAM2 return only the label):

  .venv-ml\\Scripts\\python.exe scripts\\ml\\parcel_anomaly.py build --exclude-label --box-detector --good "dataset/parcels/good/*.png"
  .venv-ml\\Scripts\\python.exe scripts\\ml\\parcel_anomaly.py score --exclude-label --box-detector --images "captures/*/*/frame.jpg"

Every image is aligned on the QR (rotation/scale/shift), exposure-normalised on
the label paper, and scored only inside the convex hull of the SAM mask, so a
torn notch is inside the scored area while the belt/sensor never are.
Evaluation 2026-11-07: docs/ml_pretrained_eval.md.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import time
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[2]
os.environ.setdefault("HF_HOME", str(PROJECT / "models" / "hf"))
sys.path.insert(0, str(PROJECT / "src"))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
import yaml  # noqa: E402
from transformers import AutoModel, Sam2Model, Sam2Processor  # noqa: E402

from machine_vision.imaging.perspective import order_quad  # noqa: E402
from machine_vision.readers.qr_reader import QrReader  # noqa: E402
from machine_vision.services.label_integrity import assess_label_integrity  # noqa: E402

MEAN, STD = np.array([0.485, 0.456, 0.406]), np.array([0.229, 0.224, 0.225])
PX = 56                                   # canvas pixels per QR side (4 DINOv2 patches)
EXTENT = (-6.6, 2.2, -4.2, 2.2)           # canvas x0, x1, y0, y1 in QR sides from QR top-left
DEFAULT_BANK = PROJECT / "models" / "parcel_anomaly_bank.npz"
# Held-out calibration 2026-11-07 (same label): good max 0.21, torn labels 0.48-0.61.
# Scores within the training images are ~0, so the threshold is NOT derived from them.
DEFAULT_THRESHOLD = 0.33


class ParcelAnomaly:
    def __init__(self, threads: int = 6, exclude_label: bool = False, box_detector: bool = False):
        # Shipping-label print differs on every parcel, and a patch memory bank flags
        # unseen text as anomalous: score the carton only and leave label tears to
        # the geometric check in services/label_integrity.py.
        self.exclude_label = exclude_label
        self.label_settings: dict = {}
        # A QR point prompt makes SAM2 return the LABEL; a Grounding DINO box
        # ("a cardboard box") makes it return the whole carton (~8 s/frame on CPU).
        self.box_detector = box_detector
        self._gdino = None
        torch.set_num_threads(threads)
        config = yaml.safe_load((PROJECT / "config" / "default.yaml").read_text(encoding="utf-8"))
        self.reader = QrReader(config["qr"])
        self.label_settings = config.get("damage", {})
        self.dino = AutoModel.from_pretrained("facebook/dinov2-small").eval()
        self.sam_processor = Sam2Processor.from_pretrained("facebook/sam2.1-hiera-tiny")
        self.sam = Sam2Model.from_pretrained("facebook/sam2.1-hiera-tiny").eval()
        self.bank: torch.Tensor | None = None
        self.threshold: float | None = None

    # ---- geometry -------------------------------------------------------------
    def box_prompt(self, rgb: np.ndarray) -> list[float] | None:
        from PIL import Image
        from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

        if self._gdino is None:
            name = "IDEA-Research/grounding-dino-tiny"
            self._gdino = (AutoProcessor.from_pretrained(name),
                           AutoModelForZeroShotObjectDetection.from_pretrained(name).eval())
        processor, model = self._gdino
        inputs = processor(images=Image.fromarray(rgb), text="a cardboard box. a parcel.", return_tensors="pt")
        with torch.inference_mode():
            output = model(**inputs)
        result = processor.post_process_grounded_object_detection(
            output, inputs.input_ids, threshold=0.3, text_threshold=0.25, target_sizes=[rgb.shape[:2]]
        )[0]
        if not len(result["boxes"]):
            return None
        return result["boxes"][int(result["scores"].argmax())].tolist()

    def parcel_mask(self, frame: np.ndarray, quad: np.ndarray, scale: float = 0.5) -> np.ndarray | None:
        """SAM2 multimask from the QR centre; keep the largest plausible parcel mask."""
        small = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
        if self.box_detector:
            box = self.box_prompt(rgb)
            if box is None:
                return None
            inputs = self.sam_processor(images=rgb, input_boxes=[[box]], return_tensors="pt")
            with torch.inference_mode():
                output = self.sam(**inputs, multimask_output=False)
            mask = self.sam_processor.post_process_masks(output.pred_masks, inputs["original_sizes"])[0][0, 0].numpy()
            return cv2.resize(mask.astype(np.uint8) * 255, (frame.shape[1], frame.shape[0]),
                              interpolation=cv2.INTER_NEAREST)
        cx, cy = quad.mean(axis=0) * scale
        inputs = self.sam_processor(
            images=rgb, input_points=[[[[float(cx), float(cy)]]]], input_labels=[[[1]]], return_tensors="pt",
        )
        with torch.inference_mode():
            output = self.sam(**inputs, multimask_output=True)
        masks = self.sam_processor.post_process_masks(output.pred_masks, inputs["original_sizes"])[0][0].numpy()
        best = None
        for mask in masks:
            ys, xs = np.where(mask)
            if not len(xs):
                continue
            borders = (xs.min() <= 2) + (ys.min() <= 2) + (xs.max() >= mask.shape[1] - 3) + (ys.max() >= mask.shape[0] - 3)
            if mask.mean() < 0.6 and borders < 2 and (best is None or mask.mean() > best.mean()):
                best = mask
        if best is None:
            return None
        return cv2.resize(best.astype(np.uint8) * 255, (frame.shape[1], frame.shape[0]),
                          interpolation=cv2.INTER_NEAREST)

    @staticmethod
    def canvas(
        frame: np.ndarray, quad: np.ndarray, mask: np.ndarray, exclude: np.ndarray | None = None
    ) -> tuple[np.ndarray, np.ndarray]:
        """QR-aligned crop plus convex hull of the parcel mask, exposure-normalised."""
        side = float(np.mean(np.linalg.norm(quad - np.roll(quad, 1, axis=0), axis=1)))
        ex = (quad[1] - quad[0]) / np.linalg.norm(quad[1] - quad[0])
        ey = np.array([-ex[1], ex[0]])
        scale = PX / side
        x0, x1, y0, y1 = EXTENT
        width, height = int((x1 - x0) * PX) // 14 * 14, int((y1 - y0) * PX) // 14 * 14
        matrix = np.array([[scale * ex[0], scale * ex[1], 0], [scale * ey[0], scale * ey[1], 0]])
        matrix[:, 2] = np.array([-x0 * PX, -y0 * PX]) - matrix[:, :2] @ quad[0]
        image = cv2.warpAffine(frame, matrix, (width, height), flags=cv2.INTER_AREA)
        warped = cv2.warpAffine(mask, matrix, (width, height), flags=cv2.INTER_NEAREST)
        hull = np.zeros_like(warped)
        contours, _ = cv2.findContours(warped, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            cv2.fillPoly(hull, [cv2.convexHull(max(contours, key=cv2.contourArea))], 255)
        if exclude is not None:
            removed = cv2.warpAffine(exclude, matrix, (width, height), flags=cv2.INTER_NEAREST)
            hull[cv2.dilate(removed, np.ones((15, 15), np.uint8)) > 0] = 0
        luminance = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)[hull > 0]
        if luminance.size:
            gain = 220.0 / max(1.0, float(np.percentile(luminance, 90)))
            image = np.clip(image.astype(np.float32) * gain, 0, 255).astype(np.uint8)
        return image, hull

    def prepare(self, frame: np.ndarray) -> tuple[np.ndarray, np.ndarray] | None:
        read = self.reader.read(frame)
        if read is None or read.corners is None:
            return None
        quad = order_quad(read.corners)
        mask = self.parcel_mask(frame, quad)
        if mask is None:
            return None
        exclude = None
        if self.exclude_label:
            label = assess_label_integrity(frame, read.corners, self.label_settings)
            if label.mask is None:
                return None
            exclude = label.mask
        image, hull = self.canvas(frame, quad, mask, exclude)
        # Label covers (almost) the whole top face: nothing left to score.
        return None if np.count_nonzero(hull) < 4 * 14 * 14 else (image, hull)

    # ---- features -------------------------------------------------------------
    def features(self, image: np.ndarray) -> np.ndarray:
        x = (cv2.cvtColor(image, cv2.COLOR_BGR2RGB) / 255.0 - MEAN) / STD
        tensor = torch.from_numpy(x.transpose(2, 0, 1)).float()[None]
        with torch.inference_mode():
            hidden = self.dino(pixel_values=tensor).last_hidden_state[0, 1:]
        normalised = torch.nn.functional.normalize(hidden, dim=1).numpy()
        return normalised.reshape(image.shape[0] // 14, image.shape[1] // 14, -1)

    @staticmethod
    def patch_mask(hull: np.ndarray, grid: tuple[int, int]) -> np.ndarray:
        return cv2.resize(hull, (grid[1], grid[0]), interpolation=cv2.INTER_AREA) > 200

    def build(self, frames: list[np.ndarray]) -> dict:
        patches = []
        used = 0
        for frame in frames:
            prepared = self.prepare(frame)
            if prepared is None:
                continue
            image, hull = prepared
            grid_mask = None
            # Nuisance augmentation: defocus varies across the belt, exposure drifts.
            for variant in (image, cv2.GaussianBlur(image, (0, 0), 1.0), cv2.GaussianBlur(image, (0, 0), 2.0),
                            np.clip(image * 0.85, 0, 255).astype(np.uint8),
                            np.clip(image * 1.12, 0, 255).astype(np.uint8)):
                features = self.features(variant)
                grid_mask = self.patch_mask(hull, features.shape[:2]) if grid_mask is None else grid_mask
                patches.append(features[grid_mask])
            used += 1
        if not used:
            raise ValueError("Không ảnh kiện tốt nào đọc được QR")
        self.bank = torch.from_numpy(np.concatenate(patches)).float()
        self.threshold = DEFAULT_THRESHOLD
        return {"images": used, "patches": int(self.bank.shape[0]), "threshold": self.threshold}

    def score_prepared(self, image: np.ndarray, hull: np.ndarray) -> tuple[float, np.ndarray]:
        features = self.features(image)
        grid_mask = self.patch_mask(hull, features.shape[:2])
        query = torch.from_numpy(features.reshape(-1, features.shape[-1])).float()
        distance = (1 - (query @ self.bank.T).max(dim=1).values).numpy().reshape(features.shape[:2])
        distance[~grid_mask] = 0
        values = distance[grid_mask]
        if not values.size:
            return 0.0, distance
        top = max(1, int(0.01 * values.size))
        return float(np.sort(values)[-top:].mean()), distance

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, bank=self.bank.numpy().astype(np.float16), threshold=self.threshold)

    def load(self, path: Path) -> None:
        data = np.load(path)
        self.bank = torch.from_numpy(data["bank"].astype(np.float32))
        self.threshold = float(data["threshold"])


def heatmap(image: np.ndarray, distance: np.ndarray, threshold: float) -> np.ndarray:
    normalised = np.uint8(np.clip(distance / max(1.5 * threshold, 1e-6), 0, 1) * 255)
    colour = cv2.resize(cv2.applyColorMap(normalised, cv2.COLORMAP_JET), (image.shape[1], image.shape[0]),
                        interpolation=cv2.INTER_NEAREST)
    return np.hstack([image, cv2.addWeighted(image, 0.5, colour, 0.5, 0)])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    build = sub.add_parser("build", help="memory bank from GOOD parcel images")
    build.add_argument("--good", required=True, help="glob of good images (PNG/JPG)")
    build.add_argument("--bank", default=str(DEFAULT_BANK))
    build.add_argument("--threshold", type=float, default=DEFAULT_THRESHOLD)
    score = sub.add_parser("score", help="score images, write heatmaps + scores.json")
    score.add_argument("--images", required=True)
    score.add_argument("--bank", default=str(DEFAULT_BANK))
    score.add_argument("--out", default=str(PROJECT / "tmp" / "anomaly_out"))
    score.add_argument("--threshold", type=float, help="override the bank threshold")
    for item in (build, score):
        item.add_argument("--exclude-label", action="store_true",
                          help="score the carton only (recommended for real parcels)")
        item.add_argument("--box-detector", action="store_true",
                          help="Grounding DINO box -> SAM2 whole-carton mask (slow on CPU)")
    args = parser.parse_args()
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")

    model = ParcelAnomaly(exclude_label=args.exclude_label, box_detector=args.box_detector)
    if args.command == "build":
        paths = sorted(glob.glob(args.good))
        info = model.build([cv2.imread(p) for p in paths])
        model.threshold = info["threshold"] = args.threshold
        model.save(Path(args.bank))
        print(json.dumps({**info, "bank": args.bank}, ensure_ascii=False))
        return 0

    model.load(Path(args.bank))
    if args.threshold is not None:
        model.threshold = args.threshold
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for path in sorted(glob.glob(args.images)):
        started = time.perf_counter()
        prepared = model.prepare(cv2.imread(path))
        if prepared is None:
            rows.append({"image": path, "score": None, "verdict": "NOT_SCORED"})
            print(f"NOT_SCORED (no QR / mask / carton area)  {path}")
            continue
        value, distance = model.score_prepared(*prepared)
        verdict = "ANOMALY" if value > model.threshold else "OK"
        name = Path(path).parent.name if Path(path).name == "frame.jpg" else Path(path).stem
        cv2.imwrite(str(out / f"{name}_{verdict}.jpg"), heatmap(prepared[0], distance, model.threshold))
        rows.append({"image": path, "score": round(value, 4), "verdict": verdict,
                     "seconds": round(time.perf_counter() - started, 2)})
        print(f"{verdict:8s} {value:.3f} (thr {model.threshold:.3f})  {path}")
    (out / "scores.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
