from __future__ import annotations

from pathlib import Path
from typing import Protocol

import cv2
import numpy as np

from machine_vision.imaging.enhancement import crop_fraction
from machine_vision.models import DamageAssessment, PackageDetection
from machine_vision.services.label_integrity import LabelIntegrity, assess_label_integrity


class DamageDetector(Protocol):
    def inspect(
        self, frame: np.ndarray, package: PackageDetection | None = None
    ) -> tuple[DamageAssessment, dict[str, np.ndarray]]: ...

    def set_bench_mode(self, enabled: bool) -> None: ...

    def save_bench_reference(self, frame: np.ndarray) -> Path: ...

    def reference_ready(self) -> bool: ...


class PackageDamageDetector:
    """Parcel damage: background-free label tear check + background-difference outline/holes."""

    def __init__(
        self, settings: dict, project_root: Path, writable_root: Path | None = None
    ):
        self.settings = settings
        self.project_root = project_root
        self.writable_root = writable_root or project_root
        reference_path = Path(str(settings.get("background_path", "")))
        self.reference_path = (
            reference_path if reference_path.is_absolute() else project_root / reference_path
        )
        self.production_reference = (
            cv2.imread(str(self.reference_path)) if self.reference_path.is_file() else None
        )
        bench_path = Path(
            str(settings.get("bench_background_path", "config/calibration/empty_bench.png"))
        )
        self.bench_reference_path = (
            bench_path if bench_path.is_absolute() else self.writable_root / bench_path
        )
        bundled_bench_path = bench_path if bench_path.is_absolute() else project_root / bench_path
        bench_source = (
            self.bench_reference_path
            if self.bench_reference_path.is_file()
            else bundled_bench_path
        )
        self.bench_reference = (
            cv2.imread(str(bench_source)) if bench_source.is_file() else None
        )
        self._bench_mode = False
        self.reference = self.production_reference

    def set_bench_mode(self, enabled: bool) -> None:
        self._bench_mode = bool(enabled)
        self.reference = self.bench_reference if self._bench_mode else self.production_reference

    def reference_ready(self) -> bool:
        return self.reference is not None

    def save_bench_reference(self, frame: np.ndarray) -> Path:
        """Save the empty-scene reference of the ACTIVE mode (bench or conveyor).

        Conveyor mode (2026-11-23): the camera trigger compares every frame with this
        image, so it must be re-captured whenever the camera, lamp or belt moved.
        """
        conveyor = not self._bench_mode and bool(self.settings.get("background_path"))
        target = self.reference_path if conveyor else self.bench_reference_path
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = target.with_name(target.stem + ".tmp" + target.suffix)
        if not cv2.imwrite(str(temporary), frame):
            raise RuntimeError(f"Unable to save background: {target}")
        temporary.replace(target)
        if conveyor:
            self.production_reference = frame.copy()
            self.reference = self.production_reference
        else:
            self.bench_reference = frame.copy()
            if self._bench_mode:
                self.reference = self.bench_reference
        return target

    @staticmethod
    def _clip_box(
        box: tuple[int, int, int, int], shape: tuple[int, ...]
    ) -> tuple[int, int, int, int] | None:
        x, y, width, height = box
        x0, y0 = max(0, x), max(0, y)
        x1, y1 = min(shape[1], x + width), min(shape[0], y + height)
        if x1 - x0 < 20 or y1 - y0 < 20:
            return None
        return x0, y0, x1 - x0, y1 - y0

    def _segment_parcel(
        self,
        frame: np.ndarray,
        seed: tuple[int, int],
        scale: float | None,
        known: np.ndarray | None = None,
    ) -> dict:
        """Parcel = background-difference component containing the QR (not the largest blob).

        Returns ``valid=False`` with a reason when the empty-scene reference no longer
        matches the scene (camera moved, fixtures changed): then the component floods
        the frame and any outline/hole verdict would be noise.
        """
        if self.reference is None:
            return {"valid": False, "code": "NO_REFERENCE", "reason": "chưa có ảnh nền trống"}
        reference = self.reference
        if reference.shape[:2] != frame.shape[:2]:
            reference = cv2.resize(reference, (frame.shape[1], frame.shape[0]))
        blur = int(self.settings.get("blur_kernel", 9)) | 1
        current = cv2.cvtColor(cv2.GaussianBlur(frame, (blur, blur), 0), cv2.COLOR_BGR2LAB)
        baseline = cv2.cvtColor(cv2.GaussianBlur(reference, (blur, blur), 0), cv2.COLOR_BGR2LAB)
        delta = cv2.absdiff(current, baseline).astype(np.float32)
        # Lightness plus chroma: a white label on a grey board differs mostly in L,
        # brown cardboard on a green belt mostly in a/b.
        difference = np.maximum(delta[:, :, 0], 2.0 * np.maximum(delta[:, :, 1], delta[:, :, 2]))
        changed = np.uint8(difference > float(self.settings.get("difference_threshold", 25))) * 255
        size = int(self.settings.get("morphology_kernel", 11))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (size, size))
        mask = cv2.morphologyEx(changed, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        if known is not None:
            # Label paper is part of the parcel even where it matches the reference
            # (white label over the white belt rails left slits in the outline).
            mask = cv2.bitwise_or(mask, known)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        if count < 2:
            return {"valid": False, "code": "NO_PARCEL", "reason": "không tách được kiện khỏi nền"}
        x, y = seed
        component = int(labels[min(max(y, 0), mask.shape[0] - 1), min(max(x, 0), mask.shape[1] - 1)])
        if component == 0:
            # Seed on an unchanged pixel (e.g. QR printed on a label matching the
            # background): take the component closest to it.
            centres = [
                (np.hypot(*(np.array([sx + sw / 2, sy + sh / 2]) - np.array(seed))), index)
                for index, (sx, sy, sw, sh, _area) in enumerate(stats[1:], start=1)
            ]
            component = min(centres)[1]
        parcel = np.uint8(labels == component) * 255
        if scale:
            # Cut thin tails (moving-belt glare, cables) still touching the parcel.
            cut = int(float(self.settings.get("parcel_open_qr", 0.15)) * scale) | 1
            opened = cv2.morphologyEx(parcel, cv2.MORPH_OPEN, np.ones((cut, cut), np.uint8))
            count_open, labels_open = cv2.connectedComponents(opened)
            if count_open > 1:
                seeds = labels_open[parcel > 0]
                seeds = seeds[seeds > 0]
                if seeds.size:
                    parcel = np.uint8(labels_open == np.bincount(seeds).argmax()) * 255
        contours, _ = cv2.findContours(parcel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contour = max(contours, key=cv2.contourArea)
        area_ratio = float(cv2.contourArea(contour)) / parcel.size
        px, py, pw, ph = cv2.boundingRect(contour)
        margin = int(self.settings.get("border_margin_px", 10))
        borders = sum(
            (px <= margin, py <= margin,
             px + pw >= frame.shape[1] - margin, py + ph >= frame.shape[0] - margin)
        )
        reach = scale if scale else 0.1 * np.sqrt(max(1.0, cv2.contourArea(contour)))
        # Pixels farther than ``reach`` from the parcel (distance transform: a
        # 250 px elliptic dilation took ~2 s on a 2K frame).
        outside = cv2.distanceTransform(cv2.bitwise_not(parcel), cv2.DIST_L2, 3) > reach / 2
        background_change = float(np.count_nonzero(changed[outside])) / max(1, np.count_nonzero(outside))
        result = {
            "valid": True, "mask": parcel, "contour": contour, "changed": changed,
            "difference": difference, "metrics": {
                "parcel_area_ratio": area_ratio,
                "parcel_border_contacts": float(borders),
                "background_change_ratio": background_change,
            },
        }
        if background_change > float(self.settings.get("max_background_change", 0.05)) or (
            area_ratio > float(self.settings.get("max_parcel_area_ratio", 0.70))
        ):
            result.update(valid=False, code="BACKGROUND_MISMATCH", reason=(
                f"ảnh nền không còn khớp cảnh ({background_change:.0%} vùng nền thay đổi, "
                f"kiện chiếm {area_ratio:.0%} khung) — chụp lại nền trống"
            ))
        elif borders >= 1 and self.settings.get("require_full_contour", True):
            result.update(valid=False, code="TOUCHES_BORDER", reason="kiện chạm mép ảnh, không thấy đủ viền")
        return result

    def _parcel_defects(
        self, frame: np.ndarray, parcel: dict, label: LabelIntegrity | None, scale: float
    ) -> tuple[list[str], dict[str, float], list[np.ndarray]]:
        contour = parcel["contour"]
        area = max(1.0, float(cv2.contourArea(contour)))
        width, height = cv2.minAreaRect(contour)[1]
        rectangularity = area / max(1.0, float(width * height))
        depth = 0.0
        hull = cv2.convexHull(contour, returnPoints=False)
        if hull is not None and len(hull) > 3:
            defects = cv2.convexityDefects(contour, hull)
            # OpenCV 4.x returns (N, 1, 4), 5.x returns (N, 4).
            defects = None if defects is None else defects.reshape(-1, 1, 4)
            if defects is not None:
                depth = float(defects[:, 0, 3].max()) / 256.0
        types: list[str] = []
        if rectangularity < float(self.settings.get("min_rectangularity", 0.82)):
            types.append("deformation")
        if depth / scale > float(self.settings.get("parcel_max_notch_qr", 0.40)):
            types.append("dent_or_edge_tear")

        # Holes: only strictly inside the parcel, away from its edge and the label.
        # Search the *filled* outline: a hole that matches the reference behind it
        # (dark puncture over the dark belt head) is missing from the change mask
        # itself, so the raw mask would hide exactly the hole being looked for.
        filled = np.zeros_like(parcel["mask"])
        cv2.drawContours(filled, [contour], -1, 255, cv2.FILLED)
        interior = cv2.erode(
            filled,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (int(0.12 * scale) | 1,) * 2),
        )
        if label is not None and label.mask is not None:
            interior[cv2.dilate(label.mask, np.ones((int(0.06 * scale) | 1,) * 2, np.uint8)) > 0] = 0
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        surface = gray[interior > 0]
        holes: list[np.ndarray] = []
        if surface.size:
            dark = float(np.median(surface)) - float(self.settings.get("surface_hole_contrast", 45))
            # A hole is darker than the parcel surface, or shows the empty background.
            candidate = ((gray < dark) | (parcel["changed"] == 0)) & (interior > 0)
            candidate = cv2.morphologyEx(
                np.uint8(candidate) * 255, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
            )
            minimum = (float(self.settings.get("hole_min_diameter_qr", 0.10)) * scale) ** 2
            maximum_aspect = float(self.settings.get("surface_hole_max_aspect", 3.2))
            for blob in cv2.findContours(candidate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
                _bx, _by, bw, bh = cv2.boundingRect(blob)
                blob_area = float(cv2.contourArea(blob))
                if (
                    blob_area >= minimum
                    and max(bw / max(bh, 1), bh / max(bw, 1)) <= maximum_aspect
                    and blob_area / max(1.0, float(bw * bh)) >= 0.30
                ):
                    holes.append(blob)
        if holes:
            types.append("hole_or_puncture")
        metrics = {
            "parcel_rectangularity": rectangularity,
            "parcel_notch_qr": depth / scale,
            "surface_holes": float(len(holes)),
        }
        return types, metrics, holes

    def _inspect_target(
        self, frame: np.ndarray, package: PackageDetection
    ) -> tuple[DamageAssessment, dict[str, np.ndarray]]:
        label = (
            assess_label_integrity(frame, package.qr_corners, self.settings)
            if package.qr_corners is not None
            else None
        )
        scale = label.metrics.get("label_qr_side_px") if label is not None else None
        if package.qr_corners is not None:
            cx, cy = np.asarray(package.qr_corners, dtype=np.float32).reshape(-1, 2).mean(axis=0)
        else:
            lx, ly, lw, lh = package.label_bbox
            cx, cy = lx + lw / 2, ly + lh / 2
        if self.settings.get("check_parcel_surface", True):
            parcel = self._segment_parcel(
                frame, (int(cx), int(cy)), scale, label.mask if label is not None else None
            )
        else:
            # 2026-11-23: station checks only the QR and the label (tear); carton
            # dents/holes are out of scope, so no empty-belt reference is needed here.
            parcel = {"valid": False, "code": "DISABLED", "reason": "không kiểm tra bề mặt thùng"}
        if scale is None and parcel.get("contour") is not None:
            # No QR geometry: use a fifth of the parcel size as the length unit.
            scale = 0.2 * np.sqrt(max(1.0, float(cv2.contourArea(parcel["contour"]))))
        scale = float(scale or 100.0)

        damage_types: list[str] = []
        notes: list[str] = []
        metrics: dict[str, float] = dict(parcel.get("metrics", {}))
        holes: list[np.ndarray] = []
        if label is not None:
            metrics.update(label.metrics)
            if label.ok is False:
                damage_types.append("label_tear")
            notes.append(label.reason)
        if parcel["valid"]:
            types, parcel_metrics, holes = self._parcel_defects(frame, parcel, label, scale)
            damage_types += types
            metrics.update(parcel_metrics)
            notes.append("viền kiện đạt" if not types else "viền/bề mặt kiện có lỗi")
        else:
            notes.append("chưa kiểm tra viền kiện: " + parcel["reason"])

        if damage_types:
            damaged: bool | None = True
        elif (
            label is not None and label.ok is None and self.settings.get("require_label_check", True)
        ) or (not parcel["valid"] and self.settings.get("require_parcel_outline", False)):
            damaged = None
        elif (label is not None and label.ok) or parcel["valid"]:
            damaged = False
        else:
            damaged = None
        reason = "; ".join(notes)
        if damage_types:
            reason = "Phát hiện lỗi: " + ", ".join(damage_types) + " — " + reason

        overlay = frame.copy()
        red, green, blue = (0, 0, 255), (55, 220, 125), (255, 170, 45)
        bbox = None
        if parcel.get("contour") is not None and parcel["valid"]:
            bad = any(t in damage_types for t in ("deformation", "dent_or_edge_tear", "hole_or_puncture"))
            cv2.drawContours(overlay, [parcel["contour"]], -1, red if bad else green, 4)
            bbox = cv2.boundingRect(parcel["contour"])
        if label is not None and label.contour is not None:
            cv2.drawContours(overlay, [label.contour], -1, red if label.ok is False else blue, 3)
            if label.ok is False and label.notch_point is not None:
                cv2.circle(overlay, label.notch_point, int(0.3 * scale), red, 5)
                cv2.putText(overlay, "RACH NHAN", (label.notch_point[0] - 60, label.notch_point[1] - int(0.35 * scale)),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, red, 3, cv2.LINE_AA)
            bbox = bbox or cv2.boundingRect(label.contour)
        for blob in holes:
            hx, hy, hw, hh = cv2.boundingRect(blob)
            cv2.rectangle(overlay, (hx, hy), (hx + hw, hy + hh), red, 3)
            cv2.putText(overlay, "THUNG LO", (hx, max(25, hy - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, red, 2, cv2.LINE_AA)
        if not parcel["valid"]:
            # cv2.putText has no Vietnamese glyphs: ASCII status only.
            cv2.putText(overlay, "VIEN KIEN: CHUA KIEM TRA - " + parcel.get("code", "NO_REFERENCE"),
                        (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 255), 3, cv2.LINE_AA)
        hole_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.drawContours(hole_mask, holes, -1, 255, cv2.FILLED)
        debug = {"damage_overlay": overlay, "surface_hole_mask": hole_mask}
        if parcel.get("mask") is not None:
            debug["damage_mask"] = parcel["mask"]
        if label is not None and label.mask is not None:
            debug["label_paper_mask"] = label.mask
        confidence = 0.9 if damaged is not None else 0.0
        return (
            DamageAssessment(
                calibrated=True,
                damaged=damaged,
                damage_types=tuple(dict.fromkeys(damage_types)),
                confidence=confidence,
                metrics={key: float(value) for key, value in metrics.items()},
                bbox=bbox if bbox is not None else package.bbox,
                reason=reason,
            ),
            debug,
        )

    def inspect(
        self, frame: np.ndarray, package: PackageDetection | None = None
    ) -> tuple[DamageAssessment, dict[str, np.ndarray]]:
        if package is not None:
            return self._inspect_target(frame, package)
        roi = self.settings.get("roi", [0.02, 0.02, 0.96, 0.96])
        image = crop_fraction(frame, roi)
        if self.reference is None:
            missing_path = self.bench_reference_path if self._bench_mode else self.reference_path
            return (
                DamageAssessment(
                    calibrated=False,
                    damaged=None,
                    reason=f"Empty-scene reference not found: {missing_path}",
                ),
                {"damage_roi": image},
            )

        reference = crop_fraction(self.reference, roi)
        if reference.shape[:2] != image.shape[:2]:
            reference = cv2.resize(reference, (image.shape[1], image.shape[0]))

        blur_size = int(self.settings.get("blur_kernel", 9))
        blur_size = blur_size if blur_size % 2 else blur_size + 1
        current_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        reference_gray = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY)
        current_gray = cv2.GaussianBlur(current_gray, (blur_size, blur_size), 0)
        reference_gray = cv2.GaussianBlur(reference_gray, (blur_size, blur_size), 0)
        difference = cv2.absdiff(current_gray, reference_gray)
        threshold = int(self.settings.get("difference_threshold", 25))
        mask = cv2.threshold(difference, threshold, 255, cv2.THRESH_BINARY)[1]
        kernel_size = int(self.settings.get("morphology_kernel", 11))
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (kernel_size, kernel_size)
        )
        shape_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(shape_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        minimum_area = float(self.settings.get("min_package_area_ratio", 0.08)) * mask.size
        candidates = [contour for contour in contours if cv2.contourArea(contour) >= minimum_area]
        if not candidates:
            return (
                DamageAssessment(
                    calibrated=True,
                    damaged=None,
                    reason="Package contour was not found in the damage ROI",
                ),
                {"damage_roi": image, "damage_mask": mask},
            )

        contour = max(candidates, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        rotated = cv2.minAreaRect(contour)
        rect_area = max(1.0, float(rotated[1][0] * rotated[1][1]))
        hull = cv2.convexHull(contour)
        hull_area = max(1.0, float(cv2.contourArea(hull)))
        perimeter = max(1.0, float(cv2.arcLength(contour, True)))
        epsilon = float(self.settings.get("approx_epsilon_ratio", 0.025)) * perimeter
        vertices = len(cv2.approxPolyDP(contour, epsilon, True))
        rectangularity = area / rect_area
        solidity = area / hull_area
        area_ratio = area / mask.size

        minimum_rectangularity = float(self.settings.get("min_rectangularity", 0.82))
        minimum_solidity = float(self.settings.get("min_solidity", 0.94))
        maximum_vertices = int(self.settings.get("max_vertices", 8))
        minimum_hole_ratio = float(self.settings.get("min_hole_area_ratio", 0.002))
        hole_margin = max(3, int(self.settings.get("hole_border_margin_px", 9)))
        hole_margin = hole_margin if hole_margin % 2 else hole_margin + 1
        filled_package = np.zeros_like(mask)
        cv2.drawContours(filled_package, [contour], -1, 255, cv2.FILLED)
        interior = cv2.erode(
            filled_package,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (hole_margin, hole_margin)),
        )
        void_mask = cv2.bitwise_and(interior, cv2.bitwise_not(shape_mask))
        void_contours, _ = cv2.findContours(
            void_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        minimum_hole_area = minimum_hole_ratio * area
        hole_contours = [
            item for item in void_contours if cv2.contourArea(item) >= minimum_hole_area
        ]
        hole_area_ratio = (
            max((cv2.contourArea(item) for item in hole_contours), default=0.0) / area
        )
        damage_types: list[str] = []
        if rectangularity < minimum_rectangularity:
            damage_types.append("deformation")
        if solidity < minimum_solidity:
            damage_types.append("dent_or_edge_tear")
        if vertices > maximum_vertices:
            damage_types.append("irregular_outline")
        if hole_contours:
            damage_types.append("hole_or_puncture")

        deficits = (
            max(0.0, minimum_rectangularity - rectangularity)
            / max(minimum_rectangularity, 1e-6),
            max(0.0, minimum_solidity - solidity) / max(minimum_solidity, 1e-6),
            max(0.0, vertices - maximum_vertices) / max(maximum_vertices, 1),
            min(1.0, hole_area_ratio / max(minimum_hole_ratio, 1e-6)),
        )
        confidence = min(1.0, max(deficits) * 4.0) if damage_types else min(
            1.0,
            min(
                rectangularity / max(minimum_rectangularity, 1e-6),
                solidity / max(minimum_solidity, 1e-6),
            ),
        )
        x, y, width, height = cv2.boundingRect(contour)
        border_margin = int(self.settings.get("border_margin_px", 10))
        touches_border = (
            x <= border_margin
            or y <= border_margin
            or x + width >= image.shape[1] - border_margin
            or y + height >= image.shape[0] - border_margin
        )
        overlay = image.copy()
        color = (0, 0, 255) if damage_types else (0, 200, 80)
        cv2.drawContours(overlay, [contour], -1, color, 3)
        cv2.polylines(overlay, [cv2.boxPoints(rotated).astype(np.int32)], True, (255, 160, 0), 2)
        if hole_contours:
            cv2.drawContours(overlay, hole_contours, -1, (0, 0, 255), 3)
        debug = {
            "damage_roi": image,
            "damage_mask": mask,
            "damage_overlay": overlay,
            "damage_holes": void_mask,
        }
        if touches_border and self.settings.get("require_full_contour", True):
            return (
                DamageAssessment(
                    calibrated=True,
                    damaged=None,
                    metrics={
                        "area_ratio": float(area_ratio),
                        "rectangularity": float(rectangularity),
                        "solidity": float(solidity),
                        "vertices": float(vertices),
                        "hole_area_ratio": float(hole_area_ratio),
                        "holes": float(len(hole_contours)),
                    },
                    bbox=(x, y, width, height),
                    reason="The full package outline is not visible inside the damage ROI",
                ),
                debug,
            )
        reason = (
            "Package shape defect detected: " + ", ".join(damage_types)
            if damage_types
            else "Package outline passed the configured shape gates"
        )
        assessment = DamageAssessment(
            calibrated=True,
            damaged=bool(damage_types),
            damage_types=tuple(damage_types),
            confidence=float(confidence),
            metrics={
                "area_ratio": float(area_ratio),
                "rectangularity": float(rectangularity),
                "solidity": float(solidity),
                "vertices": float(vertices),
                "hole_area_ratio": float(hole_area_ratio),
                "holes": float(len(hole_contours)),
            },
            bbox=(x, y, width, height),
            reason=reason,
        )
        return assessment, debug
