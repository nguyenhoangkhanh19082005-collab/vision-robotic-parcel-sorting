"""Background-free label tear check, anchored on the decoded QR code.

The label paper is the brightest, most neutral surface around the QR. Its
outline is segmented by lightness *relative to the QR quiet zone* (so it follows
exposure and lamp changes) and must stay a convex rectangle: a torn label leaves
a notch where cardboard or background shows through. Notch depth is measured in
QR side lengths, so the check is independent of camera height and resolution.

2026-11-04 station captures: intact label max notch 0.16 QR, five torn labels
0.39-0.41 QR (close/open 0.20 QR).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

from machine_vision.imaging.perspective import order_quad


@dataclass
class LabelIntegrity:
    ok: bool | None
    reason: str
    metrics: dict[str, float] = field(default_factory=dict)
    contour: np.ndarray | None = None
    notch_point: tuple[int, int] | None = None
    mask: np.ndarray | None = None


def _kernel(size: float) -> np.ndarray:
    side = max(3, int(size) | 1)
    return cv2.getStructuringElement(cv2.MORPH_RECT, (side, side))


def assess_label_integrity(
    frame: np.ndarray, qr_corners: np.ndarray, settings: dict | None = None
) -> LabelIntegrity:
    settings = settings or {}
    quad = order_quad(qr_corners).astype(np.float32)
    side = float(np.mean(np.linalg.norm(quad - np.roll(quad, 1, axis=0), axis=1)))
    if side < 20:
        return LabelIntegrity(None, "QR quá nhỏ để kiểm tra nhãn")

    lab = cv2.cvtColor(cv2.GaussianBlur(frame, (0, 0), 3), cv2.COLOR_BGR2LAB).astype(np.float32)
    # Quiet zone ring around the QR is guaranteed label paper.
    centre = quad.mean(axis=0)
    ring = np.zeros(frame.shape[:2], dtype=np.uint8)
    cv2.fillConvexPoly(ring, np.round(centre + (quad - centre) * 1.35).astype(np.int32), 255)
    cv2.fillConvexPoly(ring, np.round(centre + (quad - centre) * 1.12).astype(np.int32), 0)
    ring_pixels = lab[ring > 0]
    if ring_pixels.size == 0:
        return LabelIntegrity(None, "Không lấy được màu giấy quanh QR")
    reference = np.median(ring_pixels, axis=0)

    chroma = np.hypot(lab[:, :, 1] - reference[1], lab[:, :, 2] - reference[2])
    paper = np.uint8(
        (lab[:, :, 0] > float(settings.get("label_paper_lightness_ratio", 0.80)) * reference[0])
        & (chroma < float(settings.get("label_paper_max_chroma", 7.0)))
    ) * 255
    # Close fills print (text, barcode, QR); open cuts thin bright leaks such as rails.
    paper = cv2.morphologyEx(
        paper, cv2.MORPH_CLOSE, _kernel(float(settings.get("label_close_qr", 0.20)) * side)
    )
    paper = cv2.morphologyEx(
        paper, cv2.MORPH_OPEN, _kernel(float(settings.get("label_open_qr", 0.20)) * side)
    )
    count, labels = cv2.connectedComponents(paper)
    seeds = labels[(ring > 0) & (paper > 0)]
    seeds = seeds[seeds > 0]
    if count < 2 or seeds.size == 0:
        return LabelIntegrity(None, "Không tách được vùng giấy nhãn")
    component = np.uint8(labels == np.bincount(seeds).argmax()) * 255

    contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    width, height = cv2.minAreaRect(contour)[1]
    rectangularity = area / max(1.0, float(width * height))
    height_px, width_px = frame.shape[:2]
    x, y, w, h = cv2.boundingRect(contour)
    margin = 3
    touches_border = x <= margin or y <= margin or x + w >= width_px - margin or y + h >= height_px - margin

    depth = 0.0
    notch_point = None
    hull = cv2.convexHull(contour, returnPoints=False)
    if hull is not None and len(hull) > 3:
        defects = cv2.convexityDefects(contour, hull)
        # OpenCV 4.x returns (N, 1, 4), 5.x returns (N, 4).
        defects = None if defects is None else defects.reshape(-1, 1, 4)
        if defects is not None:
            index = int(np.argmax(defects[:, 0, 3]))
            depth = float(defects[index, 0, 3]) / 256.0
            point = contour[int(defects[index, 0, 2])][0]
            notch_point = (int(point[0]), int(point[1]))
    metrics = {
        "label_qr_side_px": side,
        "label_area_qr2": area / side**2,
        "label_rectangularity": rectangularity,
        "label_notch_qr": depth / side,
    }
    if touches_border:
        return LabelIntegrity(
            None, "Nhãn chạm mép ảnh, không thấy đủ viền", metrics, contour, None, component
        )
    if area / side**2 < float(settings.get("label_min_area_qr2", 4.0)):
        return LabelIntegrity(
            None, "Vùng giấy nhãn quá nhỏ (phân vùng lỗi)", metrics, contour, None, component
        )
    limit = float(settings.get("label_max_notch_qr", 0.27))
    if depth / side > limit:
        return LabelIntegrity(
            False,
            f"Nhãn bị rách/khuyết: khía sâu {depth / side:.2f} cạnh QR (> {limit:.2f})",
            metrics,
            contour,
            notch_point,
            component,
        )
    return LabelIntegrity(True, "Viền nhãn nguyên vẹn", metrics, contour, notch_point, component)
