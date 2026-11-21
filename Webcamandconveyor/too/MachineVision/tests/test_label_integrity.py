"""Label tear check, damage segmentation guards and QR reader fast path (2026-11-04 redesign)."""

from pathlib import Path
from tempfile import TemporaryDirectory

import cv2
import numpy as np
import pytest

from machine_vision.models import PackageDetection
from machine_vision.readers.qr_reader import QrReader
from machine_vision.services.damage import PackageDamageDetector
from machine_vision.services.label_integrity import assess_label_integrity

SETTINGS = {"label_close_qr": 0.20, "label_open_qr": 0.20, "label_max_notch_qr": 0.27}


def _scene(tear: bool = False, label_box=(300, 200, 900, 760), hole=None):
    """Dark belt, brown parcel, white label with a QR (side 160 px) and print."""
    frame = np.full((1000, 1400, 3), (40, 70, 35), dtype=np.uint8)
    cv2.rectangle(frame, (250, 150), (950, 810), (70, 120, 160), -1)  # cardboard
    x0, y0, x1, y1 = label_box
    cv2.rectangle(frame, (x0, y0), (x1, y1), (235, 235, 235), -1)
    for row in range(y0 + 40, y0 + 260, 40):
        cv2.line(frame, (x0 + 40, row), (x0 + 320, row), (20, 20, 20), 6)
    qr = cv2.QRCodeEncoder_create().encode("SPXVN31601072127")
    qr = cv2.resize(qr, (160, 160), interpolation=cv2.INTER_NEAREST)
    frame[y1 - 240 : y1 - 80, x1 - 240 : x1 - 80] = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)
    if tear:
        notch = np.array([[x1, y0 + 150], [x1 - 75, y0 + 200], [x1, y0 + 260]], dtype=np.int32)
        cv2.fillPoly(frame, [notch], (70, 120, 160))  # cardboard shows through
    if hole is not None:
        cv2.circle(frame, hole, 22, (10, 10, 10), -1)
    return frame


def _corners(frame):
    read = QrReader({"upscales": [1.0, 2.0]}).read(frame)
    assert read is not None and read.corners is not None
    return read.corners


def test_intact_label_passes_and_torn_label_is_detected():
    intact = _scene()
    torn = _scene(tear=True)

    ok = assess_label_integrity(intact, _corners(intact), SETTINGS)
    bad = assess_label_integrity(torn, _corners(torn), SETTINGS)

    assert ok.ok is True, ok.reason
    assert ok.metrics["label_notch_qr"] < 0.27
    assert bad.ok is False, bad.reason
    assert bad.metrics["label_notch_qr"] > 0.27
    assert bad.notch_point is not None and bad.notch_point[0] > 800


def test_label_touching_image_border_is_inconclusive():
    frame = _scene(label_box=(0, 200, 900, 760))
    result = assess_label_integrity(frame, _corners(frame), SETTINGS)
    assert result.ok is None
    assert "mép ảnh" in result.reason


def _detector(root: Path, background: np.ndarray, **extra) -> PackageDamageDetector:
    cv2.imwrite(str(root / "empty.png"), background)
    return PackageDamageDetector(
        {"background_path": "empty.png", "difference_threshold": 25, "blur_kernel": 5,
         "morphology_kernel": 7, **SETTINGS, **extra},
        root,
    )


def _package(frame):
    corners = _corners(frame)
    x, y, w, h = cv2.boundingRect(corners.astype(np.int32))
    # Deliberately oversized box (the old detector's failure mode): must not matter.
    return PackageDetection(bbox=(0, 0, frame.shape[1], frame.shape[0]), label_bbox=(x, y, w, h),
                            confidence=0.62, qr_corners=corners)


def test_dark_fixture_outside_parcel_is_not_a_hole_but_hole_inside_is():
    background = np.full((1000, 1400, 3), (40, 70, 35), dtype=np.uint8)
    cv2.rectangle(background, (1150, 700), (1300, 820), (15, 15, 15), -1)  # sensor, in background
    clean = _scene()
    cv2.rectangle(clean, (1150, 700), (1300, 820), (15, 15, 15), -1)
    punctured = _scene(hole=(500, 650))
    cv2.rectangle(punctured, (1150, 700), (1300, 820), (15, 15, 15), -1)

    with TemporaryDirectory() as temporary:
        detector = _detector(Path(temporary), background)
        ok, _ = detector.inspect(clean, _package(clean))
        bad, debug = detector.inspect(punctured, _package(punctured))

    assert ok.damaged is False, ok.reason
    assert "hole_or_puncture" not in ok.damage_types
    assert bad.damaged is True
    assert "hole_or_puncture" in bad.damage_types
    assert debug["surface_hole_mask"].any()


def test_stale_background_skips_outline_but_keeps_label_verdict():
    stale = np.full((1000, 1400, 3), 200, dtype=np.uint8)  # scene changed completely
    torn = _scene(tear=True)
    intact = _scene()
    with TemporaryDirectory() as temporary:
        detector = _detector(Path(temporary), stale)
        bad, _ = detector.inspect(torn, _package(torn))
        ok, _ = detector.inspect(intact, _package(intact))
        strict = _detector(Path(temporary), stale, require_parcel_outline=True)
        unsure, _ = strict.inspect(intact, _package(intact))

    assert bad.damaged is True and bad.damage_types == ("label_tear",)
    assert ok.damaged is False and "chưa kiểm tra viền kiện" in ok.reason
    assert unsure.damaged is None


def test_qr_reader_runs_slow_opencv_fallback_once(monkeypatch):
    reader = QrReader({"upscales": [2.0, 3.0]})
    calls = []

    class _CountingDetector:
        def detectAndDecode(self, image):  # noqa: N802 - OpenCV API name
            calls.append(image.shape)
            return "", None, None

    monkeypatch.setattr(reader, "_opencv", _CountingDetector())
    assert reader.read(np.full((200, 300, 3), 128, dtype=np.uint8)) is None
    assert calls == [(200, 300)]


def test_background_capture_refuses_frames_with_a_parcel():
    from machine_vision.services.pipeline import VisionPipeline

    with TemporaryDirectory() as temporary:
        root = Path(temporary)
        damage = PackageDamageDetector({"bench_background_path": "bench.png"}, root, root)
        pipeline = VisionPipeline.__new__(VisionPipeline)
        pipeline.qr_reader = QrReader({"upscales": [1.0]})
        pipeline.damage_detector = damage
        with pytest.raises(ValueError, match="mã QR"):
            pipeline.capture_bench_background([_scene()] * 3)
        empty = np.full((1000, 1400, 3), (40, 70, 35), dtype=np.uint8)
        assert pipeline.capture_bench_background([empty] * 3).is_file()
