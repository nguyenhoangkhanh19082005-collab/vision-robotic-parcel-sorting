"""Regression tests for the 2026-10-29 image-quality / responsiveness changes."""

import time
from pathlib import Path
from tempfile import TemporaryDirectory

import cv2
import numpy as np
import pytest

from machine_vision.camera import opencv_camera
from machine_vision.imaging.enhancement import enhance_for_display
from machine_vision.imaging.perspective import (
    rectify_from_qr_anchor,
    similarity_transform,
    square_qr_target,
)
from machine_vision.models import DamageAssessment, LogoAssessment
from machine_vision.readers.logo_reader import TemplateLogoDetector
from machine_vision.readers.qr_reader import QrReader
from machine_vision.repositories.shipments import InMemoryShipmentRepository
from machine_vision.services.classifier import ParcelClassifier
from machine_vision.services.pipeline import VisionPipeline
from machine_vision.ui import camera_worker
from machine_vision.ui.camera_worker import CameraWorker, FrameBuffer

# --- item 4: square QR target -------------------------------------------------


def test_square_qr_target_derives_height_from_width():
    x, y, w, h = square_qr_target([0.70, 0.54, 0.20, 0.20], 1050, 1480)
    assert (x, y, w) == (0.70, 0.54, 0.20)
    assert w * 1050 == pytest.approx(h * 1480)


def test_rectified_qr_stays_square_even_with_non_square_config():
    image = np.zeros((600, 800, 3), dtype=np.uint8)
    corners = np.array([[300, 200], [420, 200], [420, 320], [300, 320]], dtype=np.float32)
    cv2.rectangle(image, (300, 200), (420, 320), (0, 0, 255), -1)

    label = rectify_from_qr_anchor(image, corners, [0.70, 0.54, 0.20, 0.20], 1050, 1480)

    # Red marker only: the warp border is filled white, which must not count.
    mask = (label[:, :, 2] > 128) & (label[:, :, 0] < 100)
    ys, xs = np.where(mask)
    width = xs.max() - xs.min() + 1
    height = ys.max() - ys.min() + 1
    assert width == pytest.approx(210, abs=3)
    assert height == pytest.approx(width, abs=3)


def test_similarity_transform_recovers_rotation_scale_and_shift():
    angle, scale, shift = np.deg2rad(7.0), 1.8, np.array([40.0, -25.0])
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    source = np.array([[100, 100], [350, 100], [350, 350], [100, 350]], dtype=np.float64)
    destination = (scale * (rotation @ source.T)).T + shift

    matrix = similarity_transform(source, destination)

    mapped = (matrix[:, :2] @ source.T).T + matrix[:, 2]
    assert np.allclose(mapped, destination, atol=1e-3)


def test_similarity_rectification_is_less_sensitive_to_qr_corner_noise():
    image = np.zeros((900, 1200, 3), dtype=np.uint8)
    corners = np.array([[700, 400], [950, 400], [950, 650], [700, 650]], dtype=np.float32)
    noisy = corners + np.array([[2, -2], [-2, -2], [2, 2], [-2, 2]], dtype=np.float32)
    # A marker far from the QR (where the logo sits) shows how far noise moves it.
    cv2.circle(image, (200, 120), 6, (0, 0, 255), -1)
    target = [0.742857, 0.418919, 0.20, 0.141892]

    def marker(transform, quad):
        label = rectify_from_qr_anchor(image, quad, target, 1050, 1480, transform=transform)
        ys, xs = np.where((label[:, :, 2] > 128) & (label[:, :, 0] < 100))
        return np.array([xs.mean(), ys.mean()])

    for transform in ("perspective", "similarity"):
        assert np.allclose(marker(transform, corners), marker("similarity", corners), atol=2)
    drift_perspective = np.linalg.norm(marker("perspective", noisy) - marker("perspective", corners))
    drift_similarity = np.linalg.norm(marker("similarity", noisy) - marker("similarity", corners))
    assert drift_similarity < 0.5 * drift_perspective


# --- item 5: display enhancement ------------------------------------------------


def test_display_enhancement_sharpens_without_changing_shape_or_colour_balance():
    image = np.full((80, 120, 3), (60, 140, 200), dtype=np.uint8)
    cv2.putText(image, "SPX", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (20, 20, 20), 3)
    soft = cv2.GaussianBlur(image, (0, 0), 1.5)

    enhanced = enhance_for_display(soft)

    assert enhanced.shape == soft.shape
    assert cv2.Laplacian(enhanced, cv2.CV_64F).var() > cv2.Laplacian(soft, cv2.CV_64F).var()
    # Luminance-only sharpening keeps flat areas at the same colour.
    assert np.abs(enhanced[2:8, 100:110].astype(int) - soft[2:8, 100:110].astype(int)).max() <= 2


# --- item 6: multiple logo templates and blurry-template guard -----------------


def _logo(text: str) -> np.ndarray:
    image = np.full((60, 160, 3), 235, dtype=np.uint8)
    cv2.putText(image, text, (6, 44), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (10, 10, 10), 3)
    return image


def test_extra_template_detects_second_label_layout():
    with TemporaryDirectory() as temporary:
        root = Path(temporary)
        cv2.imwrite(str(root / "primary.png"), _logo("XPRESS"))
        cv2.imwrite(str(root / "stacked.png"), _logo("SPX 24"))
        label = np.full((300, 500, 3), 235, dtype=np.uint8)
        label[200:260, 300:460] = _logo("SPX 24")
        settings = {
            "template_path": "primary.png",
            "calibration_path": "none.json",
            "search_roi": [0.0, 0.0, 0.5, 0.4],
            "extra_templates": [
                {"name": "spx_stacked", "path": "stacked.png", "search_roi": [0.5, 0.5, 0.5, 0.5]}
            ],
            "threshold": 0.10,
            "min_good_matches": 4,
            "min_inlier_ratio": 0.5,
        }

        without_extra = TemplateLogoDetector({**settings, "extra_templates": []}, root)
        with_extra = TemplateLogoDetector(settings, root)
        single, _ = without_extra.detect(label)
        multi, _ = with_extra.detect(label)

    assert single.present is False
    assert multi.present is True
    assert "spx_stacked" in multi.reason


def test_calibration_rejects_blurry_template():
    label = np.full((300, 500, 3), 235, dtype=np.uint8)
    label[60:120, 80:240] = _logo("XPRESS")
    blurry = cv2.GaussianBlur(label, (0, 0), 6)
    with TemporaryDirectory() as temporary:
        root = Path(temporary)
        detector = TemplateLogoDetector(
            {"template_path": "logo.png", "min_template_features": 1}, root
        )
        with pytest.raises(ValueError, match="quá mờ"):
            detector.calibrate(blurry, (70, 50, 180, 80))
        assert not (root / "logo.png").exists()
        result = detector.calibrate(label, (70, 50, 180, 80))
    assert result["laplacian"] >= 8.0


# --- item 8: the full-frame QR is reused for rectification --------------------


class _Logo:
    def detect(self, image):
        return LogoAssessment(True, True, score=0.9, threshold=0.5), image


class _Damage:
    def inspect(self, _image, _package=None):
        return DamageAssessment(True, False, confidence=0.99), {}


class _CountingReader(QrReader):
    calls = 0
    shapes: list = []

    def read(self, image):
        type(self).calls += 1
        type(self).shapes.append(image.shape[:2])
        return super().read(image)


def test_inspect_decodes_full_frame_qr_once_and_returns_display_label():
    frame = np.full((900, 1400, 3), 255, dtype=np.uint8)
    qr = cv2.resize(
        cv2.QRCodeEncoder_create().encode("VN123456789"), (300, 300),
        interpolation=cv2.INTER_NEAREST,
    )
    frame[300:600, 900:1200] = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)
    config = {
        "acquisition": {"min_laplacian": 0, "min_tenengrad": 0, "motion_threshold": 999},
        "label": {
            "roi": [0, 0, 1, 1],
            "rectify_width": 1050,
            "rectify_height": 1480,
            "qr_roi": [0.70, 0.38, 0.28, 0.22],
            "rectification": {
                "enabled": True,
                "method": "qr_anchor",
                "qr_target_roi": [0.742857, 0.418919, 0.20, 0.141892],
            },
        },
    }
    reader = _CountingReader({"upscale": 2.0})
    _CountingReader.calls = 0
    _CountingReader.shapes = []
    pipeline = VisionPipeline(
        config,
        reader,
        _Logo(),
        _Damage(),
        ParcelClassifier(
            {"reject_bin": "REJECT", "unknown_bin": "MANUAL_REVIEW", "require_qr": True},
            InMemoryShipmentRepository([]),
        ),
    )

    result = pipeline.inspect([frame, frame])

    # The 2K frame is decoded once; the read-ratio check only decodes small QR crops.
    assert _CountingReader.shapes.count(frame.shape[:2]) == 1
    assert all(h < 900 and w < 1400 for h, w in _CountingReader.shapes[1:])
    assert result.classification.parcel.qr_text == "VN123456789"
    assert result.classification.parcel.qr_read_ratio == 1.0
    assert result.label_image.shape == (1480, 1050, 3)


# --- item 3: negotiated resolution check --------------------------------------


class _FakeCapture:
    def __init__(self, shape, fourcc="YUY2", focus=-1.0):
        self.shape = shape
        self.props = {
            cv2.CAP_PROP_FOURCC: float(sum(ord(c) << (8 * i) for i, c in enumerate(fourcc))),
            cv2.CAP_PROP_FOCUS: focus,
        }

    def isOpened(self):
        return True

    def set(self, prop, value):
        return True

    def get(self, prop):
        return self.props.get(prop, 0.0)

    def read(self):
        return True, np.zeros(self.shape, dtype=np.uint8)

    def release(self):
        pass


def test_camera_warns_when_driver_falls_back_to_lower_resolution(monkeypatch):
    monkeypatch.setattr(
        opencv_camera.cv2, "VideoCapture", lambda *_a: _FakeCapture((720, 1280, 3))
    )
    camera = opencv_camera.OpenCVCamera(
        {"width": 2560, "height": 1440, "fourcc": "YUY2", "focus": 120, "warmup_frames": 2}
    )
    text = " ".join(camera.warnings)
    assert "1280x720" in text and "2560x1440" in text
    assert "focus" in text


def test_camera_has_no_warning_when_mode_matches(monkeypatch):
    monkeypatch.setattr(
        opencv_camera.cv2, "VideoCapture", lambda *_a: _FakeCapture((1440, 2560, 3))
    )
    camera = opencv_camera.OpenCVCamera(
        {"width": 2560, "height": 1440, "fourcc": "YUY2", "warmup_frames": 1}
    )
    assert camera.warnings == []


class _RecordingCapture(_FakeCapture):
    def __init__(self, shape):
        super().__init__(shape)
        self.log = []

    def set(self, prop, value):
        self.log.append(("set", prop, value))
        return True

    def read(self):
        self.log.append(("read",))
        return super().read()


@pytest.mark.parametrize(("exposure", "auto_value"), [(-7, 0.25), (None, 0.75)])
def test_exposure_is_written_after_stream_starts_and_mode_is_always_explicit(
    monkeypatch, exposure, auto_value
):
    capture = _RecordingCapture((1440, 2560, 3))
    monkeypatch.setattr(opencv_camera.cv2, "VideoCapture", lambda *_a: capture)
    opencv_camera.OpenCVCamera({"exposure": exposure, "warmup_frames": 1})

    first_read = capture.log.index(("read",))
    auto_index = capture.log.index(("set", cv2.CAP_PROP_AUTO_EXPOSURE, auto_value))
    assert first_read < auto_index
    exposure_writes = [i for i, e in enumerate(capture.log) if e[:2] == ("set", cv2.CAP_PROP_EXPOSURE)]
    if exposure is None:
        assert exposure_writes == []
    else:
        assert exposure_writes and exposure_writes[0] > auto_index


# --- item 7: camera worker thread ----------------------------------------------


class _FakeCamera:
    def __init__(self, _settings):
        self.warnings = ["test warning"]
        self.count = 0

    def read(self):
        time.sleep(0.005)
        self.count += 1
        frame = np.zeros((90, 160, 3), dtype=np.uint8)
        frame[:, : self.count % 160] = 255
        return frame

    def close(self):
        pass


def test_camera_worker_buffers_every_frame_but_never_floods_gui(qt_app, monkeypatch):
    monkeypatch.setattr(camera_worker, "OpenCVCamera", _FakeCamera)
    buffer = FrameBuffer(15)
    delivered, opened = [], []
    worker = CameraWorker({}, [0, 0, 1, 1], buffer, detect_package=lambda _f: None)
    worker.opened.connect(opened.append)
    # Do not acknowledge: a stalled GUI must receive exactly one frame.
    worker.frame_ready.connect(lambda frame, metrics, det: delivered.append((metrics, det)))
    worker.start()
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline and len(buffer) < 15:
        qt_app.processEvents()
        time.sleep(0.01)
    qt_app.processEvents()
    assert opened == [["test warning"]]
    assert len(buffer) == 15
    assert len(delivered) == 1
    assert delivered[0][1] is False  # first frame searched, nothing found

    worker.frame_consumed()
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline and len(delivered) < 2:
        qt_app.processEvents()
        time.sleep(0.01)
    worker.stop()
    assert worker.wait(2000)
    assert len(delivered) == 2
    assert np.isfinite(delivered[1][0].motion)
