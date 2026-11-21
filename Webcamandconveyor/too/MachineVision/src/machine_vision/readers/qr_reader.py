from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from machine_vision.imaging.enhancement import qr_variants


@dataclass(frozen=True)
class QrRead:
    text: str
    format: str
    variant_index: int
    corners: np.ndarray | None = None


class QrReader:
    """ZXing-C++ reader with OpenCV fallback and preprocessing variants."""

    def __init__(self, settings: dict):
        self.settings = settings
        self._opencv = cv2.QRCodeDetector()
        try:
            import zxingcpp  # type: ignore
        except ImportError:
            zxingcpp = None
        self._zxing = zxingcpp

    def read(self, image: np.ndarray) -> QrRead | None:
        configured = self.settings.get("upscales")
        upscales = (
            configured
            if isinstance(configured, list) and configured
            else [self.settings.get("upscale", 2.0)]
        )
        variant_index = 0
        for upscale in upscales:
            variants = qr_variants(
                image,
                upscale=float(upscale),
                inverted=bool(self.settings.get("try_inverted", True)),
            )
            for variant in variants:
                result = self._read_zxing(variant)
                if result:
                    return self._scaled(result, image, variant, variant_index)
                variant_index += 1
        # OpenCV's detector is slow on upscaled frames (seconds at 7680x4320), so it
        # runs once on the original image after every ZXing attempt has failed.
        base = variants[0]
        result = self._read_opencv(base)
        return self._scaled(result, image, base, variant_index) if result else None

    @staticmethod
    def _scaled(
        result: tuple[str, str, np.ndarray | None], image: np.ndarray, variant: np.ndarray, index: int
    ) -> QrRead:
        text, code_format, corners = result
        if corners is not None:
            scale_x = image.shape[1] / variant.shape[1]
            scale_y = image.shape[0] / variant.shape[0]
            corners = corners * np.array([scale_x, scale_y], dtype=np.float32)
        return QrRead(text, code_format, index, corners)

    def _read_zxing(self, image: np.ndarray) -> tuple[str, str, np.ndarray | None] | None:
        if self._zxing is None:
            return self._read_opencv(image)
        # QR-only search is 10-40x faster than scanning every 1D/2D format, and the
        # global-histogram binarizer recovers soft codes the local one misses.
        for binarizer in (self._zxing.Binarizer.LocalAverage, self._zxing.Binarizer.GlobalHistogram):
            results = self._zxing.read_barcodes(
                image, formats=self._zxing.BarcodeFormat.QRCode, binarizer=binarizer
            )
            qr_results = [value for value in results if "QR" in str(value.format).upper()]
            if qr_results:
                value = qr_results[0]
                text = value.text.strip()
                if text:
                    position = value.position
                    corners = np.array(
                        [
                            [position.top_left.x, position.top_left.y],
                            [position.top_right.x, position.top_right.y],
                            [position.bottom_right.x, position.bottom_right.y],
                            [position.bottom_left.x, position.bottom_left.y],
                        ],
                        dtype=np.float32,
                    )
                    return text, str(value.format), corners
        return None

    def _read_opencv(self, image: np.ndarray) -> tuple[str, str, np.ndarray | None] | None:
        text, points, _straight = self._opencv.detectAndDecode(image)
        if text.strip():
            corners = None if points is None else np.asarray(points, dtype=np.float32).reshape(4, 2)
            return text.strip(), "QR_CODE", corners
        return None
