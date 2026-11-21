from __future__ import annotations

from collections.abc import Iterator

import cv2
import numpy as np


class CameraError(RuntimeError):
    pass


class OpenCVCamera:
    """UVC camera adapter with explicit read-back of negotiated properties."""

    BACKENDS = {
        "dshow": cv2.CAP_DSHOW,
        "msmf": cv2.CAP_MSMF,
        "any": cv2.CAP_ANY,
    }

    def __init__(self, settings: dict):
        self.settings = settings
        backend = self.BACKENDS.get(str(settings.get("backend", "dshow")).lower(), cv2.CAP_ANY)
        self.capture = cv2.VideoCapture(int(settings.get("index", 0)), backend)
        if not self.capture.isOpened():
            raise CameraError("Cannot open camera")
        self.warnings: list[str] = []
        self._configure()
        self._verify_negotiation()

    def _configure(self) -> None:
        fourcc = str(self.settings.get("fourcc", "MJPG"))
        if len(fourcc) == 4:
            self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.settings.get("width", 2560)))
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.settings.get("height", 1440)))
        self.capture.set(cv2.CAP_PROP_FPS, float(self.settings.get("fps", 30)))

        self._last_warmup: np.ndarray | None = None
        # MSMF ignores exposure writes made before the stream runs and drops into
        # a stuck over-bright manual state instead (verified 2026-10-31).
        for _ in range(int(self.settings.get("pre_control_frames", 5))):
            ok, frame = self.capture.read()
            if ok and frame is not None:
                self._last_warmup = frame

        autofocus = self.settings.get("autofocus")
        if autofocus is not None:
            self.capture.set(cv2.CAP_PROP_AUTOFOCUS, 1 if autofocus else 0)
        exposure = self.settings.get("exposure")
        # The manual/auto mode persists in the driver across sessions, so it is
        # always written: 0.25 = manual, 0.75 = auto (DShow and MSMF).
        self.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75 if exposure is None else 0.25)
        for key, prop in (
            ("focus", cv2.CAP_PROP_FOCUS),
            ("exposure", cv2.CAP_PROP_EXPOSURE),
            ("gain", cv2.CAP_PROP_GAIN),
        ):
            value = self.settings.get(key)
            if value is not None:
                self.capture.set(prop, float(value))

        for _ in range(int(self.settings.get("warmup_frames", 12))):
            ok, frame = self.capture.read()
            if ok and frame is not None:
                self._last_warmup = frame

    def _verify_negotiation(self) -> None:
        """Compare what the driver actually delivers with what the config asked for.

        UVC drivers silently fall back to a lower mode (e.g. 1280x720) when the
        requested one is unsupported or another camera sits at the same index.
        Every ROI is a frame fraction, so the pipeline keeps running – just on a
        much softer image. The real frame shape is the only trustworthy value.
        """
        requested = (int(self.settings.get("width", 2560)), int(self.settings.get("height", 1440)))
        frame = self._last_warmup
        if frame is None:
            ok, frame = self.capture.read()
            if not ok or frame is None:
                raise CameraError("Camera opened but returned no frame")
        actual = (int(frame.shape[1]), int(frame.shape[0]))
        if actual != requested:
            self.warnings.append(
                f"Camera cấp {actual[0]}x{actual[1]} thay vì {requested[0]}x{requested[1]} "
                "như cấu hình. Kiểm tra camera index/cổng USB; ảnh sẽ kém nét hơn dự kiến."
            )
        fourcc = str(self.settings.get("fourcc", "")).upper()
        negotiated = self.fourcc()
        # MSMF reports its decoded output subtype (e.g. 0x16 = RGB24), not a FOURCC;
        # only compare when the driver returned a printable four-character code.
        printable = len(negotiated) == 4 and all(32 < ord(char) < 127 for char in negotiated)
        if len(fourcc) == 4 and printable and negotiated.upper() != fourcc:
            self.warnings.append(f"Driver dùng định dạng {negotiated} thay vì {fourcc}.")
        if self.settings.get("focus") is not None and self.capture.get(cv2.CAP_PROP_FOCUS) < 0:
            self.warnings.append("Driver không hỗ trợ đặt focus thủ công (CAP_PROP_FOCUS).")

    def fourcc(self) -> str:
        value = int(self.capture.get(cv2.CAP_PROP_FOURCC))
        return "".join(chr((value >> (8 * i)) & 0xFF) for i in range(4))

    def negotiated_properties(self) -> dict[str, float | str]:
        return {
            "width": self.capture.get(cv2.CAP_PROP_FRAME_WIDTH),
            "height": self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT),
            "fps": self.capture.get(cv2.CAP_PROP_FPS),
            "fourcc": self.fourcc(),
            "focus": self.capture.get(cv2.CAP_PROP_FOCUS),
            "exposure": self.capture.get(cv2.CAP_PROP_EXPOSURE),
            "gain": self.capture.get(cv2.CAP_PROP_GAIN),
        }

    def read(self) -> np.ndarray:
        ok, frame = self.capture.read()
        if not ok or frame is None:
            raise CameraError("Camera returned an empty frame")
        return frame

    def frames(self) -> Iterator[np.ndarray]:
        while self.capture.isOpened():
            yield self.read()

    def close(self) -> None:
        self.capture.release()

    def __enter__(self) -> OpenCVCamera:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

