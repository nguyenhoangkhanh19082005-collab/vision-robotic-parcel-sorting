"""Background threads for camera acquisition and inspection.

Before this module the GUI thread did everything every 33 ms: a blocking
``VideoCapture.read`` on a 2560x1440 stream, focus metrics (~50 ms per 2K frame)
and drawing, so the live view could not keep up and ``inspect`` (0.5-3 s) froze
the whole window. Now the camera is opened, read and measured in its own thread
and the GUI only paints the newest frame.
"""

from __future__ import annotations

import threading
from collections import deque
from collections.abc import Callable, Iterator
from typing import Any

import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal

from machine_vision.camera.opencv_camera import CameraError, OpenCVCamera
from machine_vision.imaging.enhancement import crop_fraction
from machine_vision.imaging.focus import assess_focus
from machine_vision.models import FocusMetrics


class FrameBuffer:
    """Thread-safe drop-in for the ``deque`` of recent frames used by the GUI."""

    def __init__(self, maxlen: int):
        self._frames: deque[np.ndarray] = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def append(self, frame: np.ndarray) -> None:
        with self._lock:
            self._frames.append(frame)

    def clear(self) -> None:
        with self._lock:
            self._frames.clear()

    def snapshot(self) -> list[np.ndarray]:
        with self._lock:
            return list(self._frames)

    def __len__(self) -> int:
        with self._lock:
            return len(self._frames)

    def __bool__(self) -> bool:
        return len(self) > 0

    def __getitem__(self, index: int) -> np.ndarray:
        with self._lock:
            return self._frames[index]

    def __iter__(self) -> Iterator[np.ndarray]:
        return iter(self.snapshot())


class CameraWorker(QThread):
    """Open the camera, read frames and measure focus off the GUI thread.

    Every frame goes into ``buffer`` (the burst used by ``inspect``). Only the
    newest frame is handed to the GUI; if the GUI is still painting the previous
    one, the new frame is not queued, so signals never pile up behind a slow UI.
    Focus metrics are computed on the full-resolution label ROI because the
    quality-gate thresholds were calibrated at that resolution.
    """

    opened = pyqtSignal(list)  # driver warnings (list[str])
    failed = pyqtSignal(str)
    frame_ready = pyqtSignal(object, object, object)  # frame, FocusMetrics, detection | None

    def __init__(
        self,
        camera_settings: dict,
        label_roi: list[float],
        buffer: FrameBuffer,
        detect_package: Callable[[np.ndarray], Any] | None = None,
        detection_interval: int = 10,
    ):
        super().__init__()
        self.camera_settings = camera_settings
        self.label_roi = label_roi
        self.buffer = buffer
        self.detect_package = detect_package
        self.detection_interval = max(1, int(detection_interval))
        self._running = True
        self._gui_busy = threading.Event()
        self._reset_motion = threading.Event()

    def stop(self) -> None:
        self._running = False

    def frame_consumed(self) -> None:
        """Called by the GUI after painting, allowing the next frame through."""
        self._gui_busy.clear()

    def reset_motion(self) -> None:
        """Forget the previous label so motion is not measured across a reset."""
        self._reset_motion.set()

    def run(self) -> None:
        try:
            camera = OpenCVCamera(self.camera_settings)
        except CameraError as exc:
            self.failed.emit(str(exc))
            return
        self.opened.emit(list(camera.warnings))
        previous_label: np.ndarray | None = None
        counter = 0
        try:
            while self._running:
                try:
                    frame = camera.read()
                except CameraError as exc:
                    self.failed.emit(str(exc))
                    return
                self.buffer.append(frame)
                if self._reset_motion.is_set():
                    self._reset_motion.clear()
                    previous_label = None
                label = crop_fraction(frame, self.label_roi)
                metrics: FocusMetrics = assess_focus(label, previous_label)
                previous_label = label
                detection: Any = None
                counter += 1
                if self.detect_package is not None and counter % self.detection_interval == 1:
                    # ``False`` means "searched, nothing found" (vs None: not searched).
                    detection = self.detect_package(frame) or False
                if not self._gui_busy.is_set():
                    self._gui_busy.set()
                    self.frame_ready.emit(frame, metrics, detection)
        finally:
            camera.close()


class TaskThread(QThread):
    """Run one blocking callable (e.g. ``pipeline.inspect``) in the background."""

    succeeded = pyqtSignal(object)
    errored = pyqtSignal(object)  # the raised exception

    def __init__(self, task: Callable[[], Any]):
        super().__init__()
        self.task = task

    def run(self) -> None:
        try:
            result = self.task()
        except Exception as exc:  # delivered to the GUI thread for display
            self.errored.emit(exc)
            return
        self.succeeded.emit(result)
