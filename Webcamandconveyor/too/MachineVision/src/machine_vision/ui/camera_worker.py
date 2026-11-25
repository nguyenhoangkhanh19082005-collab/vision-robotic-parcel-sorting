"""Background threads for camera acquisition and inspection.

Before this module the GUI thread did everything every 33 ms: a blocking
``VideoCapture.read`` on a 2560x1440 stream, focus metrics (~50 ms per 2K frame)
and drawing, so the live view could not keep up and ``inspect`` (0.5-3 s) froze
the whole window. Now the camera is opened, read and measured in its own thread
and the GUI only paints the newest frame.
"""

from __future__ import annotations

import threading
import time
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
        self._version = 0
        self._stamp = 0.0

    def append(self, frame: np.ndarray) -> None:
        with self._lock:
            self._frames.append(frame)
            self._version += 1
            self._stamp = time.monotonic()

    def latest(self) -> tuple[int, float, np.ndarray] | None:
        """(counter, capture time, frame) of the newest frame; the counter never resets."""
        with self._lock:
            if not self._frames:
                return None
            return self._version, self._stamp, self._frames[-1]

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
    """Open the camera and read frames; measure focus for the GUI on a second thread.

    The capture loop only reads and appends to ``buffer`` (the burst used by
    ``inspect`` and the frames the camera arrival trigger analyses), so it keeps the
    camera rate (30 fps). 2026-11-23: focus metrics on the full-resolution label ROI
    take ~67 ms per frame; computed inline they cut the capture to ~12 fps and the
    arrival trigger missed parcels on the moving belt. The preview thread now takes
    only the newest frame whenever the GUI has finished painting the previous one.
    Focus metrics stay on the full-resolution label ROI because the quality-gate
    thresholds were calibrated at that resolution.
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
        # Counted in *captured* frames (config value); the preview sees about a
        # third of them, so the package search runs at roughly the same rate.
        self.detection_interval = max(1, int(detection_interval) // 3)
        self._running = True
        self._gui_busy = threading.Event()
        self._reset_motion = threading.Event()
        self._new_frame = threading.Event()

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
        preview = threading.Thread(target=self._preview_loop, name="camera-preview", daemon=True)
        preview.start()
        try:
            while self._running:
                try:
                    frame = camera.read()
                except CameraError as exc:
                    self.failed.emit(str(exc))
                    return
                self.buffer.append(frame)
                self._new_frame.set()
        finally:
            self._running = False
            self._new_frame.set()
            preview.join(timeout=2.0)
            camera.close()

    def _preview_loop(self) -> None:
        """Focus metrics + package overlay on the newest frame, at the GUI's pace."""
        previous_label: np.ndarray | None = None
        seen = -1
        counter = 0
        while self._running:
            self._new_frame.wait(0.2)
            self._new_frame.clear()
            if not self._running or self._gui_busy.is_set():
                continue
            latest = self.buffer.latest()
            if latest is None or latest[0] == seen:
                continue
            seen, frame = latest[0], latest[2]
            if self._reset_motion.is_set():
                self._reset_motion.clear()
                previous_label = None
            label = crop_fraction(frame, self.label_roi)
            metrics: FocusMetrics = assess_focus(label, previous_label)
            previous_label = label
            detection: Any = None
            counter += 1
            if self.detect_package is not None and counter % self.detection_interval == 1 % self.detection_interval:
                # ``False`` means "searched, nothing found" (vs None: not searched).
                detection = self.detect_package(frame) or False
            self._gui_busy.set()
            self.frame_ready.emit(frame, metrics, detection)


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
