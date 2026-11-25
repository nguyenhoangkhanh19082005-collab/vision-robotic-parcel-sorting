"""Mouse wheel never changes a slider, spin box or combo box by accident (2026-11-25).

Operator report: scrolling the mouse wheel over the GUI moved the drag sliders (belt PWM,
robot joint jog in the Robot tab) and changed their values. With the robot connected a
stray wheel turn over a joint slider is a motion command, so the wheel is blocked on
every slider / spin box / combo box in the whole application. The wheel is handed to
the nearest scroll area instead, so pages and lists still scroll. Values change only by
dragging, clicking the arrows or typing.
"""

from __future__ import annotations

from PyQt6.QtCore import QEvent, QObject
from PyQt6.QtWidgets import (
    QAbstractScrollArea,
    QAbstractSlider,
    QAbstractSpinBox,
    QApplication,
    QComboBox,
    QScrollBar,
)

GUARDED = (QAbstractSlider, QAbstractSpinBox, QComboBox)


class WheelGuard(QObject):
    def eventFilter(self, watched: QObject, event: QEvent) -> bool:  # noqa: N802 - Qt API
        if event.type() != QEvent.Type.Wheel:
            return False
        if not isinstance(watched, GUARDED) or isinstance(watched, QScrollBar):
            return False
        # The spin box / combo box line edit receives the wheel too: guard its owner.
        area = watched.parentWidget()
        while area is not None and not isinstance(area, QAbstractScrollArea):
            area = area.parentWidget()
        if area is not None:
            QApplication.sendEvent(area.viewport(), event)
        return True


def install_wheel_guard(app: QApplication) -> WheelGuard:
    guard = WheelGuard(app)
    app.installEventFilter(guard)
    return guard
