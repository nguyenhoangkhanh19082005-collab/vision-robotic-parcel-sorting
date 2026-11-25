"""Mouse wheel must not change sliders / spin boxes (operator report 2026-11-25)."""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtCore import QPoint, QPointF, Qt  # noqa: E402
from PyQt6.QtGui import QWheelEvent  # noqa: E402
from PyQt6.QtWidgets import (  # noqa: E402
    QApplication,
    QScrollArea,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from machine_vision.ui.wheel_guard import install_wheel_guard  # noqa: E402


def _wheel(widget, steps=-3):
    event = QWheelEvent(
        QPointF(5, 5), QPointF(widget.mapToGlobal(QPoint(5, 5))), QPoint(0, 0), QPoint(0, 120 * steps),
        Qt.MouseButton.NoButton, Qt.KeyboardModifier.NoModifier, Qt.ScrollPhase.NoScrollPhase, False,
    )
    QApplication.sendEvent(widget, event)


def test_wheel_scrolls_the_page_but_never_moves_a_slider_or_spin_box():
    app = QApplication.instance() or QApplication([])
    page = QWidget()
    column = QVBoxLayout(page)
    slider, spin = QSlider(Qt.Orientation.Horizontal), QSpinBox()
    slider.setRange(0, 255)
    slider.setValue(40)
    spin.setValue(9)
    column.addWidget(slider)
    column.addWidget(spin)
    column.addSpacing(3000)                     # tall page inside a scroll area
    area = QScrollArea()
    area.setWidget(page)
    area.resize(300, 200)
    area.show()
    app.processEvents()

    _wheel(slider)                              # without the guard the value changes
    assert slider.value() != 40
    slider.setValue(40)

    guard = install_wheel_guard(app)
    try:
        _wheel(slider)
        _wheel(spin)
        assert slider.value() == 40 and spin.value() == 9
        assert area.verticalScrollBar().value() > 0     # the page scrolled instead
    finally:
        app.removeEventFilter(guard)
        area.close()
