"""Visual system of the sorting-station HMI (2026-11-25 redesign).

The palette comes from the station itself: the dark green-grey of the conveyor belt is
the background, the kraft brown of a carton marks "the parcel being handled" and the
primary action. Status colours are reserved for decisions (pass / reject / review) and
the web link. Bahnschrift (Windows' DIN-style face, with Vietnamese) sets headings and
numbers; Segoe UI the running text. Labels are sentence case, no all-caps chrome.

The one emphasised element is ``ProcessRoute``: the numbered route of a parcel through
the station (camera -> pick point -> waiting -> robot -> web), lit at the current step.
"""

from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QFrame, QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout

PALETTE = {
    "bg": "#131b1a",        # belt
    "surface": "#1b2523",   # panels
    "raised": "#22302d",    # inputs, hovered rows
    "line": "#304240",
    "text": "#e7edea",
    "muted": "#93a39e",
    "faint": "#61716c",
    "kraft": "#d49b4e",     # parcel / primary action
    "kraft_hi": "#e3ae66",
    "ink": "#1d1407",       # text on kraft
    "pass": "#4fb985",
    "reject": "#e35d4f",
    "review": "#e2b347",
    "info": "#5ca9db",      # data / web
    "danger": "#c8372d",
}
P = PALETTE
DISPLAY = "Bahnschrift"
BODY = "Segoe UI"

DECISION_COLORS = {"PASS": P["pass"], "REJECT": P["reject"], "MANUAL_REVIEW": P["review"]}
DECISION_TEXT = {
    "PASS": "Đạt",
    "REJECT": "Loại",
    "MANUAL_REVIEW": "Cần xem lại",
}

STYLE = f"""
QMainWindow, QWidget {{ background: {P['bg']}; color: {P['text']};
    font-family: "{BODY}"; font-size: 12px; }}
QLabel, QCheckBox, QRadioButton {{ background: transparent; }}
QToolTip {{ background: {P['raised']}; color: {P['text']}; border: 1px solid {P['line']}; padding: 5px; }}
QFrame#panel {{ background: {P['surface']}; border: 1px solid {P['line']}; border-radius: 4px; }}
QFrame#topbar {{ background: {P['surface']}; border: 0; border-bottom: 1px solid {P['line']}; }}
QFrame#subpanel {{ background: {P['bg']}; border: 1px solid {P['line']}; border-radius: 3px; }}
QFrame#statusbar {{ background: transparent; border: 0; }}
QLabel#title {{ font-family: "{DISPLAY}"; font-size: 20px; font-weight: 600; color: {P['text']}; }}
QLabel#subtitle {{ color: {P['muted']}; font-size: 11px; }}
QLabel#sectionTitle {{ font-family: "{DISPLAY}"; color: {P['text']}; font-size: 14px; font-weight: 600; }}
QLabel#hint {{ color: {P['muted']}; font-size: 11px; }}
QLabel#metricTitle {{ color: {P['muted']}; font-size: 11px; }}
QLabel#metricValue {{ font-family: "{DISPLAY}"; color: {P['text']}; font-size: 26px; font-weight: 600; }}
QLabel#metricUnit {{ color: {P['faint']}; font-size: 10px; }}
QLabel#fieldName {{ color: {P['muted']}; font-size: 11px; }}
QLabel#fieldValue {{ color: {P['text']}; font-weight: 600; }}
QLabel#assetReady {{ color: {P['pass']}; background: rgba(79,185,133,30); border-radius: 3px;
    padding: 4px 7px; font-size: 10px; font-weight: 700; }}
QLabel#assetWaiting {{ color: {P['review']}; background: rgba(226,179,71,30); border-radius: 3px;
    padding: 4px 7px; font-size: 10px; font-weight: 700; }}
QLabel#reasonBox {{ background: {P['bg']}; color: {P['muted']}; border: 1px solid {P['line']};
    border-radius: 3px; padding: 8px; }}
QPushButton {{ background: {P['raised']}; color: {P['text']}; border: 1px solid {P['line']};
    border-radius: 3px; padding: 8px 14px; font-weight: 600; }}
QPushButton:hover {{ border-color: {P['muted']}; }}
QPushButton:pressed {{ background: {P['line']}; }}
QPushButton:disabled {{ background: {P['surface']}; color: {P['faint']}; border-color: {P['raised']}; }}
QPushButton:focus {{ outline: none; border-color: {P['kraft']}; }}
QPushButton#primary {{ background: {P['kraft']}; color: {P['ink']}; border: 1px solid {P['kraft_hi']};
    font-family: "{DISPLAY}"; font-size: 15px; padding: 12px 16px; }}
QPushButton#primary:hover {{ background: {P['kraft_hi']}; }}
QPushButton#primary:disabled {{ background: {P['raised']}; color: {P['faint']}; border-color: {P['line']}; }}
QPushButton#running {{ background: {P['bg']}; color: {P['kraft']}; border: 2px solid {P['kraft']};
    font-family: "{DISPLAY}"; font-size: 15px; padding: 11px 16px; }}
QPushButton#confirm {{ background: {P['pass']}; color: #0c1f15; border: 1px solid #7fd2a8;
    font-family: "{DISPLAY}"; font-size: 14px; padding: 10px 14px; }}
QPushButton#confirm:disabled {{ background: {P['surface']}; color: {P['faint']}; border-color: {P['raised']}; }}
QPushButton#danger {{ background: {P['danger']}; color: white; border: 1px solid #ef7a70;
    font-family: "{DISPLAY}"; font-size: 14px; padding: 8px 18px; }}
QPushButton#danger:hover {{ background: #dc4337; }}
QPushButton#secondary {{ background: {P['raised']}; color: {P['text']}; border: 1px solid {P['line']}; }}
QPushButton#viewPreset {{ background: transparent; color: {P['muted']}; border: 1px solid {P['line']};
    border-radius: 3px; padding: 5px 10px; font-size: 11px; }}
QPushButton#viewPreset:checked {{ background: {P['raised']}; color: {P['text']}; border-color: {P['kraft']}; }}
QTableWidget {{ background: {P['surface']}; alternate-background-color: {P['bg']};
    gridline-color: transparent; border: 1px solid {P['line']}; border-radius: 3px;
    selection-background-color: {P['raised']}; selection-color: {P['text']}; }}
QHeaderView::section {{ background: {P['surface']}; color: {P['muted']}; padding: 6px;
    border: 0; border-bottom: 1px solid {P['line']}; font-size: 11px; }}
QCheckBox {{ color: {P['muted']}; spacing: 7px; }}
QCheckBox::indicator {{ width: 15px; height: 15px; border: 1px solid {P['muted']}; border-radius: 3px; background: {P['bg']}; }}
QCheckBox::indicator:checked {{ background: {P['kraft']}; border-color: {P['kraft_hi']}; }}
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{ background: {P['bg']}; color: {P['text']};
    border: 1px solid {P['line']}; border-radius: 3px; padding: 5px; }}
QTreeWidget {{ background: transparent; border: 0; color: {P['muted']}; outline: 0;
    alternate-background-color: {P['bg']}; }}
QTreeWidget::item {{ min-height: 26px; }}
QTreeWidget::item:selected {{ background: {P['raised']}; color: {P['text']}; }}
QSplitter::handle {{ background: transparent; width: 6px; }}
QTabWidget::pane {{ border: 0; border-top: 1px solid {P['line']}; background: {P['bg']}; }}
QTabBar::tab {{ background: transparent; color: {P['muted']}; padding: 9px 18px; border: 0;
    border-bottom: 2px solid transparent; font-size: 13px; }}
QTabBar::tab:hover {{ color: {P['text']}; }}
QTabBar::tab:selected {{ color: {P['text']}; border-bottom: 2px solid {P['kraft']}; font-weight: 600; }}
QTabWidget#inspectorTabs QTabBar::tab {{ padding: 6px 9px; font-size: 11px; }}
QSlider::groove:horizontal {{ height: 4px; background: {P['line']}; border-radius: 2px; }}
QSlider::sub-page:horizontal {{ background: {P['kraft']}; border-radius: 2px; }}
QSlider::handle:horizontal {{ width: 14px; margin: -6px 0; background: {P['text']};
    border: 1px solid {P['kraft']}; border-radius: 7px; }}
QScrollBar:vertical {{ background: {P['bg']}; width: 10px; margin: 0; }}
QScrollBar::handle:vertical {{ background: {P['line']}; min-height: 28px; border-radius: 4px; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
QProgressBar {{ background: {P['bg']}; border: 1px solid {P['line']}; border-radius: 3px;
    color: {P['text']}; text-align: center; }}
QProgressBar::chunk {{ background: {P['kraft']}; }}
"""

# Pill states are codes used across the code base; the pill shows Vietnamese.
STATE_TEXT = {
    "ONLINE": "trực tuyến",
    "OFFLINE": "mất kết nối",
    "READY": "sẵn sàng",
    "FAULT": "lỗi",
    "NOT CONFIGURED": "chưa cấu hình",
    "LOCAL": "cục bộ",
    "LOCAL TEST": "thử cục bộ",
    "BYPASSED": "không gửi",
    "MÔ PHỎNG": "mô phỏng",
}
STATE_COLORS = {
    "ONLINE": P["pass"],
    "READY": P["pass"],
    "OFFLINE": P["reject"],
    "FAULT": P["reject"],
    "NOT CONFIGURED": P["review"],
    "LOCAL": P["info"],
    "LOCAL TEST": P["review"],
    "BYPASSED": P["review"],
    "MÔ PHỎNG": P["muted"],
}


class ProcessRoute(QFrame):
    """Numbered route of the current parcel; one step lit at a time."""

    STEPS = (
        ("Camera kiểm tra", "Chờ kiện tới camera"),
        ("Tới điểm gắp", "Băng tải chạy tới cảm biến"),
        ("Chờ gắp", "Kiện dừng ở điểm gắp"),
        ("Robot thả vào thùng", "Robot hoặc người lấy kiện"),
        ("Lưu lên web", "Ghi kết quả lên Supabase"),
    )

    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("route")
        self.setStyleSheet(f"QFrame#route {{ background: {P['bg']}; border: 0; }}")
        row = QHBoxLayout(self)
        row.setContentsMargins(16, 8, 16, 8)
        row.setSpacing(0)
        self._steps: list[tuple[QFrame, QLabel, QLabel, QLabel]] = []
        for index, (name, detail) in enumerate(self.STEPS):
            if index:
                link = QFrame()
                link.setFixedHeight(2)
                link.setMinimumWidth(18)
                link.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
                link.setStyleSheet(f"background: {P['line']};")
                row.addWidget(link, 1, Qt.AlignmentFlag.AlignVCenter)
            box = QFrame()
            box.setObjectName("step")
            inner = QHBoxLayout(box)
            inner.setContentsMargins(10, 6, 14, 6)
            inner.setSpacing(10)
            number = QLabel(str(index + 1))
            number.setProperty("step", index + 1)
            number.setFixedSize(26, 26)
            number.setAlignment(Qt.AlignmentFlag.AlignCenter)
            texts = QVBoxLayout()
            texts.setSpacing(0)
            title = QLabel(name)
            sub = QLabel(detail)
            texts.addWidget(title)
            texts.addWidget(sub)
            inner.addWidget(number)
            inner.addLayout(texts)
            row.addWidget(box, 0)
            self._steps.append((box, number, title, sub))
        self.detail = [detail for _name, detail in self.STEPS]
        self.web_state = ("pending", self.STEPS[4][1])
        self.set_active(None)

    def set_active(self, index: int | None, text: str | None = None, tone: str = "active") -> None:
        """Light step ``index`` (0..3); earlier steps show as done. ``tone``: active/fault/reject."""
        for position, (box, number, title, sub) in enumerate(self._steps[:4]):
            if index is None:
                state = "pending"
            elif position < index:
                state = "done"
            elif position == index:
                state = tone
            else:
                state = "pending"
            detail = text if (position == index and text) else self.detail[position]
            self._paint(box, number, title, sub, state, detail)
        self._paint(*self._steps[4], *self.web_state)

    def set_web(self, state: str, text: str) -> None:
        """Web step: "done" (sent), "fault" (offline / rows waiting), "pending" (not configured)."""
        self.web_state = (state, text)
        self._paint(*self._steps[4], state, text)

    @staticmethod
    def _paint(box: QFrame, number: QLabel, title: QLabel, sub: QLabel, state: str, detail: str) -> None:
        colours = {
            "pending": (P["faint"], "transparent", P["line"], P["muted"]),
            "done": (P["pass"], "transparent", P["pass"], P["text"]),
            "active": (P["ink"], P["kraft"], P["kraft"], P["text"]),
            "reject": (P["text"], P["reject"], P["reject"], P["text"]),
            "fault": (P["text"], P["danger"], P["danger"], P["text"]),
        }
        num_fg, num_bg, ring, title_fg = colours.get(state, colours["pending"])
        number.setText("✓" if state == "done" else str(number.property("step")))
        number.setStyleSheet(
            f"background:{num_bg}; color:{num_fg}; border:2px solid {ring}; border-radius:13px;"
            f"font-family:'{DISPLAY}'; font-size:13px; font-weight:600;"
        )
        box.setStyleSheet(
            "QFrame#step { background: %s; border: 1px solid %s; border-radius: 4px; }"
            % (P["surface"] if state in ("active", "reject", "fault") else "transparent",
               ring if state in ("active", "reject", "fault") else "transparent")
        )
        title.setStyleSheet(
            f"font-family:'{DISPLAY}'; font-size:14px; font-weight:600; color:{title_fg}; background:transparent;"
        )
        sub.setText(detail)
        sub.setStyleSheet(f"font-size:11px; color:{P['muted']}; background:transparent;")
