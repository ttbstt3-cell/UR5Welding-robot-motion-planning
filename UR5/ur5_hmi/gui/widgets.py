from __future__ import annotations

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QLabel, QPushButton, QHBoxLayout, QSizePolicy, QWidget


def set_status_label(label: QLabel, text: str, level: str = "unknown", scale: float = 1.0) -> None:
    colors = {
        "ok": ("#d8f5dc", "#146c2e"),
        "warn": ("#fff2cc", "#8a5a00"),
        "bad": ("#ffd9d9", "#9b1c1c"),
        "unknown": ("#e8e8e8", "#444444"),
    }
    bg, fg = colors.get(level, colors["unknown"])
    label.setText(text)
    label.setAlignment(Qt.AlignCenter)
    label.setMinimumWidth(round(170 * scale))
    compact_row_height = round(30 + 8 * (float(scale) - 1.0))
    label.setMinimumHeight(compact_row_height)
    label.setStyleSheet(
        f"QLabel {{ background-color:{bg}; color:{fg}; "
        f"border:1px solid #999; border-radius:4px; padding:{round(3 * scale)}px {round(8 * scale)}px; "
        f"font-size:{round(14 * scale)}px; }}"
    )


class StatusRow(QWidget):
    """A readable ``name | colored value`` row used by status cards."""

    def __init__(self, name: str, scale: float = 1.0, parent=None):
        super().__init__(parent)
        self.name_label = QLabel(name)
        self.value_label = QLabel("Unknown")
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(max(6, round(8 * scale)))
        layout.addWidget(self.name_label, 1)
        layout.addStretch(1)
        layout.addWidget(self.value_label, 0)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.set_scale(scale)

    def set_scale(self, scale: float) -> None:
        scale = min(1.5, max(1.0, float(scale)))
        compact_row_height = round(30 + 8 * (scale - 1.0))
        self.setMinimumHeight(compact_row_height)
        self.name_label.setMinimumWidth(round(145 * scale))
        self.name_label.setMinimumHeight(compact_row_height)
        self.name_label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        self.name_label.setStyleSheet(f"font-size:{round(15 * scale)}px;")
        set_status_label(self.value_label, self.value_label.text(), "unknown", scale)
        self.value_label.setMinimumWidth(round(170 * scale))

    def set_value(self, text: str, level: str = "unknown", scale: float = 1.0) -> None:
        set_status_label(self.value_label, text, level, scale)
        self.value_label.setMinimumWidth(round(170 * scale))


class JogButtons(QWidget):
    def __init__(self, index: int, on_jog, scale: float = 1.0, parent=None):
        super().__init__(parent)
        self.index = index
        self.minus = QPushButton("−")
        self.plus = QPushButton("+")
        # Jog is deliberately one-shot.  An accidental mouse/touch hold must
        # never turn into a stream of independent motion goals.
        self.minus.setAutoRepeat(False)
        self.plus.setAutoRepeat(False)
        self.minus.clicked.connect(lambda: on_jog(index, -1.0))
        self.plus.clicked.connect(lambda: on_jog(index, 1.0))
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.minus)
        layout.addWidget(self.plus)
        self.set_scale(scale)

    def set_scale(self, scale: float) -> None:
        # Deliberately use the prescribed 72/84/96 × 46/54/62 sizes instead
        # of letting a parent table compress the two controls.
        button_width = round(72 + 48 * (scale - 1.0))
        button_height = round(46 + 32 * (scale - 1.0))
        font_size = round(22 + 16 * (scale - 1.0))
        style = (
            f"QPushButton {{ font-size:{font_size}px; font-weight:bold; }}"
            "QPushButton:hover { background:#dbeafe; }"
            "QPushButton:pressed { background:#93c5fd; }"
            "QPushButton:disabled { color:#9ca3af; background:#e5e7eb; }"
        )
        for button in (self.minus, self.plus):
            button.setMinimumSize(button_width, button_height)
            button.setStyleSheet(style)
        self.setMinimumSize(button_width * 2 + round(8 * scale), button_height)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

    def set_enabled(self, enabled: bool) -> None:
        self.minus.setEnabled(enabled)
        self.plus.setEnabled(enabled)
