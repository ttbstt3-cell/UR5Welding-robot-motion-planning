"""Single source of truth for HMI dimensions.

The robot/control code does not use this module.  It only keeps Qt layout
dimensions consistent when the operator changes the visual scale.
"""

from __future__ import annotations


class UiMetrics:
    def __init__(self, scale: float = 1.0):
        self.scale = min(1.5, max(1.0, float(scale)))

    def px(self, value: float) -> int:
        return max(1, round(float(value) * self.scale))

    @property
    def status_row(self) -> int:
        # Compact status rows keep the complete right panel visible at 125%.
        return round(30 + 8 * (self.scale - 1.0))

    @property
    def tcp_row(self) -> int:
        return self.px(28)

    @property
    def joint_row(self) -> int:
        # Keep all six rows visible in the Jog tab.  The row still grows with
        # the UI scale and is always tall enough for the scaled JogButtons
        # (46/54/62 px at 100/125/150%).
        return round(50 + 24 * (self.scale - 1.0))

    @property
    def button_height(self) -> int:
        return self.px(44)

    @property
    def tab_height(self) -> int:
        return self.px(42)

    @property
    def spacing(self) -> int:
        return self.px(8)

    @property
    def panel_margin(self) -> int:
        return self.px(10)
