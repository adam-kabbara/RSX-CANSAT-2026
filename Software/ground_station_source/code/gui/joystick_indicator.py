import math
from PyQt6.QtCore import Qt, QPointF, QRectF
from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPainter, QPen, QColor, QFont


class JoystickIndicator(QWidget):
    """
    Radar-style joystick position display.
    Accepts roll/pitch in the range -1.0 … +1.0.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(140, 140) # todo Ensure it's large enough to look good
        self._roll  = 0.0
        self._pitch = 0.0

    def set_roll(self, val: float):
        self._roll = max(-1.0, min(1.0, val))
        self.update()

    def set_pitch(self, val: float):
        self._pitch = max(-1.0, min(1.0, val))
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setRenderHint(QPainter.RenderHint.TextAntialiasing)

        w, h   = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0
        radius = min(cx, cy) - 20

        # Background circle
        p.setPen(QPen(QColor("#4CAF50"), 2))
        p.setBrush(QColor(0, 0, 0, 60))
        p.drawEllipse(QPointF(cx, cy), radius, radius)

        # Crosshair
        p.setPen(QPen(QColor("#444444"), 1, Qt.PenStyle.DashLine))
        p.drawLine(QPointF(cx - radius, cy), QPointF(cx + radius, cy))
        p.drawLine(QPointF(cx, cy - radius), QPointF(cx, cy + radius))

        # Dot position, clamped to circle boundary
        x = self._roll  * radius
        y = self._pitch * radius
        mag = math.sqrt(x * x + y * y)
        if mag > radius:
            x = x / mag * radius
            y = y / mag * radius

        dot_r = max(6.0, radius * 0.08)
        p.setBrush(QColor("#FF5722"))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(cx + x, cy + y), dot_r, dot_r)

        # Value label
        p.setPen(QColor("#00FF88"))
        p.setFont(QFont("Courier New", 8, QFont.Weight.Bold))
        p.drawText(
            QRectF(0, h - 18, w, 16),
            Qt.AlignmentFlag.AlignCenter,
            f"R:{self._roll:+.2f}  P:{self._pitch:+.2f}",
        )