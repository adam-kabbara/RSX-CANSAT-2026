"""
attitude_indicator.py
---------------------
Drop-in PyQt6 Attitude Indicator (Artificial Horizon) widget.

Usage:
    from .attitude_indicator import AttitudeIndicator
    ai = AttitudeIndicator()
    layout.addWidget(ai)
    ai.set_roll(degrees)    # -180 … +180
    ai.set_pitch(degrees)   # -90  … +90
    ai.set_yaw(degrees)     # 0    … 360
"""

import math
from PyQt6.QtCore    import Qt, QPointF, QRectF
from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui     import (QPainter, QPen, QColor, QBrush, QPainterPath,
                              QLinearGradient, QFont, QPolygonF)


class AttitudeIndicator(QWidget):
    """
    Artificial-horizon attitude indicator.

    Paints:
      - Sky / ground with smooth gradients that rotate on roll
      - Pitch ladder  (±5° minor, ±10° major with degree labels)
      - Bank-angle arc (-60 … +60°) with fixed ticks and moving pointer
      - Fixed gold aircraft reference symbol
      - Heading (yaw) readout box at the bottom
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(140, 140) # todo, might need tuning for the gnd station screen dims
        self._roll  = 0.0
        self._pitch = 0.0
        self._yaw   = 0.0

    def set_roll(self, degrees: float):
        self._roll = degrees
        self.update()

    def set_pitch(self, degrees: float):
        self._pitch = max(-90.0, min(90.0, degrees))
        self.update()

    def set_yaw(self, degrees: float):
        self._yaw = degrees % 360.0
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setRenderHint(QPainter.RenderHint.TextAntialiasing)

        w, h   = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0
        R      = min(cx, cy) - 3.0

        p.translate(cx, cy)

        clip = QPainterPath()
        clip.addEllipse(QPointF(0.0, 0.0), R, R)
        p.setClipPath(clip)

        p.save()
        p.rotate(-self._roll)

        py = self._pitch * (R / 45.0)
        D  = R * 2.5

        sky = QLinearGradient(QPointF(0.0, -D), QPointF(0.0, py))
        sky.setColorAt(0.0, QColor("#0b3d6b"))
        sky.setColorAt(1.0, QColor("#4a90c4"))
        p.fillRect(QRectF(-D, -D, D * 2.0, D + py), QBrush(sky))

        gnd = QLinearGradient(QPointF(0.0, py), QPointF(0.0, D))
        gnd.setColorAt(0.0, QColor("#7a4520"))
        gnd.setColorAt(1.0, QColor("#2c1508"))
        p.fillRect(QRectF(-D, py, D * 2.0, D), QBrush(gnd))

        p.setPen(QPen(QColor("white"), 2.0))
        p.drawLine(QPointF(-D, py), QPointF(D, py))

        self._draw_pitch_ladder(p, R, py)
        p.restore()

        self._draw_bank_arc(p, R)
        self._draw_aircraft(p, R)

        p.setClipping(False)
        p.setPen(QPen(QColor("#111111"), 5))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(QPointF(0.0, 0.0), R, R)

        self._draw_heading_box(p, R)

    def _draw_pitch_ladder(self, p: QPainter, R: float, py: float):
        fnt = QFont("Courier New", 7)
        p.setFont(fnt)

        for deg in range(-30, 31, 5):
            if deg == 0:
                continue
            y = py - deg * (R / 45.0)
            if abs(y) > R * 0.88:
                continue

            major = (deg % 10 == 0)
            hw    = R * (0.30 if major else 0.17)

            p.setPen(QPen(QColor("white"), 1.5 if major else 0.8))
            p.drawLine(QPointF(-hw, y), QPointF(hw, y))

            if major:
                cap = 4.0
                p.drawLine(QPointF(-hw, y), QPointF(-hw, y - cap))
                p.drawLine(QPointF( hw, y), QPointF( hw, y - cap))

                p.setPen(QColor("white"))
                lbl = str(abs(deg))
                p.drawText(QRectF(hw + 4, y - 7, 24, 14),
                           Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, lbl)
                p.drawText(QRectF(-hw - 28, y - 7, 24, 14),
                           Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, lbl)

    def _draw_bank_arc(self, p: QPainter, R: float):
        arc_r = R - 18.0
        rect  = QRectF(-arc_r, -arc_r, arc_r * 2.0, arc_r * 2.0)

        p.setPen(QPen(QColor("#777777"), 1.0))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawArc(rect, int((90 - 60) * 16), int(120 * 16))

        for ang in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            p.save()
            p.rotate(ang)
            major = ang in (-60, -30, 0, 30, 60)
            tlen  = 11.0 if major else 6.0
            p.setPen(QPen(QColor("white"), 1.8 if major else 1.0))
            p.drawLine(QPointF(0.0, -arc_r), QPointF(0.0, -arc_r + tlen))
            p.restore()

        p.save()
        p.rotate(self._roll)
        tip_y  = -arc_r + 1.0
        base_y = -arc_r + 14.0
        tri = QPolygonF([
            QPointF(0.0, tip_y),
            QPointF(-5.5, base_y),
            QPointF( 5.5, base_y),
        ])
        p.setBrush(QColor("white"))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(tri)
        p.restore()

    def _draw_aircraft(self, p: QPainter, R: float):
        pen = QPen(QColor("#FFD700"), 3.0, Qt.PenStyle.SolidLine,
                   Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin)
        p.setPen(pen)
        p.setBrush(Qt.BrushStyle.NoBrush)

        W  = R * 0.52
        nb = R * 0.12

        p.drawLine(QPointF(-W, 0.0), QPointF(-R * 0.16, 0.0))
        p.drawLine(QPointF(-R * 0.16, 0.0), QPointF(-R * 0.16, nb))
        p.drawLine(QPointF(R * 0.16, 0.0), QPointF(W, 0.0))
        p.drawLine(QPointF(R * 0.16, 0.0), QPointF(R * 0.16, nb))

        p.setBrush(QColor("#FFD700"))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(0.0, 0.0), 3.5, 3.5)

    def _draw_heading_box(self, p: QPainter, R: float):
        bw, bh = 68.0, 19.0
        by = R - bh - 5.0

        p.setBrush(QColor(0, 0, 0, 190))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRoundedRect(QRectF(-bw / 2.0, by, bw, bh), 3.0, 3.0)

        p.setPen(QColor("#00FF88"))
        fnt = QFont("Courier New", 9, QFont.Weight.Bold)
        p.setFont(fnt)
        p.drawText(QRectF(-bw / 2.0, by, bw, bh),
                   Qt.AlignmentFlag.AlignCenter, f"HDG {int(self._yaw):03d}°")
