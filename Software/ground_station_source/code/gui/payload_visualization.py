"""
Lightweight payload orientation visualization.

This is intentionally a 2.5D Qt canvas instead of a real 3D renderer so it can
stay responsive on the Raspberry Pi ground station.
"""

import math
import os
import time
from dataclasses import dataclass
from typing import Any

from PyQt6.QtCore import QEvent, QPointF, QRectF, Qt, QTimer
from PyQt6.QtGui import (
    QBrush,
    QColor,
    QFont,
    QIcon,
    QPainter,
    QPen,
    QPolygonF,
)
from PyQt6.QtWidgets import QMainWindow, QWidget

from . import cosmetics


Vector3 = tuple[float, float, float]


@dataclass
class TimedVector:
    vector: Vector3
    magnitude: float
    updated_at: float


class PayloadVisualizationCanvas(QWidget):
    """Draws a simple payload body with derived telemetry vectors."""

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setMinimumSize(720, 560)
        self.setAutoFillBackground(False)
        self.setAttribute(Qt.WidgetAttribute.WA_AcceptTouchEvents, True)
        self.setCursor(Qt.CursorShape.OpenHandCursor)

        self._roll = math.radians(-6.0)
        self._pitch = math.radians(10.0)
        self._yaw = math.radians(-18.0)
        self._view_yaw = math.radians(0.0)
        self._view_pitch = math.radians(0.0)
        self._drag_sensitivity = 0.006
        self._dragging = False
        self._last_drag_pos = None
        self._last_gyro_time = None
        self._last_packet_at = None

        self._accel = None
        self._speed = None
        self._last_gps_sample = None
        self._state_text = "Unknown"
        self._mission_time = "00:00:00"
        self._packet_count = "0"

        self._frame_timer = QTimer(self)
        self._frame_timer.setInterval(50)
        self._frame_timer.setTimerType(Qt.TimerType.CoarseTimer)
        self._frame_timer.timeout.connect(self.update)
        self._frame_timer.start()

    def event(self, event):
        if event.type() in (
            QEvent.Type.TouchBegin,
            QEvent.Type.TouchUpdate,
            QEvent.Type.TouchEnd,
            QEvent.Type.TouchCancel,
        ):
            return self._handle_touch_event(event)
        return super().event(event)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._dragging = True
            self._last_drag_pos = event.position()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._dragging and self._last_drag_pos is not None:
            self._apply_drag(event.position())
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._dragging = False
            self._last_drag_pos = None
            self.setCursor(Qt.CursorShape.OpenHandCursor)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._view_yaw = 0.0
            self._view_pitch = 0.0
            self.update()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)

    def update_telemetry(self, data: Any):
        """Accept a parsed TelemetryData-like object."""
        now = time.monotonic()
        self._last_packet_at = now

        self._state_text = self._clean_text(getattr(data, "STATE", None), "Unknown")
        self._mission_time = self._clean_text(getattr(data, "MISSION_TIME", None), "00:00:00")
        self._packet_count = self._clean_text(getattr(data, "PACKET_COUNT", None), "0")

        self._update_orientation(data, now)
        self._update_accel(data, now)
        self._update_speed(data, now)
        self.update()

    def paintEvent(self, event):
        del event

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)

        rect = self.rect()
        painter.fillRect(rect, cosmetics.theme_qcolor("visualization", "background", "#f6f8fb"))

        self._draw_header(painter, rect)
        self._draw_reference_grid(painter, rect)
        body_points = self._draw_payload_body(painter, rect)
        self._draw_vectors(painter, rect)
        self._draw_gyro_cue(painter, rect, body_points["top_center"])
        self._draw_footer(painter, rect)

    def _update_orientation(self, data: Any, now: float):
        gyro = (
            self._as_float(getattr(data, "GYRO_R", None)),
            self._as_float(getattr(data, "GYRO_P", None)),
            self._as_float(getattr(data, "GYRO_Y", None)),
        )
        if any(value is None for value in gyro):
            return

        if self._last_gyro_time is None:
            self._last_gyro_time = now
            return

        dt = max(0.0, min(now - self._last_gyro_time, 1.0))
        self._last_gyro_time = now

        roll_rate, pitch_rate, yaw_rate = gyro
        self._roll += math.radians(roll_rate) * dt
        self._pitch += math.radians(pitch_rate) * dt
        self._yaw += math.radians(yaw_rate) * dt

        # Keep the first-stage estimate visually stable. The source is gyro
        # rate only, so unrestricted integration would drift quickly.
        self._roll = self._clamp(self._roll * 0.995, math.radians(-55), math.radians(55))
        self._pitch = self._clamp(self._pitch * 0.995, math.radians(-55), math.radians(55))
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

    def _update_accel(self, data: Any, now: float):
        values = (
            self._as_float(getattr(data, "ACCEL_R", None)),
            self._as_float(getattr(data, "ACCEL_P", None)),
            self._as_float(getattr(data, "ACCEL_Y", None)),
        )
        if any(value is None for value in values):
            return

        magnitude = self._magnitude(values)
        smoothed = self._smooth_vector(self._accel.vector if self._accel else None, values, 0.35)
        self._accel = TimedVector(smoothed, magnitude, now)

    def _update_speed(self, data: Any, now: float):
        lat = self._as_float(getattr(data, "GPS_LATITUDE", None))
        lon = self._as_float(getattr(data, "GPS_LONGITUDE", None))
        alt = self._as_float(getattr(data, "GPS_ALTITUDE", None))
        if lat is None or lon is None:
            return

        alt = alt if alt is not None else 0.0
        sample = (lat, lon, alt, now)

        if self._last_gps_sample is None:
            self._last_gps_sample = sample
            return

        last_lat, last_lon, last_alt, last_time = self._last_gps_sample
        dt = now - last_time
        self._last_gps_sample = sample
        if dt <= 0:
            return

        lat_mid = math.radians((lat + last_lat) * 0.5)
        north_m = (lat - last_lat) * 111_320.0
        east_m = (lon - last_lon) * 111_320.0 * math.cos(lat_mid)
        up_m = alt - last_alt
        velocity = (east_m / dt, north_m / dt, up_m / dt)
        magnitude = self._magnitude(velocity)
        smoothed = self._smooth_vector(self._speed.vector if self._speed else None, velocity, 0.4)
        self._speed = TimedVector(smoothed, magnitude, now)

    def _draw_header(self, painter: QPainter, rect):
        painter.setPen(cosmetics.theme_qcolor("visualization", "title", "#111827"))
        painter.setFont(QFont("Consolas", 18, QFont.Weight.DemiBold))
        painter.drawText(QPointF(24, 36), "Payload Orientation")

        painter.setFont(QFont("Consolas", 10))
        painter.setPen(cosmetics.theme_qcolor("visualization", "subtitle", "#4b5563"))
        status = f"State: {self._state_text}   Time: {self._mission_time}   Packet: {self._packet_count}"
        painter.drawText(QPointF(24, 58), status)

        if self._last_packet_at is None:
            painter.setPen(cosmetics.theme_qcolor("visualization", "muted", "#9ca3af"))
            painter.drawText(QPointF(24, 80), "Waiting for telemetry")

        legend_x = rect.width() - 210
        self._draw_legend_item(painter, legend_x, 30, cosmetics.theme_qcolor("visualization", "accel", "#2563eb"), "Acceleration")
        self._draw_legend_item(painter, legend_x, 52, cosmetics.theme_qcolor("visualization", "speed", "#059669"), "Speed / direction")
        self._draw_legend_item(painter, legend_x, 74, cosmetics.theme_qcolor("visualization", "midline", "#6b7280"), "Payload midline")

    def _draw_reference_grid(self, painter: QPainter, rect):
        painter.save()
        painter.setPen(QPen(cosmetics.theme_qcolor("visualization", "grid", "#e5e7eb"), 1))

        y = rect.height() - 96
        painter.drawLine(QPointF(42, y), QPointF(rect.width() - 42, y))
        for i in range(7):
            x = 70 + i * ((rect.width() - 140) / 6)
            painter.drawLine(QPointF(x, y - 4), QPointF(x, y + 4))

        painter.restore()

    def _draw_payload_body(self, painter: QPainter, rect):
        front_z = 1.62
        rear_z = -1.62
        rotor_centers = [
            (-1.28, 0.08, 1.10),
            (1.28, 0.08, 1.10),
            (-1.28, 0.08, -1.10),
            (1.28, 0.08, -1.10),
        ]

        painter.save()

        self._draw_quadcopter_shadow(painter, rect, rotor_centers)
        self._draw_quadcopter_arms(painter, rect, rotor_centers)
        self._draw_quadcopter_rotors(painter, rect, rotor_centers)
        self._draw_quadcopter_body(painter, rect)
        self._draw_midline(painter, rect, front_z, rear_z)

        painter.restore()

        return {
            "top_center": self._project(self._rotate((0.0, 0.22, front_z)), rect),
            "center": self._project((0.0, 0.0, 0.0), rect),
        }

    def _draw_quadcopter_shadow(self, painter: QPainter, rect, rotor_centers: list[Vector3]):
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(cosmetics.theme_qcolor("visualization", "shadow", [17, 24, 39, 28])))

        shadow_rotors = [(x, -0.20, z) for x, _, z in rotor_centers]
        for center in shadow_rotors:
            painter.drawPolygon(self._rotor_disc(center, 0.44, rect))

        painter.setPen(QPen(cosmetics.theme_qcolor("visualization", "shadow", [17, 24, 39, 34]), 8))
        for center in shadow_rotors:
            painter.drawLine(self._point((0.0, -0.20, 0.0), rect), self._point(center, rect))

        body_shadow = [
            (-0.55, -0.20, 0.36),
            (0.55, -0.20, 0.36),
            (0.62, -0.20, -0.28),
            (0.0, -0.20, -0.58),
            (-0.62, -0.20, -0.28),
        ]
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawPolygon(self._polygon(body_shadow, rect))

    def _draw_quadcopter_arms(self, painter: QPainter, rect, rotor_centers: list[Vector3]):
        painter.setPen(QPen(cosmetics.theme_qcolor("visualization", "aircraft_detail", "#64748b"), 9, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        for center in rotor_centers:
            painter.drawLine(self._point((0.0, 0.03, 0.0), rect), self._point(center, rect))

        painter.setPen(QPen(cosmetics.theme_qcolor("visualization", "aircraft_outline", "#111827"), 3, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        for center in rotor_centers:
            painter.drawLine(self._point((0.0, 0.10, 0.0), rect), self._point(center, rect))

    def _draw_quadcopter_rotors(self, painter: QPainter, rect, rotor_centers: list[Vector3]):
        for i, center in enumerate(rotor_centers):
            fill_key = "front_rotor_fill" if center[2] > 0 else "rear_rotor_fill"
            fill_fallback = "#dbeafe" if center[2] > 0 else "#e2e8f0"
            self._draw_model_polygon(
                painter,
                rect,
                self._rotor_points(center, 0.43),
                cosmetics.theme_color("visualization", fill_key, fill_fallback),
                cosmetics.theme_color("visualization", "aircraft_outline", "#111827"),
                2,
            )

            blade_angle = math.radians(25 + i * 45)
            blade_vector = (math.cos(blade_angle) * 0.35, 0.0, math.sin(blade_angle) * 0.35)
            blade_start = self._add(center, self._scale(blade_vector, -1.0))
            blade_end = self._add(center, blade_vector)
            painter.setPen(QPen(cosmetics.theme_qcolor("visualization", "propeller_blade", "#334155"), 4, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            painter.drawLine(self._point(blade_start, rect), self._point(blade_end, rect))

            hub_points = self._rotor_points((center[0], center[1] + 0.04, center[2]), 0.12, samples=18)
            self._draw_model_polygon(
                painter,
                rect,
                hub_points,
                cosmetics.theme_color("visualization", "rotor_hub_fill", "#ffffff"),
                cosmetics.theme_color("visualization", "aircraft_outline", "#111827"),
                1,
            )

    def _draw_quadcopter_body(self, painter: QPainter, rect):
        body = [
            (-0.50, 0.16, 0.38),
            (0.50, 0.16, 0.38),
            (0.58, 0.16, -0.24),
            (0.0, 0.16, -0.58),
            (-0.58, 0.16, -0.24),
        ]
        self._draw_model_polygon(
            painter,
            rect,
            body,
            cosmetics.theme_color("visualization", "fuselage_fill", "#ffffff"),
            cosmetics.theme_color("visualization", "aircraft_outline", "#111827"),
            2,
        )

        front_marker = [
            (-0.17, 0.24, 0.47),
            (0.17, 0.24, 0.47),
            (0.0, 0.25, 0.72),
        ]
        self._draw_model_polygon(
            painter,
            rect,
            front_marker,
            cosmetics.theme_color("visualization", "front_marker_fill", "#ef4444"),
            cosmetics.theme_color("visualization", "aircraft_outline", "#111827"),
            1,
        )

        battery = [
            (-0.28, 0.25, 0.10),
            (0.28, 0.25, 0.10),
            (0.28, 0.25, -0.28),
            (-0.28, 0.25, -0.28),
        ]
        self._draw_model_polygon(
            painter,
            rect,
            battery,
            cosmetics.theme_color("visualization", "cockpit_fill", "#bcd7f3"),
            cosmetics.theme_color("visualization", "cockpit_outline", "#2563eb"),
            1,
        )

    def _draw_midline(self, painter: QPainter, rect, front_z: float, rear_z: float):
        start = self._project(self._rotate((0.0, 0.30, rear_z - 0.10)), rect)
        end = self._project(self._rotate((0.0, 0.30, front_z + 0.10)), rect)
        painter.setPen(QPen(cosmetics.theme_qcolor("visualization", "midline", "#6b7280"), 2, Qt.PenStyle.DashLine))
        painter.drawLine(start, end)

    def _draw_model_polygon(
        self,
        painter: QPainter,
        rect,
        points: list[Vector3],
        fill_color: str,
        outline_color: str,
        outline_width: int,
    ):
        painter.setBrush(QBrush(QColor(fill_color)))
        painter.setPen(QPen(QColor(outline_color), outline_width))
        painter.drawPolygon(self._polygon(points, rect))

    def _rotor_disc(self, center: Vector3, radius: float, rect) -> QPolygonF:
        return QPolygonF([self._point(point, rect) for point in self._rotor_points(center, radius)])

    def _rotor_points(self, center: Vector3, radius: float, samples: int = 36) -> list[Vector3]:
        cx, cy, cz = center
        return [
            (
                cx + math.cos((2 * math.pi * i) / samples) * radius,
                cy,
                cz + math.sin((2 * math.pi * i) / samples) * radius,
            )
            for i in range(samples)
        ]

    def _polygon(self, points: list[Vector3], rect) -> QPolygonF:
        return QPolygonF([self._point(point, rect) for point in points])

    def _point(self, point: Vector3, rect) -> QPointF:
        return self._project(self._rotate(point), rect)

    def _draw_vectors(self, painter: QPainter, rect):
        origin = (0.0, 0.0, 0.0)
        now = time.monotonic()

        if self._accel is not None:
            accel_world = self._rotate(self._accel.vector)
            self._draw_vector(
                painter,
                rect,
                origin,
                accel_world,
                self._accel,
                cosmetics.theme_qcolor("visualization", "accel", "#2563eb"),
                "Accel",
                "m/s^2",
                cap=18.0,
                max_units=1.55,
                now=now,
            )

        if self._speed is not None:
            self._draw_vector(
                painter,
                rect,
                origin,
                self._speed.vector,
                self._speed,
                cosmetics.theme_qcolor("visualization", "speed", "#059669"),
                "Speed",
                "m/s",
                cap=35.0,
                max_units=1.75,
                now=now,
            )

    def _draw_vector(
        self,
        painter: QPainter,
        rect,
        origin: Vector3,
        vector: Vector3,
        reading: TimedVector,
        color: QColor,
        label: str,
        unit: str,
        cap: float,
        max_units: float,
        now: float,
    ):
        age = now - reading.updated_at
        if age > 5.0:
            return

        magnitude = self._magnitude(vector)
        if magnitude < 0.05:
            return

        alpha = 255 if age <= 2.0 else int(255 * max(0.0, 1.0 - ((age - 2.0) / 3.0)))
        draw_color = QColor(color)
        draw_color.setAlpha(alpha)

        scale = max_units * min(1.0, reading.magnitude / cap)
        direction = self._scale(vector, scale / magnitude)
        end = self._add(origin, direction)

        start_point = self._project(origin, rect)
        end_point = self._project(end, rect)

        painter.save()
        painter.setPen(QPen(draw_color, 4, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        painter.drawLine(start_point, end_point)
        self._draw_arrow_head(painter, start_point, end_point, draw_color)

        painter.setFont(QFont("Consolas", 10, QFont.Weight.DemiBold))
        painter.setPen(draw_color)
        painter.drawText(end_point + QPointF(8, -8), f"{label} {reading.magnitude:.1f} {unit}")
        painter.restore()

    def _draw_gyro_cue(self, painter: QPainter, rect, top_center: QPointF):
        del rect

        painter.save()
        painter.setPen(QPen(cosmetics.theme_qcolor("visualization", "title", "#111827"), 2))
        cue_rect = QRectF(top_center.x() - 46, top_center.y() - 58, 92, 44)
        painter.drawArc(cue_rect, 20 * 16, 285 * 16)

        angle = math.radians(20)
        arrow_tip = QPointF(
            cue_rect.center().x() + math.cos(angle) * cue_rect.width() * 0.5,
            cue_rect.center().y() - math.sin(angle) * cue_rect.height() * 0.5,
        )
        painter.setBrush(QBrush(cosmetics.theme_qcolor("visualization", "title", "#111827")))
        painter.drawPolygon(
            QPolygonF(
                [
                    arrow_tip,
                    arrow_tip + QPointF(-14, -2),
                    arrow_tip + QPointF(-8, 10),
                ]
            )
        )

        painter.setFont(QFont("Consolas", 10))
        painter.setPen(cosmetics.theme_qcolor("visualization", "label", "#374151"))
        painter.drawText(top_center + QPointF(56, -38), "Gyro est.")
        painter.restore()

    def _draw_footer(self, painter: QPainter, rect):
        painter.save()
        painter.setFont(QFont("Consolas", 9))
        painter.setPen(cosmetics.theme_qcolor("visualization", "midline", "#6b7280"))
        text = "Orientation is estimated from gyro rate; speed is derived from GPS deltas."
        painter.drawText(QPointF(24, rect.height() - 24), text)
        painter.restore()

    def _draw_legend_item(self, painter: QPainter, x: float, y: float, color: QColor, text: str):
        painter.save()
        painter.setPen(QPen(color, 4, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        painter.drawLine(QPointF(x, y), QPointF(x + 24, y))
        painter.setFont(QFont("Consolas", 10))
        painter.setPen(cosmetics.theme_qcolor("visualization", "label", "#374151"))
        painter.drawText(QPointF(x + 34, y + 5), text)
        painter.restore()

    def _draw_arrow_head(self, painter: QPainter, start: QPointF, end: QPointF, color: QColor):
        dx = end.x() - start.x()
        dy = end.y() - start.y()
        length = math.hypot(dx, dy)
        if length < 4.0:
            return

        ux = dx / length
        uy = dy / length
        size = 14.0
        left = QPointF(
            end.x() - ux * size - uy * size * 0.45,
            end.y() - uy * size + ux * size * 0.45,
        )
        right = QPointF(
            end.x() - ux * size + uy * size * 0.45,
            end.y() - uy * size - ux * size * 0.45,
        )

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(color))
        painter.drawPolygon(QPolygonF([end, left, right]))

    def _project(self, point: Vector3, rect) -> QPointF:
        point = self._rotate_view(point)
        x, y, z = point
        scale = min(rect.width(), rect.height()) * 0.15
        center = QPointF(rect.width() * 0.48, rect.height() * 0.56)
        return QPointF(
            center.x() + (x + y * 0.38) * scale,
            center.y() - (z - y * 0.20) * scale,
        )

    def _rotate(self, point: Vector3) -> Vector3:
        x, y, z = point

        cr = math.cos(self._roll)
        sr = math.sin(self._roll)
        y, z = y * cr - z * sr, y * sr + z * cr

        cp = math.cos(self._pitch)
        sp = math.sin(self._pitch)
        x, z = x * cp + z * sp, -x * sp + z * cp

        cy = math.cos(self._yaw)
        sy = math.sin(self._yaw)
        x, y = x * cy - y * sy, x * sy + y * cy

        return (x, y, z)

    def _rotate_view(self, point: Vector3) -> Vector3:
        x, y, z = point

        cy = math.cos(self._view_yaw)
        sy = math.sin(self._view_yaw)
        x, y = x * cy - y * sy, x * sy + y * cy

        cp = math.cos(self._view_pitch)
        sp = math.sin(self._view_pitch)
        y, z = y * cp - z * sp, y * sp + z * cp

        return (x, y, z)

    def _handle_touch_event(self, event):
        position = self._touch_position(event)
        if position is None:
            return False

        if event.type() == QEvent.Type.TouchBegin:
            self._dragging = True
            self._last_drag_pos = position
        elif event.type() == QEvent.Type.TouchUpdate and self._last_drag_pos is not None:
            self._apply_drag(position)
        else:
            self._dragging = False
            self._last_drag_pos = None

        event.accept()
        return True

    def _touch_position(self, event):
        points = event.points()
        if not points:
            return None
        return points[0].position()

    def _apply_drag(self, position: QPointF):
        delta = position - self._last_drag_pos
        self._last_drag_pos = position

        self._view_yaw += delta.x() * self._drag_sensitivity
        self._view_pitch = self._clamp(
            self._view_pitch - delta.y() * self._drag_sensitivity,
            math.radians(-65.0),
            math.radians(65.0),
        )
        self._view_yaw = math.atan2(math.sin(self._view_yaw), math.cos(self._view_yaw))
        self.update()

    @staticmethod
    def _smooth_vector(previous: Vector3 | None, current: Vector3, factor: float) -> Vector3:
        if previous is None:
            return current
        return tuple(previous[i] * (1.0 - factor) + current[i] * factor for i in range(3))

    @staticmethod
    def _magnitude(vector: Vector3) -> float:
        return math.sqrt(sum(component * component for component in vector))

    @staticmethod
    def _scale(vector: Vector3, factor: float) -> Vector3:
        return tuple(component * factor for component in vector)

    @staticmethod
    def _add(left: Vector3, right: Vector3) -> Vector3:
        return tuple(left[i] + right[i] for i in range(3))

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _as_float(value):
        if value is None:
            return None
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(parsed):
            return None
        return parsed

    @staticmethod
    def _clean_text(value, fallback: str) -> str:
        if value is None:
            return fallback
        text = str(value).strip()
        return text if text else fallback


class PayloadVisualizationWindow(QMainWindow):
    """Top-level window for the stage-one isolated visualization."""

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("Payload Orientation")
        icon_path = os.path.join(os.path.dirname(__file__), "..", "media", "icon.png")
        self.setWindowIcon(QIcon(icon_path))

        self.canvas = PayloadVisualizationCanvas(self)
        self.setCentralWidget(self.canvas)
        self.resize(860, 640)

    def update_telemetry(self, data: Any):
        self.canvas.update_telemetry(data)
