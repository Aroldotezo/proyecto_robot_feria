from collections.abc import Sequence

from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QColor, QPainter, QPen

from robotic_arm.domain.geometry import NormalizedPoint


class CalibrationOverlay:
    """Dibuja los puntos de calibracion (P1, P2...) y prueba los puntos en el video."""

    def __init__(self) -> None:
        self._reference_points: tuple[NormalizedPoint, ...] = ()
        self._test_point: NormalizedPoint | None = None

    def set_reference_points(self, points: Sequence[NormalizedPoint]) -> None:
        self._reference_points = tuple(points)

    def set_test_point(self, point: NormalizedPoint | None) -> None:
        self._test_point = point

    def paint(self, painter: QPainter, width: int, height: int) -> None:
        painter.setBrush(Qt.BrushStyle.NoBrush)

        painter.setPen(QPen(QColor("#00E5FF"), 2))
        for number, point in enumerate(self._reference_points, start=1):
            center = QPointF(point.x * width, point.y * height)
            painter.drawEllipse(center, 7, 7)
            self._draw_cross(painter, center, 12)
            painter.drawText(QPointF(center.x() + 10, center.y() - 10), f"P{number}")

        if self._test_point is not None:
            painter.setPen(QPen(QColor("#FF2D95"), 2))
            self._draw_cross(painter, QPointF(self._test_point.x * width, self._test_point.y * height), 16)

    @staticmethod
    def _draw_cross(painter: QPainter, center: QPointF, arm: float) -> None:
        painter.drawLine(QPointF(center.x() - arm, center.y()), QPointF(center.x() + arm, center.y()))
        painter.drawLine(QPointF(center.x(), center.y() - arm), QPointF(center.x(), center.y() + arm))