from collections.abc import Sequence

from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPen

from robotic_arm.domain.zone import Zone


class ZoneOverlay:
    """Dibuja la zona de evaluacion y la zona de destino."""

    def __init__(self) -> None:
        self._zones: tuple[Zone, ...] = ()

    def set_zones(self, zones: Sequence[Zone]) -> None:
        self._zones = tuple(zones)

    def paint(self, painter: QPainter, width: int, height: int) -> None:
        for zone in self._zones:
            region = zone.region
            rect = QRectF(region.x * width, region.y * height, region.width * width, region.height * height)
            color = QColor(zone.color)
            fill = QColor(color)
            fill.setAlpha(70)
            painter.setPen(QPen(color, 2))
            painter.setBrush(fill)
            painter.drawRoundedRect(rect, 6, 6)
            self._draw_label(painter, rect, zone.label)

    @staticmethod
    def _draw_label(painter: QPainter, rect: QRectF, text: str) -> None:
        metrics = painter.fontMetrics()
        chip = QRectF(rect.left() + 4, rect.top() + 4, metrics.horizontalAdvance(text) + 12, metrics.height() + 4)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(0, 0, 0, 150))
        painter.drawRoundedRect(chip, 4, 4)
        painter.setPen(QColor("white"))
        painter.drawText(chip, Qt.AlignmentFlag.AlignCenter, text)