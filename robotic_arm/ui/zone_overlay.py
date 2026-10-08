from collections.abc import Sequence

from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPen

from robotic_arm.domain.zone import Zone


class ZoneOverlay:
    """Dibuja la zona de evaluacion y la zona de destino."""

    def __init__(self) -> None:
        self._zones: tuple[Zone, ...] = ()
        self._selected_zone_id: str | None = None

    def set_zones(self, zones: Sequence[Zone]) -> None:
        self._zones = tuple(zones)

    def set_selected_zone(self, zone_id: str | None) -> None:
        self._selected_zone_id = zone_id

    def paint(self, painter: QPainter, width: int, height: int) -> None:
        for zone in self._zones:
            region = zone.region
            rect = QRectF(region.x * width, region.y * height, region.width * width, region.height * height)
            color = QColor(zone.color)
            fill = QColor(color)
            is_selected = (zone.id == self._selected_zone_id)
            fill.setAlpha(115 if is_selected else 65)
            pen_width = 3 if is_selected else 2
            painter.setPen(QPen(color, pen_width))
            painter.setBrush(fill)
            painter.drawRoundedRect(rect, 6, 6)
            self._draw_label(painter, rect, zone.label, is_selected)

    @staticmethod
    def _draw_label(painter: QPainter, rect: QRectF, text: str, is_selected: bool = False) -> None:
        metrics = painter.fontMetrics()
        label_text = f"★ {text}" if is_selected else text
        chip = QRectF(rect.left() + 4, rect.top() + 4, metrics.horizontalAdvance(label_text) + 12, metrics.height() + 4)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(0, 0, 0, 180 if is_selected else 150))
        painter.drawRoundedRect(chip, 4, 4)
        painter.setPen(QColor("#FFE066" if is_selected else "white"))
        painter.drawText(chip, Qt.AlignmentFlag.AlignCenter, label_text)