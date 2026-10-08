"""Overlay visual para detecciones de YOLO: bounding boxes, clase, confianza, centro y coordenadas."""

from dataclasses import dataclass

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen

from robotic_arm.domain.arm_space import ArmPoint
from robotic_arm.domain.camera.detection import Detection
from robotic_arm.domain.geometry import NormalizedPoint


@dataclass(frozen=True, slots=True)
class DetectionInfo:
    """All computed information about a single detection for display and tracking."""

    detection: Detection
    normalized_center: NormalizedPoint
    arm_point: ArmPoint | None
    destination_label: str | None
    category_id: str | None
    is_tracked: bool = False


class DetectionOverlay:
    """Draws YOLO detections over the displayed frame, following the Overlay protocol."""

    def __init__(self) -> None:
        self._detections: list[DetectionInfo] = []
        self._frame_width: int = 0
        self._frame_height: int = 0

    def set_detections(
        self,
        detections: list[DetectionInfo],
        frame_width: int,
        frame_height: int,
    ) -> None:
        self._detections = detections
        self._frame_width = frame_width
        self._frame_height = frame_height

    def clear(self) -> None:
        self._detections = []

    def paint(self, painter: QPainter, width: int, height: int) -> None:
        if not self._detections or self._frame_width <= 0 or self._frame_height <= 0:
            return

        sx = width / self._frame_width
        sy = height / self._frame_height

        for info in self._detections:
            det = info.detection
            color = _color_for_class(det.clase.value)

            # Bounding box
            rect = QRectF(det.x * sx, det.y * sy, det.width * sx, det.height * sy)
            pen_width = 3 if info.is_tracked else 2
            painter.setPen(QPen(color, pen_width))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawRect(rect)

            # Center cross
            cx, cy = det.center
            center_screen = QPointF(cx * sx, cy * sy)
            arm_size = 8
            painter.setPen(QPen(QColor("#FFFFFF"), 2))
            painter.drawLine(
                QPointF(center_screen.x() - arm_size, center_screen.y()),
                QPointF(center_screen.x() + arm_size, center_screen.y()),
            )
            painter.drawLine(
                QPointF(center_screen.x(), center_screen.y() - arm_size),
                QPointF(center_screen.x(), center_screen.y() + arm_size),
            )

            # Center dot
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(color)
            painter.drawEllipse(center_screen, 4, 4)

            # Label chip
            self._draw_label(painter, rect, info, color)

    def _draw_label(
        self,
        painter: QPainter,
        rect: QRectF,
        info: DetectionInfo,
        color: QColor,
    ) -> None:
        det = info.detection
        lines: list[str] = []

        # Line 1: class + confidence
        class_label = det.clase.value.upper()
        conf = f"{det.reliability:.0%}"
        tracked_marker = " ★" if info.is_tracked else ""
        lines.append(f"{class_label} {conf}{tracked_marker}")

        # Line 2: pixel center + normalized
        cx, cy = det.center
        nc = info.normalized_center
        lines.append(f"px({cx},{cy}) n({nc.x:.3f},{nc.y:.3f})")

        # Line 3: ArmPoint
        if info.arm_point is not None:
            lines.append(f"Arm({info.arm_point.x:.1f}, {info.arm_point.y:.1f}) mm")

        # Line 4: destination
        if info.destination_label is not None:
            lines.append(f"→ {info.destination_label}")

        font = QFont()
        font.setPointSize(8)
        font.setBold(True)
        painter.setFont(font)
        metrics = painter.fontMetrics()
        line_height = metrics.height() + 2
        max_text_width = max(metrics.horizontalAdvance(line) for line in lines)
        chip_width = max_text_width + 12
        chip_height = line_height * len(lines) + 6

        chip_x = rect.left()
        chip_y = rect.top() - chip_height - 2
        if chip_y < 0:
            chip_y = rect.bottom() + 2

        chip_rect = QRectF(chip_x, chip_y, chip_width, chip_height)

        # Background
        bg = QColor(0, 0, 0, 200)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(bg)
        painter.drawRoundedRect(chip_rect, 4, 4)

        # Color bar on left
        bar = QRectF(chip_x, chip_y, 3, chip_height)
        painter.setBrush(color)
        painter.drawRect(bar)

        # Text
        painter.setPen(QColor("#FFFFFF"))
        for i, line in enumerate(lines):
            painter.drawText(
                QPointF(chip_x + 8, chip_y + (i + 1) * line_height),
                line,
            )


_CLASS_COLORS = {
    "organico": QColor("#51CF66"),
    "inorganico": QColor("#FF8787"),
    "defectuoso": QColor("#4DABF7"),
}


def _color_for_class(class_value: str) -> QColor:
    return _CLASS_COLORS.get(class_value, QColor("#CCCCCC"))
