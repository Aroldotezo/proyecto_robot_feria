from typing import Protocol

import numpy as np
from numpy.typing import NDArray
from PySide6.QtCore import QEvent, QPointF, Qt, Signal
from PySide6.QtGui import QImage, QMouseEvent, QPainter, QPixmap
from PySide6.QtWidgets import QLabel, QSizePolicy, QWidget

from robotic_arm.domain.geometry import NormalizedPoint


class Overlay(Protocol):
    def paint(self, painter: QPainter, width: int, height: int) -> None:
        """Draws over the displayed frame; (width, height) is the size of the displayed image."""


class VideoView(QLabel):
    """Shows BGR frames scaled to the available space (keeping the aspect ratio), draws overlays
    on top and reports mouse positions as normalized image coordinates."""

    image_clicked = Signal(object)
    image_hovered = Signal(object) 

    def __init__(self, placeholder: str = "No camera selected", parent: QWidget | None = None) -> None:
        super().__init__(placeholder, parent)
        self._placeholder = placeholder
        self._pixmap: QPixmap | None = None
        self._shown_size: tuple[int, int] | None = None
        self._overlays: list[Overlay] = []
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.setMouseTracking(True)
        self.setStyleSheet("background-color: #1e1e1e; color: #aaaaaa;")

    def add_overlay(self, overlay: Overlay) -> None:
        self._overlays.append(overlay)

    def refresh(self) -> None:
        """Repinta el frame actual."""
        self._render()

    def show_frame(self, frame: NDArray[np.uint8]) -> None:
        frame = np.ascontiguousarray(frame)
        height, width = frame.shape[:2]
        image = QImage(frame.data, width, height, frame.strides[0], QImage.Format.Format_BGR888)
        self._pixmap = QPixmap.fromImage(image) 
        self._render()

    def clear_frame(self) -> None:
        self._pixmap = None
        self._shown_size = None
        self.setText(self._placeholder)

    def resizeEvent(self, event) -> None: 
        super().resizeEvent(event)
        self._render()

    def mousePressEvent(self, event: QMouseEvent) -> None: 
        point = self._to_normalized(event.position())
        if event.button() == Qt.MouseButton.LeftButton and point is not None:
            self.image_clicked.emit(point)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:  
        self.image_hovered.emit(self._to_normalized(event.position()))
        super().mouseMoveEvent(event)

    def leaveEvent(self, event: QEvent) -> None: 
        self.image_hovered.emit(None)
        super().leaveEvent(event)

    def _render(self) -> None:
        if self._pixmap is None:
            return
        scaled = self._pixmap.scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.FastTransformation)
        if self._overlays:
            painter = QPainter(scaled)
            painter.setRenderHint(QPainter.RenderHint.Antialiasing)
            for overlay in self._overlays:
                overlay.paint(painter, scaled.width(), scaled.height())
            painter.end()
        self._shown_size = (scaled.width(), scaled.height())
        self.setPixmap(scaled)

    def _to_normalized(self, position: QPointF) -> NormalizedPoint | None:
        """Posicion de widget -> posicion de imagen, tomando margenes y cajas de texto."""
        if self._shown_size is None:
            return None
        shown_width, shown_height = self._shown_size
        if shown_width <= 0 or shown_height <= 0:
            return None
        x = (position.x() - (self.width() - shown_width) / 2) / shown_width
        y = (position.y() - (self.height() - shown_height) / 2) / shown_height
        if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0):
            return None
        return NormalizedPoint(x, y)