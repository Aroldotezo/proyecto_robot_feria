import numpy as np
from numpy.typing import NDArray
from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QLabel, QSizePolicy, QWidget


class VideoView(QLabel):
    """Muestra la vista de la camara escalada al contenedor visual."""

    def __init__(self, placeholder: str = "No camera selected", parent: QWidget | None = None) -> None:
        super().__init__(placeholder, parent)
        self._placeholder = placeholder
        self._pixmap: QPixmap | None = None
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.setStyleSheet("background-color: #1e1e1e; color: #aaaaaa;")

    def show_frame(self, frame: NDArray[np.uint8]) -> None:
        frame = np.ascontiguousarray(frame)
        height, width = frame.shape[:2]
        image = QImage(frame.data, width, height, frame.strides[0], QImage.Format.Format_BGR888)
        self._pixmap = QPixmap.fromImage(image)
        self._render()

    def clear_frame(self) -> None:
        self._pixmap = None
        self.setText(self._placeholder)

    def resizeEvent(self, event) -> None: 
        super().resizeEvent(event)
        self._render()

    def _render(self) -> None:
        if self._pixmap is None:
            return
        self.setPixmap(
            self._pixmap.scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.FastTransformation)
        )