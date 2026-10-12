"""Widget contenedor que muestra dos cámaras en paralelo:
1. Cámara Cenital (Superior): Detección X, Y y rotación con YOLO
2. Cámara Lateral (Izquierda): Monitoreo y lectura de elevación Z
"""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from robotic_arm.ui.video_view import VideoView


class DualVideoWidget(QWidget):
    """Contenedor de doble cámara (Cenital + Lateral) con etiquetas de estado y controles limpios."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        splitter = QSplitter(Qt.Orientation.Horizontal, self)

        # ---- Panel Cámara Cenital (Principal X, Y) ----
        top_container = QFrame()
        top_container.setFrameShape(QFrame.Shape.StyledPanel)
        top_layout = QVBoxLayout(top_container)
        top_layout.setContentsMargins(4, 4, 4, 4)

        top_header = QHBoxLayout()
        top_title = QLabel("CAMARA 1: CENITAL (Plano X, Y)")
        top_title.setStyleSheet("font-weight: bold; color: #4dabf7; font-size: 13px;")
        self._top_status = QLabel("Desconectada")
        self._top_status.setStyleSheet("color: #888888; font-size: 11px;")
        top_header.addWidget(top_title)
        top_header.addStretch()
        top_header.addWidget(self._top_status)
        top_layout.addLayout(top_header)

        self._top_view = VideoView(placeholder="Cámara cenital desconectada")
        top_layout.addWidget(self._top_view, stretch=1)

        # ---- Panel Cámara Lateral (Altura Z) ----
        side_container = QFrame()
        side_container.setFrameShape(QFrame.Shape.StyledPanel)
        side_layout = QVBoxLayout(side_container)
        side_layout.setContentsMargins(4, 4, 4, 4)

        side_header = QHBoxLayout()
        side_title = QLabel("CAMARA 2: LATERAL (Altura Z)")
        side_title.setStyleSheet("font-weight: bold; color: #51cf66; font-size: 13px;")
        self._side_status = QLabel("Desconectada")
        self._side_status.setStyleSheet("color: #888888; font-size: 11px;")
        side_header.addWidget(side_title)
        side_header.addStretch()
        side_header.addWidget(self._side_status)
        side_layout.addLayout(side_header)

        self._side_view = VideoView(placeholder="Cámara lateral desconectada")
        side_layout.addWidget(self._side_view, stretch=1)

        splitter.addWidget(top_container)
        splitter.addWidget(side_container)
        splitter.setSizes([640, 640])

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.addWidget(splitter)

    @property
    def top_view(self) -> VideoView:
        return self._top_view

    @property
    def side_view(self) -> VideoView:
        return self._side_view

    def set_top_status(self, text: str) -> None:
        self._top_status.setText(text)

    def set_side_status(self, text: str) -> None:
        self._side_status.setText(text)
