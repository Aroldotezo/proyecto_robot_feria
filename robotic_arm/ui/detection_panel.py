"""Panel de interfaz para la detección YOLO y el switch de seguridad ARM TRACKING."""

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)


class DetectionPanel(QWidget):
    """Panel de control para detección YOLO y tracking del brazo."""

    arm_tracking_toggled = Signal(bool)
    confidence_changed = Signal(float)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        # ---- ARM TRACKING switch ----
        tracking_group = QGroupBox("Control de tracking físico")
        tracking_layout = QVBoxLayout(tracking_group)

        self._tracking_button = QPushButton("ARM TRACKING: OFF")
        self._tracking_button.setCheckable(True)
        self._tracking_button.setChecked(False)
        self._tracking_button.setMinimumHeight(48)
        self._update_tracking_style(False)
        self._tracking_button.toggled.connect(self._on_tracking_toggled)
        tracking_layout.addWidget(self._tracking_button)

        self._tracking_status = QLabel("El brazo NO se moverá automáticamente.")
        self._tracking_status.setWordWrap(True)
        tracking_layout.addWidget(self._tracking_status)

        # ---- Confidence slider ----
        conf_group = QGroupBox("Confianza mínima de detección")
        conf_layout = QHBoxLayout(conf_group)

        self._conf_slider = QSlider(Qt.Orientation.Horizontal)
        self._conf_slider.setRange(10, 95)
        self._conf_slider.setValue(50)
        self._conf_slider.setTickInterval(5)
        self._conf_label = QLabel("50%")
        self._conf_label.setMinimumWidth(40)
        conf_layout.addWidget(self._conf_slider, stretch=1)
        conf_layout.addWidget(self._conf_label)
        self._conf_slider.valueChanged.connect(self._on_conf_changed)

        # ---- Detection info ----
        info_group = QGroupBox("Detección activa")
        info_layout = QVBoxLayout(info_group)
        self._info_output = QPlainTextEdit()
        self._info_output.setReadOnly(True)
        self._info_output.setMinimumHeight(200)
        self._info_output.setPlaceholderText("Esperando detecciones YOLO...")
        font = QFont("monospace")
        font.setStyleHint(QFont.StyleHint.Monospace)
        self._info_output.setFont(font)
        info_layout.addWidget(self._info_output)

        # ---- Layout ----
        layout = QVBoxLayout(self)
        layout.addWidget(tracking_group)
        layout.addWidget(conf_group)
        layout.addWidget(info_group)
        layout.addStretch()

    @property
    def arm_tracking_enabled(self) -> bool:
        return self._tracking_button.isChecked()

    def set_detection_info(self, text: str) -> None:
        self._info_output.setPlainText(text)

    def _on_tracking_toggled(self, checked: bool) -> None:
        self._update_tracking_style(checked)
        if checked:
            self._tracking_button.setText("ARM TRACKING: ON")
            self._tracking_status.setText(
                "⚠ El brazo SE MOVERÁ automáticamente siguiendo las detecciones."
            )
        else:
            self._tracking_button.setText("ARM TRACKING: OFF")
            self._tracking_status.setText("El brazo NO se moverá automáticamente.")
        self.arm_tracking_toggled.emit(checked)

    def _on_conf_changed(self, value: int) -> None:
        self._conf_label.setText(f"{value}%")
        self.confidence_changed.emit(value / 100.0)

    def _update_tracking_style(self, active: bool) -> None:
        if active:
            self._tracking_button.setStyleSheet(
                "QPushButton { background-color: #e03131; color: white; "
                "font-size: 16px; font-weight: bold; border-radius: 6px; }"
            )
        else:
            self._tracking_button.setStyleSheet(
                "QPushButton { background-color: #2b8a3e; color: white; "
                "font-size: 16px; font-weight: bold; border-radius: 6px; }"
            )
