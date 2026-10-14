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
    manual_pick_requested = Signal()
    servo_debug_requested = Signal()        # todos a 90° → 40°
    servo_grasp_test_requested = Signal()   # prueba posición de agarre actual
    servo_drop_test_requested = Signal()    # prueba posición de soltado actual
    web_stream_toggled = Signal(bool)       # switch de transmisión WebSocket

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        tracking_group = QGroupBox("Control de tracking físico")
        tracking_layout = QVBoxLayout(tracking_group)

        self._tracking_button = QPushButton("BRAZO ROBOT: APAGADO")
        self._tracking_button.setCheckable(True)
        self._tracking_button.setChecked(False)
        self._tracking_button.setMinimumHeight(48)
        self._update_tracking_style(False)
        self._tracking_button.toggled.connect(self._on_tracking_toggled)
        tracking_layout.addWidget(self._tracking_button)

        self._manual_pick_button = QPushButton("Iniciar secuencia de colocacion")
        self._manual_pick_button.setMinimumHeight(38)
        self._manual_pick_button.setStyleSheet(
            "background-color: #2b8a3e; color: white; font-weight: bold; border-radius: 4px;"
        )
        self._manual_pick_button.clicked.connect(self.manual_pick_requested)
        tracking_layout.addWidget(self._manual_pick_button)

        self._debug_button = QPushButton("🔧 Debug Servos (90° → -50°)")
        self._debug_button.setMinimumHeight(34)
        self._debug_button.setStyleSheet(
            "background-color: #e67700; color: white; font-weight: bold; border-radius: 4px;"
        )
        self._debug_button.clicked.connect(self.servo_debug_requested)
        tracking_layout.addWidget(self._debug_button)

        self._grasp_test_button = QPushButton("⤵ Probar posición GRASP")
        self._grasp_test_button.setMinimumHeight(30)
        self._grasp_test_button.setStyleSheet(
            "background-color: #1864ab; color: white; font-weight: bold; border-radius: 4px;"
        )
        self._grasp_test_button.clicked.connect(self.servo_grasp_test_requested)
        tracking_layout.addWidget(self._grasp_test_button)

        self._drop_test_button = QPushButton("⤴ Probar posición DROP")
        self._drop_test_button.setMinimumHeight(30)
        self._drop_test_button.setStyleSheet(
            "background-color: #5c3bc0; color: white; font-weight: bold; border-radius: 4px;"
        )
        self._drop_test_button.clicked.connect(self.servo_drop_test_requested)
        tracking_layout.addWidget(self._drop_test_button)

        self._tracking_status = QLabel("El brazo NO se moverá automáticamente.")
        self._tracking_status.setWordWrap(True)
        tracking_layout.addWidget(self._tracking_status)

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

        # Transmisión Web (WebSocket hacia Django en Render)
        web_group = QGroupBox("Transmisión Web (Dashboard Django)")
        web_layout = QVBoxLayout(web_group)

        self._web_stream_button = QPushButton("📡 STREAMING WEB: APAGADO")
        self._web_stream_button.setCheckable(True)
        self._web_stream_button.setChecked(False)
        self._web_stream_button.setMinimumHeight(40)
        self._update_web_stream_style(False)
        self._web_stream_button.toggled.connect(self._on_web_stream_toggled)
        web_layout.addWidget(self._web_stream_button)

        self._web_stream_status = QLabel("Desconectado (wss://app-brazo-robot.onrender.com)")
        self._web_stream_status.setWordWrap(True)
        web_layout.addWidget(self._web_stream_status)

        layout = QVBoxLayout(self)
        layout.addWidget(tracking_group)
        layout.addWidget(web_group)
        layout.addWidget(conf_group)
        layout.addWidget(info_group)
        layout.addStretch()

    def set_web_stream_status(self, text: str) -> None:
        self._web_stream_status.setText(text)

    def set_web_streaming(self, active: bool) -> None:
        if self._web_stream_button.isChecked() != active:
            self._web_stream_button.setChecked(active)

    def _on_web_stream_toggled(self, checked: bool) -> None:
        self._update_web_stream_style(checked)
        self.web_stream_toggled.emit(checked)

    def _update_web_stream_style(self, active: bool) -> None:
        if active:
            self._web_stream_button.setText("📡 TRANSMITIENDO EN VIVO (CAM_01)")
            self._web_stream_button.setStyleSheet(
                "QPushButton { background-color: #0c8599; color: white; "
                "font-size: 13px; font-weight: bold; border-radius: 6px; }"
            )
        else:
            self._web_stream_button.setText("📡 STREAMING WEB: APAGADO")
            self._web_stream_button.setStyleSheet(
                "QPushButton { background-color: #495057; color: white; "
                "font-size: 13px; font-weight: bold; border-radius: 6px; }"
            )

    @property
    def arm_tracking_enabled(self) -> bool:
        return self._tracking_button.isChecked()

    def set_detection_info(self, text: str) -> None:
        self._info_output.setPlainText(text)

    def _on_tracking_toggled(self, checked: bool) -> None:
        self._update_tracking_style(checked)
        if checked:
            self._tracking_button.setText("BRAZO ROBOT: ENCENDIDO")
            self._tracking_status.setText(
                "El brazo se mueve automáticamente siguiendo las detecciones."
            )
        else:
            self._tracking_button.setText("BRAZO ROBOT: APAGADO")
            self._tracking_status.setText("El brazo no se mueve automáticamente.")
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
