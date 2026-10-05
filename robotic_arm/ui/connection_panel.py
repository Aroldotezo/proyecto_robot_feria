from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QComboBox, QHBoxLayout, QLabel, QPushButton, QSlider, QVBoxLayout, QWidget

from robotic_arm.domain.port_info import PortInfo
from robotic_arm.ports.arm_controller import MAX_SPEED, MIN_SPEED

INITIAL_SPEED = 30  # Velocidad del brazo para testeo


class ConnectionPanel(QWidget):
    refresh_requested = Signal()
    connect_requested = Signal(str)  # port device
    disconnect_requested = Signal()
    speed_changed = Signal(int)
    test_requested = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._connected = False
        self._busy = False

        self._ports = QComboBox()
        self._ports.setPlaceholderText("Ningun puerto encontrado")
        self._refresh_button = QPushButton("Refrescar")
        self._connect_button = QPushButton("Conectar")
        self._test_button = QPushButton("Mover a posicion inicial (test)")
        self._test_button.setToolTip("Mover todos los servo a 90 grados. Mantener el area limpia.")
        self._speed_slider = QSlider(Qt.Orientation.Horizontal)
        self._speed_slider.setRange(MIN_SPEED, MAX_SPEED)
        self._speed_slider.setValue(INITIAL_SPEED)
        self._speed_label = QLabel(str(INITIAL_SPEED))
        self._speed_label.setMinimumWidth(28)

        self._build_layout()
        self._wire_signals()
        self._update_enabled_state()

    @property
    def speed(self) -> int:
        return self._speed_slider.value()

    def set_ports(self, ports: list[PortInfo]) -> None:
        previous = self._ports.currentData()
        self._ports.clear()
        for port in ports:
            self._ports.addItem(port.label, port.device)
        index = self._ports.findData(previous)
        if index >= 0:
            self._ports.setCurrentIndex(index)
        self._update_enabled_state()

    def set_connected(self, connected: bool) -> None:
        self._connected = connected
        self._connect_button.setText("Desconectar" if connected else "Conectar")
        self._update_enabled_state()

    def set_busy(self, busy: bool) -> None:
        self._busy = busy
        self._update_enabled_state()

    # ------------------------------------------------------------------ internals

    def _build_layout(self) -> None:
        port_row = QHBoxLayout()
        port_row.addWidget(self._ports, stretch=1)
        port_row.addWidget(self._refresh_button)

        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("Velocidad"))
        speed_row.addWidget(self._speed_slider, stretch=1)
        speed_row.addWidget(self._speed_label)

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Puerto usb"))
        layout.addLayout(port_row)
        layout.addWidget(self._connect_button)
        layout.addLayout(speed_row)
        layout.addWidget(self._test_button)
        layout.addStretch()

    def _wire_signals(self) -> None:
        self._refresh_button.clicked.connect(self.refresh_requested)
        self._connect_button.clicked.connect(self._on_connect_clicked)
        self._test_button.clicked.connect(self.test_requested)
        self._speed_slider.valueChanged.connect(self._on_speed_value_changed)
        self._speed_slider.sliderReleased.connect(self._on_speed_released)

    def _on_connect_clicked(self) -> None:
        if self._connected:
            self.disconnect_requested.emit()
            return
        device = self._ports.currentData()
        if device:
            self.connect_requested.emit(device)

    def _on_speed_value_changed(self, value: int) -> None:
        self._speed_label.setText(str(value))
        if not self._speed_slider.isSliderDown():  # Deteccion de teclado en tiempo real
            self.speed_changed.emit(value)

    def _on_speed_released(self) -> None:
        self.speed_changed.emit(self._speed_slider.value())

    def _update_enabled_state(self) -> None:
        idle = not self._busy
        self._ports.setEnabled(idle and not self._connected)
        self._refresh_button.setEnabled(idle and not self._connected)
        self._connect_button.setEnabled(idle and (self._connected or self._ports.count() > 0))
        self._test_button.setEnabled(idle and self._connected)
        self._speed_slider.setEnabled(idle)