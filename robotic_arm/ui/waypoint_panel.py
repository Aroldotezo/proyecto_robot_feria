from PySide6.QtCore import Signal
from PySide6.QtGui import QFontDatabase
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QLabel,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from robotic_arm.domain.gripper_state import GripperState

COORDINATE_LIMIT_MM = 1000.0
HINT = (
    "Posición de la punta de la garra (TCP) en el sistema del brazo: origen en el eje de la base, "
    "+X al frente, +Y a la izquierda, +Z hacia arriba (z = 0 es la mesa). "
    "Empieza con puntos cercanos y seguros, con velocidad baja, y mantén las manos fuera del área de trabajo."
)


class WaypointPanel(QWidget):
    """View only: lets you type a cartesian point, see the servo angles it gives and move the arm there."""

    calculate_requested = Signal(float, float, float, object)  # x, y, z, GripperState
    move_requested = Signal(float, float, float, object)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._x = self._coordinate_box(100.0)
        self._y = self._coordinate_box(0.0)
        self._z = self._coordinate_box(50.0)
        self._gripper = QComboBox()
        self._gripper.addItem("Abierta", GripperState.OPEN)
        self._gripper.addItem("Cerrada", GripperState.CLOSED)
        self._calculate_button = QPushButton("Calcular ángulos (sin mover)")
        self._move_button = QPushButton("Mover el brazo a este punto")
        self._output = QPlainTextEdit()
        self._output.setReadOnly(True)
        self._output.setFont(QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont))
        self._output.setMinimumHeight(170)

        hint = QLabel(HINT)
        hint.setWordWrap(True)
        form_box = QGroupBox("Punto objetivo (TCP)")
        form = QFormLayout(form_box)
        form.addRow("X", self._x)
        form.addRow("Y", self._y)
        form.addRow("Z", self._z)
        form.addRow("Garra", self._gripper)

        layout = QVBoxLayout(self)
        layout.addWidget(hint)
        layout.addWidget(form_box)
        layout.addWidget(self._calculate_button)
        layout.addWidget(self._move_button)
        layout.addWidget(self._output)
        layout.addStretch()

        self._calculate_button.clicked.connect(self._on_calculate_clicked)
        self._move_button.clicked.connect(self._on_move_clicked)

    def set_busy(self, busy: bool) -> None:
        self._calculate_button.setEnabled(not busy)
        self._move_button.setEnabled(not busy)

    def show_result(self, text: str) -> None:
        self._output.setPlainText(text)

    # ------------------------------------------------------------------ internals

    @staticmethod
    def _coordinate_box(value: float) -> QDoubleSpinBox:
        box = QDoubleSpinBox()
        box.setRange(-COORDINATE_LIMIT_MM, COORDINATE_LIMIT_MM)
        box.setDecimals(1)
        box.setSuffix(" mm")
        box.setValue(value)
        return box

    def _on_calculate_clicked(self) -> None:
        self.calculate_requested.emit(self._x.value(), self._y.value(), self._z.value(), self._gripper.currentData())

    def _on_move_clicked(self) -> None:
        answer = QMessageBox.question(
            self,
            "Mover el brazo",
            "El brazo se moverá al punto indicado. Mantén el área de trabajo despejada y usa velocidad baja "
            "en las primeras pruebas.\n\n¿Continuar?",
        )
        if answer == QMessageBox.StandardButton.Yes:
            self.move_requested.emit(self._x.value(), self._y.value(), self._z.value(), self._gripper.currentData())