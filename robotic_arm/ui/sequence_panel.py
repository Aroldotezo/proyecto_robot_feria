from collections.abc import Sequence

from PySide6.QtCore import Signal
from PySide6.QtGui import QFontDatabase
from PySide6.QtWidgets import (
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

HINT = (
    "Prueba de la secuencia completa de recoger y soltar, con las poses validadas en el brazo. "
    "1) Calibra los puntos de referencia. 2) En la pestaña de calibración, haz clic sobre el video para "
    "elegir el punto donde estaría el objeto. 3) Elige el destino. Usa velocidad baja y mantén el área despejada."
)


class SequencePanel(QWidget):
    """View only: shows the planned sequence and lets the user run it on the arm."""

    calculate_requested = Signal(str)  # destination category id
    execute_requested = Signal(str)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._test_point_label = QLabel("Punto de prueba: haz clic sobre el video")
        self._test_point_label.setWordWrap(True)
        self._destination_combo = QComboBox()
        self._calculate_button = QPushButton("Calcular secuencia (sin mover)")
        self._execute_button = QPushButton("Ejecutar secuencia en el brazo")
        self._output = QPlainTextEdit()
        self._output.setReadOnly(True)
        self._output.setFont(QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont))
        self._output.setMinimumHeight(260)

        hint = QLabel(HINT)
        hint.setWordWrap(True)
        destination_row = QHBoxLayout()
        destination_row.addWidget(QLabel("Destino"))
        destination_row.addWidget(self._destination_combo, stretch=1)

        box = QGroupBox("Secuencia de prueba")
        box_layout = QVBoxLayout(box)
        box_layout.addWidget(self._test_point_label)
        box_layout.addLayout(destination_row)
        box_layout.addWidget(self._calculate_button)
        box_layout.addWidget(self._execute_button)

        layout = QVBoxLayout(self)
        layout.addWidget(hint)
        layout.addWidget(box)
        layout.addWidget(self._output)
        layout.addStretch()

        self._calculate_button.clicked.connect(self._on_calculate_clicked)
        self._execute_button.clicked.connect(self._on_execute_clicked)

    def set_destinations(self, items: Sequence[tuple[str, str]]) -> None:
        """(label, category id) pairs."""
        self._destination_combo.clear()
        for label, category_id in items:
            self._destination_combo.addItem(label, category_id)

    def set_test_point_text(self, text: str) -> None:
        self._test_point_label.setText(f"Punto de prueba: {text}")

    def set_busy(self, busy: bool) -> None:
        self._calculate_button.setEnabled(not busy)
        self._execute_button.setEnabled(not busy)

    def show_text(self, text: str) -> None:
        self._output.setPlainText(text)

    def _on_calculate_clicked(self) -> None:
        category_id = self._destination_combo.currentData()
        if category_id:
            self.calculate_requested.emit(category_id)

    def _on_execute_clicked(self) -> None:
        category_id = self._destination_combo.currentData()
        if not category_id:
            return
        answer = QMessageBox.question(
            self,
            "Ejecutar secuencia",
            "El brazo recorrerá toda la secuencia. Mantén el área de trabajo despejada, ten a la mano la "
            "desconexión del brazo y usa velocidad baja en las primeras pruebas.\n\n¿Continuar?",
        )
        if answer == QMessageBox.StandardButton.Yes:
            self.execute_requested.emit(category_id)