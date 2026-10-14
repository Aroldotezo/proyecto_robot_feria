"""Panel de interfaz gráfica para la rutina programada de demostración (Fallback)."""

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QCheckBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class FallbackRoutinePanel(QWidget):
    """Permite seleccionar las tareas programadas y ejecutarlas en bucle continuo."""

    start_requested = Signal(list, bool)  # (tareas_seleccionadas, es_bucle)
    stop_requested = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._build_ui()

    def _build_ui(self) -> None:
        main_layout = QVBoxLayout(self)

        # 1. Cabecera explicativa
        header_group = QGroupBox("Modo Demostración / Feria")
        header_layout = QVBoxLayout(header_group)
        info_label = QLabel(
            "Ejecuta ciclos de Pick & Place con ángulos fijos precalibrados.\n"
            "Ideal para demostraciones en feria sin depender de calibración."
        )
        info_label.setWordWrap(True)
        info_label.setStyleSheet("color: #adb5bd; font-size: 11px;")
        header_layout.addWidget(info_label)
        main_layout.addWidget(header_group)

        # 2. Selección de Tareas
        tasks_group = QGroupBox("Secuencia de Clasificación")
        tasks_layout = QVBoxLayout(tasks_group)

        self._check_organic = QCheckBox("1. Orgánico (Tolva Derecha - S1 = 140°)")
        self._check_organic.setChecked(True)
        self._check_organic.setStyleSheet("color: #51cf66; font-weight: bold;")
        tasks_layout.addWidget(self._check_organic)

        self._check_defective = QCheckBox("2. Defectuoso (Tolva Centro - S1 = 100°)")
        self._check_defective.setChecked(True)
        self._check_defective.setStyleSheet("color: #4dabf7; font-weight: bold;")
        tasks_layout.addWidget(self._check_defective)

        self._check_inorganic = QCheckBox("3. Inorgánico (Tolva Izquierda - S1 = 60°)")
        self._check_inorganic.setChecked(True)
        self._check_inorganic.setStyleSheet("color: #ff8787; font-weight: bold;")
        tasks_layout.addWidget(self._check_inorganic)

        main_layout.addWidget(tasks_group)

        # 3. Opciones de Ejecución
        options_group = QGroupBox("Modo de Operación")
        options_layout = QVBoxLayout(options_group)
        self._check_loop = QCheckBox("Repetir en bucle continuo (Loop infinito)")
        self._check_loop.setChecked(True)
        self._check_loop.setStyleSheet("font-weight: bold; color: #ffd43b;")
        options_layout.addWidget(self._check_loop)
        main_layout.addWidget(options_group)

        # 4. Botones de Control
        control_group = QGroupBox("Control")
        control_layout = QVBoxLayout(control_group)

        self._start_button = QPushButton("▶ INICIAR RUTINA PROGRAMADA")
        self._start_button.setMinimumHeight(44)
        self._start_button.setStyleSheet(
            "QPushButton { background-color: #2b8a3e; color: white; "
            "font-size: 13px; font-weight: bold; border-radius: 6px; } "
            "QPushButton:hover { background-color: #2f9e44; }"
        )
        self._start_button.clicked.connect(self._on_start_clicked)
        control_layout.addWidget(self._start_button)

        self._stop_button = QPushButton("⏹ DETENER RUTINA")
        self._stop_button.setMinimumHeight(38)
        self._stop_button.setEnabled(False)
        self._stop_button.setStyleSheet(
            "QPushButton { background-color: #c92a2a; color: white; "
            "font-size: 12px; font-weight: bold; border-radius: 6px; } "
            "QPushButton:disabled { background-color: #495057; color: #868e96; }"
        )
        self._stop_button.clicked.connect(self.stop_requested)
        control_layout.addWidget(self._stop_button)

        main_layout.addWidget(control_group)

        # 5. Monitor de Estado
        status_group = QGroupBox("Monitor de Rutina")
        status_layout = QVBoxLayout(status_group)

        self._status_label = QLabel("Estado: En espera")
        self._status_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        status_layout.addWidget(self._status_label)

        self._action_label = QLabel("Paso: -")
        self._action_label.setStyleSheet("color: #74c0fc; font-family: monospace;")
        self._action_label.setWordWrap(True)
        status_layout.addWidget(self._action_label)

        self._cycle_label = QLabel("Ciclos completados: 0")
        self._cycle_label.setStyleSheet("color: #ced4da; font-size: 11px;")
        status_layout.addWidget(self._cycle_label)

        main_layout.addWidget(status_group)

        # 6. Pie de nota técnica
        config_path_label = QLabel("Configuración de ángulos en:\nrobotic_arm/domain/fallback_routine_config.py")
        config_path_label.setStyleSheet("color: #868e96; font-size: 10px; font-style: italic;")
        config_path_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(config_path_label)

        main_layout.addStretch()

    def _on_start_clicked(self) -> None:
        tasks = []
        if self._check_organic.isChecked():
            tasks.append("organic")
        if self._check_defective.isChecked():
            tasks.append("defective")
        if self._check_inorganic.isChecked():
            tasks.append("inorganic")

        is_loop = self._check_loop.isChecked()
        self.start_requested.emit(tasks, is_loop)

    def set_running(self, running: bool) -> None:
        """Actualiza el estado de los botones e indicadores."""
        self._start_button.setEnabled(not running)
        self._stop_button.setEnabled(running)
        self._check_organic.setEnabled(not running)
        self._check_defective.setEnabled(not running)
        self._check_inorganic.setEnabled(not running)
        self._check_loop.setEnabled(not running)

        if running:
            self._status_label.setText("Estado: 🟢 EJECUTANDO EN VIVO")
            self._status_label.setStyleSheet("color: #40c057; font-weight: bold; font-size: 12px;")
        else:
            self._status_label.setText("Estado: ⚪ DETENIDO")
            self._status_label.setStyleSheet("color: #adb5bd; font-weight: bold; font-size: 12px;")
            self._action_label.setText("Paso: En reposo")

    def set_step_progress(self, description: str, current: int, total: int) -> None:
        self._action_label.setText(f"Paso {current}/{total}: {description}")

    def set_cycle_count(self, count: int) -> None:
        self._cycle_label.setText(f"Ciclos completados: {count}")
