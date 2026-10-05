from collections.abc import Sequence

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFontDatabase
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from robotic_arm.domain.arm_heights import ArmHeights
from robotic_arm.domain.motion_step import MotionStep
from robotic_arm.domain.reference_point import ReferencePoint

COORDINATE_LIMIT_MM = 2000.0
HEIGHT_MIN_MM = -500.0
HEIGHT_MAX_MM = 1000.0
ADD_TEXT = "Add point (click on video)"
CANCEL_TEXT = "Click on the video... (cancel)"
HINT = (
    "Click 'Add point', then click a known spot on the video and type that spot's position in the arm frame "
    "(mm): origin at the base axis, +X forward, +Y left. Use 4 points or more, spread over the work area "
    "(with 5 or more, the RMS error reveals a wrong value)."
)


class CalibrationPanel(QWidget):
    """View only: reference-point table, heights and a sequence preview. Knows nothing about services."""

    add_point_toggled = Signal(bool)  # True: waiting for a click on the video
    remove_point_requested = Signal(int)
    clear_points_requested = Signal()
    arm_point_edited = Signal(int, float, float)  # row, arm x, arm y
    heights_changed = Signal(float, float, float)  # z_safe, z_pick, z_drop
    preview_requested = Signal(str)  # category id

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._table = QTableWidget(0, 5)
        self._add_button = QPushButton(ADD_TEXT)
        self._remove_button = QPushButton("Remove selected")
        self._clear_button = QPushButton("Clear all")
        self._status_label = QLabel()
        self._z_safe = self._height_box()
        self._z_pick = self._height_box()
        self._z_drop = self._height_box()
        self._test_point_label = QLabel("Click on the video to choose a test point")
        self._destination_combo = QComboBox()
        self._preview_button = QPushButton("Preview pick and place sequence")
        self._plan_output = QPlainTextEdit()

        self._configure_widgets()
        self._build_layout()
        self._wire_signals()

    # ------------------------------------------------------------------ API for the controller

    def set_reference_points(self, points: Sequence[ReferencePoint]) -> None:
        self._table.setRowCount(len(points))
        for row, point in enumerate(points):
            self._set_text_item(row, 0, str(row + 1))
            self._set_text_item(row, 1, f"{point.image.x:.3f}")
            self._set_text_item(row, 2, f"{point.image.y:.3f}")
            self._table.setCellWidget(row, 3, self._coordinate_box(point.arm.x, row))
            self._table.setCellWidget(row, 4, self._coordinate_box(point.arm.y, row))

    def set_status(self, text: str, ok: bool) -> None:
        self._status_label.setText(text)
        self._status_label.setStyleSheet("color: #2e7d32;" if ok else "color: #c62828;")

    def set_heights(self, heights: ArmHeights) -> None:
        for box, value in ((self._z_safe, heights.z_safe), (self._z_pick, heights.z_pick), (self._z_drop, heights.z_drop)):
            box.blockSignals(True)
            box.setValue(value)
            box.blockSignals(False)

    def set_destinations(self, items: Sequence[tuple[str, str]]) -> None:
        """(label, category id) pairs."""
        self._destination_combo.clear()
        for label, category_id in items:
            self._destination_combo.addItem(label, category_id)

    def set_adding_point(self, active: bool) -> None:
        self._add_button.blockSignals(True)
        self._add_button.setChecked(active)
        self._add_button.setText(CANCEL_TEXT if active else ADD_TEXT)
        self._add_button.blockSignals(False)

    def set_test_point_text(self, text: str) -> None:
        self._test_point_label.setText(text)

    def show_plan(self, steps: Sequence[MotionStep]) -> None:
        self._plan_output.setPlainText("\n".join(self._format_step(n, step) for n, step in enumerate(steps, start=1)))

    def show_plan_message(self, text: str) -> None:
        self._plan_output.setPlainText(text)

    # ------------------------------------------------------------------ construction

    def _configure_widgets(self) -> None:
        self._table.setHorizontalHeaderLabels(["#", "Image X", "Image Y", "Arm X (mm)", "Arm Y (mm)"])
        header = self._table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self._table.verticalHeader().setVisible(False)
        self._table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.setMinimumHeight(150)
        self._add_button.setCheckable(True)
        self._status_label.setWordWrap(True)
        self._test_point_label.setWordWrap(True)
        self._plan_output.setReadOnly(True)
        self._plan_output.setFont(QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont))
        self._plan_output.setMinimumHeight(170)

    def _build_layout(self) -> None:
        hint = QLabel(HINT)
        hint.setWordWrap(True)
        buttons = QHBoxLayout()
        buttons.addWidget(self._add_button, stretch=1)
        buttons.addWidget(self._remove_button)
        buttons.addWidget(self._clear_button)

        points_box = QGroupBox("Reference points")
        points_layout = QVBoxLayout(points_box)
        points_layout.addWidget(hint)
        points_layout.addWidget(self._table)
        points_layout.addLayout(buttons)
        points_layout.addWidget(self._status_label)

        heights_box = QGroupBox("Heights (mm, +Z up)")
        heights_form = QFormLayout(heights_box)
        heights_form.addRow("Z_SAFE (travel)", self._z_safe)
        heights_form.addRow("Z_PICK (grasp)", self._z_pick)
        heights_form.addRow("Z_DROP (release)", self._z_drop)

        destination_row = QHBoxLayout()
        destination_row.addWidget(QLabel("Destination"))
        destination_row.addWidget(self._destination_combo, stretch=1)

        test_box = QGroupBox("Test")
        test_layout = QVBoxLayout(test_box)
        test_layout.addWidget(self._test_point_label)
        test_layout.addLayout(destination_row)
        test_layout.addWidget(self._preview_button)
        test_layout.addWidget(self._plan_output)

        layout = QVBoxLayout(self)
        layout.addWidget(points_box)
        layout.addWidget(heights_box)
        layout.addWidget(test_box)
        layout.addStretch()

    def _wire_signals(self) -> None:
        self._add_button.toggled.connect(self._on_add_toggled)
        self._remove_button.clicked.connect(self._on_remove_clicked)
        self._clear_button.clicked.connect(self._on_clear_clicked)
        self._preview_button.clicked.connect(self._on_preview_clicked)
        for box in (self._z_safe, self._z_pick, self._z_drop):
            box.valueChanged.connect(self._emit_heights_changed)

    # ------------------------------------------------------------------ internals

    @staticmethod
    def _height_box() -> QDoubleSpinBox:
        box = QDoubleSpinBox()
        box.setRange(HEIGHT_MIN_MM, HEIGHT_MAX_MM)
        box.setDecimals(1)
        box.setSuffix(" mm")
        box.setKeyboardTracking(False)
        return box

    def _coordinate_box(self, value: float, row: int) -> QDoubleSpinBox:
        box = QDoubleSpinBox()
        box.setRange(-COORDINATE_LIMIT_MM, COORDINATE_LIMIT_MM)
        box.setDecimals(1)
        box.setKeyboardTracking(False)
        box.setValue(value)  # set before connecting so building the table emits nothing
        box.valueChanged.connect(lambda _value, row=row: self._emit_arm_point_edited(row))
        return box

    def _set_text_item(self, row: int, column: int, text: str) -> None:
        item = QTableWidgetItem(text)
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self._table.setItem(row, column, item)

    def _emit_arm_point_edited(self, row: int) -> None:
        x_box = self._table.cellWidget(row, 3)
        y_box = self._table.cellWidget(row, 4)
        if x_box is not None and y_box is not None:
            self.arm_point_edited.emit(row, x_box.value(), y_box.value())

    def _emit_heights_changed(self, _value: float = 0.0) -> None:
        self.heights_changed.emit(self._z_safe.value(), self._z_pick.value(), self._z_drop.value())

    def _on_add_toggled(self, checked: bool) -> None:
        self._add_button.setText(CANCEL_TEXT if checked else ADD_TEXT)
        self.add_point_toggled.emit(checked)

    def _on_remove_clicked(self) -> None:
        row = self._table.currentRow()
        if row >= 0:
            self.remove_point_requested.emit(row)

    def _on_clear_clicked(self) -> None:
        answer = QMessageBox.question(self, "Clear reference points", "Remove all reference points?")
        if answer == QMessageBox.StandardButton.Yes:
            self.clear_points_requested.emit()

    def _on_preview_clicked(self) -> None:
        category_id = self._destination_combo.currentData()
        if category_id:
            self.preview_requested.emit(category_id)

    @staticmethod
    def _format_step(number: int, step: MotionStep) -> str:
        if step.waypoint is None:
            return f"{number}. {step.note or step.action.value}"
        w = step.waypoint
        return f"{number}. X={w.x:7.1f}  Y={w.y:7.1f}  Z={w.z:6.1f}   {step.note}"