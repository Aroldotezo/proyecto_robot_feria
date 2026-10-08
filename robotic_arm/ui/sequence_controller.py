from collections.abc import Sequence

from PySide6.QtCore import QObject, QThreadPool, Signal, Slot

from robotic_arm.application.arm_service import ArmService
from robotic_arm.application.calibration_service import CalibrationService
from robotic_arm.application.motion_planner import MotionPlanner
from robotic_arm.domain.exceptions.errors import CalibrationException, JointLimitException
from robotic_arm.domain.geometry import NormalizedPoint
from robotic_arm.domain.motion_step import MotionStep
from robotic_arm.ui.sequence_panel import SequencePanel
from robotic_arm.ui.worker import Worker


class SequenceController(QObject):
    status_message = Signal(str)
    error_occurred = Signal(str)
    _progress = Signal(str)  # lets the worker thread report steps safely to the UI thread

    def __init__(
        self,
        panel: SequencePanel,
        service: ArmService,
        planner: MotionPlanner,
        calibration: CalibrationService,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._panel = panel
        self._service = service
        self._planner = planner
        self._calibration = calibration
        self._pool = QThreadPool.globalInstance()
        self._test_point: NormalizedPoint | None = None
        self._plan_text = ""

        panel.calculate_requested.connect(self._on_calculate)
        panel.execute_requested.connect(self._on_execute)
        self._progress.connect(self._on_progress)
        self.reload()

    def reload(self) -> None:
        zones = self._calibration.profile.destination_zones
        self._panel.set_destinations([(zone.label, zone.category_id) for zone in zones])

    def set_test_point(self, point: NormalizedPoint, description: str) -> None:
        self._test_point = point
        self._panel.set_test_point_text(description)

    # ------------------------------------------------------------------ actions

    def _on_calculate(self, category_id: str) -> None:
        steps = self._plan(category_id)
        if steps is not None:
            self._plan_text = self._describe(steps)
            self._panel.show_text(self._plan_text)

    def _on_execute(self, category_id: str) -> None:
        if not self._service.connected:
            self._panel.show_text("Conecta primero el brazo (pestaña de conexión del brazo).")
            return
        steps = self._plan(category_id)
        if steps is None:
            return
        self._plan_text = self._describe(steps)
        self._panel.show_text(self._plan_text)
        self._panel.set_busy(True)

        worker = Worker(lambda: self._service.execute(steps, on_step=self._report_step))
        worker.signals.finished.connect(self._on_finished)
        worker.signals.failed.connect(self._on_failed)
        self._pool.start(worker)

    def _report_step(self, index: int, total: int, step: MotionStep) -> None:
        self._progress.emit(f"Paso {index}/{total}: {step.note}")  # runs in the worker thread

    @Slot(str)
    def _on_progress(self, text: str) -> None:
        self.status_message.emit(text)
        self._panel.show_text(f"{self._plan_text}\n\n{text}")

    @Slot(object)
    def _on_finished(self, _result: object) -> None:
        self._panel.set_busy(False)
        self.status_message.emit("Secuencia completada")
        self._panel.show_text(f"{self._plan_text}\n\nSecuencia completada.")

    @Slot(str)
    def _on_failed(self, message: str) -> None:
        self._panel.set_busy(False)
        self.status_message.emit("La secuencia se detuvo por un error")
        self.error_occurred.emit(message)

    # ------------------------------------------------------------------ helpers

    def _plan(self, category_id: str) -> tuple[MotionStep, ...] | None:
        if self._test_point is None:
            self._panel.show_text("Elige primero un punto de prueba: haz clic sobre el video (pestaña de calibración).")
            return None
        try:
            return self._planner.plan_pick_and_place(self._test_point, category_id)
        except (ValueError, CalibrationException, JointLimitException) as e:
            self._panel.show_text(f"No se puede planear la secuencia: {e}")
            return None

    def _describe(self, steps: Sequence[MotionStep]) -> str:
        lines = ["Control de S1 al soltar (enseñado vs. calculado desde la calibración):"]
        for check in self._planner.drop_angle_checks():
            taught = f"{check.configured}°" if check.configured is not None else "sin valor"
            if check.computed is None:
                computed = "no disponible"
            elif check.configured is None:
                computed = f"{check.computed}°"
            else:
                computed = f"{check.computed}° (diferencia {check.computed - check.configured:+d}°)"
            lines.append(f"  {check.zone_label}: enseñado {taught} | calculado {computed}")
        lines.append("")
        for number, step in enumerate(steps, start=1):
            lines.append(f"{number}. {step.note}")
            lines.append(f"   S1..S6 = {list(step.position.angles)}")
        return "\n".join(lines)