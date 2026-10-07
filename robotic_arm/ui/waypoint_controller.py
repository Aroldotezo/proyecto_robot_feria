from PySide6.QtCore import QObject, QThreadPool, Signal, Slot

from robotic_arm.application.arm_service import ArmService
from robotic_arm.domain.camera.arm_position import ArmPosition
from robotic_arm.domain.arm_space import ArmWaypoint
from robotic_arm.domain.exceptions.errors import InverseKinematicsException
from robotic_arm.domain.gripper_state import GripperState
from robotic_arm.ui.waypoint_panel import WaypointPanel
from robotic_arm.ui.worker import Worker

SERVO_NAMES = ("base", "hombro", "codo", "muñeca", "giro de muñeca", "garra")


class WaypointController(QObject):
    status_message = Signal(str)
    error_occurred = Signal(str)

    def __init__(self, panel: WaypointPanel, service: ArmService, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._panel = panel
        self._service = service
        self._pool = QThreadPool.globalInstance()
        panel.calculate_requested.connect(self._on_calculate)
        panel.move_requested.connect(self._on_move)

    def _on_calculate(self, x: float, y: float, z: float, gripper: GripperState) -> None:
        pose = self._solve(ArmWaypoint(x, y, z), gripper)
        if pose is not None:
            self._panel.show_result(self._describe(pose))

    def _on_move(self, x: float, y: float, z: float, gripper: GripperState) -> None:
        waypoint = ArmWaypoint(x, y, z)
        pose = self._solve(waypoint, gripper)  # reject before touching the arm, with a clear message
        if pose is None:
            return
        self._panel.show_result(self._describe(pose) + "\n\nMoviendo el brazo...")
        self._panel.set_busy(True)
        self.status_message.emit("Moviendo el brazo al punto indicado...")

        worker = Worker(lambda: self._service.move_to_waypoint(waypoint, gripper))
        worker.signals.finished.connect(self._on_move_finished)
        worker.signals.failed.connect(self._on_move_failed)
        self._pool.start(worker)

    @Slot(object)
    def _on_move_finished(self, _result: object) -> None:
        self._panel.set_busy(False)
        self.status_message.emit("Movimiento completado")

    @Slot(str)
    def _on_move_failed(self, message: str) -> None:
        self._panel.set_busy(False)
        self.status_message.emit("No se pudo mover el brazo")
        self.error_occurred.emit(message)

    def _solve(self, waypoint: ArmWaypoint, gripper: GripperState) -> ArmPosition | None:
        try:
            return self._service.compute_pose(waypoint, gripper)
        except InverseKinematicsException as e:
            self._panel.show_result(f"Punto rechazado: {e}")
            return None

    @staticmethod
    def _describe(pose: ArmPosition) -> str:
        lines = [f"Servo {number} ({name}): {angle:3d}°" for number, (name, angle) in enumerate(zip(SERVO_NAMES, pose.angles), 1)]
        return "Ángulos calculados:\n" + "\n".join(lines)