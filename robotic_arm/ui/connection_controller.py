from collections.abc import Callable

from PySide6.QtCore import QObject, QThreadPool, Signal, Slot

from robotic_arm.application.arm_service import ArmService
from robotic_arm.ports.port_scanner import PortScanner
from robotic_arm.ui.connection_panel import ConnectionPanel
from robotic_arm.ui.worker import Worker


class ConnectionController(QObject):
    status_message = Signal(str)
    error_occurred = Signal(str)

    def __init__(
        self,
        panel: ConnectionPanel,
        service: ArmService,
        scanner: PortScanner,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._panel = panel
        self._service = service
        self._scanner = scanner
        self._pool = QThreadPool.globalInstance()
        self._on_success: Callable[[object], None] | None = None 

        panel.refresh_requested.connect(self.refresh_ports)
        panel.connect_requested.connect(self._connect)
        panel.disconnect_requested.connect(self._disconnect)
        panel.speed_changed.connect(self._change_speed)
        panel.test_requested.connect(self._test_connection)

    def refresh_ports(self) -> None:
        ports = self._scanner.scan()
        self._panel.set_ports(ports)
        self.status_message.emit(f"{len(ports)} serial port(s) found")

    def shutdown(self) -> None:
        try:
            self._service.disconnect()
        except Exception:
            pass

    # ------------------------------------------------------------------ actions

    def _connect(self, port: str) -> None:
        speed = self._panel.speed
        self._run(
            task=lambda: self._service.connect(port, speed),
            busy_message=f"Connecting to {port}...",
            on_success=lambda _: self._after_connect(port),
        )

    def _disconnect(self) -> None:
        self._run(
            task=self._service.disconnect,
            busy_message="Disconnecting...",
            on_success=lambda _: self._after_disconnect(),
        )

    def _change_speed(self, speed: int) -> None:
        if not self._service.connected:
            return  # la velocidad seleccionada es excesiva
        self._run(
            task=lambda: self._service.assign_speed(speed),
            busy_message=f"Setting speed to {speed}...",
            on_success=lambda _: self.status_message.emit(f"Speed set to {speed}"),
        )

    def _test_connection(self) -> None:
        self._run(
            task=self._service.move_to_rest,
            busy_message="Moving to rest position...",
            on_success=lambda _: self.status_message.emit("Arm reached the rest position: connection OK"),
        )

    def _after_connect(self, port: str) -> None:
        self._panel.set_connected(True)
        self.status_message.emit(f"Connected to {port}")

    def _after_disconnect(self) -> None:
        self._panel.set_connected(False)
        self.status_message.emit("Disconnected")

    # ------------------------------------------------------------------ background tasks

    def _run(self, task: Callable[[], object], busy_message: str, on_success: Callable[[object], None]) -> None:
        self._on_success = on_success
        self._panel.set_busy(True)
        self.status_message.emit(busy_message)

        worker = Worker(task)
        worker.signals.finished.connect(self._on_task_finished)
        worker.signals.failed.connect(self._on_task_failed)
        self._pool.start(worker)

    @Slot(object)
    def _on_task_finished(self, result: object) -> None:
        self._panel.set_busy(False)
        callback, self._on_success = self._on_success, None
        if callback is not None:
            callback(result)

    @Slot(str)
    def _on_task_failed(self, message: str) -> None:
        self._on_success = None
        self._panel.set_busy(False)
        self._panel.set_connected(self._service.connected)
        self.status_message.emit("Operation failed")
        self.error_occurred.emit(message)