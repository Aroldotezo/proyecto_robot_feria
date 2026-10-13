from robotic_arm.domain.worker_signals import WorkerSignals
from collections.abc import Callable

from PySide6.QtCore import QRunnable


class Worker(QRunnable):
    """Corre la aplicacion en multiples hilos para que la interfaz no se bloquee."""

    def __init__(self, task: Callable[[], object]) -> None:
        super().__init__()
        self._task = task
        self.signals = WorkerSignals()

    def run(self) -> None:
        try:
            result = self._task()
        except Exception as e:
            try:
                self.signals.failed.emit(str(e))
            except RuntimeError:
                pass  # El receptor de la señal ya fue destruido por cierre de ventana
        else:
            try:
                self.signals.finished.emit(result)
            except RuntimeError:
                pass