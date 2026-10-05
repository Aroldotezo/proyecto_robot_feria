from collections.abc import Callable

from PySide6.QtCore import QObject, QRunnable, Signal


class WorkerSignals(QObject):
    finished = Signal(object)  # resultado de la tarea
    failed = Signal(str)       # mensaje de error


class Worker(QRunnable):
    """Corre la aplicacion en multiples hilos para que la interfaz no se bloquee."""

    def __init__(self, task: Callable[[], object]) -> None:
        super().__init__()
        self._task = task
        self.signals = WorkerSignals()

    def run(self) -> None:
        try:
            result = self._task()
        except Exception as e:  #Se bloquea la UI
            self.signals.failed.emit(str(e))
        else:
            self.signals.finished.emit(result)