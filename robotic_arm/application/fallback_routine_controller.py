"""Controlador para ejecutar la rutina programada de demostración (Fallback)."""

import logging
import threading
import time

from PySide6.QtCore import QObject, Signal, Slot

from robotic_arm.application.arm_service import ArmService
from robotic_arm.domain.camera.arm_position import ArmPosition
from robotic_arm.domain.fallback_routine_config import FallbackRoutineConfig, StepPose

log = logging.getLogger(__name__)


class FallbackRoutineController(QObject):
    """Ejecuta secuencias de clasificación fija en bucle para simular el comportamiento del robot."""

    status_message = Signal(str)
    step_progress = Signal(str, int, int)      # (descripción, paso_actual, total_pasos)
    routine_state_changed = Signal(bool)      # True = ejecutando, False = detenido
    cycle_completed = Signal(int)             # número de vueltas completadas

    def __init__(
        self,
        arm_service: ArmService,
        config: FallbackRoutineConfig | None = None,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._arm_service = arm_service
        self._config = config or FallbackRoutineConfig()

        self._stop_event = threading.Event()
        self._worker_thread: threading.Thread | None = None
        self._cycle_count = 0

    @property
    def is_running(self) -> bool:
        return self._worker_thread is not None and self._worker_thread.is_alive()

    @property
    def config(self) -> FallbackRoutineConfig:
        return self._config

    def start_routine(self, tasks: list[str], loop: bool = True) -> bool:
        """Inicia la rutina para las tareas especificadas (ej: ['organic', 'defective', 'inorganic'])."""
        if self.is_running:
            self.status_message.emit("⚠ La rutina ya está en ejecución.")
            return False

        if not self._arm_service.connected:
            self.status_message.emit("⚠ Conecta el brazo primero para iniciar la rutina.")
            return False

        if not tasks:
            self.status_message.emit("⚠ Selecciona al menos una tarea (Orgánico, Defectuoso o Inorgánico).")
            return False

        self._stop_event.clear()
        self._cycle_count = 0
        self._worker_thread = threading.Thread(
            target=self._run_loop,
            args=(list(tasks), loop),
            daemon=True,
        )
        self._worker_thread.start()
        self.routine_state_changed.emit(True)
        self.status_message.emit(f"▶ Rutina iniciada: {', '.join(tasks)} (Bucle: {'SÍ' if loop else 'NO'})")
        return True

    def stop_routine(self) -> None:
        """Detiene la rutina de inmediato."""
        if not self.is_running:
            return
        self.status_message.emit("⏹ Deteniendo rutina...")
        self._stop_event.set()

    def _sleep_interruptible(self, seconds: float) -> bool:
        """Espera el tiempo indicado pero se interrumpe inmediatamente si se solicita detener."""
        return self._stop_event.wait(timeout=seconds)

    def _run_loop(self, tasks: list[str], loop: bool) -> None:
        """Hilo de trabajo en segundo plano."""
        log.info("Iniciando hilo de rutina programada: tasks=%s, loop=%s", tasks, loop)
        try:
            arm = self._arm_service._require_arm()

            while not self._stop_event.is_set():
                for task_key in tasks:
                    if self._stop_event.is_set():
                        break

                    steps = self._config.build_task_steps(task_key)
                    total = len(steps)

                    for idx, step in enumerate(steps, start=1):
                        if self._stop_event.is_set():
                            break

                        self.step_progress.emit(f"[{task_key.upper()}] {step.name}", idx, total)
                        self.status_message.emit(f"({idx}/{total}) {step.name}")

                        pose = ArmPosition(step.angles)
                        arm.move_to(pose)

                        # Espera entre pasos
                        if self._sleep_interruptible(step.delay_seconds):
                            break

                self._cycle_count += 1
                self.cycle_completed.emit(self._cycle_count)

                if not loop or self._stop_event.is_set():
                    break

                self.status_message.emit(f"Ciclo #{self._cycle_count} completado. Siguiente ciclo en {self._config.cycle_delay}s...")
                if self._sleep_interruptible(self._config.cycle_delay):
                    break

        except Exception as e:
            log.exception("Error durante la rutina programada: %s", e)
            self.status_message.emit(f"⚠ Error en rutina: {e}")
        finally:
            self.routine_state_changed.emit(False)
            self.status_message.emit("Rutina finalizada")
            log.info("Hilo de rutina programada finalizado")
