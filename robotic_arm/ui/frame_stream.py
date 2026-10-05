import threading
import time

import numpy as np
from numpy.typing import NDArray
from PySide6.QtCore import QObject, QThread, Signal

from robotic_arm.domain.errors import CameraErrorException
from robotic_arm.ports.frame_source import FrameSource


class FrameStream(QThread):
    """
    Captura la camara en su propio hilo.
    """

    frame_available = Signal()
    fps_updated = Signal(float)
    failed = Signal(str)

    def __init__(self, source: FrameSource, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._source = source
        self._lock = threading.Lock()
        self._latest: NDArray[np.uint8] | None = None
        self._notified = False

    def take_frame(self) -> NDArray[np.uint8] | None:
        with self._lock:
            frame, self._latest, self._notified = self._latest, None, False
        return frame

    def stop(self) -> None:
        self.requestInterruption()
        self.wait(5000)

    def run(self) -> None:
        try:
            self._source.open()
            self._capture_loop()
        except Exception as e: 
            self.failed.emit(str(e))
        finally:
            self._source.release()

    def _capture_loop(self) -> None:
        frames, window_start = 0, time.monotonic()
        while not self.isInterruptionRequested():
            frame = self._source.read()
            if frame is None:
                raise CameraErrorException("La camara ha parado de compartir su entrada")
            self._publish(frame)

            frames += 1
            elapsed = time.monotonic() - window_start
            if elapsed >= 1.0:
                self.fps_updated.emit(frames / elapsed)
                frames, window_start = 0, time.monotonic()

    def _publish(self, frame: NDArray[np.uint8]) -> None:
        with self._lock:
            self._latest = frame
            should_notify = not self._notified
            self._notified = True
        if should_notify:
            self.frame_available.emit()