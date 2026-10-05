import math
import time

import cv2
import numpy as np
from numpy.typing import NDArray

from robotic_arm.ports.frame_source import FrameSource


class FakeCamera(FrameSource):
    """Synthetic video (a red dot moving over a dark background) for development without a camera."""

    def __init__(self, index: int = 0, width: int = 640, height: int = 480, fps: int = 30) -> None:
        self._width = width
        self._height = height
        self._fps = fps
        self._started_at = 0.0

    def open(self) -> None:
        self._started_at = time.monotonic()

    def read(self) -> NDArray[np.uint8] | None:
        time.sleep(1 / self._fps)
        t = time.monotonic() - self._started_at
        frame = np.full((self._height, self._width, 3), 40, dtype=np.uint8)
        center = (int(self._width / 2 + 120 * math.sin(t)), int(self._height / 2 + 60 * math.cos(1.3 * t)))
        cv2.circle(frame, center, 30, (0, 0, 255), -1)
        cv2.putText(frame, "SIMULATED CAMERA", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        return frame

    def release(self) -> None:
        pass