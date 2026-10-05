import cv2
import numpy as np
from numpy.typing import NDArray

from robotic_arm.adapters.opencv_backend import preferred_backend
from robotic_arm.domain.exceptions.errors import CameraErrorException
from robotic_arm.ports.frame_source import FrameSource


class OpenCvCamera(FrameSource):
    def __init__(self, index: int, width: int = 640, height: int = 480) -> None:
        self._index = index
        self._width = width
        self._height = height
        self._capture: cv2.VideoCapture | None = None

    def open(self) -> None:
        capture = cv2.VideoCapture(self._index, preferred_backend())
        if not capture.isOpened():
            capture.release()
            raise CameraErrorException(f"No se pudo abrir la camara {self._index}. Esta siendo usada por otro programa?")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._capture = capture

    def read(self) -> NDArray[np.uint8] | None:
        if self._capture is None:
            raise CameraErrorException("La camara no esta abierta")
        ok, frame = self._capture.read()
        return frame if ok else None

    def release(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None