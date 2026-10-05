from abc import ABC, abstractmethod

import numpy as np
from numpy.typing import NDArray

from robotic_arm.domain.detection import Detection


class Detector(ABC):
    """Puerto de reconocimiento: recibe un frame BGR (formato OpenCV) y devuelve lo detectado."""

    @abstractmethod
    def detect(self, frame: NDArray[np.uint8]) -> list[Detection]: ...