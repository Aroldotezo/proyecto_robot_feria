from abc import ABC, abstractmethod

import numpy as np
from numpy.typing import NDArray


class FrameSource(ABC):
    """Puerto de salida: recurso que la camara reconoce (OpenCV format)."""

    @abstractmethod
    def open(self) -> None:
        """Raises CameraErrorException if the source cannot be opened."""

    @abstractmethod
    def read(self) -> NDArray[np.uint8] | None:
        """Next frame, or None if the source stopped delivering."""

    @abstractmethod
    def release(self) -> None:
        """Safe to call even if the source was never opened."""