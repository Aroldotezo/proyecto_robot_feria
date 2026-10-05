from abc import ABC, abstractmethod

from robotic_arm.domain.camera.camera_info import CameraInfo


class CameraScanner(ABC):
    """Puerto de salida: clase que permite descubrir cuales camaras estan disponibles."""

    @abstractmethod
    def scan(self) -> list[CameraInfo]: ...