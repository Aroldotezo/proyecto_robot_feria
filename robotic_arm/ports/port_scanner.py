from abc import ABC, abstractmethod

from robotic_arm.domain.port_info import PortInfo


class PortScanner(ABC):
    """Salida del puerto: Ayuda a saber si el puerto esta conectado."""

    @abstractmethod
    def scan(self) -> list[PortInfo]:
        """PUertos disponibles, Uno de ellos se selecciona para el puerto del arduino."""