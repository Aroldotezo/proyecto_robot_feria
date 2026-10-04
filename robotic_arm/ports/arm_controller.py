from abc import ABC, abstractmethod

from robotic_arm.domain.arm_position import ArmPosition

MIN_SPEED = 0
MAX_SPEED = 100


class ArmController(ABC):
    """Puerto de salida: lo que la aplicación necesita del brazo, sin saber cómo se conecta."""

    @property
    @abstractmethod
    def connected(self) -> bool: ...

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def assign_speed(self, velocidad: int) -> None:
        """Velocidad de 0 a 100."""

    @abstractmethod
    def move_to(self, pose: ArmPosition) -> None:
        """Mueve el brazo y BLOQUEA hasta que confirme que llegó."""