from abc import ABC, abstractmethod

from robotic_arm.domain.classification_profile import ClassificationProfile


class ProfileRepository(ABC):
    """Puerto de salida: donde estan los perfiles de calibracion almacenados."""

    @abstractmethod
    def names(self) -> list[str]: ...

    @abstractmethod
    def load(self, name: str) -> ClassificationProfile:
        """Puede soltar una excepcion: ProfileNotFoundException if si no existe."""

    @abstractmethod
    def save(self, profile: ClassificationProfile) -> None: ...