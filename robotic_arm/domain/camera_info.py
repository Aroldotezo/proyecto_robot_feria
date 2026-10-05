from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class CameraInfo:
    """Clase que representa la camara disponible en el sistema."""

    index: int
    label: str