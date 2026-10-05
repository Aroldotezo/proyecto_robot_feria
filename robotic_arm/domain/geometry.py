from dataclasses import dataclass

_EPSILON = 1e-9


@dataclass(frozen=True, slots=True)
class NormalizedPoint:
    """Posicion de las fracciones o regiones de la imagen, no depende completamente de la resolucin de la camara."""

    x: float
    y: float

    def __post_init__(self) -> None:
        if not (0.0 <= self.x <= 1.0 and 0.0 <= self.y <= 1.0):
            raise ValueError(f"La normalizacion de las coordenadas debe ser entre 0..1, seleccionadas: ({self.x}, {self.y})")


@dataclass(frozen=True, slots=True)
class NormalizedRect:
    """Region rectangular de la imagen."""

    x: float
    y: float
    width: float
    height: float

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("La region debe ser positiva entre el ancho y el alto")
        if self.x < 0 or self.y < 0 or self.x + self.width > 1 + _EPSILON or self.y + self.height > 1 + _EPSILON:
            raise ValueError("Una region no esta dentro de las coordenadas: (0..1)")

    @property
    def center(self) -> NormalizedPoint:
        return NormalizedPoint(self.x + self.width / 2, self.y + self.height / 2)

    def contains(self, point: NormalizedPoint) -> bool:
        return self.x <= point.x <= self.x + self.width and self.y <= point.y <= self.y + self.height