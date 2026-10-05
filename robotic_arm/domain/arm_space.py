from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class ArmPoint:
    """Posicion en la que se trabaja el plano, esta en milimetros."""

    x: float
    y: float


@dataclass(frozen=True, slots=True)
class ArmWaypoint:
    """Posicion del brazo que debe moverse en el espacio"""

    x: float
    y: float
    z: float