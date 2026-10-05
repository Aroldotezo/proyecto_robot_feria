from dataclasses import dataclass
from enum import Enum

from robotic_arm.domain.geometry import NormalizedRect


class ZoneRole(Enum):
    EVALUATION = "evaluation"
    DESTINATION = "destination"


@dataclass(frozen=True, slots=True)
class Zone:
    """Region de la vista de la camara."""

    id: str
    label: str
    role: ZoneRole
    region: NormalizedRect
    category_id: str | None = None
    color: str = "#888888"

    def __post_init__(self) -> None:
        if self.role is ZoneRole.DESTINATION and not self.category_id:
            raise ValueError(f"Zona de destino '{self.id}' necesita una category_id")
        if self.role is ZoneRole.EVALUATION and self.category_id:
            raise ValueError("La zona evaluada no tiene un category_id")