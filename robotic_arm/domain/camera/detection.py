from dataclasses import dataclass

from robotic_arm.domain.object_class import ObjectClass

@dataclass(frozen=True, slots=True)
class Detection:
    """Un objeto detectado en el frame: clase, confianza (0..1) y caja en pixeles."""

    clase: ObjectClass
    reliability: float
    x: int
    y: int
    width: int
    height: int

    def __post_init__(self) -> None:
        if not 0.0 <= self.reliability <= 1.0:
            raise ValueError(f"La confianza debe estar entre 0 y 1, no {self.reliability}")

    @property
    def center(self) -> tuple[int, int]:
        return self.x + self.width // 2, self.y + self.height // 2