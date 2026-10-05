from dataclasses import dataclass, replace

TOTAL_SERVOS = 6
MIN_ANGLE = 0
MAX_ANGLE = 180
BASE_INDEX = 0    # servo 1
CLAW_INDEX = 5   # servo 6


@dataclass(frozen=True, slots=True)
class ArmPosition:
    """Angulos de los 6 servos (orden 1..6). Inmutable y siempre válida."""

    angles: tuple[int, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "angles", tuple(int(a) for a in self.angles))
        if len(self.angles) != TOTAL_SERVOS:
            raise ValueError(f"Se requieren exactamente {TOTAL_SERVOS} ángulos")
        for angle in self.angles:
            if not MIN_ANGLE <= angle <= MAX_ANGLE:
                raise ValueError(f"Angulo fuera de rango ({MIN_ANGLE}..{MAX_ANGLE}): {angle}")

    @classmethod
    def repose(cls) -> "ArmPosition":
        """Todos los servos a 90°, igual que la inicializacion del firmware."""
        return cls((90,) * TOTAL_SERVOS)

    @classmethod
    def limited(cls, angles: list[float] | tuple[float, ...]) -> "ArmPosition":
        """Construye una pose recortando cada angulo a 0..180 (util tras interpolar)."""
        return cls(tuple(max(MIN_ANGLE, min(MAX_ANGLE, round(a))) for a in angles))

    def with_claw(self, angle: int) -> "ArmPosition":
        """Copia de la pose con otro angulo en la garra (reemplaza el list(...)[5] = ... de main.py)."""
        further = list(self.angles)
        further[CLAW_INDEX] = angle
        return replace(self, angles=tuple(further))