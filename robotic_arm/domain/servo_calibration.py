from dataclasses import dataclass

_TOLERANCE = 1e-9


@dataclass(frozen=True, slots=True)
class ServoCalibration:
    """Relation between a joint's mathematical angle and its servo angle, plus its allowed range.

        servo_angle = offset + direction * mathematical_angle

    offset: servo angle (degrees) when the mathematical angle is 0.
    direction: +1, or -1 when the servo turns the opposite way.
    min_angle / max_angle: mechanical range the joint may be commanded to (within 0..180).
    """

    offset: float
    direction: int = 1
    min_angle: float = 0.0
    max_angle: float = 180.0

    def __post_init__(self) -> None:
        if self.direction not in (-1, 1):
            raise ValueError("La direccion debe ser entre +1 or -1")
        if not 0.0 <= self.min_angle <= self.max_angle <= 180.0:
            raise ValueError("El rango debe satisfacer la condicion 0 <= angulo minimo <= angulo maximo <= 180")

    def to_servo(self, mathematical_angle: float) -> float:
        return self.offset + self.direction * mathematical_angle

    def to_mathematical(self, servo_angle: float) -> float:
        return self.direction * (servo_angle - self.offset)

    def allows(self, servo_angle: float) -> bool:
        return self.min_angle - _TOLERANCE <= servo_angle <= self.max_angle + _TOLERANCE