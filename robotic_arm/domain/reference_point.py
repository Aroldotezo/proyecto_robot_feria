from dataclasses import dataclass

from robotic_arm.domain.arm_space import ArmPoint
from robotic_arm.domain.geometry import NormalizedPoint


@dataclass(frozen=True, slots=True)
class ReferencePoint:
    """El spot de la imagen para saber donde esta el brazo."""

    image: NormalizedPoint
    arm: ArmPoint