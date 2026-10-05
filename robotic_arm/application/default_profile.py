from dataclasses import replace

from robotic_arm.domain.arm_space import ArmPoint
from robotic_arm.domain.classification_profile import ClassificationProfile
from robotic_arm.domain.geometry import NormalizedPoint, NormalizedRect
from robotic_arm.domain.reference_point import ReferencePoint
from robotic_arm.domain.zone import Zone, ZoneRole

_DEMO_REFERENCE_POINTS = (
    ReferencePoint(NormalizedPoint(0.1, 0.1), ArmPoint(120.0, -150.0)),
    ReferencePoint(NormalizedPoint(0.9, 0.1), ArmPoint(120.0, 150.0)),
    ReferencePoint(NormalizedPoint(0.9, 0.9), ArmPoint(300.0, 150.0)),
    ReferencePoint(NormalizedPoint(0.1, 0.9), ArmPoint(300.0, -150.0)),
)


def create_default_profile() -> ClassificationProfile:
    """Organic sorting layout: evaluation zone on top, three destination zones below."""
    return ClassificationProfile(
        name="Organic sorting",
        zones=(
            Zone("evaluation", "Evaluation zone", ZoneRole.EVALUATION, NormalizedRect(0.04, 0.04, 0.92, 0.42), color="#FFD43B"),
            Zone("inorganic", "Inorganic", ZoneRole.DESTINATION, NormalizedRect(0.04, 0.50, 0.29, 0.46), "inorganic", "#FF8787"),
            Zone("defective", "Defective", ZoneRole.DESTINATION, NormalizedRect(0.355, 0.50, 0.29, 0.46), "defective", "#4DABF7"),
            Zone("organic", "Organic", ZoneRole.DESTINATION, NormalizedRect(0.67, 0.50, 0.29, 0.46), "organic", "#51CF66"),
        ),
    )


def create_demo_profile() -> ClassificationProfile:
    """Perfil por defecto puede almacenar mas perfiles."""
    return replace(create_default_profile(), reference_points=_DEMO_REFERENCE_POINTS)