from dataclasses import dataclass, field

from robotic_arm.domain.arm_heights import ArmHeights
from robotic_arm.domain.geometry import NormalizedPoint
from robotic_arm.domain.reference_point import ReferencePoint
from robotic_arm.domain.zone import Zone, ZoneRole


@dataclass(frozen=True, slots=True)
class ClassificationProfile:
    """Escenario completo: zonas, referencia de calibracion y alto del brazo."""

    name: str
    zones: tuple[Zone, ...]
    reference_points: tuple[ReferencePoint, ...] = ()
    heights: ArmHeights = field(default_factory=ArmHeights)

    def __post_init__(self) -> None:
        if sum(1 for zone in self.zones if zone.role is ZoneRole.EVALUATION) != 1:
            raise ValueError("El perfil necesita excatamente una zona de evaluacion")
        ids = [zone.id for zone in self.zones]
        if len(set(ids)) != len(ids):
            raise ValueError("Los ids de la zona deben ser unicos")
        categories = [zone.category_id for zone in self.destination_zones]
        if len(set(categories)) != len(categories):
            raise ValueError("Cada categoria solo necesita una zona de destino")

    @property
    def evaluation_zone(self) -> Zone:
        return next(zone for zone in self.zones if zone.role is ZoneRole.EVALUATION)

    @property
    def destination_zones(self) -> tuple[Zone, ...]:
        return tuple(zone for zone in self.zones if zone.role is ZoneRole.DESTINATION)

    def destination_for(self, category_id: str) -> Zone:
        for zone in self.destination_zones:
            if zone.category_id == category_id:
                return zone
        raise ValueError(f"No destination zone for category '{category_id}'")

    def zone_at(self, point: NormalizedPoint) -> Zone | None:
        return next((zone for zone in self.zones if zone.region.contains(point)), None)