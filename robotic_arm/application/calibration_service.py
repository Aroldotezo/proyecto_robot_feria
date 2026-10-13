from dataclasses import dataclass, replace

from robotic_arm.domain.arm_heights import ArmHeights
from robotic_arm.domain.arm_space import ArmPoint
from robotic_arm.domain.classification_profile import ClassificationProfile
from robotic_arm.domain.exceptions.errors import CalibrationException
from robotic_arm.domain.geometry import NormalizedPoint, NormalizedRect
from robotic_arm.domain.plane_calibration import MIN_REFERENCE_POINTS, PlaneCalibration
from robotic_arm.domain.reference_point import ReferencePoint
from robotic_arm.ports.profile_repository import ProfileRepository


@dataclass(frozen=True, slots=True)
class CalibrationStatus:
    calibrated: bool
    point_count: int
    message: str


class CalibrationService:
    """Edicion del perfil activo y convierte las imagenes en posiciones del brazo."""

    def __init__(self, repository: ProfileRepository, profile_name: str) -> None:
        self._repository = repository
        self._profile = repository.load(profile_name)
        self._calibration: PlaneCalibration | None = None
        self._problem = ""
        self._rebuild()

    @property
    def profile(self) -> ClassificationProfile:
        return self._profile

    def profile_names(self) -> list[str]:
        return self._repository.names()

    def use_profile(self, name: str) -> None:
        self._profile = self._repository.load(name)
        self._rebuild()

    def add_reference_point(self, image: NormalizedPoint, arm: ArmPoint) -> None:
        self._update(reference_points=(*self._profile.reference_points, ReferencePoint(image, arm)))

    def update_reference_arm_point(self, index: int, arm: ArmPoint) -> None:
        points = list(self._profile.reference_points)
        points[index] = replace(points[index], arm=arm)
        self._update(reference_points=tuple(points))

    def remove_reference_point(self, index: int) -> None:
        points = list(self._profile.reference_points)
        del points[index]
        self._update(reference_points=tuple(points))

    def clear_reference_points(self) -> None:
        self._update(reference_points=())

    def set_heights(self, heights: ArmHeights) -> None:
        self._update(heights=heights)

    def update_zone_region(self, zone_id: str, region: NormalizedRect) -> None:
        zones = [
            replace(z, region=region) if z.id == zone_id else z
            for z in self._profile.zones
        ]
        self._update(zones=tuple(zones))

    def reset_default_zones(self) -> None:
        from robotic_arm.application.default_profile import create_default_profile

        default_zones = create_default_profile().zones
        self._update(zones=default_zones)

    def image_to_arm(self, point: NormalizedPoint) -> ArmPoint:
        """Convierte punto normalizado [0, 1] de la cámara cenital a coordenadas (X, Y) en mm del brazo.
        Si hay homografía calibrada por puntos, la usa. Si no, calcula directamente por geometría analítica:
        - X_arm: Rango de alcance frontal [130 mm .. 250 mm] según posición vertical de la imagen (y).
        - Y_arm: Rango lateral [-120 mm .. +120 mm] según posición horizontal de la imagen (x).
        """
        if self._calibration is not None:
            return self._calibration.to_arm(point)

        # Mapeo analítico directo por defecto (cero calibración manual requerida)
        # La cámara cenital ve: ABAJO = más lejos (+X, adelante), ARRIBA = más cerca (base)
        # Confirmado físicamente: decrementar servos → brazo hacia adelante → coincide con y=1 (abajo)
        # izquierda (x < 0.5) = +Y (hacia la izquierda del brazo), derecha = -Y
        arm_x = 130.0 + point.y * 120.0   # y=1 (abajo) → 250mm (adelante), y=0 (arriba) → 130mm (cerca)
        arm_y = (0.5 - point.x) * 240.0
        return ArmPoint(round(arm_x, 1), round(arm_y, 1))

    def status(self) -> CalibrationStatus:
        count = len(self._profile.reference_points)
        if self._calibration is None:
            return CalibrationStatus(False, count, self._problem)
        return CalibrationStatus(True, count, f"Calibrados: {count} puntos, RMS error {self._calibration.rms_error:.1f} mm")

    def _update(self, **changes: object) -> None:
        updated = replace(self._profile, **changes)
        self._repository.save(updated)
        self._profile = updated
        self._rebuild()

    def _rebuild(self) -> None:
        points = self._profile.reference_points
        self._calibration = None
        if len(points) < MIN_REFERENCE_POINTS:
            self._problem = f"Falta de Calibracion: Se tienen {MIN_REFERENCE_POINTS} puntos de referencia ({len(points)}/{MIN_REFERENCE_POINTS})"
            return
        try:
            self._calibration = PlaneCalibration.from_reference_points(points)
        except CalibrationException as e:
            self._problem = f"No esta calibrado: {e}"