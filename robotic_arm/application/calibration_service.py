from dataclasses import dataclass, replace

from robotic_arm.domain.arm_heights import ArmHeights
from robotic_arm.domain.arm_space import ArmPoint
from robotic_arm.domain.classification_profile import ClassificationProfile
from robotic_arm.domain.exceptions.errors import CalibrationException
from robotic_arm.domain.geometry import NormalizedPoint
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

    def image_to_arm(self, point: NormalizedPoint) -> ArmPoint:
        """Puede soltar la excepcion: CalibrationException mientras el perfil no esta definido."""
        if self._calibration is None:
            raise CalibrationException(self._problem)
        return self._calibration.to_arm(point)

    def status(self) -> CalibrationStatus:
        count = len(self._profile.reference_points)
        if self._calibration is None:
            return CalibrationStatus(False, count, self._problem)
        return CalibrationStatus(True, count, f"Calibrados: {count} puntos, RMS error {self._calibration.rms_error:.1f} mm")

    def _update(self, **changes: object) -> None:
        self._profile = replace(self._profile, **changes)
        self._repository.save(self._profile)
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