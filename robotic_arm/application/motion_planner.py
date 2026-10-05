from robotic_arm.application.calibration_service import CalibrationService
from robotic_arm.domain.arm_space import ArmPoint, ArmWaypoint
from robotic_arm.domain.geometry import NormalizedPoint
from robotic_arm.domain.motion_step import MotionAction, MotionStep


class MotionPlanner:
    """Construye la secuencia de manipulacion del brazo."""

    def __init__(self, calibration: CalibrationService) -> None:
        self._calibration = calibration

    def plan_pick_and_place(self, object_position: NormalizedPoint, category_id: str) -> tuple[MotionStep, ...]:
        """Puede soltar ValueError o CalibrationException."""
        profile = self._calibration.profile
        if not profile.evaluation_zone.region.contains(object_position):
            raise ValueError("El objecto esta fuera de la zona de evaluacion")

        heights = profile.heights
        source = self._calibration.image_to_arm(object_position)
        destination = self._calibration.image_to_arm(profile.destination_for(category_id).region.center)

        def at(point: ArmPoint, z: float) -> ArmWaypoint:
            return ArmWaypoint(point.x, point.y, z)

        return (
            MotionStep(MotionAction.GO_HOME, note="HOME"),
            MotionStep(MotionAction.MOVE_TO, at(source, heights.z_safe), "XY object, at Z_SAFE"),
            MotionStep(MotionAction.MOVE_TO, at(source, heights.z_pick), "down to Z_PICK"),
            MotionStep(MotionAction.CLOSE_GRIPPER, note="close gripper"),
            MotionStep(MotionAction.MOVE_TO, at(source, heights.z_safe), "up to Z_SAFE"),
            MotionStep(MotionAction.MOVE_TO, at(destination, heights.z_safe), "XY destination, at Z_SAFE"),
            MotionStep(MotionAction.MOVE_TO, at(destination, heights.z_drop), "down to Z_DROP"),
            MotionStep(MotionAction.OPEN_GRIPPER, note="open gripper"),
            MotionStep(MotionAction.MOVE_TO, at(destination, heights.z_safe), "up to Z_SAFE"),
        )