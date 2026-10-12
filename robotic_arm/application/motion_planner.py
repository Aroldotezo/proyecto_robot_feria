import math

from robotic_arm.application.calibration_service import CalibrationService
from robotic_arm.domain.camera.arm_position import ArmPosition
from robotic_arm.domain.arm_space import ArmPoint
from robotic_arm.domain.drop_angle_check import DropAngleCheck
from robotic_arm.domain.exceptions.errors import CalibrationException, JointLimitException
from robotic_arm.domain.geometry import NormalizedPoint
from robotic_arm.domain.motion_step import MotionAction, MotionStep
from robotic_arm.domain.physical_arm_config import PhysicalArmConfig, ShoulderElbowPose
from robotic_arm.domain.zone import Zone


from robotic_arm.domain.arm_kinematics import ArmKinematics
from robotic_arm.domain.arm_space import ArmPoint, ArmWaypoint


class MotionPlanner:
    """Plans the pick-and-place sequence using physical kinematics and calibrated parameters.
    Can operate with physical preset poses or compute dynamic inverse kinematics targets."""

    def __init__(
        self,
        calibration: CalibrationService,
        config: PhysicalArmConfig,
        kinematics: ArmKinematics | None = None,
    ) -> None:
        self._calibration = calibration
        self._config = config
        self._kinematics = kinematics

    @property
    def kinematics(self) -> ArmKinematics | None:
        return self._kinematics

    def plan_pick_and_place(
        self,
        object_position: NormalizedPoint,
        category_id: str,
        target_z: float | None = None,
    ) -> tuple[MotionStep, ...]:
        """Raises ValueError (outside the evaluation zone, unknown category, on the base axis),
        CalibrationException (not calibrated), UnreachablePointException or JointLimitException."""
        profile = self._calibration.profile
        if not profile.evaluation_zone.region.contains(object_position):
            raise ValueError("El objeto está fuera de la zona de evaluación")
        destination = profile.destination_for(category_id)

        arm_point = self._calibration.image_to_arm(object_position)
        pick_s1 = self.base_angle_for(arm_point)
        drop_s1 = self._drop_base_angle(category_id, destination)

        # Si hay cinemática inversa configurada, validar alcanzabilidad del waypoint
        if self._kinematics is not None:
            z = target_z if target_z is not None else 10.0
            waypoint = ArmWaypoint(arm_point.x, arm_point.y, z)
            # Verifica alcanzabilidad y límites físicos de los 6 servos:
            self._kinematics.inverse(waypoint)

        return self._build_sequence(pick_s1, drop_s1)

    def base_angle_for(self, point: ArmPoint) -> int:
        """S1 for a position on the work plane (arm frame, mm). Never clamped: out of range is an error."""
        if math.hypot(point.x, point.y) < 1e-9:
            raise ValueError("El punto está sobre el eje de la base: la dirección de giro no está definida")
        base = self._config.base
        servo = base.to_servo(math.degrees(math.atan2(point.y, point.x)))
        rounded = round(servo)
        if not base.allows(servo) or not base.allows(rounded):
            raise JointLimitException(
                f"La base necesitaría S1 = {servo:.1f}° y su rango físico es {base.min_angle:.0f}°-{base.max_angle:.0f}°"
            )
        return rounded

    def drop_angle_checks(self) -> list[DropAngleCheck]:
        """For each destination: the taught S1 against the S1 the calibration gives for the zone center."""
        checks = []
        for zone in self._calibration.profile.destination_zones:
            assert zone.category_id is not None
            checks.append(DropAngleCheck(zone.label, zone.category_id, self._config.drop_base_angle(zone.category_id), self._computed_for(zone)))
        return checks

    # ------------------------------------------------------------------ internals

    def _drop_base_angle(self, category_id: str, destination: Zone) -> int:
        taught = self._config.drop_base_angle(category_id)
        if taught is not None:
            return taught
        return self.base_angle_for(self._calibration.image_to_arm(destination.region.center))  # zone center

    def _computed_for(self, zone: Zone) -> int | None:
        try:
            return self.base_angle_for(self._calibration.image_to_arm(zone.region.center))
        except (CalibrationException, JointLimitException, ValueError):
            return None

    def _build_sequence(self, pick_s1: int, drop_s1: int) -> tuple[MotionStep, ...]:
        config = self._config
        opened, closed = config.gripper_open_angle, config.gripper_closed_angle

        def pose(s1: int, stage: ShoulderElbowPose, gripper: int) -> ArmPosition:
            return ArmPosition((s1, stage.s2, stage.s3, config.wrist_pitch_angle, config.wrist_roll_angle, gripper))

        steps = [MotionStep(MotionAction.GO_HOME, ArmPosition.repose(), "Ir a la posición inicial (HOME)")]

        def move(note: str, position: ArmPosition) -> None:
            if position != steps[-1].position:  # skip a step that would not move anything
                steps.append(MotionStep(MotionAction.MOVE_TO, position, note))

        move(f"Girar la base hacia el objeto (S1 = {pick_s1}°) con el brazo elevado", pose(pick_s1, config.travel, opened))
        move("Aproximar: bajar a la zona de agarre", pose(pick_s1, config.grasp, opened))
        move("Elevar ligeramente antes de cerrar", pose(pick_s1, config.lift, opened))
        steps.append(MotionStep(MotionAction.CLOSE_GRIPPER, pose(pick_s1, config.lift, closed), "Cerrar la garra (S1, S2 y S3 no cambian)"))
        move("Elevar con el objeto", pose(pick_s1, config.travel, closed))
        move(f"Girar la base hacia el destino (S1 = {drop_s1}°)", pose(drop_s1, config.travel, closed))
        move("Bajar para soltar", pose(drop_s1, config.drop, closed))
        steps.append(MotionStep(MotionAction.OPEN_GRIPPER, pose(drop_s1, config.drop, opened), "Abrir la garra (S1, S2 y S3 no cambian)"))
        move("Elevar tras soltar", pose(drop_s1, config.travel, opened))
        return tuple(steps)