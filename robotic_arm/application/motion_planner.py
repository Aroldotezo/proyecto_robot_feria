import logging
import math

from robotic_arm.application.calibration_service import CalibrationService
from robotic_arm.domain.camera.arm_position import ArmPosition
from robotic_arm.domain.arm_space import ArmPoint
from robotic_arm.domain.drop_angle_check import DropAngleCheck
from robotic_arm.domain.exceptions.errors import CalibrationException, JointLimitException, UnreachablePointException
from robotic_arm.domain.geometry import NormalizedPoint
from robotic_arm.domain.motion_step import MotionAction, MotionStep
from robotic_arm.domain.physical_arm_config import PhysicalArmConfig, ShoulderElbowPose
from robotic_arm.domain.zone import Zone

from robotic_arm.domain.arm_kinematics import ArmKinematics
from robotic_arm.domain.arm_space import ArmPoint, ArmWaypoint

from robotic_arm.application.default_arm_model import create_default_arm_model

log = logging.getLogger(__name__)

# Margen de tolerancia (2 %) al verificar si el objeto está dentro de la zona de evaluación.
# Evita rechazos espurios cuando el centro de la detección cae exactamente en el borde del recuadro.
_EVAL_ZONE_MARGIN = 0.02

_WRIST_MIN, _WRIST_MAX = 0, 175   
_GRASP_LIFT_MM = 15.0

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
        self._kinematics = kinematics or ArmKinematics(create_default_arm_model())

    @property
    def config(self) -> PhysicalArmConfig:
        return self._config

    @property
    def kinematics(self) -> ArmKinematics | None:
        return self._kinematics

    def plan_pick_and_place(
        self,
        object_position: NormalizedPoint,
        category_id: str,
        target_z: float | None = None,
    ) -> tuple[MotionStep, ...]:
        profile = self._calibration.profile
        eval_region = profile.evaluation_zone.region
        expanded_x = max(0.0, eval_region.x - _EVAL_ZONE_MARGIN)
        expanded_y = max(0.0, eval_region.y - _EVAL_ZONE_MARGIN)
        expanded_w = min(1.0 - expanded_x, eval_region.width + 2 * _EVAL_ZONE_MARGIN)
        expanded_h = min(1.0 - expanded_y, eval_region.height + 2 * _EVAL_ZONE_MARGIN)
        inside = (
            expanded_x <= object_position.x <= expanded_x + expanded_w
            and expanded_y <= object_position.y <= expanded_y + expanded_h
        )
        if not inside:
            raise ValueError("El objeto está fuera de la zona de evaluación")

        destination = profile.destination_for(category_id)
        arm_point = self._calibration.image_to_arm(object_position)
        pick_s1 = self._safe_base_angle(arm_point, label="pick")
        drop_s1 = self._safe_drop_base_angle(category_id, destination)

        # S2/S3/S4 salen de la cinemática inversa en el plano vertical del objeto (y=0: la base no cuenta aquí).
        radius = math.hypot(arm_point.x, arm_point.y)
        object_height = target_z if target_z is not None else 20.0
        grasp_z = max(10.0, 0.5 * object_height)  # agarra a media altura del objeto
        grasp = self._stage_at(radius, grasp_z)
        lift = self._stage_at(radius, grasp_z + _GRASP_LIFT_MM)

        return self._build_sequence(pick_s1, drop_s1, grasp=grasp, lift=lift)

    @staticmethod
    def _vertical_wrist(s2: int, s3: int) -> int:
        """S4 that keeps the tool pointing straight down for a given S2/S3 (checked against the taught pose 60/50/70)."""
        return max(_WRIST_MIN, min(_WRIST_MAX, 180 - s2 - s3))

    def stage_at(self, radius: float, z: float) -> ShoulderElbowPose:
        """IK for a point `radius` mm from the base axis and `z` mm above the table.
        An unreachable point is rejected: it must never fall back to a folded preset pose."""
        try:
            angles = self._kinematics.inverse(ArmWaypoint(radius, 0.0, z)).angles
        except (UnreachablePointException, JointLimitException) as e:
            raise ValueError(f"Objeto fuera de alcance (r = {radius:.0f} mm, z = {z:.0f} mm): {e}") from e
        s2, s3, s4 = angles[1], angles[2], angles[3]
        if s2 < self._config.grasp_s2_min_limit:
            raise ValueError(
                f"Objeto demasiado lejos (r = {radius:.0f} mm): S2 = {s2}° bajaría de {self._config.grasp_s2_min_limit}°"
            )
        return ShoulderElbowPose(s2=s2, s3=s3, s4=s4)

    def build_pose(self, s1: int, stage: ShoulderElbowPose, gripper: int) -> ArmPosition:
        """Full 6-servo pose. S4 keeps the tool pointing straight down unless the stage defines it."""
        s4 = stage.s4 if stage.s4 is not None else self._vertical_wrist(stage.s2, stage.s3)
        return ArmPosition((s1, stage.s2, stage.s3, s4, self._config.wrist_roll_angle, gripper))


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

    # ------------------------------------------------------------------ fallback internals

    def _safe_base_angle(self, point: ArmPoint, label: str = "") -> int:
        """Like base_angle_for() but clamps to the servo limits instead of raising, and logs a warning."""
        if math.hypot(point.x, point.y) < 1e-9:
            raise ValueError("El punto está sobre el eje de la base: la dirección de giro no está definida")
        base = self._config.base
        servo = base.to_servo(math.degrees(math.atan2(point.y, point.x)))
        rounded = round(servo)
        if not base.allows(servo) or not base.allows(rounded):
            clamped = int(max(base.min_angle, min(base.max_angle, rounded)))
            log.warning(
                "⚠ Alcance %s: S1 ideal = %.1f° está fuera del rango %d°-%d°. "
                "El brazo avanzará hasta el límite (S1 = %d°). "
                "Ajusta el recuadro de calibración si este aviso es frecuente.",
                label, servo, base.min_angle, base.max_angle, clamped,
            )
            return clamped
        return rounded

    def _safe_drop_base_angle(self, category_id: str, destination: Zone) -> int:
        taught = self._config.drop_base_angle(category_id)
        if taught is not None:
            return taught
        try:
            return self._safe_base_angle(
                self._calibration.image_to_arm(destination.region.center), label=f"drop({category_id})"
            )
        except (CalibrationException, ValueError):
            # Si el centro de la zona no se puede proyectar, usa el neutro de la base
            base = self._config.base
            fallback = int((base.min_angle + base.max_angle) / 2)
            log.warning(
                "⚠ No se pudo calcular el ángulo de drop para '%s'; usando neutro S1 = %d°.",
                category_id, fallback,
            )
            return fallback

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

    def _build_sequence(
        self,
        pick_s1: int,
        drop_s1: int,
        grasp: ShoulderElbowPose,
        lift: ShoulderElbowPose,
    ) -> tuple[MotionStep, ...]:
        config = self._config
        opened, closed = config.gripper_open_angle, config.gripper_closed_angle

        def pose(s1: int, stage: ShoulderElbowPose, gripper: int) -> ArmPosition:
            s4 = stage.s4 if stage.s4 is not None else self._vertical_wrist(stage.s2, stage.s3)
            return ArmPosition((s1, stage.s2, stage.s3, s4, config.wrist_roll_angle, gripper))

        steps = [MotionStep(MotionAction.GO_HOME, ArmPosition.repose(), "Ir a la posición inicial (HOME)")]

        def move(note: str, position: ArmPosition) -> None:
            if position != steps[-1].position:
                steps.append(MotionStep(MotionAction.MOVE_TO, position, note))

        move(f"Girar la base hacia el objeto (S1 = {pick_s1}°) con el brazo elevado", pose(pick_s1, config.travel, opened))
        move("Aproximar: bajar a la zona de agarre", pose(pick_s1, grasp, opened))
        move("Elevar ligeramente antes de cerrar", pose(pick_s1, lift, opened))
        steps.append(MotionStep(MotionAction.CLOSE_GRIPPER, pose(pick_s1, lift, closed), "Cerrar la garra (S1, S2 y S3 no cambian)"))
        move("Elevar con el objeto", pose(pick_s1, config.travel, closed))
        move(f"Girar la base hacia el destino (S1 = {drop_s1}°)", pose(drop_s1, config.travel, closed))
        move("Bajar para soltar", pose(drop_s1, config.drop, closed))
        steps.append(MotionStep(MotionAction.OPEN_GRIPPER, pose(drop_s1, config.drop, opened), "Abrir la garra (S1, S2 y S3 no cambian)"))
        move("Elevar tras soltar", pose(drop_s1, config.travel, opened))
        return tuple(steps)