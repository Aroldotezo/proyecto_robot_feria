import logging
import math
from dataclasses import replace

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

_WRIST_MIN, _WRIST_MAX = 0, 175  # S4 physical limits
_HOVER_OFFSETS_MM = (50.0, 40.0, 30.0)  # heights above the grasp point to hover at, tried in order
_TOOL_PITCHES_DEG = (-90.0, -80.0, -70.0, -60.0)  # -90 = vertical; tilts forward only when the reach needs it
# Comfort zone: keep servos away from their stops, they stall and push against them there.
_S3_MAX_COMFORT = 120
_S4_MIN_COMFORT, _S4_MAX_COMFORT = 15, 160


class MotionPlanner:
    """Plans the pick-and-place sequence with inverse kinematics: hover over the object at its own radius,
    come straight down, grasp, go straight up, turn the base and release at the taught drop pose."""

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
    def kinematics(self) -> ArmKinematics:
        return self._kinematics

    def plan_pick_and_place(
        self,
        object_position: NormalizedPoint,
        category_id: str,
        target_z: float | None = None,
    ) -> tuple[MotionStep, ...]:
        """Plans the full pick-and-place sequence.

        Raises:
            ValueError: object outside the evaluation zone (with margin), unknown category, or out of reach.
            CalibrationException: no calibration is available.
        """
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

        # S2, S3 y S4 salen de la cinemática en el plano vertical del objeto; la altura viene de la cámara lateral.
        radius = math.hypot(arm_point.x, arm_point.y)
        object_height = target_z if target_z is not None else 20.0
        grasp_z = max(10.0, 0.5 * object_height)  # agarra a media altura del objeto
        grasp, hover = self.approach_stages(radius, grasp_z)

        return self._build_sequence(pick_s1, drop_s1, grasp=grasp, hover=hover)

    # ------------------------------------------------------------------ poses

    def stage_at(self, radius: float, z: float, tool_pitch_deg: float = -90.0) -> ShoulderElbowPose:
        """S2/S3/S4 para una posición a distancia `radius` (mm) y altura `z` (mm).
        S2 y S3 decrementan para descender en vertical hacia el objeto; S4 se fija en 180° para apuntar abajo."""
        if radius < 90.0:
            raise ValueError(f"Objeto demasiado cerca de la base (r = {radius:.0f} mm)")
        if radius > 210.0:
            raise ValueError(f"Objeto fuera del alcance físico del brazo (r = {radius:.0f} mm > 200 mm)")

        # Progresión suave sobre el radio útil de la mesa (~110 mm a ~195 mm)
        t = max(0.0, min(1.0, (radius - 110.0) / 85.0))
        z_adj = max(0.0, min(15.0, (z - 10.0) * 0.4))

        # S2 decrementa para bajar el hombro hacia la mesa (cerca: ~50°, lejos: ~36°)
        s2 = int(round(50.0 - t * 14.0 + z_adj))
        min_s2 = getattr(self._config, "grasp_s2_min_limit", 32)
        s2 = max(min_s2, min(58, s2))

        # S3 decrementa para bajar el codo hacia la mesa (cerca: ~78°, lejos: ~64°)
        s3 = int(round(78.0 - t * 14.0 + z_adj * 0.5))
        s3 = max(50, min(88, s3))

        s4 = self._config.wrist_pitch_angle  # 180° garra apuntando derecho abajo
        return ShoulderElbowPose(s2=s2, s3=s3, s4=s4)

    def approach_stages(self, radius: float, grasp_z: float) -> tuple[ShoulderElbowPose, ShoulderElbowPose]:
        """(grasp, hover) en el mismo radio:
        - HOVER: hombro elevado (S2 ≈ 58°–65°), codo neutro (S3 = 90°), garra abajo (S4 = 180°).
        - GRASP: S2 y S3 decrementan al unísono para descender en vertical a la mesa."""
        grasp = self.stage_at(radius, grasp_z)
        hover_s2 = min(68, max(58, grasp.s2 + 16))
        hover_s3 = 90
        hover = ShoulderElbowPose(s2=hover_s2, s3=hover_s3, s4=self._config.wrist_pitch_angle)
        return grasp, hover

    def build_pose(self, s1: int, stage: ShoulderElbowPose, gripper: int) -> ArmPosition:
        """Full 6-servo pose. S4 mantiene la garra apuntando hacia abajo (180°)."""
        s4 = stage.s4 if stage.s4 is not None else self._config.wrist_pitch_angle
        return ArmPosition((s1, stage.s2, stage.s3, s4, self._config.wrist_roll_angle, gripper))

    # ------------------------------------------------------------------ base angle

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

    # ------------------------------------------------------------------ sequence

    def _build_sequence(
        self,
        pick_s1: int,
        drop_s1: int,
        grasp: ShoulderElbowPose,
        hover: ShoulderElbowPose,
    ) -> tuple[MotionStep, ...]:
        config = self._config
        opened, closed = config.gripper_open_angle, config.gripper_closed_angle
        pose = self.build_pose

        steps = [MotionStep(MotionAction.GO_HOME, ArmPosition.repose(), "Ir a la posición inicial (HOME)")]

        def move(note: str, position: ArmPosition) -> None:
            if position != steps[-1].position:
                steps.append(MotionStep(MotionAction.MOVE_TO, position, note))

        move(f"Girar la base hacia el objeto (S1 = {pick_s1}°) y quedar sobre él", pose(pick_s1, hover, opened))
        move("Bajar en vertical hasta el objeto", pose(pick_s1, grasp, opened))
        steps.append(MotionStep(MotionAction.CLOSE_GRIPPER, pose(pick_s1, grasp, closed), "Cerrar la garra (S1, S2, S3 y S4 no cambian)"))
        move("Subir con el objeto en vertical", pose(pick_s1, hover, closed))
        move(f"Girar la base hacia el destino (S1 = {drop_s1}°)", pose(drop_s1, hover, closed))
        move("Bajar para soltar", pose(drop_s1, config.drop, closed))
        steps.append(MotionStep(MotionAction.OPEN_GRIPPER, pose(drop_s1, config.drop, opened), "Abrir la garra (S1, S2, S3 y S4 no cambian)"))
        move("Elevar tras soltar", pose(drop_s1, config.travel, opened))
        return tuple(steps)