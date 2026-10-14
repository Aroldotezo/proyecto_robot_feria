import math

from robotic_arm.domain.arm_model import ArmModel
from robotic_arm.domain.camera.arm_position import ArmPosition
from robotic_arm.domain.arm_space import ArmWaypoint
from robotic_arm.domain.exceptions.errors import JointLimitException, UnreachablePointException
from robotic_arm.domain.gripper_state import GripperState
from robotic_arm.domain.servo_calibration import ServoCalibration
from robotic_arm.domain.tool_pose import ToolPose

_EPSILON = 1e-9
_HOME_SERVO_ANGLE = 90.0 


class ArmKinematics:
    """Cartesian <-> servo angles for this arm. Pure math: it knows nothing about cameras, detectors,
    homographies, serial ports or the Arduino.
    """

    def __init__(self, model: ArmModel) -> None:
        self._model = model

    @property
    def model(self) -> ArmModel:
        return self._model

    def gripper_angle(self, gripper: GripperState) -> int:
        model = self._model
        angle = model.gripper_open_angle if gripper is GripperState.OPEN else model.gripper_closed_angle
        min_g, max_g = model.gripper_range
        if not (min_g - _EPSILON <= angle <= max_g + _EPSILON):
            raise JointLimitException(
                f"El servo 6 (gripper) necesitaría {angle}° y su rango permitido es {min_g:.0f}°-{max_g:.0f}°"
            )
        return angle

    # ------------------------------------------------------------------ inverse

    def inverse(self, waypoint: ArmWaypoint, gripper: GripperState = GripperState.OPEN) -> ArmPosition:
        """TCP position -> the six servo angles.

        Raises UnreachablePointException (outside the workspace) or JointLimitException (reachable on paper
        but beyond a servo's allowed range). Angles are never clamped: an invalid solution is rejected.
        """
        model, geometry = self._model, self._model.geometry

        if waypoint.z < model.min_tcp_z - _EPSILON:
            raise UnreachablePointException(
                f"El punto está por debajo de la altura mínima segura (z = {waypoint.z:.0f} mm, mínimo {model.min_tcp_z:.0f} mm)"
            )
        radius = math.hypot(waypoint.x, waypoint.y)
        if radius < _EPSILON:
            raise UnreachablePointException("El punto está sobre el eje de la base: la dirección de giro no está definida")
        yaw = math.degrees(math.atan2(waypoint.y, waypoint.x))

        pitch = math.radians(model.tool_pitch_deg)
        tool_length = geometry.tool_length(gripper)
        wrist_radius = radius - tool_length * math.cos(pitch)
        wrist_height = waypoint.z - tool_length * math.sin(pitch) - geometry.base_height  # relative to the shoulder axis
        distance = math.hypot(wrist_radius, wrist_height)

        upper, fore = geometry.shoulder_length, geometry.forearm_length
        if distance > upper + fore + _EPSILON:
            raise UnreachablePointException(
                f"Fuera de alcance: la muñeca quedaría a {distance:.0f} mm del hombro y el máximo es {upper + fore:.0f} mm"
            )
        if distance < abs(upper - fore) - _EPSILON:
            raise UnreachablePointException(
                f"Demasiado cerca de la base: la muñeca quedaría a {distance:.0f} mm del hombro y el mínimo es {abs(upper - fore):.0f} mm"
            )

        cos_elbow = (distance**2 - upper**2 - fore**2) / (2 * upper * fore)
        elbow_magnitude = math.acos(max(-1.0, min(1.0, cos_elbow)))

        valid: list[tuple[float, tuple[float, float, float, float]]] = []
        violations_of_first: list[str] = []
        for index, elbow in enumerate((-elbow_magnitude, elbow_magnitude)):  # elbow up first, then elbow down
            shoulder = math.atan2(wrist_height, wrist_radius) - math.atan2(
                fore * math.sin(elbow), upper + fore * math.cos(elbow)
            )
            wrist = pitch - (shoulder + elbow)
            servo = (
                model.base.to_servo(yaw),
                model.shoulder.to_servo(_wrap(math.degrees(shoulder))),
                model.elbow.to_servo(_wrap(math.degrees(elbow))),
                model.wrist.to_servo(_wrap(math.degrees(wrist))),
            )
            problems = self._limit_problems(servo)
            if not problems:
                valid.append((sum(abs(value - _HOME_SERVO_ANGLE) for value in servo[1:]), servo))
            elif index == 0:
                violations_of_first = problems

        if not valid:
            raise JointLimitException(
                "El punto es alcanzable matemáticamente pero excede los límites de los servos: "
                + "; ".join(violations_of_first)
            )

        _, best = min(valid, key=lambda candidate: candidate[0])  # first minimum wins: deterministic
        rounded = tuple(round(value) for value in best)
        late_problems = self._limit_problems(rounded)  # rounding must not push an angle out of range
        if late_problems:
            raise JointLimitException("Al redondear a grados enteros se excede un límite: " + "; ".join(late_problems))
        return ArmPosition((*rounded, model.wrist_roll_angle, self.gripper_angle(gripper)))


    def inverse_with_fixed_elbow(
        self,
        radius: float,
        z: float,
        elbow_servo: int,
        gripper: GripperState = GripperState.OPEN,
    ) -> tuple[int, int]:
        """(S2, S4) that put the TCP at `radius` mm from the base axis and `z` mm above the table while the
        elbow (S3) stays at `elbow_servo`. The shoulder does the reaching; the wrist pitch absorbs the rest,
        so the tool angle is whatever results. Never clamped: raises UnreachablePointException / JointLimitException.
        """
        model, geometry = self._model, self._model.geometry
        if z < model.min_tcp_z - _EPSILON:
            raise UnreachablePointException(
                f"El punto está por debajo de la altura mínima segura (z = {z:.0f} mm, mínimo {model.min_tcp_z:.0f} mm)"
            )
        elbow = math.radians(model.elbow.to_mathematical(elbow_servo))
        tool_length = geometry.tool_length(gripper)
        upper, fore = geometry.shoulder_length, geometry.forearm_length

        def wrist_at(shoulder_servo: float) -> tuple[float, float, float]:
            shoulder = math.radians(model.shoulder.to_mathematical(shoulder_servo))
            forearm = shoulder + elbow
            return (
                upper * math.cos(shoulder) + fore * math.cos(forearm),
                geometry.base_height + upper * math.sin(shoulder) + fore * math.sin(forearm),
                forearm,
            )

        def gap(shoulder_servo: float) -> float:
            wrist_radius, wrist_height, _ = wrist_at(shoulder_servo)
            return math.hypot(radius - wrist_radius, z - wrist_height) - tool_length

        # Scan from the upright shoulder (S2 = 90) forward/down: the first sign change is the solution.
        high = min(model.shoulder.to_servo(90.0), model.shoulder.max_angle)
        low = model.shoulder.min_angle
        step = 0.25
        upper_bound = high
        root: float | None = None
        while upper_bound - step >= low - _EPSILON:
            lower_bound = upper_bound - step
            if gap(upper_bound) * gap(lower_bound) <= 0.0:
                a, b = lower_bound, upper_bound
                for _ in range(40):  # bisection
                    mid = (a + b) / 2
                    if gap(a) * gap(mid) <= 0.0:
                        b = mid
                    else:
                        a = mid
                root = (a + b) / 2
                break
            upper_bound = lower_bound
        if root is None:
            raise UnreachablePointException(
                f"Fuera de alcance con el codo en S3 = {elbow_servo}°: el hombro (S2 entre {low:.0f}° y {high:.0f}°) "
                f"no llega a r = {radius:.0f} mm, z = {z:.0f} mm"
            )

        shoulder_servo = round(root)
        if not model.shoulder.allows(shoulder_servo):
            raise JointLimitException(
                f"servo 2 (hombro) necesitaría {shoulder_servo}° y su rango permitido es {low:.0f}°-{model.shoulder.max_angle:.0f}°"
            )
        wrist_radius, wrist_height, forearm = wrist_at(shoulder_servo)  # recomputed with the rounded S2
        tool_angle = math.atan2(z - wrist_height, radius - wrist_radius)
        wrist_servo = round(model.wrist.to_servo(_wrap(math.degrees(tool_angle - forearm))))
        if not model.wrist.allows(wrist_servo):
            raise JointLimitException(
                f"servo 4 (muñeca) necesitaría {wrist_servo}° y su rango permitido es "
                f"{model.wrist.min_angle:.0f}°-{model.wrist.max_angle:.0f}°"
            )
        return shoulder_servo, wrist_servo
    # ------------------------------------------------------------------ forward (validation)

    def forward(self, position: ArmPosition, gripper: GripperState) -> ToolPose:
        """Servo angles -> where the TCP ends up. Meant for validating the model, not for control."""
        model, geometry = self._model, self._model.geometry
        angles = position.angles
        yaw = math.radians(model.base.to_mathematical(angles[0]))
        shoulder = math.radians(model.shoulder.to_mathematical(angles[1]))
        elbow = math.radians(model.elbow.to_mathematical(angles[2]))
        wrist = math.radians(model.wrist.to_mathematical(angles[3]))

        forearm_angle = shoulder + elbow
        tool_angle = forearm_angle + wrist
        elbow_radius = geometry.shoulder_length * math.cos(shoulder)
        elbow_height = geometry.base_height + geometry.shoulder_length * math.sin(shoulder)
        wrist_radius = elbow_radius + geometry.forearm_length * math.cos(forearm_angle)
        wrist_height = elbow_height + geometry.forearm_length * math.sin(forearm_angle)
        tool_length = geometry.tool_length(gripper)
        tip_radius = wrist_radius + tool_length * math.cos(tool_angle)
        tip_height = wrist_height + tool_length * math.sin(tool_angle)

        tcp = ArmWaypoint(tip_radius * math.cos(yaw), tip_radius * math.sin(yaw), tip_height)
        return ToolPose(tcp, _wrap(math.degrees(tool_angle)))

    # ------------------------------------------------------------------ internals

    def _limit_problems(self, servo: tuple[float, ...]) -> list[str]:
        model = self._model
        joints: tuple[tuple[int, str, ServoCalibration], ...] = (
            (1, "base", model.base),
            (2, "hombro", model.shoulder),
            (3, "codo", model.elbow),
            (4, "muñeca", model.wrist),
        )
        problems = [
            f"servo {number} ({name}) necesitaría {value:.1f}° y su rango permitido es {calibration.min_angle:.0f}°-{calibration.max_angle:.0f}°"
            for (number, name, calibration), value in zip(joints, servo)
            if not calibration.allows(value)
        ]

        # Validar servo 5 (muñeca roll)
        min_roll, max_roll = model.wrist_roll_range
        if not (min_roll - _EPSILON <= model.wrist_roll_angle <= max_roll + _EPSILON):
            problems.append(
                f"servo 5 (roll) configurado en {model.wrist_roll_angle}° excede su rango permitido {min_roll:.0f}°-{max_roll:.0f}°"
            )

        return problems


def _wrap(angle_deg: float) -> float:
    """Normalizes an angle to (-180, 180]."""
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped