from collections.abc import Callable

from robotic_arm.domain.arm_position import ArmPosition
from robotic_arm.domain.errors import DisconnectedArmException
from robotic_arm.ports.arm_controller import ArmController


class ArmService:
    """Casos de uso del brazo que se van a consumir."""

    def __init__(self, arm_factory: Callable[[str], ArmController]) -> None:
        self._arm_factory = arm_factory
        self._arm: ArmController | None = None

    @property
    def connected(self) -> bool:
        return self._arm is not None and self._arm.connected

    def connect(self, port: str, speed: int) -> None:
        if self.connected:
            return
        arm = self._arm_factory(port)
        arm.connect()
        try:
            arm.assign_speed(speed)
        except Exception:
            arm.disconnect()
            raise
        self._arm = arm

    def disconnect(self) -> None:
        if self._arm is not None:
            self._arm.disconnect()
            self._arm = None

    def assign_speed(self, speed: int) -> None:
        self._require_arm().assign_speed(speed)

    def move_to_rest(self) -> None:
        self._require_arm().move_to(ArmPosition.rest())

    def _require_arm(self) -> ArmController:
        if self._arm is None:
            raise DisconnectedArmException("El brazo no esta conectado")
        return self._arm