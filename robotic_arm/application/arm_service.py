from collections.abc import Callable, Sequence

from robotic_arm.domain.camera.arm_position import ArmPosition
from robotic_arm.domain.exceptions.errors import DisconnectedArmException
from robotic_arm.domain.motion_step import MotionStep
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
        self._require_arm().move_to(ArmPosition.repose())

    def execute(
        self,
        steps: Sequence[MotionStep],
        on_step: Callable[[int, int, MotionStep], None] | None = None,
    ) -> None:
        """Runs a planned sequence one pose at a time and stops on the first error."""
        arm = self._require_arm()

        for index, step in enumerate(steps, start=1):
            if on_step is not None:
                on_step(index, len(steps), step)

            arm.move_to(step.position)

    def _require_arm(self) -> ArmController:
        if self._arm is None:
            raise DisconnectedArmException("El brazo no esta conectado")

        return self._arm