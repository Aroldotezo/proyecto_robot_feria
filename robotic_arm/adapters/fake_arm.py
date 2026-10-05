import logging
import time

from robotic_arm.domain.arm_position import ArmPosition
from robotic_arm.domain.errors import DisconnectedArmException
from robotic_arm.ports.arm_controller import MAX_SPEED, MIN_SPEED, ArmController

logger = logging.getLogger(__name__)


class FakeArm(ArmController):
    """Simulacion de brazo: Uso en desarrrollo."""

    def __init__(self, port: str, move_delay: float = 1.0) -> None:
        self._port = port
        self._move_delay = move_delay
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        time.sleep(0.5)
        self._connected = True
        logger.info("FakeArm connected on %s", self._port)

    def disconnect(self) -> None:
        self._connected = False
        logger.info("FakeArm disconnected")

    def assign_speed(self, speed: int) -> None:
        if not MIN_SPEED <= speed <= MAX_SPEED:
            raise ValueError(f"Speed must be between {MIN_SPEED} and {MAX_SPEED}")
        self._require_connection()
        logger.info("FakeArm speed set to %d", speed)

    def move_to(self, pose: ArmPosition) -> None:
        self._require_connection()
        logger.info("FakeArm moving to %s", pose.angles)
        time.sleep(self._move_delay)

    def _require_connection(self) -> None:
        if not self._connected:
            raise DisconnectedArmException("El brazo no esta conectado")