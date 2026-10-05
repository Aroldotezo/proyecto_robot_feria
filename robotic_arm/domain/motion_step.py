from dataclasses import dataclass
from enum import Enum

from robotic_arm.domain.arm_space import ArmWaypoint


class MotionAction(Enum):
    GO_HOME = "go_home"
    MOVE_TO = "move_to"
    CLOSE_GRIPPER = "close_gripper"
    OPEN_GRIPPER = "open_gripper"


@dataclass(frozen=True, slots=True)
class MotionStep:
    """Pasos a los que se mueve el brazo. Solo MOVE_TO tiene el waypoint."""

    action: MotionAction
    waypoint: ArmWaypoint | None = None
    note: str = ""

    def __post_init__(self) -> None:
        if (self.action is MotionAction.MOVE_TO) != (self.waypoint is not None):
            raise ValueError("Solo la posicion de destino necesita un waypoint")