from dataclasses import dataclass
from enum import Enum

from robotic_arm.domain.camera.arm_position import ArmPosition


class MotionAction(Enum):
    GO_HOME = "go_home"
    MOVE_TO = "move_to"
    CLOSE_GRIPPER = "close_gripper"
    OPEN_GRIPPER = "open_gripper"


@dataclass(frozen=True, slots=True)
class MotionStep:
    """One step of a manipulation sequence: the exact pose to send to the arm (one ArmPosition)."""

    action: MotionAction
    position: ArmPosition
    note: str = ""