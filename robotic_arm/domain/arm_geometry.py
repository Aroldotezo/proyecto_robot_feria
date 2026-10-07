from dataclasses import dataclass

from robotic_arm.domain.gripper_state import GripperState


@dataclass(frozen=True, slots=True)
class ArmGeometry:
    """Physical dimensions of the arm in millimeters.
    """

    base_height: float        
    shoulder_length: float    
    forearm_length: float     
    tool_length_open: float   
    tool_length_closed: float 

    def __post_init__(self) -> None:
        for name in ("shoulder_length", "forearm_length", "tool_length_open", "tool_length_closed"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if self.base_height < 0:
            raise ValueError("base_height cannot be negative")

    def tool_length(self, gripper: GripperState) -> float:
        return self.tool_length_open if gripper is GripperState.OPEN else self.tool_length_closed