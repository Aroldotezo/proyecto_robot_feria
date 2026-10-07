from dataclasses import dataclass

from robotic_arm.domain.arm_space import ArmWaypoint


@dataclass(frozen=True, slots=True)
class ToolPose:
    """Where the TCP is and how the tool axis points (degrees above the horizontal, -90 = down)."""

    tcp: ArmWaypoint
    pitch_deg: float