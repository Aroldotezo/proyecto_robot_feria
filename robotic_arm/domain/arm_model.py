from dataclasses import dataclass

from robotic_arm.domain.arm_geometry import ArmGeometry
from robotic_arm.domain.servo_calibration import ServoCalibration


@dataclass(frozen=True, slots=True)
class ArmModel:
    """Everything the kinematics needs to know about this particular arm. Nothing here is code:
    geometry, servo conventions, limits and tool orientation can all be recalibrated."""

    geometry: ArmGeometry
    base: ServoCalibration       # servo 1, yaw (+ = counter-clockwise seen from above, towards +Y)
    shoulder: ServoCalibration   # servo 2, angle of the upper arm above the horizontal
    elbow: ServoCalibration      # servo 3, angle of the forearm relative to the upper arm
    wrist: ServoCalibration      # servo 4, angle of the tool relative to the forearm
    wrist_roll_angle: int        # servo 5, kept fixed
    gripper_open_angle: int      # servo 6
    gripper_closed_angle: int    # servo 6
    tool_pitch_deg: float        # tool axis above the horizontal: -90 = pointing straight down
    min_tcp_z: float             # safety floor: the TCP is never sent below this height (mm)