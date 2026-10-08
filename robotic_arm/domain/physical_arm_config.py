from collections.abc import Mapping
from dataclasses import dataclass, field

from robotic_arm.domain.servo_calibration import ServoCalibration


@dataclass(frozen=True, slots=True)
class ShoulderElbowPose:
    """Servo angles of the shoulder (S2) and the elbow (S3) for one stage of the manipulation."""

    s2: int
    s3: int

    def __post_init__(self) -> None:
        for name, angle in (("S2", self.s2), ("S3", self.s3)):
            if not 0 <= angle <= 180:
                raise ValueError(f"{name} must be within 0..180, got {angle}")


@dataclass(frozen=True, slots=True)
class PhysicalArmConfig:
    """The poses and ranges validated on the real arm. The software replays them deterministically:
    there is no continuous inverse kinematics and S4/S5 never take part in picking.

    S1 (base) is the only servo computed from the object's X/Y position. S2/S3 come from the stage poses.
    Opening or closing S6 never changes S1/S2/S3.
    """

    base: ServoCalibration                   # S1 = offset + direction * yaw(deg); its range is the physical limit
    grasp: ShoulderElbowPose                 # lowest approach, over the object
    lift: ShoulderElbowPose                  # slightly raised, where the gripper closes
    travel: ShoulderElbowPose                # carried height while the base turns
    drop: ShoulderElbowPose                  # release height
    drop_base_angles: Mapping[str, int] = field(default_factory=dict)  # category id -> taught S1 for releasing
    wrist_pitch_angle: int = 180             # S4, fixed
    wrist_roll_angle: int = 150              # S5, fixed
    gripper_open_angle: int = 30             # S6
    gripper_closed_angle: int = 120          # S6
    grasp_s2_min_limit: int = 45             # S2 must never go below this in the grasp zone

    def __post_init__(self) -> None:
        for name, angle in (
            ("wrist_pitch_angle", self.wrist_pitch_angle),
            ("wrist_roll_angle", self.wrist_roll_angle),
            ("gripper_open_angle", self.gripper_open_angle),
            ("gripper_closed_angle", self.gripper_closed_angle),
        ):
            if not 0 <= angle <= 180:
                raise ValueError(f"{name} must be within 0..180, got {angle}")
        if self.grasp.s2 < self.grasp_s2_min_limit:
            raise ValueError(f"The grasp S2 ({self.grasp.s2}) is below its physical limit ({self.grasp_s2_min_limit})")
        for category, angle in self.drop_base_angles.items():
            if not self.base.allows(angle):
                raise ValueError(f"The drop S1 for '{category}' ({angle}) is outside the base range")

    def drop_base_angle(self, category_id: str) -> int | None:
        return self.drop_base_angles.get(category_id)