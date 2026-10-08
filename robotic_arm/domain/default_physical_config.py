from robotic_arm.domain.physical_arm_config import PhysicalArmConfig, ShoulderElbowPose
from robotic_arm.domain.servo_calibration import ServoCalibration


def create_default_physical_config() -> PhysicalArmConfig:
    """Poses and ranges measured on the real arm. Tune them here while the arm is being calibrated.

    Servo direction model (physically validated):
      S2: INCREMENTS to pick/extend forward (>90°), DECREMENTS returns arm backward
      S3: INCREMENTS when picking (above 90° = arm extends forward), at 90° = neutral
      S1: rotates only; computed from object X/Y position
    """
    return PhysicalArmConfig(
        base=ServoCalibration(offset=90.0, direction=1, min_angle=50.0, max_angle=150.0),
        grasp=ShoulderElbowPose(s2=120, s3=110),  # S2 INCREMENT forward + S3 INCREMENT → arm extends toward X+
        lift=ShoulderElbowPose(s2=90, s3=90),      # neutral/home position, gripper closes here
        travel=ShoulderElbowPose(s2=90, s3=90),    # arm neutral for base rotation
        drop=ShoulderElbowPose(s2=40, s3=20),
        drop_base_angles={"organic": 140, "defective": 100, "inorganic": 60},
        wrist_pitch_angle=180,
        wrist_roll_angle=150,
        gripper_open_angle=30,
        gripper_closed_angle=160,      # physical close requires 160°, not 120°
        grasp_s2_min_limit=45,
    )