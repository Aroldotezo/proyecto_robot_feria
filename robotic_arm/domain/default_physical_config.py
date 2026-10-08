from robotic_arm.domain.physical_arm_config import PhysicalArmConfig, ShoulderElbowPose
from robotic_arm.domain.servo_calibration import ServoCalibration


def create_default_physical_config() -> PhysicalArmConfig:
    """Poses and ranges measured on the real arm. Tune them here while the arm is being calibrated.
    """
    return PhysicalArmConfig(
        base=ServoCalibration(offset=100.0, direction=1, min_angle=50.0, max_angle=150.0),
        grasp=ShoulderElbowPose(s2=55, s3=70),
        lift=ShoulderElbowPose(s2=60, s3=90),
        travel=ShoulderElbowPose(s2=60, s3=90),
        drop=ShoulderElbowPose(s2=40, s3=20),
        drop_base_angles={"organic": 140, "defective": 100, "inorganic": 60},
        wrist_pitch_angle=180,
        wrist_roll_angle=150,
        gripper_open_angle=30,
        gripper_closed_angle=120,
        grasp_s2_min_limit=45,
    )