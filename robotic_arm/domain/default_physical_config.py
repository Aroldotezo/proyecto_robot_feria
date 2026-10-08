from robotic_arm.domain.physical_arm_config import PhysicalArmConfig, ShoulderElbowPose
from robotic_arm.domain.servo_calibration import ServoCalibration


def create_default_physical_config() -> PhysicalArmConfig:
    """Poses and ranges measured on the real arm. Tune them here while the arm is being calibrated.

    Servo direction model (physically validated):
      S2: DECREMENTS to pick/extend forward (e.g. 70°), INCREMENTS to lift/travel (90°)
      S3: INCREMENTS when picking (e.g. 140° = arm extends forward/down), at 90° = neutral
      S1: rotates only; computed from object X/Y position
    """
    return PhysicalArmConfig(
        base=ServoCalibration(offset=90.0, direction=1, min_angle=50.0, max_angle=150.0),
        grasp=ShoulderElbowPose(s2=70, s3=140),   # S2 DECREMENT + S3 INCREMENT → arm extends toward X+
        lift=ShoulderElbowPose(s2=80, s3=90),      # slightly raised from grasp, where gripper closes
        travel=ShoulderElbowPose(s2=90, s3=90),    # arm neutral (HOME) for base rotation
        drop=ShoulderElbowPose(s2=40, s3=20),
        drop_base_angles={"organic": 140, "defective": 100, "inorganic": 60},
        wrist_pitch_angle=180,
        wrist_roll_angle=150,
        gripper_open_angle=30,
        gripper_closed_angle=160,      # physical close requires 160°, not 120°
        grasp_s2_min_limit=45,
    )