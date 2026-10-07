from robotic_arm.domain.arm_geometry import ArmGeometry
from robotic_arm.domain.arm_model import ArmModel
from robotic_arm.domain.servo_calibration import ServoCalibration


def create_default_arm_model() -> ArmModel:
    """Starting point for THIS arm. Calibrate it physically and edit the values below.
    """
    return ArmModel(
        geometry=ArmGeometry(
            base_height=67.0,
            shoulder_length=160.0,
            forearm_length=110.0,
            tool_length_open=120.0,
            tool_length_closed=170.0,
        ),
        base=ServoCalibration(offset=90.0, direction=1),
        shoulder=ServoCalibration(offset=0.0, direction=1),
        elbow=ServoCalibration(offset=180.0, direction=1),
        wrist=ServoCalibration(offset=90.0, direction=1),
        wrist_roll_angle=90,
        gripper_open_angle=30,
        gripper_closed_angle=120,
        tool_pitch_deg=-90.0,
        min_tcp_z=0.0,
    )