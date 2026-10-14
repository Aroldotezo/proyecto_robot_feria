from robotic_arm.domain.arm_geometry import ArmGeometry
from robotic_arm.domain.arm_model import ArmModel
from robotic_arm.domain.servo_calibration import ServoCalibration


def create_default_arm_model() -> ArmModel:
    """Configuración física real del brazo robótico calibrado en milímetros y límites angulares.
    Medidas reales euclidianas:
      H_base: 130.0 mm
      L1 (shoulder): 127.0 mm
      L2 (forearm): 86.0 mm
      L3_open: 122.0 mm
      L3_closed: 166.0 mm
    Límites angulares [min, max]:
      S1: [50°, 125°]
      S2: [20°, 90°]
      S3: [0°, 160°]
      S4: [0°, 175°]
      S5: [0°, 170°]
      S6: [90°, 180°]
    """
    return ArmModel(
        geometry=ArmGeometry(
            base_height=130.0,
            shoulder_length=127.0,
            forearm_length=86.0,
            tool_length_open=122.0,
            tool_length_closed=166.0,
        ),
        base=ServoCalibration(offset=90.0, direction=1, min_angle=50.0, max_angle=125.0),
        # shoulder=ServoCalibration(offset=0.0, direction=1, min_angle=20.0, max_angle=160.0),
        shoulder=ServoCalibration(offset=0.0, direction=1, min_angle=20.0, max_angle=90.0),
        elbow=ServoCalibration(offset=180.0, direction=1, min_angle=0.0, max_angle=160.0),
        wrist=ServoCalibration(offset=90.0, direction=1, min_angle=0.0, max_angle=175.0),
        wrist_roll_angle=90,
        gripper_open_angle=90,
        gripper_closed_angle=180,
        tool_pitch_deg=-90.0,
        min_tcp_z=0.0,
        wrist_roll_range=(0.0, 170.0),
        gripper_range=(90.0, 180.0),
    )