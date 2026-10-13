from robotic_arm.domain.physical_arm_config import PhysicalArmConfig, ShoulderElbowPose
from robotic_arm.domain.servo_calibration import ServoCalibration


def create_default_physical_config() -> PhysicalArmConfig:
    """Poses and ranges measured on the real arm. Tune them here while the arm is being calibrated.

    Servo direction model (decreciente hacia adelante y hacia la mesa):
      S1: Base. 60° = Izquierda (Inorganic), 100° = Centro (Defective), 140° = Derecha (Organic)
      S2: Hombro. DECRECE de 90° (travel 60°, grasp 50°, drop 40°)
      S3: Codo. DECRECE de 90° (travel 90°, grasp 70°, drop 20°)
      S4: Pitch muñeca. 180° apunta garra hacia abajo
      S5: Roll muñeca. 150°
      S6: Garra. 30° abierta, 160° cerrada
    """
    return PhysicalArmConfig(
        base=ServoCalibration(offset=90.0, direction=1, min_angle=50.0, max_angle=150.0),
        grasp=ShoulderElbowPose(s2=50, s3=70),     # Decrece para inclinarse y descender a la mesa
        lift=ShoulderElbowPose(s2=60, s3=90),      # Se eleva con el objeto
        travel=ShoulderElbowPose(s2=60, s3=90),    # Postura de viaje (S2=60°, S3=90°)
        drop=ShoulderElbowPose(s2=40, s3=20),      # Poses de CONTEXT.md: S2=40°, S3=20° para soltar
        drop_base_angles={"organic": 140, "defective": 100, "inorganic": 60},
        wrist_pitch_angle=180,
        wrist_roll_angle=150,
        gripper_open_angle=30,
        gripper_closed_angle=160,       # Cierre físico completo
        grasp_s2_min_limit=35,
    )