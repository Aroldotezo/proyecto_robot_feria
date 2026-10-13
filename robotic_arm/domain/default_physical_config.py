from robotic_arm.domain.physical_arm_config import PhysicalArmConfig, ShoulderElbowPose
from robotic_arm.domain.servo_calibration import ServoCalibration


def create_default_physical_config() -> PhysicalArmConfig:
    """Poses and ranges measured on the real arm. Tune them here while the arm is being calibrated.

    Servo direction model (físicamente validado):
      S1: Base. 60° = Izquierda (Inorganic), 100° = Centro (Defective), 140° = Derecha (Organic)
      S2: Hombro. INCREMENTA de 90° hacia arriba (115°-132°) para empujar/inclinarse hacia ADELANTE
      S3: Codo. DECREMENTA de 90° hacia abajo (65°-75°) para bajar el antebrazo hacia la mesa
      S4: Pitch muñeca. 180° apunta garra hacia abajo
      S5: Roll muñeca. 150°
      S6: Garra. 30° abierta, 160° cerrada
    """
    return PhysicalArmConfig(
        base=ServoCalibration(offset=90.0, direction=1, min_angle=50.0, max_angle=150.0),
        grasp=ShoulderElbowPose(s2=125, s3=70),    # Hacia adelante (S2=125°) y hacia la mesa (S3=70°)
        lift=ShoulderElbowPose(s2=115, s3=90),     # Se eleva con el objeto
        travel=ShoulderElbowPose(s2=115, s3=90),   # Postura de viaje inclinada al frente
        drop=ShoulderElbowPose(s2=132, s3=65),     # Se extiende más adelante para soltar en cajas
        drop_base_angles={"organic": 140, "defective": 100, "inorganic": 60},
        wrist_pitch_angle=180,
        wrist_roll_angle=150,
        gripper_open_angle=30,
        gripper_closed_angle=160,       # Cierre físico completo
        grasp_s2_min_limit=45,
    )