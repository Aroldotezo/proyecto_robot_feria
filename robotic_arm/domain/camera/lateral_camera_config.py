from dataclasses import dataclass
import math

from robotic_arm.domain.arm_space import ArmWaypoint


@dataclass(frozen=True, slots=True)
class LateralCameraConfig:
    """Configuración física y calibración de la cámara lateral izquierda.
    
    Coordenadas relativas al eje central de la base (Servo 1, suelo = Z:0 mm):
    - x_cam: Avance frontal respecto al Servo 1 (mm)
    - y_cam: Separación lateral (a la izquierda) respecto al Servo 1 (mm)
    - z_cam: Altura del lente sobre la mesa (mm)
    - pitch_deg: Ángulo de inclinación hacia abajo en grados (ej: 25.0)
    """

    x_cam: float = 309.0
    y_cam: float = 397.0
    z_cam: float = 165.0
    pitch_deg: float = 25.0

    @property
    def pitch_rad(self) -> float:
        return math.radians(self.pitch_deg)

    def camera_space_vector(self, waypoint: ArmWaypoint) -> tuple[float, float, float]:
        """Calcula el vector de desplazamiento relativo desde el lente de la cámara
        hacia un punto en el espacio del robot (dx, dy, dz) en mm.
        """
        dx = waypoint.x - self.x_cam
        dy = waypoint.y - self.y_cam
        dz = waypoint.z - self.z_cam
        return dx, dy, dz

    def distance_to(self, waypoint: ArmWaypoint) -> float:
        """Distancia euclidiana tridimensional en mm entre el lente y el waypoint."""
        dx, dy, dz = self.camera_space_vector(waypoint)
        return math.hypot(dx, dy, dz)
