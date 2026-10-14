"""Configuración de poses y ángulos para la rutina programada de demostración (Fallback).

MODIFICA AQUÍ LOS ÁNGULOS MANUALES PARA CADA FASE.
Cada pose es una tupla de 6 enteros: (S1_base, S2_hombro, S3_codo, S4_pitch, S5_roll, S6_garra).
"""

from dataclasses import dataclass, field


@dataclass(frozen=True, slots=True)
class StepPose:
    """Una acción y su pose de 6 ángulos directos con tiempo de espera en segundos."""
    name: str
    angles: tuple[int, int, int, int, int, int]
    delay_seconds: float = 2.0


@dataclass(frozen=True, slots=True)
class FallbackRoutineConfig:
    """Banco de poses por pasos para la rutina programada.
    
    Convenio de servos:
      S1: Base (60° = Inorgánico / 100° = Defectuoso / 140° = Orgánico)
      S2: Hombro (decrece para bajar hacia adelante: 65° hover, 52° agarre, 45° soltado)
      S3: Codo (90° neutro, 105° extender para agarrar, 40° soltar en tolva)
      S4: Muñeca Pitch (180° = garra abajo)
      S5: Muñeca Roll (150° neutro)
      S6: Garra (30° abierta / 160° cerrada)
    """

    # --- 1. POSICIÓN DE REPOSO / HOME ---
    home_pose: tuple[int, int, int, int, int, int] = (90, 90, 90, 90, 150, 30)

    # --- 2. SECUENCIA DE AGARRE (PICK CENTRAL) ---
    # S2 invertido: 115° = inclina al frente (hover), 128° = baja a contacto sobre residuo
    pick_hover: tuple[int, int, int, int, int, int] = (90, 115, 90, 180, 150, 30)    # Sobre el residuo, garra abierta
    pick_down: tuple[int, int, int, int, int, int] = (90, 128, 105, 180, 150, 30)   # Bajar a contacto
    pick_grip: tuple[int, int, int, int, int, int] = (90, 128, 105, 180, 150, 160)  # Cerrar garra
    pick_lift: tuple[int, int, int, int, int, int] = (90, 115, 90, 180, 150, 160)   # Subir con el residuo

    # --- 3. DESTINO: ORGÁNICO (Tolva A, derecha: S1 = 140°) ---
    drop_organic_travel: tuple[int, int, int, int, int, int] = (140, 115, 90, 180, 150, 160)   # Girar hacia tolva
    drop_organic_down: tuple[int, int, int, int, int, int] = (140, 135, 40, 180, 150, 160)     # Bajar sobre tolva
    drop_organic_release: tuple[int, int, int, int, int, int] = (140, 135, 40, 180, 150, 30)   # Abrir garra (soltar)
    drop_organic_retract: tuple[int, int, int, int, int, int] = (140, 115, 90, 180, 150, 30)   # Elevar tras soltar

    # --- 4. DESTINO: DEFECTUOSO (Tolva B, centro: S1 = 100°) ---
    drop_defective_travel: tuple[int, int, int, int, int, int] = (100, 115, 90, 180, 150, 160)
    drop_defective_down: tuple[int, int, int, int, int, int] = (100, 135, 40, 180, 150, 160)
    drop_defective_release: tuple[int, int, int, int, int, int] = (100, 135, 40, 180, 150, 30)
    drop_defective_retract: tuple[int, int, int, int, int, int] = (100, 115, 90, 180, 150, 30)

    # --- 5. DESTINO: INORGÁNICO (Tolva C, izquierda: S1 = 60°) ---
    drop_inorganic_travel: tuple[int, int, int, int, int, int] = (60, 115, 90, 180, 150, 160)
    drop_inorganic_down: tuple[int, int, int, int, int, int] = (60, 135, 40, 180, 150, 160)
    drop_inorganic_release: tuple[int, int, int, int, int, int] = (60, 135, 40, 180, 150, 30)
    drop_inorganic_retract: tuple[int, int, int, int, int, int] = (60, 115, 90, 180, 150, 30)

    # Tiempos de espera por defecto entre movimientos (en segundos)
    step_delay: float = 2.0
    grip_delay: float = 1.5
    cycle_delay: float = 3.0

    def build_task_steps(self, task_key: str) -> list[StepPose]:
        """Genera la lista ordenada de poses para una tarea específica."""
        d = self.step_delay
        gd = self.grip_delay

        # Fase común de agarre (Pick)
        steps = [
            StepPose("Ir a posición inicial de agarre (Hover)", self.pick_hover, d),
            StepPose("Bajar hacia el residuo", self.pick_down, d),
            StepPose("Cerrar garra (Agarre)", self.pick_grip, gd),
            StepPose("Elevar residuo", self.pick_lift, d),
        ]

        if task_key == "organic":
            steps.extend([
                StepPose("Girar base a Orgánicos (140°)", self.drop_organic_travel, d),
                StepPose("Bajar en Tolva Orgánicos", self.drop_organic_down, d),
                StepPose("Abrir garra (Soltar Orgánico)", self.drop_organic_release, gd),
                StepPose("Elevar garra tras soltar", self.drop_organic_retract, d),
            ])
        elif task_key == "defective":
            steps.extend([
                StepPose("Girar base a Defectuosos (100°)", self.drop_defective_travel, d),
                StepPose("Bajar en Tolva Defectuosos", self.drop_defective_down, d),
                StepPose("Abrir garra (Soltar Defectuoso)", self.drop_defective_release, gd),
                StepPose("Elevar garra tras soltar", self.drop_defective_retract, d),
            ])
        elif task_key == "inorganic":
            steps.extend([
                StepPose("Girar base a Inorgánicos (60°)", self.drop_inorganic_travel, d),
                StepPose("Bajar en Tolva Inorgánicos", self.drop_inorganic_down, d),
                StepPose("Abrir garra (Soltar Inorgánico)", self.drop_inorganic_release, gd),
                StepPose("Elevar garra tras soltar", self.drop_inorganic_retract, d),
            ])

        return steps
