from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class ArmHeights:
    """Se debe colocar el encuadre en zona, la vision no se mueve: Ajusta el espacio fisico."""

    z_safe: float = 50.0 
    z_pick: float = 10.0   
    z_drop: float = 35.0  

    def __post_init__(self) -> None:
        if self.z_safe < max(self.z_pick, self.z_drop):
            raise ValueError("Z_SAFE no puede ser menor al Z_PICK o Z_DROP")