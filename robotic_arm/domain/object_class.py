from enum import Enum


class ObjectClass(Enum):
    """Destino de clasificacion de un objeto."""

    ORGANIC = "organico"
    INORGANIC = "inorganico"
    DEFECTIVE = "defectuoso"