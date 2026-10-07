class ArmErrorException(Exception):
    """Error genérico al comunicarse con el brazo."""


class DisconnectedArmException(ArmErrorException):
    """Se intento operar el brazo sin conexion abierta."""


class ArmTimeoutException(ArmErrorException):
    """El brazo no confirmo el movimiento (DONE) a tiempo."""


class CameraErrorException(Exception):
    """La camara no pudo ser abierta o esta siendo bloqueada por otra aplicacion."""


class CalibrationException(Exception):
    """La calibracion fue olvidada, incompleta o inconsistente."""


class ProfileNotFoundException(Exception):
    """El perfil de clasificacion no existe."""


class ProfileStorageException(Exception):
    """El perfil de puntos seleccionado no puede ser cargado o escrito."""


class InverseKinematicsException(Exception):
    """El punto cartesiano la posicion propuesta no es segura para el servomotor."""


class UnreachablePointException(InverseKinematicsException):
    """Punto fuera del area de trabajo: esta en una zona no mapeada del brazo."""


class JointLimitException(InverseKinematicsException):
    """Punto calculable matematicamente pero necesita estar dentro de la zona optima."""