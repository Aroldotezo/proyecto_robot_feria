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