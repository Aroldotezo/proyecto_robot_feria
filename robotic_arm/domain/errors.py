class ArmErrorException(Exception):
    """Error genérico al comunicarse con el brazo."""


class DisconnectedArmException(ArmErrorException):
    """Se intento operar el brazo sin conexion abierta."""


class ArmTimeoutException(ArmErrorException):
    """El brazo no confirmo el movimiento (DONE) a tiempo."""