import threading
import time

import serial

from robotic_arm.domain.camera.arm_position import ArmPosition
from robotic_arm.domain.exceptions.errors import (
    ArmErrorException,
    ArmTimeoutException,
    DisconnectedArmException,
)
from robotic_arm.ports.arm_controller import MAX_SPEED, MIN_SPEED, ArmController


class SerialArm(ArmController):
    """
    Habla con el Arduino por puerto serie (9600 baudios por defecto):
        "MODE 1\\n"       -> modo automático
        "SPD <0-100>\\n"  -> velocidad
        "<servo> <ang>\\n" x6 -> pose; el Arduino responde "DONE\\n" al llegar.
    Es seguro llamarlo desde varios hilos: las operaciones se serializan con un lock.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 9600,
        done_timeout: float = 15.0,
        reset_wait: float = 2.0,
    ) -> None:
        self._port = port
        self._baudrate = baudrate
        self._done_timeout = done_timeout
        self._reset_wait = reset_wait  # El Arduino UNO ser reinicia cuando el puerto esta abierto
        self._serial: serial.Serial | None = None
        self._lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self) -> None:
        with self._lock:
            if self.connected:
                return
            try:
                self._serial = serial.Serial(self._port, self._baudrate, timeout=1)
            except serial.SerialException as e:
                self._serial = None
                raise ArmErrorException(f"Could not open {self._port}: {e}") from e

            time.sleep(self._reset_wait)
            self._serial.reset_input_buffer()
            self._send("MODE 1")

    def disconnect(self) -> None:
        with self._lock:
            if self._serial is not None and self._serial.is_open:
                self._serial.close()
            self._serial = None

    def assign_speed(self, speed: int) -> None:
        if not MIN_SPEED <= speed <= MAX_SPEED:
            raise ValueError(f"Speed must be between {MIN_SPEED} and {MAX_SPEED}")
        with self._lock:
            self._send(f"SPD {int(speed)}")

    def move_to(self, pose: ArmPosition) -> None:
        with self._lock:
            connection = self._connection()
            connection.reset_input_buffer()
            self._send(*(f"{servo} {angle}" for servo, angle in enumerate(pose.angles, start=1)))
            self._wait_for_done(connection)

    # ------------------------------------------------------------------ internos

    def _connection(self) -> serial.Serial:
        if self._serial is None or not self._serial.is_open:
            raise DisconnectedArmException("El brazo no esta conectado")
        return self._serial

    def _send(self, *lines: str) -> None:
        connection = self._connection()
        try:
            for line in lines:
                connection.write(f"{line}\n".encode())
                connection.flush()
                time.sleep(0.02)  # 20ms para permitir que Arduino procese cada comando individualmente
        except serial.SerialException as e:
            raise ArmErrorException(f"Failed to write to {self._port}: {e}") from e

    def _wait_for_done(self, connection: serial.Serial) -> None:
        deadline = time.monotonic() + self._done_timeout
        try:
            while time.monotonic() < deadline:
                if connection.readline().decode(errors="ignore").strip() == "DONE":
                    return
        except serial.SerialException as e:
            raise ArmErrorException(f"Error al leer el puerto: {self._port}: {e}") from e
        raise ArmTimeoutException(f"El arduino no respondio 'DONE' en {self._done_timeout:.0f} s")