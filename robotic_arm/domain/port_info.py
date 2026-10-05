from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class PortInfo:
    """A serial port available on this machine."""

    device: str            # "COM4" o "/dev/ttyACM0"
    description: str       # lectura humana, puede estar vacio
    likely_arduino: bool   # Puerto USB id

    @property
    def label(self) -> str:
        return f"{self.device} - {self.description}" if self.description else self.device