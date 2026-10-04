from serial.tools import list_ports

from robotic_arm.domain.port_info import PortInfo
from robotic_arm.ports.port_scanner import PortScanner

# Arduino SA, Arduino.org, CH340 clones (QinHeng) and FTDI
ARDUINO_VENDOR_IDS = {0x2341, 0x2A03, 0x1A86, 0x0403}


class SerialPortScanner(PortScanner):
    def scan(self) -> list[PortInfo]:
        ports = [
            PortInfo(
                device=port.device,
                description="" if port.description in (None, "n/a") else port.description,
                likely_arduino=port.vid in ARDUINO_VENDOR_IDS,
            )
            for port in list_ports.comports()
        ]
        return sorted(ports, key=lambda p: (not p.likely_arduino, p.device))