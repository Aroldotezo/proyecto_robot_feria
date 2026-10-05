from robotic_arm.domain.port_info import PortInfo
from robotic_arm.ports.port_scanner import PortScanner


class FakePortScanner(PortScanner):
    """Always reports one simulated port, for use together with FakeArm."""

    def scan(self) -> list[PortInfo]:
        return [PortInfo(device="SIMULATED", description="Simulated arm", likely_arduino=True)]