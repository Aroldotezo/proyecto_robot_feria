import argparse
import logging
import sys

from PySide6.QtWidgets import QApplication

from robotic_arm.adapters.fake_arm import FakeArm
from robotic_arm.adapters.fake_port_scanner import FakePortScanner
from robotic_arm.adapters.serial_arm import SerialArm
from robotic_arm.adapters.serial_port_scanner import SerialPortScanner
from robotic_arm.application.arm_service import ArmService
from robotic_arm.ui.main_window import MainWindow


def main() -> int:
    parser = argparse.ArgumentParser(description="Robotic arm sorter")
    parser.add_argument("--simulate", action="store_true", help="use a simulated arm (no hardware needed)")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)

    if args.simulate:
        service, scanner = ArmService(arm_factory=FakeArm), FakePortScanner()
    else:
        service, scanner = ArmService(arm_factory=SerialArm), SerialPortScanner()

    app = QApplication(sys.argv)
    window = MainWindow(service, scanner)
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())