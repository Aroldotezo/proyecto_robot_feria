import argparse
import logging
import sys

from PySide6.QtWidgets import QApplication

from robotic_arm.adapters.fake_arm import FakeArm
from robotic_arm.adapters.fake_camera import FakeCamera
from robotic_arm.adapters.fake_camera_scanner import FakeCameraScanner
from robotic_arm.adapters.fake_port_scanner import FakePortScanner
from robotic_arm.adapters.opencv_camera import OpenCvCamera
from robotic_arm.adapters.opencv_camera_scanner import OpenCvCameraScanner
from robotic_arm.adapters.serial_arm import SerialArm
from robotic_arm.adapters.serial_port_scanner import SerialPortScanner
from robotic_arm.application.arm_service import ArmService
from robotic_arm.ui.main_window import MainWindow


def main() -> int:
    parser = argparse.ArgumentParser(description="Brazo robotico evaluador")
    parser.add_argument("--simulate", action="store_true", help="simulated arm and camera (no hardware needed)")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)

    if args.simulate:
        arm_service = ArmService(arm_factory=FakeArm)
        port_scanner = FakePortScanner()
        camera_scanner = FakeCameraScanner()
        camera_factory = FakeCamera
    else:
        arm_service = ArmService(arm_factory=SerialArm)
        port_scanner = SerialPortScanner()
        camera_scanner = OpenCvCameraScanner()
        camera_factory = OpenCvCamera

    app = QApplication(sys.argv)
    window = MainWindow(arm_service, port_scanner, camera_scanner, camera_factory)
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())