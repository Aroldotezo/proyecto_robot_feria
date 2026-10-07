import argparse
import logging
import sys
from pathlib import Path

from PySide6.QtWidgets import QApplication, QMessageBox

from robotic_arm.adapters.app_data_directory import default_data_directory
from robotic_arm.adapters.fake_arm import FakeArm
from robotic_arm.adapters.fake_camera import FakeCamera
from robotic_arm.adapters.fake_camera_scanner import FakeCameraScanner
from robotic_arm.adapters.fake_port_scanner import FakePortScanner
from robotic_arm.adapters.in_memory_profile_repository import InMemoryProfileRepository
from robotic_arm.adapters.opencv_camera import OpenCvCamera
from robotic_arm.adapters.opencv_camera_scanner import OpenCvCameraScanner
from robotic_arm.adapters.serial_arm import SerialArm
from robotic_arm.adapters.serial_port_scanner import SerialPortScanner
from robotic_arm.adapters.sqlite_profile_repository import SqliteProfileRepository
from robotic_arm.application.arm_service import ArmService
from robotic_arm.application.calibration_service import CalibrationService
from robotic_arm.application.default_profile import create_default_profile, create_demo_profile
from robotic_arm.application.motion_planner import MotionPlanner
from robotic_arm.domain.exceptions.errors import ProfileStorageException
from robotic_arm.ports.profile_repository import ProfileRepository
from robotic_arm.ui.main_window import MainWindow
from robotic_arm.application.default_arm_model import create_default_arm_model
from robotic_arm.domain.arm_kinematics import ArmKinematics

DATABASE_FILE = "profiles.db"


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Robotic arm sorter")
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="simulated arm and camera with a demo calibration kept in memory (nothing is saved)",
    )
    parser.add_argument(
        "--fake-arm",
        action="store_true",
        help="simulated arm with the real camera (profiles are saved as usual)",
    )
    parser.add_argument("--db", type=Path, default=None, help="profiles database file (default: per-user data folder)")
    return parser.parse_args()


def open_profiles(args: argparse.Namespace) -> tuple[ProfileRepository, str]:
    """Returns the profile repository and the name of the profile to use."""
    if args.simulate:
        demo = create_demo_profile()
        return InMemoryProfileRepository([demo]), demo.name

    path = args.db or default_data_directory() / DATABASE_FILE
    repository = SqliteProfileRepository(path)
    default = create_default_profile()
    if default.name not in repository.names():
        repository.save(default)
    logging.info("Profiles stored in %s", path)
    return repository, default.name


def main() -> int:
    args = parse_arguments()
    logging.basicConfig(level=logging.INFO)
    app = QApplication(sys.argv)

    kinematics = ArmKinematics(create_default_arm_model())

    if args.simulate or args.fake_arm:
        arm_service, port_scanner = (
            ArmService(arm_factory=FakeArm, kinematics=kinematics),
            FakePortScanner(),
        )
    else:
        arm_service, port_scanner = (
            ArmService(arm_factory=SerialArm, kinematics=kinematics),
            SerialPortScanner(),
        )

    try:
        repository, profile_name = open_profiles(args)
        calibration_service = CalibrationService(repository, profile_name)
    except ProfileStorageException as e:
        QMessageBox.critical(None, "Error", str(e))
        return 1

    motion_planner = MotionPlanner(calibration_service)

    if args.simulate:
        camera_scanner, camera_factory = FakeCameraScanner(), FakeCamera
    else:
        camera_scanner, camera_factory = OpenCvCameraScanner(), OpenCvCamera

    window = MainWindow(
        arm_service,
        port_scanner,
        camera_scanner,
        camera_factory,
        calibration_service,
        motion_planner,
    )
    window.show()
    return app.exec()

if __name__ == "__main__":
    sys.exit(main())