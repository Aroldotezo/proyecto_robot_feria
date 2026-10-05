from robotic_arm.domain.camera.camera_info import CameraInfo
from robotic_arm.ports.camera_scanner import CameraScanner


class FakeCameraScanner(CameraScanner):
    def scan(self) -> list[CameraInfo]:
        return [CameraInfo(index=0, label="Camara simulada")]