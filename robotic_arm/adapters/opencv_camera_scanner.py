import cv2

from robotic_arm.adapters.opencv_backend import preferred_backend
from robotic_arm.domain.camera.camera_info import CameraInfo
from robotic_arm.ports.camera_scanner import CameraScanner

MAX_PROBED_INDEXES = 5


class OpenCvCameraScanner(CameraScanner):
    """Clase modelo porque OpenCV no puede lsitar las camaras, pero puede saber el inidice."""

    def scan(self) -> list[CameraInfo]:
        cameras: list[CameraInfo] = []
        for index in range(MAX_PROBED_INDEXES):
            capture = cv2.VideoCapture(index, preferred_backend())
            try:
                if capture.isOpened() and capture.read()[0]:
                    cameras.append(CameraInfo(index=index, label=f"Camera {index}"))
            finally:
                capture.release()
        return cameras