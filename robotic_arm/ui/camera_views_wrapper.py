from PySide6.QtWidgets import QDockWidget
from robotic_arm.ui.video_view import VideoView


class CameraViewsWrapper:
    def __init__(
        self,
        top_v: VideoView,
        side_v: VideoView,
        top_d: QDockWidget,
        side_d: QDockWidget,
    ) -> None:
        self.top_view = top_v
        self.side_view = side_v
        self._top_d = top_d
        self._side_d = side_d

    def set_top_status(self, text: str) -> None:
        self._top_d.setWindowTitle(f"CÁMARA 1: CENITAL ({text})")

    def set_side_status(self, text: str) -> None:
        self._side_d.setWindowTitle(f"CÁMARA 2: LATERAL ({text})")