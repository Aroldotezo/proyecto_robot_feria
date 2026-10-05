from PySide6.QtCore import Signal
from PySide6.QtGui import QAction
from PySide6.QtWidgets import QMenu, QWidget

from robotic_arm.domain.camera_info import CameraInfo


class CameraMenu(QMenu):
    camera_selected = Signal(int) 
    stop_requested = Signal()
    refresh_requested = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__("&Camera", parent)
        self._streaming = False
        self._scanning = False
        self._camera_actions: list[QAction] = []

        self._empty_action = self.addAction("No se encontro ninguna camara")
        self._empty_action.setEnabled(False)
        self._separator = self.addSeparator()
        self._refresh_action = self.addAction("Refrescar camaras")
        self._stop_action = self.addAction("Detener camara")
        self._refresh_action.triggered.connect(self.refresh_requested)
        self._stop_action.triggered.connect(self.stop_requested)
        self._update_enabled_state()

    def set_cameras(self, cameras: list[CameraInfo]) -> None:
        for action in self._camera_actions:
            self.removeAction(action)
            action.deleteLater()
        self._camera_actions.clear()

        for camera in cameras:
            action = QAction(camera.label, self)
            action.setCheckable(True)
            action.setData(camera.index)
            action.triggered.connect(lambda _checked=False, index=camera.index: self.camera_selected.emit(index))
            self.insertAction(self._separator, action)
            self._camera_actions.append(action)

        self._empty_action.setVisible(not cameras)
        self._update_enabled_state()

    def set_active(self, index: int | None) -> None:
        for action in self._camera_actions:
            action.setChecked(action.data() == index)

    def set_streaming(self, streaming: bool) -> None:
        self._streaming = streaming
        self._update_enabled_state()

    def set_scanning(self, scanning: bool) -> None:
        self._scanning = scanning
        self._update_enabled_state()

    def _update_enabled_state(self) -> None:
        self._refresh_action.setEnabled(not self._streaming and not self._scanning)
        self._stop_action.setEnabled(self._streaming)
        for action in self._camera_actions:
            action.setEnabled(not self._scanning)