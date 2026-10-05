from collections.abc import Callable

from PySide6.QtCore import Qt
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import QDockWidget, QLabel, QMainWindow, QMessageBox

from robotic_arm.application.arm_service import ArmService
from robotic_arm.ports.camera_scanner import CameraScanner
from robotic_arm.ports.frame_source import FrameSource
from robotic_arm.ports.port_scanner import PortScanner
from robotic_arm.ui.camera_controller import CameraController
from robotic_arm.ui.camera_menu import CameraMenu
from robotic_arm.ui.connection_controller import ConnectionController
from robotic_arm.ui.connection_panel import ConnectionPanel
from robotic_arm.ui.video_view import VideoView


class MainWindow(QMainWindow):
    def __init__(
        self,
        arm_service: ArmService,
        port_scanner: PortScanner,
        camera_scanner: CameraScanner,
        camera_factory: Callable[[int], FrameSource],
    ) -> None:
        super().__init__()
        self.setWindowTitle("Brazo robotico clasificador")
        self.resize(1100, 700)

        video_view = VideoView()
        self.setCentralWidget(video_view)

        camera_menu = CameraMenu(self)
        self.menuBar().addMenu(camera_menu)

        connection_panel = ConnectionPanel()
        dock = QDockWidget("Arm connection", self)
        dock.setWidget(connection_panel)
        dock.setFeatures(
            QDockWidget.DockWidgetFeature.DockWidgetMovable | QDockWidget.DockWidgetFeature.DockWidgetFloatable
        )
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, dock)

        self._connection_controller = ConnectionController(connection_panel, arm_service, port_scanner, parent=self)
        self._camera_controller = CameraController(
            camera_menu, video_view, camera_scanner, camera_factory, parent=self
        )

        self._camera_info = QLabel()
        self.statusBar().addPermanentWidget(self._camera_info)
        self._camera_controller.camera_info.connect(self._camera_info.setText)

        for controller in (self._connection_controller, self._camera_controller):
            controller.status_message.connect(self.statusBar().showMessage)
            controller.error_occurred.connect(self._show_error)

        self._connection_controller.refresh_ports()
        self._camera_controller.refresh_cameras()

    def closeEvent(self, event: QCloseEvent) -> None:
        self._camera_controller.stop()
        self._connection_controller.shutdown()
        super().closeEvent(event)

    def _show_error(self, message: str) -> None:
        QMessageBox.critical(self, "Error", message)