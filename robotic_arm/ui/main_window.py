from collections.abc import Callable

from PySide6.QtCore import Qt
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import QDockWidget, QLabel, QMainWindow, QMessageBox, QScrollArea, QWidget

from robotic_arm.application.arm_service import ArmService
from robotic_arm.application.calibration_service import CalibrationService
from robotic_arm.application.motion_planner import MotionPlanner
from robotic_arm.ports.camera_scanner import CameraScanner
from robotic_arm.ports.frame_source import FrameSource
from robotic_arm.ports.port_scanner import PortScanner
from robotic_arm.ui.calibration_controller import CalibrationController
from robotic_arm.ui.calibration_overlay import CalibrationOverlay
from robotic_arm.ui.calibration_panel import CalibrationPanel
from robotic_arm.ui.camera_controller import CameraController
from robotic_arm.ui.camera_menu import CameraMenu
from robotic_arm.ui.connection_controller import ConnectionController
from robotic_arm.ui.connection_panel import ConnectionPanel
from robotic_arm.ui.video_view import VideoView
from robotic_arm.ui.zone_overlay import ZoneOverlay


class MainWindow(QMainWindow):
    def __init__(
        self,
        arm_service: ArmService,
        port_scanner: PortScanner,
        camera_scanner: CameraScanner,
        camera_factory: Callable[[int], FrameSource],
        calibration_service: CalibrationService,
        motion_planner: MotionPlanner,
    ) -> None:
        super().__init__()
        self.setWindowTitle("Robotic Arm Sorter")
        self.resize(1280, 760)

        video_view = VideoView()
        self.setCentralWidget(video_view)

        camera_menu = CameraMenu(self)
        self.menuBar().addMenu(camera_menu)

        connection_panel = ConnectionPanel()
        calibration_panel = CalibrationPanel()
        connection_dock = self._add_dock("Arm connection", connection_panel)
        calibration_dock = self._add_dock("Calibration", self._scrollable(calibration_panel))
        self.tabifyDockWidget(connection_dock, calibration_dock)
        connection_dock.raise_()

        self._connection_controller = ConnectionController(connection_panel, arm_service, port_scanner, parent=self)
        self._camera_controller = CameraController(camera_menu, video_view, camera_scanner, camera_factory, parent=self)
        self._calibration_controller = CalibrationController(
            calibration_panel,
            calibration_service,
            motion_planner,
            video_view,
            ZoneOverlay(),
            CalibrationOverlay(),
            parent=self,
        )

        self._camera_info = QLabel()
        self._cursor_info = QLabel()
        self.statusBar().addPermanentWidget(self._cursor_info)
        self.statusBar().addPermanentWidget(self._camera_info)
        self._camera_controller.camera_info.connect(self._camera_info.setText)
        self._calibration_controller.cursor_info.connect(self._cursor_info.setText)

        for controller in (self._connection_controller, self._camera_controller, self._calibration_controller):
            controller.status_message.connect(self.statusBar().showMessage)
        for controller in (self._connection_controller, self._camera_controller):
            controller.error_occurred.connect(self._show_error)

        self._connection_controller.refresh_ports()
        self._camera_controller.refresh_cameras()

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802 - Qt naming
        self._camera_controller.stop()
        self._connection_controller.shutdown()
        super().closeEvent(event)

    def _add_dock(self, title: str, widget: QWidget) -> QDockWidget:
        dock = QDockWidget(title, self)
        dock.setWidget(widget)
        dock.setFeatures(
            QDockWidget.DockWidgetFeature.DockWidgetMovable | QDockWidget.DockWidgetFeature.DockWidgetFloatable
        )
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, dock)
        return dock

    @staticmethod
    def _scrollable(widget: QWidget) -> QScrollArea:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(400)
        scroll.setWidget(widget)
        return scroll

    def _show_error(self, message: str) -> None:
        QMessageBox.critical(self, "Error", message)