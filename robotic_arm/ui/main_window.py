from PySide6.QtCore import Qt
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import QDockWidget, QLabel, QMainWindow, QMessageBox

from robotic_arm.application.arm_service import ArmService
from robotic_arm.ports.port_scanner import PortScanner
from robotic_arm.ui.connection_controller import ConnectionController
from robotic_arm.ui.connection_panel import ConnectionPanel


class MainWindow(QMainWindow):
    def __init__(self, service: ArmService, scanner: PortScanner) -> None:
        super().__init__()
        self.setWindowTitle("Robotic Arm Sorter")
        self.resize(1100, 700)

        video_placeholder = QLabel("Camera view will appear here")
        video_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setCentralWidget(video_placeholder)

        panel = ConnectionPanel()
        dock = QDockWidget("Arm connection", self)
        dock.setWidget(panel)
        dock.setFeatures(
            QDockWidget.DockWidgetFeature.DockWidgetMovable | QDockWidget.DockWidgetFeature.DockWidgetFloatable
        )
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, dock)

        self._controller = ConnectionController(panel, service, scanner, parent=self)
        self._controller.status_message.connect(self.statusBar().showMessage)
        self._controller.error_occurred.connect(self._show_error)
        self._controller.refresh_ports()

    def closeEvent(self, event: QCloseEvent) -> None:
        self._controller.shutdown()
        super().closeEvent(event)

    def _show_error(self, message: str) -> None:
        QMessageBox.critical(self, "Arm error", message)