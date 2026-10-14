from robotic_arm.ui.camera_views_wrapper import CameraViewsWrapper
from collections.abc import Callable

from PySide6.QtCore import Qt
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import (
    QDockWidget,
    QLabel,
    QMainWindow,
    QMessageBox,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from robotic_arm.application.arm_service import ArmService
from robotic_arm.application.calibration_service import CalibrationService
from robotic_arm.application.motion_planner import MotionPlanner
from robotic_arm.ports.camera_scanner import CameraScanner
from robotic_arm.ports.detector import Detector
from robotic_arm.ports.frame_source import FrameSource
from robotic_arm.ports.port_scanner import PortScanner
from robotic_arm.ui.calibration_controller import CalibrationController
from robotic_arm.ui.calibration_overlay import CalibrationOverlay
from robotic_arm.ui.calibration_panel import CalibrationPanel
from robotic_arm.ui.camera_controller import CameraController
from robotic_arm.ui.camera_menu import CameraMenu
from robotic_arm.ui.connection_controller import ConnectionController
from robotic_arm.ui.connection_panel import ConnectionPanel
from robotic_arm.ui.detection_controller import DetectionController
from robotic_arm.ui.detection_overlay import DetectionOverlay
from robotic_arm.ui.detection_panel import DetectionPanel
from robotic_arm.ui.video_view import VideoView
from robotic_arm.ui.zone_overlay import ZoneOverlay
from robotic_arm.ui.sequence_controller import SequenceController
from robotic_arm.ui.sequence_panel import SequencePanel


class MainWindow(QMainWindow):
    def __init__(
        self,
        arm_service: ArmService,
        port_scanner: PortScanner,
        camera_scanner: CameraScanner,
        camera_factory: Callable[[int], FrameSource],
        calibration_service: CalibrationService,
        motion_planner: MotionPlanner,
        detector: Detector | None = None,
    ) -> None:
        super().__init__()

        self.setWindowTitle("Brazo robotico clasificador")
        self.resize(1280, 760)

        # ---- Cámaras como Docks movibles / desacoplables ----
        self.setDockOptions(
            QMainWindow.DockOption.AnimatedDocks
            | QMainWindow.DockOption.AllowNestedDocks
            | QMainWindow.DockOption.AllowTabbedDocks
        )

        top_container = QWidget()
        top_layout = QVBoxLayout(top_container)
        top_layout.setContentsMargins(4, 4, 4, 4)
        top_status_label = QLabel("Desconectada")
        top_status_label.setStyleSheet("color: #888888; font-size: 11px;")
        video_view = VideoView(placeholder="Cámara cenital desconectada")
        top_layout.addWidget(video_view)

        self._top_dock = QDockWidget("CÁMARA 1: CENITAL (Plano X, Y)", self)
        self._top_dock.setWidget(top_container)
        self._top_dock.setFeatures(
            QDockWidget.DockWidgetFeature.DockWidgetMovable
            | QDockWidget.DockWidgetFeature.DockWidgetFloatable
        )
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self._top_dock)

        side_container = QWidget()
        side_layout = QVBoxLayout(side_container)
        side_layout.setContentsMargins(4, 4, 4, 4)
        side_video_view = VideoView(placeholder="Cámara lateral desconectada")
        side_layout.addWidget(side_video_view)

        self._side_dock = QDockWidget("CÁMARA 2: LATERAL (Altura Z)", self)
        self._side_dock.setWidget(side_container)
        self._side_dock.setFeatures(
            QDockWidget.DockWidgetFeature.DockWidgetMovable
            | QDockWidget.DockWidgetFeature.DockWidgetFloatable
        )
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self._side_dock)
        self.splitDockWidget(self._top_dock, self._side_dock, Qt.Orientation.Horizontal)

        dual_video = CameraViewsWrapper(
            video_view,
            side_video_view,
            self._top_dock,
            self._side_dock,
        )

        camera_menu = CameraMenu(self)
        self.menuBar().addMenu(camera_menu)

        # Panels
        connection_panel = ConnectionPanel()
        calibration_panel = CalibrationPanel()
        sequence_panel = SequencePanel()
        detection_panel = DetectionPanel()

        from robotic_arm.ui.fallback_routine_panel import FallbackRoutinePanel
        from robotic_arm.application.fallback_routine_controller import FallbackRoutineController

        fallback_panel = FallbackRoutinePanel()

        # Docks
        connection_dock = self._add_dock(
            "Conexion con el brazo",
            connection_panel,
        )

        calibration_dock = self._add_dock(
            "Calibracion",
            self._scrollable(calibration_panel),
        )

        sequence_dock = self._add_dock(
            "Prueba de secuencia",
            self._scrollable(sequence_panel),
        )

        detection_dock = self._add_dock(
            "Detección YOLO",
            self._scrollable(detection_panel),
        )

        fallback_dock = self._add_dock(
            "Rutina Demo (Fallback)",
            self._scrollable(fallback_panel),
        )

        self.tabifyDockWidget(connection_dock, calibration_dock)
        self.tabifyDockWidget(calibration_dock, sequence_dock)
        self.tabifyDockWidget(sequence_dock, detection_dock)
        self.tabifyDockWidget(detection_dock, fallback_dock)
        detection_dock.raise_()

        # Fallback Routine Controller
        self._fallback_controller = FallbackRoutineController(
            arm_service=arm_service,
            parent=self,
        )
        fallback_panel.start_requested.connect(self._fallback_controller.start_routine)
        fallback_panel.stop_requested.connect(self._fallback_controller.stop_routine)
        self._fallback_controller.routine_state_changed.connect(fallback_panel.set_running)
        self._fallback_controller.step_progress.connect(fallback_panel.set_step_progress)
        self._fallback_controller.cycle_completed.connect(fallback_panel.set_cycle_count)
        self._fallback_controller.status_message.connect(self.statusBar().showMessage)

        # Controllers
        self._connection_controller = ConnectionController(
            connection_panel,
            arm_service,
            port_scanner,
            parent=self,
        )

        from robotic_arm.ui.dual_camera_controller import DualCameraController

        self._camera_controller = DualCameraController(
            camera_menu,
            dual_video,
            camera_scanner,
            camera_factory,
            parent=self,
        )

        self._calibration_controller = CalibrationController(
            calibration_panel,
            calibration_service,
            motion_planner,
            video_view,
            ZoneOverlay(),
            CalibrationOverlay(),
            parent=self,
        )

        self._sequence_controller = SequenceController(
            sequence_panel,
            arm_service,
            motion_planner,
            calibration_service,
            parent=self,
        )

        # Detection controller (only if a detector is provided)
        self._detection_controller: DetectionController | None = None
        if detector is not None:
            detection_overlay = DetectionOverlay()
            self._detection_controller = DetectionController(
                detection_panel,
                detector,
                calibration_service,
                motion_planner,
                arm_service,
                video_view,
                detection_overlay,
                parent=self,
            )
            # Conectar frames cenital (YOLO) y lateral (Z)
            self._camera_controller.top_frame_available.connect(
                self._detection_controller.on_frame_available
            )
            self._camera_controller.side_frame_available.connect(
                self._detection_controller.on_side_frame_available
            )

        # Transmisión Web (WebSocket hacia Django en Render)
        from robotic_arm.adapters.web_streamer import WebStreamer
        self._web_streamer = WebStreamer(parent=self)
        detection_panel.web_stream_toggled.connect(
            lambda active: self._web_streamer.start() if active else self._web_streamer.stop()
        )
        self._web_streamer.status_changed.connect(
            detection_panel.set_web_stream_status
        )
        self._web_streamer.streaming_toggled.connect(
            detection_panel.set_web_streaming
        )
        # Bifurcar frames cenitales al streamer (cero impacto en YOLO)
        self._camera_controller.top_frame_available.connect(
            self._web_streamer.send_frame
        )

        self._calibration_controller.test_point_changed.connect(
            self._sequence_controller.set_test_point
        )

        # Status bar
        self._camera_info = QLabel()
        self._cursor_info = QLabel()

        self.statusBar().addPermanentWidget(self._cursor_info)
        self.statusBar().addPermanentWidget(self._camera_info)

        self._camera_controller.camera_info.connect(
            self._camera_info.setText
        )

        self._calibration_controller.cursor_info.connect(
            self._cursor_info.setText
        )

        status_controllers = [
            self._connection_controller,
            self._camera_controller,
            self._calibration_controller,
        ]
        if self._detection_controller is not None:
            status_controllers.append(self._detection_controller)

        for controller in status_controllers:
            controller.status_message.connect(
                self.statusBar().showMessage
            )

        for controller in (
            self._connection_controller,
            self._camera_controller,
        ):
            controller.error_occurred.connect(self._show_error)

        self._connection_controller.refresh_ports()
        self._camera_controller.refresh_cameras()

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802 - Qt naming
        if hasattr(self, "_fallback_controller") and self._fallback_controller is not None:
            self._fallback_controller.stop_routine()
        if hasattr(self, "_web_streamer") and self._web_streamer is not None:
            self._web_streamer.stop()
        if self._detection_controller is not None:
            self._detection_controller.stop()
        self._camera_controller.stop()
        self._connection_controller.shutdown()
        super().closeEvent(event)

    def _add_dock(self, title: str, widget: QWidget) -> QDockWidget:
        dock = QDockWidget(title, self)

        dock.setWidget(widget)

        dock.setFeatures(
            QDockWidget.DockWidgetFeature.DockWidgetMovable
            | QDockWidget.DockWidgetFeature.DockWidgetFloatable
        )

        self.addDockWidget(
            Qt.DockWidgetArea.RightDockWidgetArea,
            dock,
        )

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