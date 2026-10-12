"""Controlador para manejar dos cámaras en paralelo (Cenital + Lateral) en hilos separados."""

from collections.abc import Callable

from PySide6.QtCore import QObject, QThreadPool, Signal, Slot

from robotic_arm.domain.camera.camera_info import CameraInfo
from robotic_arm.ports.camera_scanner import CameraScanner
from robotic_arm.ports.frame_source import FrameSource
from robotic_arm.ui.camera_menu import CameraMenu
from robotic_arm.ui.dual_video_widget import DualVideoWidget
from robotic_arm.ui.frame_stream import FrameStream
from robotic_arm.ui.worker import Worker


class DualCameraController(QObject):
    """Administra la captura paralela multihilo de la Cámara Cenital y la Cámara Lateral."""

    status_message = Signal(str)
    camera_info = Signal(str)
    error_occurred = Signal(str)
    top_frame_available = Signal(object)   # BGR frame de Cámara 1 (Cenital)
    side_frame_available = Signal(object)  # BGR frame de Cámara 2 (Lateral)

    def __init__(
        self,
        menu: CameraMenu,
        dual_view: DualVideoWidget,
        scanner: CameraScanner,
        source_factory: Callable[[int], FrameSource],
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._menu = menu
        self._dual_view = dual_view
        self._scanner = scanner
        self._source_factory = source_factory
        self._pool = QThreadPool.globalInstance()

        self._top_stream: FrameStream | None = None
        self._side_stream: FrameStream | None = None
        self._top_index: int | None = None
        self._side_index: int | None = None

        self._available_cameras: list[CameraInfo] = []

        menu.camera_selected.connect(self._on_camera_selected)
        menu.stop_requested.connect(self.stop_all)
        menu.refresh_requested.connect(self.refresh_cameras)

    # ------------------------------------------------------------------ Escaneo

    def refresh_cameras(self) -> None:
        self._menu.set_scanning(True)
        self.status_message.emit("Escaneando cámaras disponibles...")
        worker = Worker(self._scanner.scan)
        worker.signals.finished.connect(self._on_scan_finished)
        worker.signals.failed.connect(self._on_scan_failed)
        self._pool.start(worker)

    @Slot(object)
    def _on_scan_finished(self, cameras: list[CameraInfo]) -> None:
        self._menu.set_scanning(False)
        self._available_cameras = list(cameras)
        self._menu.set_cameras(cameras)
        self._update_menu_active()
        self.status_message.emit(f"{len(cameras)} cámara(s) detectada(s)")

        # Auto-conectar si hay 2 cámaras disponibles y no están iniciadas
        if len(cameras) >= 2 and self._top_stream is None and self._side_stream is None:
            self.start_dual(top_index=cameras[0].index, side_index=cameras[1].index)
        elif len(cameras) == 1 and self._top_stream is None:
            self.start_top(cameras[0].index)

    @Slot(str)
    def _on_scan_failed(self, message: str) -> None:
        self._menu.set_scanning(False)
        self.error_occurred.emit(message)

    # ------------------------------------------------------------------ Control de streaming

    def _on_camera_selected(self, index: int) -> None:
        """Si se selecciona una cámara desde el menú:
        - Si la cenital está vacía, se asigna como cenital.
        - Si ya hay cenital, se asigna como lateral.
        """
        if self._top_index is None:
            self.start_top(index)
        elif self._top_index == index:
            # Alternar o reiniciar
            self.stop_top()
        elif self._side_index is None:
            self.start_side(index)
        elif self._side_index == index:
            self.stop_side()
        else:
            # Reasignar lateral
            self.stop_side()
            self.start_side(index)

    def start_top(self, index: int) -> None:
        self.stop_top()
        stream = FrameStream(self._source_factory(index), parent=self)
        stream.frame_available.connect(self._on_top_frame)
        stream.fps_updated.connect(lambda fps, idx=index: self._dual_view.set_top_status(f"Cámara {idx} | {fps:.0f} FPS"))
        stream.failed.connect(lambda err: self.error_occurred.emit(f"Error Cámara Cenital: {err}"))
        self._top_stream = stream
        self._top_index = index
        self._dual_view.set_top_status(f"Conectando Cámara {index}...")
        self._update_menu_active()
        stream.start()

    def start_side(self, index: int) -> None:
        self.stop_side()
        stream = FrameStream(self._source_factory(index), parent=self)
        stream.frame_available.connect(self._on_side_frame)
        stream.fps_updated.connect(lambda fps, idx=index: self._dual_view.set_side_status(f"Cámara {idx} | {fps:.0f} FPS"))
        stream.failed.connect(lambda err: self.error_occurred.emit(f"Error Cámara Lateral: {err}"))
        self._side_stream = stream
        self._side_index = index
        self._dual_view.set_side_status(f"Conectando Cámara {index}...")
        self._update_menu_active()
        stream.start()

    def start_dual(self, top_index: int, side_index: int) -> None:
        self.start_top(top_index)
        self.start_side(side_index)

    def stop_top(self) -> None:
        if self._top_stream is not None:
            s, self._top_stream = self._top_stream, None
            s.stop()
        self._top_index = None
        self._dual_view.top_view.clear_frame()
        self._dual_view.set_top_status("Desconectada")
        self._update_menu_active()

    def stop_side(self) -> None:
        if self._side_stream is not None:
            s, self._side_stream = self._side_stream, None
            s.stop()
        self._side_index = None
        self._dual_view.side_view.clear_frame()
        self._dual_view.set_side_status("Desconectada")
        self._update_menu_active()

    def stop(self) -> None:
        """Alias para detener de forma limpia todos los hilos al cerrar la aplicación."""
        self.stop_all()

    def stop_all(self) -> None:
        self.stop_top()
        self.stop_side()
        self.status_message.emit("Cámaras detenidas")

    # ------------------------------------------------------------------ Recepción de frames

    @Slot()
    def _on_top_frame(self) -> None:
        if self._top_stream is None:
            return
        frame = self._top_stream.take_frame()
        if frame is not None:
            self._dual_view.top_view.show_frame(frame)
            self.top_frame_available.emit(frame)

    @Slot()
    def _on_side_frame(self) -> None:
        if self._side_stream is None:
            return
        frame = self._side_stream.take_frame()
        if frame is not None:
            self._dual_view.side_view.show_frame(frame)
            self.side_frame_available.emit(frame)

    def _update_menu_active(self) -> None:
        is_streaming = (self._top_stream is not None) or (self._side_stream is not None)
        self._menu.set_streaming(is_streaming)
        active_idx = self._top_index if self._top_index is not None else self._side_index
        self._menu.set_active(active_idx)
