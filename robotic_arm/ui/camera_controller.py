from collections.abc import Callable

from PySide6.QtCore import QObject, QThreadPool, Signal, Slot

from robotic_arm.ports.camera_scanner import CameraScanner
from robotic_arm.ports.frame_source import FrameSource
from robotic_arm.ui.camera_menu import CameraMenu
from robotic_arm.ui.frame_stream import FrameStream
from robotic_arm.ui.video_view import VideoView
from robotic_arm.ui.worker import Worker


class CameraController(QObject):
    status_message = Signal(str)
    camera_info = Signal(str)
    error_occurred = Signal(str)
    new_frame = Signal(object)  # emits the BGR frame ndarray

    def __init__(
        self,
        menu: CameraMenu,
        view: VideoView,
        scanner: CameraScanner,
        source_factory: Callable[[int], FrameSource],
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._menu = menu
        self._view = view
        self._scanner = scanner
        self._source_factory = source_factory
        self._pool = QThreadPool.globalInstance()
        self._stream: FrameStream | None = None
        self._active_index: int | None = None
        self._frame_size: tuple[int, int] | None = None

        menu.camera_selected.connect(self._select_camera)
        menu.stop_requested.connect(self.stop)
        menu.refresh_requested.connect(self.refresh_cameras)

    def refresh_cameras(self) -> None:
        self._menu.set_scanning(True)
        self.status_message.emit("Buscando camaras disponibles...")
        worker = Worker(self._scanner.scan)
        worker.signals.finished.connect(self._on_scan_finished)
        worker.signals.failed.connect(self._on_scan_failed)
        self._pool.start(worker)

    def stop(self) -> None:
        stream, self._stream = self._stream, None
        if stream is None:
            return
        stream.stop()
        self._clear_state()
        self.status_message.emit("Camara detenida")

    # ------------------------------------------------------------------ scanning

    @Slot(object)
    def _on_scan_finished(self, cameras: object) -> None:
        self._menu.set_scanning(False)
        self._menu.set_cameras(cameras) 
        self._menu.set_active(self._active_index)
        self.status_message.emit(f"{len(cameras)} camara(s) encontradas") 

    @Slot(str)
    def _on_scan_failed(self, message: str) -> None:
        self._menu.set_scanning(False)
        self.error_occurred.emit(message)

    # ------------------------------------------------------------------ streaming

    def _select_camera(self, index: int) -> None:
        if index == self._active_index and self._stream is not None:
            self._menu.set_active(index) 
            return
        self.stop()
        self._start(index)

    def _start(self, index: int) -> None:
        stream = FrameStream(self._source_factory(index), parent=self)
        stream.frame_available.connect(self._on_frame_available)
        stream.fps_updated.connect(self._on_fps_updated)
        stream.failed.connect(self._on_stream_failed)
        stream.finished.connect(self._on_stream_finished)

        self._stream = stream
        self._active_index = index
        self._frame_size = None
        self._menu.set_active(index)
        self._menu.set_streaming(True)
        self.status_message.emit(f"Abriendo camara {index}...")
        stream.start()

    @property
    def stream(self) -> FrameStream | None:
        return self._stream

    @Slot()
    def _on_frame_available(self) -> None:
        if self._stream is None:
            return
        frame = self._stream.take_frame()
        if frame is not None:
            self._frame_size = (frame.shape[1], frame.shape[0])
            self._view.show_frame(frame)
            self.new_frame.emit(frame)

    @Slot(float)
    def _on_fps_updated(self, fps: float) -> None:
        if self.sender() is not self._stream or self._frame_size is None:
            return
        width, height = self._frame_size
        self.camera_info.emit(f"Camara {self._active_index} | {width}x{height} | {fps:.0f} fps")

    @Slot(str)
    def _on_stream_failed(self, message: str) -> None:
        if self.sender() is self._stream:
            self.error_occurred.emit(message)

    @Slot()
    def _on_stream_finished(self) -> None:
        stream = self.sender()
        if stream is self._stream: 
            self._stream = None
            self._clear_state()
            self.status_message.emit("Camara detenida")
        stream.deleteLater()

    def _clear_state(self) -> None:
        self._active_index = None
        self._frame_size = None
        self._view.clear_frame()
        self._menu.set_active(None)
        self._menu.set_streaming(False)
        self.camera_info.emit("")