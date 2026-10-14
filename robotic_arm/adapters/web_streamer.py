"""Servicio de transmisión de video en tiempo real hacia el dashboard web vía WebSocket."""

import base64
import json
import logging
import time

import cv2
import numpy as np
from PySide6.QtCore import QObject, QTimer, QUrl, Signal, Slot
from PySide6.QtWebSockets import QWebSocket

log = logging.getLogger(__name__)

DEFAULT_WS_URL = "wss://app-brazo-robot.onrender.com/ws/camera/CAM_01/"
DEFAULT_CAMERA_ID = "CAM_01"


class WebStreamer(QObject):
    """Bifurca los frames de la cámara y los transmite comprimidos en JPEG hacia el servidor Django vía WebSocket."""

    status_changed = Signal(str)       # Mensaje de estado ("Transmitiendo...", "Desconectado", etc.)
    streaming_toggled = Signal(bool)   # Estado activo/inactivo
    fps_updated = Signal(float)        # FPS reales de transmisión hacia la nube
    error_occurred = Signal(str)

    def __init__(
        self,
        ws_url: str = DEFAULT_WS_URL,
        camera_id: str = DEFAULT_CAMERA_ID,
        target_fps: int = 12,
        jpeg_quality: int = 65,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._ws_url = ws_url
        self._camera_id = camera_id
        self._target_fps = target_fps
        self._min_interval = 1.0 / max(1, target_fps)
        self._jpeg_quality = jpeg_quality

        self._enabled = False
        self._connected = False
        self._last_send_time = 0.0
        self._frames_sent = 0
        self._fps_timer_start = time.time()

        self._ws = QWebSocket()
        self._ws.connected.connect(self._on_ws_connected)
        self._ws.disconnected.connect(self._on_ws_disconnected)
        self._ws.textMessageReceived.connect(self._on_ws_message)
        self._ws.errorOccurred.connect(self._on_ws_error)

        # Timer de reconexión automática si está habilitado
        self._reconnect_timer = QTimer(self)
        self._reconnect_timer.setInterval(5000)
        self._reconnect_timer.timeout.connect(self._try_reconnect)

    @property
    def is_streaming(self) -> bool:
        return self._enabled and self._connected

    @property
    def ws_url(self) -> str:
        return self._ws_url

    def set_ws_url(self, url: str) -> None:
        self._ws_url = url.strip()
        if self._enabled:
            self.stop()
            self.start()

    def start(self) -> None:
        """Inicia la conexión WebSocket y la transmisión."""
        self._enabled = True
        self.streaming_toggled.emit(True)
        self.status_changed.emit("Conectando con servidor web...")
        log.info("Iniciando transmisión Web hacia %s", self._ws_url)
        self._connect_socket()

    def stop(self) -> None:
        """Detiene la transmisión y cierra el socket."""
        self._enabled = False
        self._connected = False
        self._reconnect_timer.stop()
        self._ws.close()
        self.streaming_toggled.emit(False)
        self.status_changed.emit("Streaming web detenido")
        log.info("Transmisión Web detenida")

    def toggle(self) -> None:
        if self._enabled:
            self.stop()
        else:
            self.start()

    def _connect_socket(self) -> None:
        if self._ws.isValid():
            self._ws.close()
        self._ws.open(QUrl(self._ws_url))

    def _try_reconnect(self) -> None:
        if self._enabled and not self._connected:
            log.info("Reintentando conexión WebSocket...")
            self.status_changed.emit("Reintentando conexión web...")
            self._connect_socket()

    @Slot()
    def _on_ws_connected(self) -> None:
        self._connected = True
        self._reconnect_timer.stop()
        log.info("WebSocket conectado exitosamente con %s", self._ws_url)
        self.status_changed.emit(f"En vivo: Transmitiendo ({self._camera_id})")

    @Slot()
    def _on_ws_disconnected(self) -> None:
        self._connected = False
        log.warning("WebSocket desconectado del servidor")
        if self._enabled:
            self.status_changed.emit("Conexión perdida. Reconectando...")
            self._reconnect_timer.start()
        else:
            self.status_changed.emit("Streaming desconectado")

    @Slot(str)
    def _on_ws_message(self, message: str) -> None:
        log.debug("Mensaje recibido del WebSocket: %s", message[:100])

    @Slot(object)
    def _on_ws_error(self, error) -> None:
        err_str = self._ws.errorString()
        log.error("Error en WebSocket: %s (%s)", error, err_str)
        self.error_occurred.emit(f"Error WebSocket: {err_str}")
        if self._enabled:
            self.status_changed.emit("Error de conexión web. Reintentando...")
            self._reconnect_timer.start()

    @Slot(object)
    def send_frame(self, frame: object) -> None:
        """Recibe un frame BGR de OpenCV, lo comprime a JPEG y lo envía vía WebSocket.
        Aplica throttling inteligente para respetar el target FPS y no saturar el canal.
        """
        if not self._enabled or not self._connected or frame is None:
            return

        now = time.time()
        if now - self._last_send_time < self._min_interval:
            return  # Throttling por FPS

        self._last_send_time = now

        try:
            # Reducir resolución si es muy grande (máx 640px de ancho)
            h, w = frame.shape[:2]
            if w > 640:
                scale = 640.0 / w
                frame_to_send = cv2.resize(frame, (640, int(h * scale)), interpolation=cv2.INTER_AREA)
            else:
                frame_to_send = frame

            # Compresión JPEG optimizada
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
            success, buffer = cv2.imencode(".jpg", frame_to_send, encode_param)
            if not success:
                return

            b64_str = base64.b64encode(buffer).decode("utf-8")
            data_url = f"data:image/jpeg;base64,{b64_str}"

            payload = json.dumps({
                "type": "video_frame",
                "camera_id": self._camera_id,
                "image": data_url,
                "frame": data_url,
            })

            self._ws.sendTextMessage(payload)

            self._frames_sent += 1
            elapsed = now - self._fps_timer_start
            if elapsed >= 2.0:
                fps = self._frames_sent / elapsed
                self.fps_updated.emit(fps)
                self._frames_sent = 0
                self._fps_timer_start = now

        except Exception as e:
            log.warning("Error procesando/enviando frame a WebSocket: %s", e)
