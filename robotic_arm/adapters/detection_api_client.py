"""Cliente API REST para registrar detecciones YOLO en el servidor Django."""

import base64
import logging
import time

import cv2
import requests
from PySide6.QtCore import QObject, QThreadPool, Signal, Slot

from robotic_arm.domain.camera.detection import Detection
from robotic_arm.domain.object_class import ObjectClass
from robotic_arm.ui.worker import Worker

log = logging.getLogger(__name__)

DEFAULT_API_URL = "https://app-brazo-robot.onrender.com/api/detecciones/"
DEFAULT_CAMERA_ID = "CAM_01"
DEFAULT_COOLDOWN_SECONDS = 7.0  # Tiempo mínimo de espera entre envíos automáticos

# Mapeo de nombres legibles para la base de datos
_OBJECT_NAME_MAPPING = {
    "higo": "Higo",
    "lata": "Lata de aluminio",
    "lata_defectuoso": "Lata defectuosa",
}

_CATEGORY_MAPPING = {
    ObjectClass.ORGANIC: "Orgánico",
    ObjectClass.INORGANIC: "Inorgánico",
    ObjectClass.DEFECTIVE: "Defectuoso",
}


class DetectionApiClient(QObject):
    """Envía detecciones a la API REST de Django en segundo plano con cooldown anti-spam."""

    post_success = Signal(str, str, float)  # (objeto, categoria, confianza)
    post_failed = Signal(str)               # mensaje de error
    cooldown_updated = Signal(float)        # segundos restantes de cooldown

    def __init__(
        self,
        api_url: str = DEFAULT_API_URL,
        camera_id: str = DEFAULT_CAMERA_ID,
        cooldown: float = DEFAULT_COOLDOWN_SECONDS,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._api_url = api_url
        self._camera_id = camera_id
        self._cooldown = cooldown
        self._last_post_time = 0.0
        self._last_posted_object: str | None = None
        self._is_posting = False
        self._enabled = True
        self._pool = QThreadPool.globalInstance()

    @property
    def enabled(self) -> bool:
        return self._enabled

    @enabled.setter
    def enabled(self, value: bool) -> None:
        self._enabled = value

    @property
    def cooldown(self) -> float:
        return self._cooldown

    @cooldown.setter
    def cooldown(self, value: float) -> None:
        self._cooldown = max(1.0, value)

    def can_post(self) -> bool:
        """Verifica si ha pasado el tiempo de espera (cooldown) y no hay otra petición en vuelo."""
        if not self._enabled or self._is_posting:
            return False
        return (time.time() - self._last_post_time) >= self._cooldown

    def try_log_detection(self, detection: Detection, frame=None) -> bool:
        """Intenta enviar una detección si el cooldown lo permite. Retorna True si se inició el envío."""
        if not self.can_post():
            return False

        self._is_posting = True
        self._last_post_time = time.time()

        # Determinar nombre y categoría
        raw_label = (detection.label or "").lower()
        objeto_nombre = _OBJECT_NAME_MAPPING.get(raw_label, raw_label.capitalize() or "Objeto detectado")
        categoria_nombre = _CATEGORY_MAPPING.get(detection.clase, "Inorgánico")
        confianza = round(float(detection.reliability), 2)

        # Generar recorte o captura base64 si hay frame disponible
        imagen_b64: str | None = None
        if frame is not None:
            try:
                h_img, w_img = frame.shape[:2]
                margin = 15
                x1 = max(0, detection.x - margin)
                y1 = max(0, detection.y - margin)
                x2 = min(w_img, detection.x + detection.width + margin)
                y2 = min(h_img, detection.y + detection.height + margin)

                if (x2 - x1) > 10 and (y2 - y1) > 10:
                    crop = frame[y1:y2, x1:x2]
                    # Reducir si es muy grande
                    ch, cw = crop.shape[:2]
                    if cw > 320:
                        crop = cv2.resize(crop, (320, int(ch * (320.0 / cw))))
                    success, buffer = cv2.imencode(".jpg", crop, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                    if success:
                        b64_data = base64.b64encode(buffer).decode("utf-8")
                        imagen_b64 = f"data:image/jpeg;base64,{b64_data}"
            except Exception as e:
                log.warning("No se pudo generar recorte de imagen: %s", e)

        payload = {
            "objeto": objeto_nombre,
            "categoria": categoria_nombre,
            "confianza": confianza,
            "camara_id": self._camera_id,
        }
        if imagen_b64:
            payload["imagen_base64"] = imagen_b64

        # Ejecutar POST en hilo de fondo Worker
        worker = Worker(lambda: self._do_post(payload, objeto_nombre, categoria_nombre, confianza))
        worker.signals.finished.connect(self._on_post_finished)
        worker.signals.failed.connect(self._on_post_failed)
        self._pool.start(worker)
        return True

    def _do_post(self, payload: dict, objeto: str, categoria: str, confianza: float) -> tuple[str, str, float]:
        """Ejecutado en segundo plano en el hilo del Worker."""
        log.info("Enviando POST de detección a %s: %s (%s)", self._api_url, objeto, categoria)
        response = requests.post(self._api_url, json=payload, timeout=8)
        if response.status_code in (200, 201):
            log.info("Detección registrada con éxito: HTTP %d", response.status_code)
            return (objeto, categoria, confianza)
        else:
            raise RuntimeError(f"HTTP {response.status_code}: {response.text[:120]}")

    @Slot(object)
    def _on_post_finished(self, result: object) -> None:
        self._is_posting = False
        if isinstance(result, tuple) and len(result) == 3:
            objeto, categoria, confianza = result
            self._last_posted_object = objeto
            self.post_success.emit(objeto, categoria, confianza)

    @Slot(str)
    def _on_post_failed(self, error_message: str) -> None:
        self._is_posting = False
        log.warning("Fallo al enviar detección a la API: %s", error_message)
        self.post_failed.emit(error_message)
