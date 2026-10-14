"""Controlador que integra el pipeline completo:
frame → YOLO → centro → normalización → calibración → ArmPoint → tracking → brazo.
"""

import logging

from PySide6.QtCore import QObject, Signal, Slot

from robotic_arm.application.arm_service import ArmService
from robotic_arm.application.calibration_service import CalibrationService
from robotic_arm.application.motion_planner import MotionPlanner
from robotic_arm.domain.arm_space import ArmPoint
from robotic_arm.domain.camera.detection import Detection
from robotic_arm.domain.exceptions.errors import (
    CalibrationException,
    JointLimitException,
)
from robotic_arm.domain.geometry import NormalizedPoint
from robotic_arm.domain.object_class import ObjectClass
from robotic_arm.ports.detector import Detector
from robotic_arm.ui.detection_overlay import DetectionInfo, DetectionOverlay
from robotic_arm.ui.detection_panel import DetectionPanel
from robotic_arm.ui.video_view import VideoView
from robotic_arm.ui.worker import Worker

log = logging.getLogger(__name__)

# Mapping from ObjectClass.value to category_id used in ClassificationProfile zones
_OBJECT_CLASS_TO_CATEGORY: dict[ObjectClass, str] = {
    ObjectClass.ORGANIC: "organic",
    ObjectClass.INORGANIC: "inorganic",
    ObjectClass.DEFECTIVE: "defective",
}


class DetectionController(QObject):
    """Connects YOLO detection with calibration, tracking, and arm movement.

    The full pipeline runs in a background thread to avoid blocking Qt.
    Results are sent back to the main thread via signals.
    """

    status_message = Signal(str)

    def __init__(
        self,
        panel: DetectionPanel,
        detector: Detector,
        calibration_service: CalibrationService,
        motion_planner: MotionPlanner,
        arm_service: ArmService,
        video_view: VideoView,
        overlay: DetectionOverlay,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._panel = panel
        self._detector = detector
        self._calibration = calibration_service
        self._planner = motion_planner
        self._arm_service = arm_service
        self._video_view = video_view
        self._overlay = overlay

        self._arm_tracking_enabled = False
        self._arm_busy = False  # True while the arm is executing a pick-and-place
        self._processing = False  # True while a detection cycle is running

        # Tracked target — the detection we are following
        self._tracked: DetectionInfo | None = None

        # Latest detections for display
        self._latest_infos: list[DetectionInfo] = []

        video_view.add_overlay(overlay)

        panel.arm_tracking_toggled.connect(self._on_arm_tracking_toggled)
        panel.confidence_changed.connect(self._on_confidence_changed)
        panel.manual_pick_requested.connect(self._on_manual_pick_requested)
        panel.servo_debug_requested.connect(self._on_servo_debug)
        panel.servo_grasp_test_requested.connect(self._on_grasp_test)
        panel.servo_drop_test_requested.connect(self._on_drop_test)

        self._latest_z: float = 15.0  # Altura por defecto en mm sobre la mesa

    # -------------------------------------------------------------- public

    def on_side_frame_available(self, frame) -> None:
        """Procesa frame de cámara lateral para estimar altura Z del objeto sobre la mesa."""
        try:
            import cv2
            import numpy as np
            # Detección simple y veloz de silueta por contraste sobre fondo
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 400:
                    x, y, w, h = cv2.boundingRect(c)
                    # Relación de altura en píxeles a mm (480 px ≈ 240 mm → ~0.5 mm/px)
                    self._latest_z = max(5.0, min(80.0, float(h * 0.45)))
        except Exception:
            pass

    def on_frame_available(self, frame) -> None:
        """Called from the camera controller when a new frame is available.
        If a detection cycle is already running, the frame is dropped (latest-only semantics).
        """
        if self._processing:
            return
        self._processing = True
        # Run detection in background thread via Worker
        worker = Worker(lambda f=frame: self._detect_pipeline(f))
        worker.signals.finished.connect(self._on_pipeline_done)
        worker.signals.failed.connect(self._on_pipeline_failed)
        from PySide6.QtCore import QThreadPool
        QThreadPool.globalInstance().start(worker)

    def _on_manual_pick_requested(self) -> None:
        """Fuerza la ejecución inmediata de Pick & Place si hay un objeto detectado."""
        if self._arm_busy:
            self.status_message.emit("El brazo ya está en movimiento...")
            return
        target = self._tracked or (self._latest_infos[0] if self._latest_infos else None)
        if target is None:
            self.status_message.emit("No hay ningún objeto detectado para recoger.")
            return
        if not self._arm_service.connected:
            self.status_message.emit("Conecta el brazo antes de ejecutar Pick & Place.")
            return
        self.status_message.emit("Iniciando Pick & Place manual...")
        self._execute_pick_and_place(target)

    def _on_servo_debug(self) -> None:

        """Debug físico: solo S2, 90 -> 70 -> 50 -> 90, con los demás servos a 90°.
        Si S2 decrece, el brazo debe inclinarse hacia adelante y BAJAR."""
        if self._arm_busy:
            self.status_message.emit("⚠ El brazo ya está ocupado, espera a que termine.")
            return
        if not self._arm_service.connected:
            self.status_message.emit("⚠ Conecta el brazo primero (debug requiere conexión).")
            return
        self._arm_busy = True
        self.status_message.emit("🔧 Debug S2: 90° → 70° → 50° → 90°. Observa si el brazo baja o sube.")

        def _run_debug() -> None:
            import time
            from robotic_arm.domain.camera.arm_position import ArmPosition
            arm = self._arm_service._require_arm()
            for s2 in (90, 70, 50, 90):
                arm.move_to(ArmPosition((90, s2, 90, 90, 90, 90)))
                time.sleep(2.5)

        worker = Worker(_run_debug)
        worker.signals.finished.connect(self._on_debug_finished)
        worker.signals.failed.connect(self._on_arm_failed)
        from PySide6.QtCore import QThreadPool
        QThreadPool.globalInstance().start(worker)

    def _on_grasp_test(self) -> None:
        """Prueba física de GRASP: sobre el objeto -> bajar en vertical (IK) -> volver arriba, a r = 130 mm."""
        if self._arm_busy:
            self.status_message.emit("⚠ El brazo ya está ocupado, espera a que termine.")
            return
        if not self._arm_service.connected:
            self.status_message.emit("⚠ Conecta el brazo primero para probar posiciones.")
            return
        radius, grasp_z = 130.0, 20.0
        try:
            grasp, hover = self._planner.approach_stages(radius, grasp_z)
        except ValueError as e:
            self.status_message.emit(f"⚠ No se puede calcular el grasp de prueba: {e}")
            return
        self._arm_busy = True
        opened = self._planner.config.gripper_open_angle
        hover_pose = self._planner.build_pose(90, hover, opened)
        grasp_pose = self._planner.build_pose(90, grasp, opened)
        self.status_message.emit(
            f"⤵ Probando GRASP (r={radius:.0f} mm): arriba S2/S3/S4 = {hover.s2}/{hover.s3}/{hover.s4}° "
            f"→ abajo {grasp.s2}/{grasp.s3}/{grasp.s4}°"
        )

        def _run_grasp() -> None:
            import time
            arm = self._arm_service._require_arm()
            arm.move_to(hover_pose)
            time.sleep(2.0)
            arm.move_to(grasp_pose)
            time.sleep(3.0)
            arm.move_to(hover_pose)

        worker = Worker(_run_grasp)
        worker.signals.finished.connect(self._on_arm_finished)
        worker.signals.failed.connect(self._on_arm_failed)
        from PySide6.QtCore import QThreadPool
        QThreadPool.globalInstance().start(worker)

    def _on_drop_test(self) -> None:
        """Prueba física directa de la postura DROP (soltado en zona destino)."""
        if self._arm_busy:
            self.status_message.emit("⚠ El brazo ya está ocupado, espera a que termine.")
            return
        if not self._arm_service.connected:
            self.status_message.emit("⚠ Conecta el brazo primero para probar posiciones.")
            return
        self._arm_busy = True
        config = self._planner.config
        drop = config.drop
        travel = config.travel
        drop_s1 = config.drop_base_angles.get("defective", 100)
        pose = self._planner.build_pose
        closed, opened = config.gripper_closed_angle, config.gripper_open_angle
        drop_s4 = pose(drop_s1, drop, closed).angles[3]
        self.status_message.emit(f"⤴ Probando DROP (S1={drop_s1}°, S2={drop.s2}°, S3={drop.s3}°, S4={drop_s4}°)")

        def _run_drop() -> None:
            import time
            arm = self._arm_service._require_arm()
            # 1. Travel al centro de drop
            arm.move_to(pose(drop_s1, travel, closed))
            time.sleep(2.0)
            # 2. Bajar a drop
            arm.move_to(pose(drop_s1, drop, closed))
            time.sleep(2.0)
            # 3. Abrir garra
            arm.move_to(pose(drop_s1, drop, opened))
            time.sleep(1.5)
            # 4. Volver a Travel
            arm.move_to(pose(drop_s1, travel, opened))

        worker = Worker(_run_drop)
        worker.signals.finished.connect(self._on_arm_finished)
        worker.signals.failed.connect(self._on_arm_failed)
        from PySide6.QtCore import QThreadPool
        QThreadPool.globalInstance().start(worker) 

    def stop(self) -> None:
        """Clear detections when the camera stops."""
        self._overlay.clear()
        self._tracked = None
        self._latest_infos = []
        self._panel.set_detection_info("")

    # -------------------------------------------------------------- pipeline (background thread)

    def _detect_pipeline(self, frame):
        """Runs in a worker thread: detect → compute info → return results."""
        import numpy as np
        height, width = frame.shape[:2]
        detections = self._detector.detect(frame)
        infos = self._build_infos(detections, width, height)
        # Mark the best candidate as tracked
        tracked_info = self._select_tracked(infos)
        if tracked_info is not None:
            infos = [
                DetectionInfo(
                    detection=info.detection,
                    normalized_center=info.normalized_center,
                    arm_point=info.arm_point,
                    destination_label=info.destination_label,
                    category_id=info.category_id,
                    is_tracked=(info is tracked_info),
                )
                for info in infos
            ]
        return (infos, width, height, tracked_info)

    def _build_infos(
        self,
        detections: list[Detection],
        frame_width: int,
        frame_height: int,
    ) -> list[DetectionInfo]:
        """Converts raw detections into enriched DetectionInfo with calibrated coordinates."""
        profile = self._calibration.profile
        infos: list[DetectionInfo] = []

        for det in detections:
            cx, cy = det.center
            # Normalize center to 0..1 using original frame dimensions
            nx = cx / frame_width
            ny = cy / frame_height
            # Clamp to valid range
            nx = max(0.0, min(1.0, nx))
            ny = max(0.0, min(1.0, ny))

            try:
                normalized = NormalizedPoint(nx, ny)
            except ValueError:
                continue

            # Calibration: image -> arm
            arm_point: ArmPoint | None = None
            try:
                arm_point = self._calibration.image_to_arm(normalized)
            except CalibrationException:
                pass

            # Destination zone
            category_id = _OBJECT_CLASS_TO_CATEGORY.get(det.clase)
            destination_label: str | None = None
            if category_id is not None:
                try:
                    zone = profile.destination_for(category_id)
                    destination_label = zone.label
                except ValueError:
                    pass

            infos.append(
                DetectionInfo(
                    detection=det,
                    normalized_center=normalized,
                    arm_point=arm_point,
                    destination_label=destination_label,
                    category_id=category_id,
                )
            )

        return infos

    def _select_tracked(self, infos: list[DetectionInfo]) -> DetectionInfo | None:
        """Simple tracking: pick the highest-confidence detection inside the evaluation zone."""
        profile = self._calibration.profile
        eval_zone = profile.evaluation_zone

        candidates = [
            info
            for info in infos
            if eval_zone.region.contains(info.normalized_center)
            and info.arm_point is not None
            and info.category_id is not None
        ]

        if not candidates:
            return None

        # Highest confidence
        return max(candidates, key=lambda i: i.detection.reliability)

    # -------------------------------------------------------------- results (main thread)

    @Slot(object)
    def _on_pipeline_done(self, result: object) -> None:
        self._processing = False
        if result is None:
            return
        infos, frame_w, frame_h, tracked_info = result
        self._latest_infos = infos
        self._tracked = tracked_info

        # Update overlay
        self._overlay.set_detections(infos, frame_w, frame_h)
        self._video_view.refresh()

        # Update panel info
        self._panel.set_detection_info(self._format_info(infos, tracked_info))

        # Arm tracking logic
        if (
            self._arm_tracking_enabled
            and not self._arm_busy
            and tracked_info is not None
            and tracked_info.arm_point is not None
            and tracked_info.category_id is not None
        ):
            self._execute_pick_and_place(tracked_info)

    @Slot(str)
    def _on_pipeline_failed(self, message: str) -> None:
        self._processing = False
        log.error("Detection pipeline failed: %s", message)

    # -------------------------------------------------------------- arm movement

    def _execute_pick_and_place(self, info: DetectionInfo) -> None:
        """Plans and executes the pick-and-place for the tracked object.
        Safety checks: validates ArmPoint, category, calibration, and joint limits.
        The actual movement runs in a background thread.
        """
        assert info.arm_point is not None
        assert info.category_id is not None

        # Safety: verify the detection is within the evaluation zone
        profile = self._calibration.profile
        if not profile.evaluation_zone.region.contains(info.normalized_center):
            log.warning("Tracked object outside evaluation zone, skipping movement")
            return

        # Plan the sequence
        try:
            steps = self._planner.plan_pick_and_place(
                info.normalized_center, info.category_id, target_z=self._latest_z
            )
        except (ValueError, CalibrationException, JointLimitException) as e:
            log.warning("Cannot plan pick-and-place: %s", e)
            self.status_message.emit(f"Movimiento rechazado: {e}")
            return

        if not steps:
            return

        # Check arm connection
        if not self._arm_service.connected:
            log.warning("ARM TRACKING ON but arm not connected")
            self.status_message.emit("ARM TRACKING ON pero el brazo no está conectado")
            return

        # Double-check the tracking switch right before executing
        if not self._arm_tracking_enabled:
            return

        self._arm_busy = True
        self.status_message.emit(
            f"Brazo: recogiendo {info.detection.clase.value} → {info.destination_label}"
        )

        worker = Worker(lambda: self._arm_service.execute(steps))
        worker.signals.finished.connect(self._on_arm_finished)
        worker.signals.failed.connect(self._on_arm_failed)
        from PySide6.QtCore import QThreadPool
        QThreadPool.globalInstance().start(worker)

    @Slot(object)
    def _on_arm_finished(self, _result: object) -> None:
        self._arm_busy = False
        self.status_message.emit("Secuencia de brazo completada")

    @Slot(object)
    def _on_debug_finished(self, _result: object) -> None:
        self._arm_busy = False
        self.status_message.emit("🔧 Debug completado: todos los servos llegaron a 40°. Observa el resultado físico.")

    @Slot(str)
    def _on_arm_failed(self, message: str) -> None:
        self._arm_busy = False
        log.error("Arm movement failed: %s", message)
        self.status_message.emit(f"Error del brazo: {message}")

    # -------------------------------------------------------------- tracking switch

    def _on_arm_tracking_toggled(self, enabled: bool) -> None:
        self._arm_tracking_enabled = enabled
        state = "ON" if enabled else "OFF"
        log.info("ARM TRACKING = %s", state)
        self.status_message.emit(f"ARM TRACKING = {state}")

    def _on_confidence_changed(self, value: float) -> None:
        if hasattr(self._detector, "confidence"):
            self._detector.confidence = value
            log.info("YOLO confidence threshold set to %.2f", value)

    # -------------------------------------------------------------- formatting

    def _format_info(
        self,
        infos: list[DetectionInfo],
        tracked: DetectionInfo | None,
    ) -> str:
        if not infos:
            return "Sin detecciones"

        lines: list[str] = []
        lines.append(f"Detecciones: {len(infos)}")
        lines.append(f"ARM TRACKING: {'ON' if self._arm_tracking_enabled else 'OFF'}")
        if self._arm_busy:
            lines.append("⚙ Brazo en movimiento...")
        lines.append("")

        for i, info in enumerate(infos, start=1):
            det = info.detection
            marker = " ★ TRACKED" if info.is_tracked else ""
            lines.append(f"--- Detección {i}{marker} ---")
            lines.append(f"  Clase:       {det.clase.value}")
            lines.append(f"  Confianza:   {det.reliability:.1%}")
            cx, cy = det.center
            lines.append(f"  Centro (px): ({cx}, {cy})")
            nc = info.normalized_center
            lines.append(f"  Normalizado: ({nc.x:.4f}, {nc.y:.4f})")
            if info.arm_point is not None:
                lines.append(
                    f"  ArmPoint:    ({info.arm_point.x:.1f}, {info.arm_point.y:.1f}) mm"
                )
            else:
                lines.append("  ArmPoint:    no calibrado")
            if info.destination_label:
                lines.append(f"  Destino:     {info.destination_label}")
            else:
                lines.append("  Destino:     no determinado")
            lines.append("")

        return "\n".join(lines)
