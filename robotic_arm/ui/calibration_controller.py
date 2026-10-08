from PySide6.QtCore import QObject, Signal

from robotic_arm.application.calibration_service import CalibrationService
from robotic_arm.application.motion_planner import MotionPlanner
from robotic_arm.domain.arm_heights import ArmHeights
from robotic_arm.domain.arm_space import ArmPoint
from robotic_arm.domain.exceptions.errors import CalibrationException
from robotic_arm.domain.geometry import NormalizedPoint, NormalizedRect
from robotic_arm.ui.calibration_overlay import CalibrationOverlay
from robotic_arm.ui.calibration_panel import CalibrationPanel
from robotic_arm.ui.video_view import VideoView
from robotic_arm.ui.zone_overlay import ZoneOverlay


class CalibrationController(QObject):
    status_message = Signal(str)
    cursor_info = Signal(str)
    test_point_changed = Signal(object, str)  # NormalizedPoint, readable description

    def __init__(
        self,
        panel: CalibrationPanel,
        service: CalibrationService,
        planner: MotionPlanner,
        video_view: VideoView,
        zone_overlay: ZoneOverlay,
        calibration_overlay: CalibrationOverlay,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._panel = panel
        self._service = service
        self._planner = planner
        self._video_view = video_view
        self._zone_overlay = zone_overlay
        self._calibration_overlay = calibration_overlay
        self._adding_point = False
        self._test_point: NormalizedPoint | None = None

        video_view.add_overlay(zone_overlay)
        video_view.add_overlay(calibration_overlay)
        video_view.image_clicked.connect(self._on_image_clicked)
        video_view.image_hovered.connect(self._on_image_hovered)
        panel.add_point_toggled.connect(self._on_add_point_toggled)
        panel.remove_point_requested.connect(self._on_remove_point)
        panel.clear_points_requested.connect(self._on_clear_points)
        panel.arm_point_edited.connect(self._on_arm_point_edited)
        panel.heights_changed.connect(self._on_heights_changed)
        panel.preview_requested.connect(self._on_preview_requested)
        panel.zone_region_changed.connect(self._on_zone_region_changed)
        panel.zone_selected.connect(self._on_zone_selected)
        panel.reset_zones_requested.connect(self._on_reset_zones)

        self.reload()

    def reload(self) -> None:
        """Pushes the active profile into the panel and overlays (call it after switching profile)."""
        profile = self._service.profile
        self._zone_overlay.set_zones(profile.zones)
        self._panel.set_zones(profile.zones)
        self._zone_overlay.set_selected_zone(self._panel.selected_zone_id)
        self._panel.set_destinations([(zone.label, zone.category_id) for zone in profile.destination_zones])
        self._panel.set_heights(profile.heights)
        self._refresh_points()

    # ------------------------------------------------------------------ video

    def _on_image_clicked(self, point: NormalizedPoint) -> None:
        if self._adding_point:
            self._adding_point = False
            self._panel.set_adding_point(False)
            self._service.add_reference_point(point, ArmPoint(0.0, 0.0))
            self._refresh_points()
            self.status_message.emit("Point added: type its arm coordinates (mm) in the table")
            return
        self._test_point = point
        self.test_point_changed.emit(point, self._describe(point))
        self._calibration_overlay.set_test_point(point)
        self._video_view.refresh()
        self._panel.set_test_point_text(f"Test point: {self._describe(point)}")

    def _on_image_hovered(self, point: NormalizedPoint | None) -> None:
        self.cursor_info.emit("" if point is None else self._describe(point))

    # ------------------------------------------------------------------ reference points

    def _on_add_point_toggled(self, active: bool) -> None:
        self._adding_point = active
        if active:
            self.status_message.emit("Click on the video to place the reference point")

    def _on_remove_point(self, row: int) -> None:
        self._service.remove_reference_point(row)
        self._refresh_points()

    def _on_clear_points(self) -> None:
        self._service.clear_reference_points()
        self._refresh_points()

    def _on_arm_point_edited(self, row: int, x: float, y: float) -> None:
        self._service.update_reference_arm_point(row, ArmPoint(x, y))
        self._refresh_status() 

    # ------------------------------------------------------------------ heights and preview

    def _on_heights_changed(self, z_safe: float, z_pick: float, z_drop: float) -> None:
        try:
            self._service.set_heights(ArmHeights(z_safe=z_safe, z_pick=z_pick, z_drop=z_drop))
        except ValueError as e:
            self._panel.set_heights(self._service.profile.heights)
            self.status_message.emit(f"Heights not applied: {e}")

    def _on_preview_requested(self, category_id: str) -> None:
        if self._test_point is None:
            self._panel.show_plan_message("Click on the video to choose a test point first.")
            return
        try:
            steps = self._planner.plan_pick_and_place(self._test_point, category_id)
        except (ValueError, CalibrationException) as e:
            self._panel.show_plan_message(f"No se puede evaluar: {e}")
            return
        self._panel.show_plan(steps)

    def _on_zone_region_changed(self, zone_id: str, x: float, y: float, w: float, h: float) -> None:
        try:
            rect = NormalizedRect(x, y, w, h)
            self._service.update_zone_region(zone_id, rect)
            profile = self._service.profile
            self._zone_overlay.set_zones(profile.zones)
            self._panel.set_zones(profile.zones)
            self._video_view.refresh()
            self.status_message.emit(f"Recuadro '{zone_id}' actualizado: ({x:.3f}, {y:.3f}, {w:.3f}, {h:.3f})")
        except ValueError as e:
            self.status_message.emit(f"Dimensiones no válidas: {e}")

    def _on_zone_selected(self, zone_id: str) -> None:
        self._zone_overlay.set_selected_zone(zone_id)
        self._video_view.refresh()

    def _on_reset_zones(self) -> None:
        self._service.reset_default_zones()
        self.reload()
        self._video_view.refresh()
        self.status_message.emit("Recuadros restablecidos a los valores por defecto")

    # ------------------------------------------------------------------ helpers

    def _refresh_points(self) -> None:
        points = self._service.profile.reference_points
        self._panel.set_reference_points(points)
        self._calibration_overlay.set_reference_points([point.image for point in points])
        self._refresh_status()
        self._video_view.refresh()

    def _refresh_status(self) -> None:
        status = self._service.status()
        self._panel.set_status(status.message, ok=status.calibrated)

    def _describe(self, point: NormalizedPoint) -> str:
        zone = self._service.profile.zone_at(point)
        zone_text = zone.label if zone else "outside the zones"
        try:
            arm = self._service.image_to_arm(point)
        except CalibrationException:
            return f"imagen ({point.x:.3f}, {point.y:.3f}) | {zone_text} | no calibrada"
        return f"Arm X {arm.x:.1f}  Y {arm.y:.1f} mm | {zone_text}"