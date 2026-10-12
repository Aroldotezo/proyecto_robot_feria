from dataclasses import dataclass

from robotic_arm.domain.camera.detection import Detection
from robotic_arm.domain.geometry import ArmPoint, NormalizedPoint


@dataclass(frozen=True, slots=True)
class DetectionInfo:
    """All computed information about a single detection for display and tracking."""

    detection: Detection
    normalized_center: NormalizedPoint
    arm_point: ArmPoint | None
    destination_label: str | None
    category_id: str | None
    is_tracked: bool = False
