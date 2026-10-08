from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class DropAngleCheck:
    """Compares the taught S1 of a destination with the S1 the calibration gives for its zone center."""

    zone_label: str
    category_id: str
    configured: int | None  # taught on the real arm; None when the category has no taught value
    computed: int | None    # from the zone center through the calibration; None if not calibrated or out of range