from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray

from robotic_arm.domain.arm_space import ArmPoint
from robotic_arm.domain.exceptions.errors import CalibrationException
from robotic_arm.domain.geometry import NormalizedPoint
from robotic_arm.domain.reference_point import ReferencePoint

MIN_REFERENCE_POINTS = 4
_DEGENERACY_TOLERANCE = 1e-6


class PlaneCalibration:
    """Mapeo de las posiciones de la imagen donde se define el espacio de trabajo."""

    def __init__(self, matrix: NDArray[np.float64], rms_error: float) -> None:
        self._matrix = matrix
        self._rms_error = rms_error

    @property
    def rms_error(self) -> float:
        """Average distance (mm) between where the reference points land and where they should."""
        return self._rms_error

    @classmethod
    def from_reference_points(cls, points: Sequence[ReferencePoint]) -> "PlaneCalibration":
        if len(points) < MIN_REFERENCE_POINTS:
            raise CalibrationException(f"At least {MIN_REFERENCE_POINTS} reference points are required, got {len(points)}")
        image = np.array([[p.image.x, p.image.y] for p in points], dtype=float)
        arm = np.array([[p.arm.x, p.arm.y] for p in points], dtype=float)

        matrix = _fit_homography(image, arm)
        projected = _project(matrix, image)
        rms_error = float(np.sqrt(np.mean(np.sum((projected - arm) ** 2, axis=1))))
        return cls(matrix, rms_error)

    def to_arm(self, point: NormalizedPoint) -> ArmPoint:
        x, y = _project(self._matrix, np.array([[point.x, point.y]], dtype=float))[0]
        return ArmPoint(float(x), float(y))


def _project(matrix: NDArray[np.float64], points: NDArray[np.float64]) -> NDArray[np.float64]:
    homogeneous = np.hstack([points, np.ones((len(points), 1))]) @ matrix.T
    scale = homogeneous[:, 2:3]
    if np.any(np.abs(scale) < 1e-12):
        raise CalibrationException("The point cannot be mapped (degenerate calibration)")
    return homogeneous[:, :2] / scale


def _normalization(points: NDArray[np.float64]) -> NDArray[np.float64]:
    """Translation + scale that centers the points and makes their mean distance sqrt(2)."""
    centroid = points.mean(axis=0)
    mean_distance = float(np.mean(np.linalg.norm(points - centroid, axis=1)))
    if mean_distance < 1e-9:
        raise CalibrationException("The reference points are all in the same place")
    scale = np.sqrt(2) / mean_distance
    return np.array([[scale, 0.0, -scale * centroid[0]], [0.0, scale, -scale * centroid[1]], [0.0, 0.0, 1.0]])


def _fit_homography(source: NDArray[np.float64], target: NDArray[np.float64]) -> NDArray[np.float64]:
    """Direct linear transform (least squares when there are more than 4 points)."""
    t_source, t_target = _normalization(source), _normalization(target)
    s = _project_affine(t_source, source)
    d = _project_affine(t_target, target)

    rows = []
    for (x, y), (u, v) in zip(s, d):
        rows.append([-x, -y, -1.0, 0.0, 0.0, 0.0, u * x, u * y, u])
        rows.append([0.0, 0.0, 0.0, -x, -y, -1.0, v * x, v * y, v])
    _, singular_values, vt = np.linalg.svd(np.array(rows))

    padded = np.zeros(9)
    padded[: len(singular_values)] = singular_values
    if padded[-2] < _DEGENERACY_TOLERANCE * padded[0]:
        raise CalibrationException("The reference points are collinear or repeated; spread them over the work area")

    matrix = np.linalg.inv(t_target) @ vt[-1].reshape(3, 3) @ t_source
    if abs(matrix[2, 2]) < 1e-12:
        raise CalibrationException("The reference points produce a degenerate calibration")
    return matrix / matrix[2, 2]


def _project_affine(transform: NDArray[np.float64], points: NDArray[np.float64]) -> NDArray[np.float64]:
    return (np.hstack([points, np.ones((len(points), 1))]) @ transform.T)[:, :2]