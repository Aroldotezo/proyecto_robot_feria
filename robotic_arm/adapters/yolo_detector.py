from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from ultralytics import YOLO

from robotic_arm.domain.camera.detection import Detection
from robotic_arm.domain.object_class import ObjectClass
from robotic_arm.ports.detector import Detector


class YoloDetector(Detector):
    """Adaptador de YOLO para el reconocimiento de objetos."""

    _CLASS_MAPPING = {
        "higo": ObjectClass.ORGANIC,
        "lata": ObjectClass.INORGANIC,
        "lata_defectuoso": ObjectClass.DEFECTIVE,
    }

    def __init__(
        self,
        model_path: str | Path,
        confidence: float = 0.5,
    ) -> None:
        self._model = YOLO(str(model_path))
        self._confidence = confidence

    @property
    def confidence(self) -> float:
        return self._confidence

    @confidence.setter
    def confidence(self, value: float) -> None:
        self._confidence = max(0.0, min(1.0, value))

    def detect(self, frame: NDArray[np.uint8]) -> list[Detection]:
        results = self._model.predict(
            source=frame,
            conf=self._confidence,
            verbose=False,
        )

        detections: list[Detection] = []

        for result in results:
            if result.boxes is None:
                continue

            names = result.names

            for box in result.boxes:
                class_id = int(box.cls.item())
                class_name = names[class_id]

                object_class = self._CLASS_MAPPING.get(class_name)

                if object_class is None:
                    continue

                confidence = float(box.conf.item())

                x1, y1, x2, y2 = box.xyxy[0].tolist()

                detections.append(
                    Detection(
                        clase=object_class,
                        reliability=confidence,
                        x=int(x1),
                        y=int(y1),
                        width=int(x2 - x1),
                        height=int(y2 - y1),
                    )
                )

        return detections