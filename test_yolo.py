from pathlib import Path

import cv2

from robotic_arm.adapters.yolo_detector import YoloDetector


MODEL_PATH = Path("models/best.pt")
IMAGE_PATH = Path("test/images/test1.jpeg")


detector = YoloDetector(
    model_path=MODEL_PATH,
    confidence=0.5,
)

frame = cv2.imread(str(IMAGE_PATH))

if frame is None:
    raise RuntimeError(f"No se pudo cargar la imagen: {IMAGE_PATH}")

detections = detector.detect(frame)

if not detections:
    print("No se detectaron objetos.")
else:
    for detection in detections:
        print(
            f"clase={detection.clase.value} "
            f"confianza={detection.reliability:.2f} "
            f"bbox=({detection.x}, {detection.y}, "
            f"{detection.width}, {detection.height}) "
            f"centro={detection.center}"
        )