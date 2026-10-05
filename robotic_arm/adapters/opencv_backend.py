import sys

import cv2


def preferred_backend() -> int:
    """Captura de camara por sistema operativo: DirectShow abre rapido en Windows, V4L2 es nativo en Linux."""
    if sys.platform.startswith("win"):
        return cv2.CAP_DSHOW
    if sys.platform.startswith("linux"):
        return cv2.CAP_V4L2
    return cv2.CAP_ANY