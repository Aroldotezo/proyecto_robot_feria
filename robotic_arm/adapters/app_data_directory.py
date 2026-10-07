import os
import sys
from pathlib import Path

APP_FOLDER = "RoboticArm"


def default_data_directory() -> Path:
    """Folder por usuario: cada maquina necesita su propia calibracion"""
    if sys.platform.startswith("win"):
        base = Path(os.environ.get("APPDATA") or Path.home() / "AppData" / "Roaming")
    elif sys.platform == "darwin":
        base = Path.home() / "Library" / "Application Support"
    else:
        base = Path(os.environ.get("XDG_DATA_HOME") or Path.home() / ".local" / "share")
    return base / APP_FOLDER