# Licencias de terceros

Cada componente conserva su licencia; quien use o distribuya este proyecto debe cumplirla.

| Componente | Licencia |
|---|---|
| PySide6 | LGPL-3.0 (no sustituir por PyQt6) |
| opencv-python-headless | Apache-2.0 (los wheels incluyen librerias con otras licencias, como FFmpeg) |
| NumPy | BSD-3-Clause |
| pyserial | BSD-3-Clause |
| Ultralytics (YOLO) | AGPL-3.0 o Enterprise de pago; los modelos entrenados tambien |
| SQLite / Python | Dominio publico / PSF |

Listado completo con dependencias transitivas: `pip install pip-licenses && pip-licenses --format=markdown`