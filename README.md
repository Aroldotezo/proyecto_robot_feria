# Brazo Clasificador

Aplicación de escritorio en Python que clasifica objetos (organico, inorganico y defectuoso)
mediante visión por computadora y controla un brazo robótico de 6 servos a traves de un Arduino UNO.

> Proyecto academico en desarrollo.


## Ejecutar

1. Instala Python 3.11 o 3.12 (en Windows, marca "Add python.exe to PATH").
   En Linux (Debian/Ubuntu): `sudo apt install python3 python3-venv libxcb-cursor0`
2. Doble clic en `run.bat` (Windows) o `./run.sh` (Linux; antes `chmod +x run.sh`).

La primera vez crea el entorno e instala las dependencias (tarda unos minutos).
Sin hardware: `run.bat --simulate` / `./run.sh --simulate`.

En Linux, para acceder al puerto serie del Arduino: `sudo usermod -aG dialout $USER` (cerrar sesión y volver a entrar).

## Instalación

Requiere Python 3.11 o 3.12.

Linux / macOS:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Windows (PowerShell o CMD):

```bat
py -3.12 -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

En Linux, para acceder al puerto serie del Arduino:

```bash
sudo usermod -aG dialout $USER   # cerrar sesión y volver a entrar
```

## Aviso

Proyecto academico, sin licencia propia (todos los derechos reservados por defecto).

Usa software de terceros con licencias propias (ver `THIRD_PARTY_LICENSES.md`); quien lo use o
modifique debe cumplirlas. Dos puntos importantes:

- Ultralytics YOLO es AGPL-3.0 (o licencia Enterprise de pago), y los modelos entrenados con el
  tambien. No es apto para un producto cerrado o comercial sin resolver eso.
- PySide6 es LGPL-3.0. No sustituirlo por PyQt6 (GPL o comercial de pago).

El software se ofrece "tal cual", sin garantias. Controla hardware con partes moviles: usalo bajo
tu responsabilidad, manten las manos fuera del area de trabajo y prueba con velocidad baja.