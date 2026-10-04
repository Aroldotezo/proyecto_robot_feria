#!/usr/bin/env python3
"""
Control de brazo robótico de 6 servos (Arduino por puerto serie) con visión
por computadora para detectar y descartar piezas defectuosas.

Dependencias:
    pip install pyserial opencv-python numpy
    (opcional, para YOLO)  pip install ultralytics

Protocolo Arduino (9600 baudios):
    "MODE 1\n"            -> modo automático
    "SPD <0-100>\n"       -> velocidad
    "<servo> <angulo>\n"  -> 6 líneas seguidas (servo 1..6, ángulo 0..180)
    El Arduino responde "DONE\n" cuando todos los servos llegaron a su posición.

Controles: pulsa 'q' en la ventana de video para salir limpiamente.
"""

import time
from enum import Enum, auto

import cv2
import numpy as np
import serial

# ============================== CONFIGURACIÓN ==============================

# --- Serial ---
PUERTO_SERIE = "COM4"          # Linux: "/dev/ttyACM0" o "/dev/ttyUSB0"; Mac: "/dev/tty.usbmodemXXXX"
BAUDIOS = 9600
VELOCIDAD = 60                 # 0 a 100
TIMEOUT_DONE = 15.0            # segundos máximos esperando "DONE"
ESPERA_REINICIO_ARDUINO = 2.0  # el Arduino se reinicia al abrir el puerto

# --- Posiciones (orden de servos: [1, 2, 3, 4, 5, 6]; el servo 6 es la garra) ---
GARRA_ABIERTA = 30
GARRA_CERRADA = 120

ANGULOS_REPOSO = [90, 90, 90, 90, 90, 90]

# Posición de agarre (la base, servo 1, se calcula según la posición X de la pieza)
ANGULOS_AGARRE = [90, 60, 50, 70, 90, GARRA_ABIERTA]
BASE_MIN, BASE_MAX = 40, 140   # rango del servo 1 que cubre el ancho de la imagen

# Posición sobre la papelera / zona de descarte
ANGULOS_DESCARTE = [170, 80, 70, 80, 90, GARRA_CERRADA]

# --- Cámara ---
INDICE_CAMARA = 1
ANCHO, ALTO = 640, 480

# --- Detección ---
USAR_YOLO = False              # True para usar un modelo YOLOv8 propio
RUTA_MODELO_YOLO = "defectos.pt"
CLASE_DEFECTUOSA = "defecto"   # nombre de la clase "defectuosa" en tu modelo
CONFIANZA_MIN = 0.5

# Detección por color: se considera "defectuoso" lo que sea ROJO (ajusta a tu caso)
HSV_BAJO_1, HSV_ALTO_1 = (0, 120, 70), (10, 255, 255)
HSV_BAJO_2, HSV_ALTO_2 = (170, 120, 70), (180, 255, 255)
AREA_MINIMA = 1500             # píxeles²; ignora ruido
FRAMES_CONFIRMACION = 8        # frames consecutivos con detección antes de actuar


# ============================== SERIAL / ARDUINO ==============================

def abrir_arduino(puerto, baudios):
    """Abre el puerto, activa modo automático y fija la velocidad."""
    ser = serial.Serial(puerto, baudios, timeout=1)
    time.sleep(ESPERA_REINICIO_ARDUINO)
    ser.reset_input_buffer()
    ser.write(b"MODE 1\n")
    ser.write(f"SPD {int(VELOCIDAD)}\n".encode())
    ser.flush()
    return ser


def mover_brazo(puerto, angulos, timeout=TIMEOUT_DONE):
    """
    Envía los 6 ángulos al Arduino y BLOQUEA hasta recibir "DONE".
    `puerto` es un objeto serial.Serial ya abierto.
    Lanza TimeoutError si no llega "DONE" a tiempo.
    """
    if len(angulos) != 6:
        raise ValueError("Se requieren exactamente 6 ángulos")

    puerto.reset_input_buffer()
    for servo, ang in enumerate(angulos, start=1):
        ang = max(0, min(180, int(ang)))          # limitar a 0..180
        puerto.write(f"{servo} {ang}\n".encode())  # formato "<servo> <angulo>\n"
    puerto.flush()

    inicio = time.time()
    while time.time() - inicio < timeout:
        linea = puerto.readline().decode(errors="ignore").strip()
        if linea == "DONE":
            return
    raise TimeoutError("El Arduino no respondió 'DONE' a tiempo")


# ============================== DETECCIÓN ==============================

_modelo_yolo = None
if USAR_YOLO:
    from ultralytics import YOLO
    _modelo_yolo = YOLO(RUTA_MODELO_YOLO)


def detectar_defectuoso(frame):
    """
    Busca una pieza defectuosa en el frame.
    Devuelve (cx, cy, (x, y, w, h)) de la pieza, o None si no hay.
    """
    if USAR_YOLO:
        resultado = _modelo_yolo(frame, verbose=False)[0]
        mejor = None
        for caja in resultado.boxes:
            nombre = resultado.names[int(caja.cls)]
            conf = float(caja.conf)
            if nombre == CLASE_DEFECTUOSA and conf >= CONFIANZA_MIN:
                if mejor is None or conf > mejor[0]:
                    x1, y1, x2, y2 = map(int, caja.xyxy[0])
                    mejor = (conf, x1, y1, x2 - x1, y2 - y1)
        if mejor is None:
            return None
        _, x, y, w, h = mejor
        return x + w // 2, y + h // 2, (x, y, w, h)

    # --- Detección por color + contornos ---
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (7, 7), 0), cv2.COLOR_BGR2HSV)
    mascara = cv2.inRange(hsv, HSV_BAJO_1, HSV_ALTO_1) | cv2.inRange(hsv, HSV_BAJO_2, HSV_ALTO_2)
    kernel = np.ones((5, 5), np.uint8)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernel)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel)

    contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contornos = [c for c in contornos if cv2.contourArea(c) >= AREA_MINIMA]
    if not contornos:
        return None

    c = max(contornos, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)
    return x + w // 2, y + h // 2, (x, y, w, h)


def angulos_para_pieza(cx, ancho_img):
    """Calcula los ángulos de agarre: la base (servo 1) sigue la posición X de la pieza."""
    angulos = list(ANGULOS_AGARRE)
    angulos[0] = int(np.interp(cx, [0, ancho_img], [BASE_MIN, BASE_MAX]))
    return angulos


# ============================== MÁQUINA DE ESTADOS ==============================

class Estado(Enum):
    IDLE = auto()
    RECOGER = auto()
    DESCARTAR = auto()
    REPOSO = auto()


def mostrar(frame, estado, deteccion=None):
    """Dibuja la detección y el estado actual en la ventana."""
    if deteccion:
        cx, cy, (x, y, w, h) = deteccion
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
    cv2.putText(frame, f"Estado: {estado.name}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.imshow("Brazo robotico", frame)


def main():
    arduino = abrir_arduino(PUERTO_SERIE, BAUDIOS)
    cam = cv2.VideoCapture(INDICE_CAMARA)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, ANCHO)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTO)
    if not cam.isOpened():
        arduino.close()
        raise RuntimeError("No se pudo abrir la cámara")

    estado = Estado.IDLE
    angulos_pieza = None
    confirmaciones = 0

    try:
        mover_brazo(arduino, ANGULOS_REPOSO)  # posición inicial segura

        while True:
            ok, frame = cam.read()
            if not ok:
                print("No se pudo leer de la cámara")
                break

            deteccion = None

            if estado == Estado.IDLE:
                # Buscar pieza defectuosa; exigir varios frames seguidos para evitar falsos positivos
                deteccion = detectar_defectuoso(frame)
                confirmaciones = confirmaciones + 1 if deteccion else 0
                if confirmaciones >= FRAMES_CONFIRMACION:
                    angulos_pieza = angulos_para_pieza(deteccion[0], frame.shape[1])
                    confirmaciones = 0
                    estado = Estado.RECOGER

            # Refrescar la ventana antes de las acciones bloqueantes
            mostrar(frame, estado, deteccion)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            if estado == Estado.RECOGER:
                print("Recogiendo pieza...")
                mover_brazo(arduino, angulos_pieza)                       # posicionar con garra abierta
                cerrada = list(angulos_pieza)
                cerrada[5] = GARRA_CERRADA
                mover_brazo(arduino, cerrada)                             # cerrar garra (servo 6)
                estado = Estado.DESCARTAR

            elif estado == Estado.DESCARTAR:
                print("Descartando pieza...")
                mover_brazo(arduino, ANGULOS_DESCARTE)                    # ir a la papelera
                abierta = list(ANGULOS_DESCARTE)
                abierta[5] = GARRA_ABIERTA
                mover_brazo(arduino, abierta)                             # soltar pieza
                estado = Estado.REPOSO

            elif estado == Estado.REPOSO:
                print("Volviendo a reposo...")
                mover_brazo(arduino, ANGULOS_REPOSO)
                estado = Estado.IDLE

    except TimeoutError as e:
        print(f"Error de comunicación: {e}")
    except KeyboardInterrupt:
        print("Interrumpido por el usuario")
    finally:
        # Cierre limpio: soltar cámara, ventanas y puerto serie
        cam.release()
        cv2.destroyAllWindows()
        if arduino.is_open:
            arduino.close()
        print("Programa finalizado.")


if __name__ == "__main__":
    main()