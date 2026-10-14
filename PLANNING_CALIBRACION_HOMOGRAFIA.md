# ROADMAP DE IMPLEMENTACIÓN: CALIBRACIÓN FORMAL DE PLANO (HOMOGRAFÍA) Y TRACKING PRECISO

> **Propósito de este documento:**
> Este archivo sirve como prompt y especificación técnica directa para la próxima sesión de trabajo. Define el paso a paso exacto para activar la calibración formal del plano de trabajo (homografía 2D cámara cenital ↔ brazo robótico) utilizando la infraestructura existente (SQLite, `PlaneCalibration`, `CalibrationService`) y reemplazar definitivamente las aproximaciones analíticas estimadas a mano.

---

## 1. Diagnóstico del Estado Actual en la Base de Código

1. **La base de datos SQLite ya existe y tiene las tablas listas:**
   - Ubicación: `~/.local/share/RoboticArm/profiles.db`
   - Tablas: `profiles`, `zones`, `reference_points`
   - Estado: La tabla `reference_points` actualmente está vacía, lo que fuerza a `CalibrationService.image_to_arm()` a recurrir al fallback analítico estimado (`arm_x = 130 + y * 120`).

2. **La matemática de homografía ya está programada:**
   - Archivo: `robotic_arm/domain/plane_calibration.py`
   - Utiliza `cv2.findHomography` / transformación proyectiva con mínimo 4 puntos y calcula el error RMS.

3. **La interfaz de calibración ya existe:**
   - Archivos: `robotic_arm/ui/calibration_panel.py`, `calibration_controller.py`, `calibration_overlay.py`.
   - Permite registrar pares `(NormalizedPoint_imagen, ArmPoint_robot)`.

---

## 2. Plan de Ejecución Paso a Paso (Meta: < 1 - 2 Horas)

### Fase 1: Calibración Física de la Mesa con 4 Puntos de Referencia
1. Marcar 4 puntos físicos visibles en la mesa dentro del campo visual de la cámara cenital.
2. Medir con regla/cinta métrica la posición $(X, Y)$ en milímetros de cada punto respecto al **centro del eje de rotación de la base del robot (S1)**:
   - **Punto 1 (Arriba Izquierda):** ej. $X = 130\text{ mm}, Y = -90\text{ mm}$
   - **Punto 2 (Arriba Derecha):** ej. $X = 130\text{ mm}, Y = +90\text{ mm}$
   - **Punto 3 (Abajo Derecha):** ej. $X = 200\text{ mm}, Y = +90\text{ mm}$
   - **Punto 4 (Abajo Izquierda):** ej. $X = 200\text{ mm}, Y = -90\text{ mm}$
3. Cargar estos 4 puntos en la base de datos `profiles.db` (vía UI en la pestaña de Calibración o mediante un script de inicialización directa en `default_profile.py`).

### Fase 2: Activación y Validación de la Homografía en Tiempo Real
1. Al contar con los $\ge 4$ puntos, `PlaneCalibration` se inicializa automáticamente en `CalibrationService._rebuild()`.
2. Verificar que `CalibrationService.status().calibrated == True` y que el error RMS reportado sea $< 3.0\text{ mm}$.
3. Comprobar que cualquier detección generada por `YoloDetector` en `DetectionController._build_infos()` entregue un `ArmPoint(x, y)` milimétrico real sin saltos.

### Fase 3: Conexión con Cinemática Inversa o Interpolación Acoplada
1. Con $(X, Y)$ reales garantizados:
   - El radio $r = \sqrt{X^2 + Y^2}$ representa la distancia física exacta.
   - El ángulo base $S_1 = \text{atan2}(Y, X)$ apunta directo al centroide del objeto.
2. Validar que la interpolación de $S_2$ y $S_3$ en `PhysicalArmConfig` use el rango real $[r_{\min}, r_{\max}]$ medido físicamente.

---

## 3. Modo Fallback (Demostración de Emergencia / Feria)

Para asegurar la presentación ante cualquier imprevisto de visión o servos:
- Implementar un switch en UI o atajo de teclado: `MODO_DEMOSTRACION_STANDALONE`.
- **Comportamiento del Fallback:**
  - Ejecutar una secuencia cíclica predeterminada de poses fijas (Pick en centroide ficticio $\rightarrow$ Drop en Orgánicos $\rightarrow$ Pick $\rightarrow$ Drop en Inorgánicos).
  - Animar en pantalla el overlay visual de detección para que el público observe la clasificación activa de forma fluida e interactiva.

---

## 4. Instrucción Directa para el Próximo Chat

```text
Actúa como Antigravity. Lee el archivo PLANNING_CALIBRACION_HOMOGRAFIA.md en la raíz del workspace. 
Vamos a activar la calibración formal de plano con los 4 puntos de referencia en la base de datos SQLite y desacoplar el fallback de emergencia. Guiate de las clases existentes: CalibrationService, PlaneCalibration y default_physical_config.py.
```
