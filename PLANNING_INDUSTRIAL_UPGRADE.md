# Plan de Implementación Profesional: Brazo Robótico Clasificador 3D

Este documento detalla los requerimientos físicos, mediciones necesarias, configuración de cámaras y el entorno para la migración a un sistema robótico industrial autónomo.

---

## 1. Decisión sobre las Cámaras y Resolución (640x480)

### ¿Afecta en algo que ambas cámaras sean 640x480?
- **NO afecta negativamente, al contrario, es una ventaja:**
  - **Rendimiento:** 640x480 es la resolución estándar en visión robótica de tiempo real. Permite que YOLO y OpenCV procesen a **30+ FPS estables** sin latencia de inferencia y sin sobrecargar el bus USB ni la CPU/GPU de la PC.
  - **Precisión:** En un área de trabajo de ~30x30 cm, 640 píxeles te dan una resolución de **~0.5 mm por píxel**, lo cual es más que suficiente para agarrar con una garra que tiene varios centímetros de apertura.
  - **Escalado Interno:** Como ambas cámaras tienen la misma relación de aspecto (4:3), los cálculos de píxel a milímetro y normalización son uniformes y directos.

---

## 2. Requerimientos del Entorno Físico y Posición de las Cámaras

Dado que la celda tiene pared cerrada a la derecha y el **hueco abierto a la izquierda del robot**:

```
              ┌──────────────────────────────────────┐
              │          FOCO LUZ DIFUSA             │
              │                                      │
              │       [CÁMARA 1: CENITAL]            │ (En el techo mirando 90° hacia la mesa)
              │                                      │
[HUECO        │             [OBJETO]                 │ [PARED
 ABIERTO      │                📦                    │  CERRADA
 LATERAL      │              ______                  │  DERECHA]
 IZQUIERDO]   │             [ ROBOT]                 │
   │          └─────────────┴──────┴─────────────────┘
   │
[CÁMARA 2: LATERAL] 
(Elevada a ~15-20 cm sobre la mesa, inclinada ~25°-35° hacia abajo)
```

### Posición Detallada de las Cámaras:

1. **Cámara 1 (Cenital):**
   - **Ubicación:** En el techo de la caja, mirando a 90° en vertical hacia la mesa de trabajo.
   - **Alineación:** Centrada sobre el área de pickeo.
   - **Función:** Detecta $(X, Y)$ del centro del objeto y el ángulo de rotación para la pinza.

2. **Cámara 2 (Lateral - Por el hueco izquierdo):**
   - **¿A nivel del suelo o elevada?** **NO a nivel del suelo.** Si estuviera al ras del suelo (0 cm), cualquier objeto en primer plano taparía totalmente a los que estén detrás (oclusión total) y el borde inferior de la pinza chocaría en la vista.
   - **Elevación en Z (Altura recomendada):** **15 a 20 cm sobre la superficie de la mesa.**
   - **Ángulo de inclinación (Pitch):** **Entre 25° y 35° hacia abajo**, apuntando hacia el centro del área de trabajo. Con esta inclinación puede ver con total claridad la silueta y altura real ($Z$) del objeto contra la mesa.
   - **Distancia en el eje Y (Lateral):** Colócala entre **20 cm y 30 cm a la izquierda del robot** (es decir, en $Y \approx +200 \text{ a } +300\text{ mm}$ si la izquierda es positiva, o en el límite exterior del marco del hueco). A esa distancia el lente de 640x480 tiene suficiente campo de visión (*FOV*) para encuadrar todo el área de pickeo sin distorsión ojo de pez.
   - **Posición en el eje X (Frente/Atrás):** Colócala alineada con el centro del área donde se colocan los objetos (normalmente a **$X \approx 150 \text{ a } 200\text{ mm}$ al frente de la base del robot**). De esta forma no mira desde atrás del robot ni muy desde adelante, sino directamente al flanco de la zona de agarre.

### ¿Se necesitan medidas de la cámara lateral respecto al Servo 1?
**SÍ, solo 3 medidas aproximadas (con regla / cinta métrica):**
1. **$X_{cam}$ (Avance respecto al Servo 1):** Distancia en mm desde el centro del eje del Servo 1 hacia adelante hasta el lente de la cámara (ej. ~180 mm).
2. **$Y_{cam}$ (Separación lateral respecto al Servo 1):** Distancia en mm desde el centro del eje del Servo 1 hacia la izquierda hasta el lente de la cámara (ej. ~250 mm).
3. **$Z_{cam}$ (Altura del lente sobre la mesa):** Altura vertical desde la superficie de la mesa hasta el centro del lente (ej. ~180 mm).

> [!NOTE]
> **Sobre el objeto que se ve muy cerca al extremo:** Es normal por el campo de visión de 640x480. La cámara lateral **NO se usa para saber la posición horizontal $(X, Y)$ del objeto** (eso lo hace la cámara cenital del techo con total precisión sin distorsión). La cámara lateral **SOLO se usa para leer la altura del objeto ($Z$) y verificar que la pinza baje a la altura correcta**. No importa si se ve cerca o grande en la vista lateral, siempre que el objeto y la mesa entren dentro del encuadre.

---

## 3. ¿Por qué la app muestra "ORGÁNICO" en lugar del nombre del objeto (higo, lata)?

### Diagnóstico del Software:
En el código actual ([yolo_detector.py](file:///home/pablo03/Escritorio/proyecto-feria/proyecto_robot_feria/robotic_arm/adapters/yolo_detector.py#L15-L19)):
1. El modelo YOLO sí detecta las clases específicas: `"higo"`, `"lata"`, `"lata_defectuoso"`.
2. Sin embargo, el adaptador mapea inmediatamente el nombre a la categoría de destino:
   - `"higo"` $\to$ `ORGANIC` (`"organico"`)
   - `"lata"` $\to$ `INORGANIC` (`"inorganico"`)
   - `"lata_defectuoso"` $\to$ `DEFECTIVE` (`"defectuoso"`)
3. La interfaz gráfica ([detection_overlay.py](file:///home/pablo03/Escritorio/proyecto-feria/proyecto_robot_feria/robotic_arm/ui/detection_overlay.py#L97)) dibuja únicamente `det.clase.value` (es decir, la categoría general).
4. **Solución:** Se puede agregar el campo `raw_class_name` en la detección para que el overlay dibuje: **`HIGO [ORGÁNICO] 95%`** o **`LATA [INORGÁNICO] 88%`**. El dataset está bien entrenado; es solo que la app descartaba el nombre del objeto específico para mostrar el contenedor de destino.

---

## 4. Medidas Físicas Requeridas (Hoja de Trabajo del Brazo)

Para que el modelo cinemático matemático ([arm_kinematics.py](file:///home/pablo03/Escritorio/proyecto-feria/proyecto_robot_feria/robotic_arm/domain/arm_kinematics.py)) calcule con precisión milimétrica los ángulos de los servos sin usar posturas "a ciegas", necesitamos medir con regla o calibre:

### A. Geometría de Eslabones (Eje a Eje en milímetros)
> [!IMPORTANT]
> **¿Cómo medir?** Siempre en **línea recta euclidiana** (distancia más corta directa entre los dos centros de tornillo o ejes de giro). **NO sigas las curvas ni el contorno del plástico/acrílico del brazo.** Es simplemente la distancia directa de centro de tornillo a centro de tornillo.

1. **`base_height` ($H_{base}$ - Altura total de la base):**
   - Medida real verificada: **130.0 mm** (desde la mesa hasta el centro del eje del Servo 2).
2. **`shoulder_length` ($L_1$):**
   - Medida real verificada: **127.0 mm** (eje de S2 a eje de S3).
3. **`forearm_length` ($L_2$):**
   - Medida real verificada: **86.0 mm** (eje de S3 a eje de S4).
4. **`tool_length_open` y `tool_length_closed` ($L_3$):**
   - Medida real abierta: **122.0 mm**
   - Medida real cerrada: **166.0 mm**

---

### B. Límites y Cero Físico de los Servos (Valores Reales Registrados)
- **Servo 1 (Base):** [50.0°, 125.0°]
- **Servo 2 (Hombro):** [20.0°, 90.0°]
- **Servo 3 (Codo):** [0.0°, 160.0°]
- **Servo 4 (Muñeca Pitch):** [0.0°, 175.0°]
- **Servo 5 (Muñeca Roll):** [0.0°, 170.0°]
- **Servo 6 (Garra / Gripper):** [90.0°, 180.0°] (Abierta = 90°, Cerrada = 180°)

### C. Parámetros Físicos de la Cámara Lateral (Izquierda)
- **$X_{cam}$ (Avance al frente):** 309.0 mm
- **$Y_{cam}$ (Separación lateral):** 397.0 mm
- **$Z_{cam}$ (Altura del lente sobre la mesa):** 165.0 mm
- **Ángulo de inclinación (Pitch):** 25.0° hacia abajo

---

### C. Espacio de Trabajo de la Caja (Dimensiones del Área de Pick)
1. **Límites de la mesa visibles por la cámara:**
   - Ancho ($Y$ mínimo a $Y$ máximo en mm): ej. de -150 mm a +150 mm respecto a la base.
   - Profundidad ($X$ mínimo a $X$ máximo en mm): ej. de 120 mm a 300 mm desde la base.
   - Altura de la mesa ($Z = 0$ mm).

---

## 5. Próximos Pasos en el Software una vez tomadas las medidas

1. **Actualizar el Modelo Físico:**
   - Cargar las medidas reales verificadas en [default_arm_model.py](file:///home/pablo03/Escritorio/proyecto-feria/proyecto_robot_feria/robotic_arm/application/default_arm_model.py).
2. **Script de Verificación de Cinemática Directa / Inversa:**
   - Crear una prueba simple donde le demos coordenadas $(X, Y, Z)$ en consola y el brazo se desplace suavemente tocando esos puntos físicos.
3. **Módulo Multi-Cámara:**
   - Habilitar la lectura paralela de las 2 webcams en la interfaz gráfica.
4. **Mapeo ArUco o Matriz de Proyección:**
   - Asignar la conversión de píxeles a milímetros reales de forma automática.
