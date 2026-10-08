Antes de seguir modificando la IK, quiero que adaptes el modelo a cómo funciona físicamente nuestro brazo.

Nosotros ya hicimos pruebas físicas con otra aplicación/controlador de Arduino y determinamos poses y rangos que funcionan para nuestra estructura real. Por eso NO queremos una IK genérica de 6 grados de libertad.

La estrategia real del robot es utilizar principalmente S1, S2, S3 y S6.

S4 y S5 NO deben participar en la cinemática de picking por ahora:

- Servo 4 (muñeca vertical): FIJO en 180°
- Servo 5 (giro de muñeca): FIJO en 150°

El servo 1 es la BASE:
- límite físico: 50° mínimo
- límite físico: 150° máximo
- la posición depende de dónde esté el objeto/destino en el plano XY.
- offset físico: 90° (HOME = 90° → yaw 0° = frente → servo 90°)

Servo 6 es exclusivamente la GARRA:
- OPEN = posición de garra abierta
- CLOSED = posición de garra cerrada
- abrir/cerrar NO debe recalcular S1/S2/S3.

RANGOS QUE MEDIMOS FÍSICAMENTE:

Zona de agarre:
- S2 recomendado: 50–60°
- S2 tiene un límite físico alrededor de 45° que NO debemos sobrepasar.
- S3 para la posición de agarre: 100° (el intervalo de pickeo va de 100 hacia ABAJO; valores menores extienden el brazo hacia adelante)

Elevación antes del cierre:
- S3 ≈ 90° como posición óptima
- S2 ≈ 65° como posición óptima
- S2 ≈ 60° como límite físico real

Aquí hay una ambigüedad de interpretación entre "óptimo" y "máximo real". NO la resuelvas suponiendo. Si el código necesita distinguir dirección de movimiento, pregúntame o deja esos valores como parámetros configurables.

POSICIONES DE SOLTAR QUE YA DETERMINAMOS FÍSICAMENTE:

- Orgánico: S1 ≈ 140°
- Defectuoso: S1 ≈ 100°
- Inorgánico: S1 ≈ 60°

Para la altura de soltar tenemos:
- S2 ≈ 40°
- S3 ≈ 20°

También tenemos una regla importante: al recoger un objeto no podemos simplemente bajar hasta tocar el suelo con la garra. Hay que hacer una aproximación y luego elevar ligeramente antes de cerrar la garra, porque de lo contrario la garra empuja el objeto y no consigue agarrarlo correctamente.

IMPORTANTE SOBRE LA CÁMARA:

Los puntos de calibración NO son puntos individuales de picking/drop.

Son puntos de referencia para convertir píxeles de cámara → coordenadas físicas X/Y. El detector proporciona el centro del objeto y la calibración transforma ese punto a X/Y físico.

Por tanto:

detección del objeto
→ centro en píxeles
→ conversión a X/Y físico
→ determinar S1 y la pose de agarre
→ aproximarse con S2/S3 dentro de los rangos físicos
→ cerrar S6
→ elevar
→ mover al destino
→ colocar
→ abrir S6.

Los destinos son centros de las zonas ya configuradas en el perfil.

NO quiero que por ahora intentes resolver todo mediante una IK matemática continua ni que cambies S2/S3 automáticamente solamente porque S6 pasó de OPEN a CLOSED.

Nuestro objetivo es aprovechar las poses y límites que ya validamos físicamente y hacer que el software las utilice de forma determinista.

Antes de implementar esto, revisa cómo está construido actualmente `ArmService`, `MotionPlanner`, la calibración XY y el flujo de movimiento, y dime exactamente qué parte actual sigue intentando resolver una IK libre y qué parte habría que sustituir/adaptar para trabajar con estas poses físicas.

No implementes todavía cambios grandes. Primero quiero ese diagnóstico.
