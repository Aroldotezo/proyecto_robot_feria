Trabaja directamente sobre el proyecto abierto en este workspace.


Ya existe un commit limpio antes de comenzar, así que puedes modificar directamente los archivos necesarios.


Primero inspecciona el proyecto y comprende las implementaciones reales existentes. Después implementa la tarea completa descrita abajo.


Tienes permiso para:



leer cualquier archivo del workspace;

modificar los archivos necesarios;

crear archivos si son realmente necesarios;

ejecutar comandos para verificar la implementación;

corregir errores que encuentres durante la implementación.



No quiero una explicación teórica ni una propuesta previa. Quiero que realices la implementación directamente.



Antes de crear una clase o abstracción nueva, verifica si ya existe una equivalente en el proyecto.



Al finalizar, no hagas commit. Déjame los cambios en el working tree para poder revisarlos.





Quiero que implementes directamente la integración completa de la visión artificial con el sistema de coordenadas, calibración, tracking y control del brazo utilizando la arquitectura EXISTENTE del proyecto.

No quiero una propuesta teórica ni una implementación paralela. Inspecciona el código actual, comprende las abstracciones que ya existen y construye la funcionalidad sobre ellas para que todo trabaje en armonía.

El objetivo es que la detección de YOLO no se quede aislada: debe formar parte del pipeline real que termina produciendo las coordenadas que necesita el brazo.

El modelo entrenado ya está disponible en:

`models/best.pt`

Ya comprobé manualmente que `YoloDetector` funciona correctamente con imágenes reales y que reconoce las tres clases entrenadas.

Las clases del modelo son:

* `higo`
* `lata`
* `lata_defectuoso`

El dominio ya realiza la correspondencia:

* `higo` → `ObjectClass.ORGANIC`
* `lata` → `ObjectClass.INORGANIC`
* `lata_defectuoso` → `ObjectClass.DEFECTIVE`

Esta correspondencia es intencional. El modelo no necesita devolver literalmente conceptos como "orgánico" o "inorgánico"; esos son conceptos del dominio y deben continuar siendo representados por `ObjectClass`.

No entrenes otro modelo y no modifiques el dataset.

---

## Integración completa

Integra YOLO con el flujo real de cámara existente.

Utiliza las abstracciones actuales del proyecto, especialmente las relacionadas con:

* cámara;
* `FrameStream`;
* `FrameSource`;
* `VideoView`;
* `YoloDetector`;
* `Detection`;
* `ObjectClass`;
* `NormalizedPoint`;
* `NormalizedRect`;
* `CalibrationService`;
* `ReferencePoint`;
* `ArmPoint`;
* `ClassificationProfile`;
* `Zone`;
* `ZoneRole`;
* ports/adapters existentes para el brazo;
* cualquier servicio de aplicación existente relacionado con tracking, clasificación o movimiento.

No reemplaces estas abstracciones por soluciones improvisadas si ya existe una forma adecuada de utilizarlas.

La integración debe conectar realmente:

captura de frame → detección → centro de detección → coordenadas normalizadas → calibración → `ArmPoint` → lógica de tracking → movimiento del brazo.

---

## Coordenadas y calibración

Las coordenadas de calibración que ya existen en el proyecto son la FUENTE DE VERDAD para transformar la posición visual de los objetos al espacio del brazo.

No inventes una transformación alternativa.

No hardcodees nuevas coordenadas físicas.

No reemplaces los `ReferencePoint` existentes por valores inventados.

Utiliza el `CalibrationService` y las estructuras de dominio existentes para realizar la transformación correspondiente.

El `Detection.center` representa la posición del objeto en el frame de cámara.

Ese punto debe pasar por las abstracciones existentes de geometría/normalización y posteriormente por el mecanismo de calibración existente para obtener el `ArmPoint` correspondiente.

Respeta las coordenadas y referencias que ya están definidas en:

* `ReferencePoint`;
* `NormalizedPoint`;
* `ArmPoint`;
* `CalibrationService`;
* `ClassificationProfile`.

Si alguna de estas abstracciones necesita una adaptación para integrarse con YOLO, modifica lo mínimo necesario y mantén su responsabilidad original.

No crees una segunda implementación de calibración.

---

## Resolución del frame y normalización

Ten en cuenta la resolución real del frame entregado por la cámara.

El `Detection.center` debe interpretarse respecto al frame original.

Si `VideoView` escala el frame para mostrarlo, no confundas las coordenadas visuales del widget con las coordenadas originales de la cámara.

La calibración debe recibir el punto correspondiente al espacio de imagen que las abstracciones actuales esperan.

Reutiliza cualquier utilidad de normalización que ya exista en el proyecto.

No dupliques lógica existente.

---

## Zonas de clasificación

Utiliza el `ClassificationProfile` existente y sus `Zone`.

El perfil actual ya define:

* `evaluation`;
* `inorganic`;
* `defective`;
* `organic`.

También existen colores asociados a estas zonas.

Respeta esas zonas y sus coordenadas normalizadas.

No inventes otra distribución de zonas.

Las zonas deben formar parte del flujo visual y de la lógica correspondiente del sistema.

Cuando una detección corresponda a una clase determinada, utiliza las abstracciones existentes para determinar su zona/destino correspondiente.

La relación conceptual ya definida es:

* `ORGANIC` → zona `organic`;
* `INORGANIC` → zona `inorganic`;
* `DEFECTIVE` → zona `defective`.

No pongas esta lógica dentro de `YoloDetector`. Debe permanecer en la capa correspondiente del dominio/aplicación.

---

## Overlay visual

Quiero que el reconocimiento sea muy evidente visualmente.

Cada objeto detectado debe mostrar sobre el video:

* bounding box;
* clase;
* confianza;
* centro de la detección;
* información útil de la coordenada calculada cuando sea razonable.

El usuario debe poder observar claramente qué está detectando YOLO y dónde considera que está el objeto.

Los bounding boxes y etiquetas deben contrastar correctamente con los colores de las zonas existentes.

No cambies innecesariamente los colores del `ClassificationProfile`.

Si la interfaz ya tiene un sistema de overlays, reutilízalo.

Si no existe uno adecuado, crea la abstracción necesaria de forma coherente con la arquitectura actual.

También quiero que las zonas de clasificación puedan visualizarse como guía sobre el video, respetando las coordenadas y colores ya definidos por el perfil.

---

## Información de coordenadas

Durante la ejecución debe ser posible observar el resultado de la cadena de transformación.

Para una detección debería ser posible conocer, al menos durante la depuración/visualización:

* clase detectada;
* confianza;
* centro en píxeles;
* punto normalizado;
* `ArmPoint` resultante;
* zona/destino correspondiente.

No inventes valores de ejemplo para esto.

Utiliza siempre los valores reales producidos por el pipeline.

Esto es especialmente importante porque actualmente estamos validando que las coordenadas de calibración realmente correspondan con la posición física esperada.

---

## Tracking

Integra el reconocimiento con el tracking utilizando las abstracciones existentes del proyecto.

No conviertas esto simplemente en una lista de detecciones independientes por frame si ya existe una estructura destinada al seguimiento.

El sistema debe poder utilizar una detección como objetivo y mantener el contexto necesario para que posteriormente el brazo pueda actuar sobre ella.

No hace falta introducir un algoritmo de tracking externo o excesivamente complejo si el proyecto ya dispone de una solución adecuada.

Prioriza una implementación coherente con el diseño existente.

---

## Control del brazo

Integra el resultado final con los ports/adapters existentes del brazo.

No hagas que YOLO controle directamente motores, servos o comunicación física.

`YoloDetector` solamente debe encargarse de detección.

La capa correspondiente debe utilizar el `ArmPoint` producido por la calibración para generar la acción del brazo.

Utiliza las abstracciones existentes para el movimiento.

No inventes un segundo controlador de hardware.

---

## SWITCH DE SEGURIDAD PARA EL TRACKING FÍSICO

Esto es obligatorio.

Necesito un control visible en la interfaz que permita activar/desactivar el movimiento físico automático del brazo.

El estado inicial SIEMPRE debe ser:

`ARM TRACKING = OFF`

Cuando esté desactivado, quiero que el pipeline completo continúe funcionando normalmente:

* cámara;
* YOLO;
* detecciones;
* bounding boxes;
* centros;
* normalización;
* calibración;
* `ArmPoint`;
* tracking lógico;
* identificación del destino.

Es decir, quiero poder comprobar que las coordenadas calculadas son correctas SIN permitir que el brazo se mueva.

El switch solamente debe bloquear la salida física hacia el brazo.

Cuando el usuario active explícitamente:

`ARM TRACKING = ON`

entonces el resultado del pipeline podrá llegar al controlador físico del brazo.

La protección debe ser real.

No debe ser solamente un indicador visual en la interfaz.

La capa que finalmente solicita el movimiento al hardware debe comprobar que el tracking físico está habilitado.

---

## Seguridad del movimiento

Incluso con `ARM TRACKING = ON`, nunca envíes al brazo una coordenada inválida.

No debe producirse movimiento cuando:

* no existe una detección válida;
* la detección desaparece;
* la calibración falla;
* no existe `ArmPoint`;
* la coordenada está fuera de los límites válidos del espacio del brazo;
* existe un error en la transformación;
* el destino no puede determinarse correctamente.

Ante cualquiera de estos casos, no inventes una coordenada ni intentes aproximarla arbitrariamente.

El sistema debe rechazar la acción de forma segura.

---

## Estado y feedback de la interfaz

La interfaz debe dejar claro si el movimiento físico está permitido.

Debe existir una indicación evidente de:

`ARM TRACKING: OFF`

o

`ARM TRACKING: ON`

El estado OFF debe ser el predeterminado cada vez que se inicia la aplicación.

El usuario debe realizar una acción explícita para permitir el movimiento físico.

También debe ser posible distinguir visualmente entre:

* objeto detectado;
* objeto actualmente seguido;
* coordenada calculada;
* destino correspondiente;
* estado del tracking físico.

No hace falta sobrecargar la interfaz: prioriza claridad.

---

## Rendimiento

La integración debe respetar el modelo actual de concurrencia.

No bloquees el hilo principal de Qt con inferencias YOLO.

Aprovecha `FrameStream` y la arquitectura existente.

Evita acumular frames antiguos innecesariamente.

Si la arquitectura requiere descartar frames mientras se procesa uno anterior, implementa ese comportamiento de forma coherente con el diseño existente.

No introduzcas complejidad innecesaria.

---

## No modificar innecesariamente

No quiero que cambies:

* el modelo;
* el dataset;
* el entrenamiento;
* `data.yaml`;
* el mapeo de clases;
* la arquitectura de dominio existente;
* la calibración existente;
* las coordenadas de referencia existentes;

salvo que sea estrictamente necesario para conectar correctamente las piezas.

Si algo ya existe y funciona, intégralo en lugar de reemplazarlo.

---

## Tests

NO CREES TESTS UNITARIOS.

No agregues:

* pytest;
* nuevos archivos `test_*.py`;
* mocks;
* tests de `YoloDetector`;
* tests de `FrameStream`;
* tests de calibración;
* tests del brazo.

La validación de esta tarea debe hacerse ejecutando la aplicación real con la cámara y observando el pipeline completo.

Ya existe una prueba manual de `YoloDetector` que confirma que el modelo funciona.

---

## Regla principal de implementación

No quiero que me expliques cómo hacerlo antes de implementarlo.

No quiero una propuesta.

No quiero que me devuelvas pseudocódigo.

Quiero que inspecciones el proyecto existente, tomes las decisiones necesarias basándote en las abstracciones que ya están implementadas y MODIFIQUES EL CÓDIGO directamente.

Utiliza las coordenadas, referencias, servicios, zonas, ports y adapters que ya existen como fuente de verdad.

No inventes una segunda arquitectura.

No simplifiques el pipeline eliminando la calibración.

No dejes YOLO aislado.

Quiero que todo quede integrado y funcionando en armonía, desde la cámara hasta el brazo, con el movimiento físico protegido por el switch `ARM TRACKING`.

Implementa la funcionalidad directamente y prioriza que el resultado sea ejecutable y coherente con la arquitectura existente.




