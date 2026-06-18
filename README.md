Práctica 4 - Control de servomotor mediante LDR, filtro de mediana, colas y Task Manager

Integrantes

1. Victor Hugo Barrera Garcia
2. Sergio Garcia Hernandez

---

Descripción de la práctica

En esta práctica se implementó un sistema embebido en tiempo real utilizando una ESP32, FreeRTOS, un sensor LDR y un servomotor.

El sistema permite iniciar una operación mediante un botón físico. Una vez iniciada, se habilita la lectura del sensor LDR, se toman muestras del ADC, se aplica un filtro de mediana y se determina un ángulo objetivo para el servomotor.

Dependiendo del nivel de iluminación detectado, el servo se mueve gradualmente hacia `0°` o `180°`. Además, durante la operación se puede mantener presionado el botón de velocidad para activar un movimiento más rápido. Al soltarlo, el sistema regresa al modo lento.

La comunicación principal entre tareas se realiza mediante colas de FreeRTOS. Esto permite que las tareas intercambien mensajes de forma ordenada sin modificar directamente el estado de otras tareas. También se utiliza `pvParameters` para configurar cada tarea al momento de crearla y `TaskHandle_t` para que el `TaskManager` pueda suspender y reanudar tareas según el estado del sistema.

---

Funcionamiento general

El sistema realiza la siguiente secuencia:

1. Al iniciar, el sistema queda en estado listo.
2. El LED de listo parpadea indicando que se puede iniciar una nueva operación.
3. Al presionar el botón `Start-operation`, el sistema comienza la operación.
4. Se detiene el parpadeo del LED de listo.
5. Se habilitan la tarea del sensor, la tarea del servo y el botón de velocidad.
6. El sensor LDR toma muestras del ADC cada `500 ms`.
7. Se aplica un filtro de mediana con ventana de `5` muestras.
8. El `TaskManager` recibe la lectura filtrada y define el ángulo objetivo.
9. El servo se mueve gradualmente hacia `0°` o `180°`.
10. Si se mantiene presionado el botón de velocidad, el servo usa modo rápido.
11. Al soltar el botón de velocidad, el servo regresa al modo lento.
12. Al alcanzar el objetivo, el sistema mantiene la posición durante `8 segundos`.
13. Después de ese tiempo, el sistema suspende las tareas necesarias y vuelve al estado listo.

---

Comportamiento del sensor y servo

| Condición detectada | Acción del sistema |
|--------------------|-------------------|
| Poca o nula luz | El servo se mueve hacia `0°` |
| Luz suficiente | El servo se mueve hacia `180°` |

El movimiento del servo no es instantáneo. La tarea del servo recibe comandos por cola y mueve el motor de forma gradual hasta llegar al objetivo.

---

Modos de velocidad

| Modo | Delay usado en el código | Comportamiento |
|------|--------------------------|----------------|
| Lento | `112 ms` por paso | Movimiento gradual más lento |
| Rápido | `56 ms` por paso | Movimiento gradual más rápido |

El modo rápido solo permanece activo mientras el botón de velocidad está presionado.

---

Tabla de pines

| Elemento | GPIO | Función |
|---------|------|---------|
| Sensor LDR | GPIO34 / ADC1_CH6 | Entrada analógica |
| Servomotor | GPIO25 | Salida PWM LEDC a 50 Hz |
| Botón de velocidad | GPIO18 | Entrada digital con pull-up |
| Botón Start-operation | GPIO19 | Entrada digital con pull-up |
| LED listo | GPIO2 | Indicador de sistema listo |

---

Arquitectura de tareas

| Tarea | Función |
|------|---------|
| `StartButtonTask` | Detecta el botón Start-operation y envía el evento al TaskManager |
| `SpeedButtonTask` | Detecta si el botón de velocidad está presionado o liberado |
| `SensorTask` | Lee el LDR mediante ADC, aplica filtro de mediana y envía el resultado |
| `ServoTask` | Recibe comandos por cola y mueve gradualmente el servomotor |
| `ReadyLedTask` | Hace parpadear el LED cuando el sistema está listo |
| `ManagerTask` | Coordina la operación, procesa eventos, administra colas y controla tareas |

---

Comunicación mediante colas

La práctica utiliza colas de FreeRTOS para comunicar las tareas principales.

Ejemplos de comunicación del sistema:

1. Los botones envían eventos al `TaskManager`.
2. El sensor envía lecturas filtradas del LDR al `TaskManager`.
3. El `TaskManager` envía comandos de movimiento a `ServoTask`.
4. El servo reporta su estado de operación al `TaskManager`.

Este diseño evita que una tarea controle directamente a otra. Por ejemplo, el sensor no mueve el servo por sí mismo; solo envía la información filtrada para que el `TaskManager` tome la decisión.

---

Uso de pvParameters

Todas las tareas reciben una estructura de configuración mediante `pvParameters`.

Esto permite pasar información como:

1. GPIO asignado.
2. Nombre de la tarea.
3. Periodos de muestreo.
4. Tamaño de ventana del filtro.
5. Configuración PWM del servo.
6. Tiempos de parpadeo del LED.
7. Tipo de botón o evento.

Gracias a esto, el código es más flexible y evita crear una función diferente para cada tarea.

---

Uso de TaskHandle_t

El `TaskManager` utiliza `TaskHandle_t` para controlar las tareas del sistema.

Con estos handles puede:

1. Suspender tareas cuando no deben ejecutarse.
2. Reanudar tareas cuando inicia una operación.
3. Evitar que el botón Start-operation reinicie el sistema mientras el servo está en movimiento.
4. Habilitar el botón de velocidad únicamente durante la operación.
5. Administrar la secuencia completa de operación.

---

Filtro de mediana

El sensor LDR puede entregar lecturas con ruido o picos repentinos. Para reducir ese efecto, se implementó un filtro de mediana.

El filtro toma varias muestras, las ordena y selecciona el valor central. En esta práctica se utiliza una ventana de `5` muestras.

Ventajas del filtro de mediana:

1. Rechaza picos aislados.
2. Reduce lecturas erróneas del ADC.
3. Entrega una señal más estable.
4. Mejora la decisión del ángulo objetivo.
5. Es más robusto ante valores extremos que un promedio simple.

Una ventana más grande hace que la señal sea más estable, pero también puede hacer que el sistema responda más lento ante cambios reales de iluminación.

---

Estructura del proyecto

```text
include/
    app_config.hpp
    messages.hpp
    app_context.hpp
    sensor_task.hpp
    button_task.hpp
    servo_task.hpp
    ready_led_task.hpp
    task_manager.hpp

src/
    main.cpp
    app_context.cpp
    sensor_task.cpp
    button_task.cpp
    servo_task.cpp
    ready_led_task.cpp
    task_manager.cpp
````

---

Archivos principales

| Archivo                  | Descripción                                                                       |
| ------------------------ | --------------------------------------------------------------------------------- |
| `main.cpp`               | Punto de entrada del programa. Llama a la creación de tareas.                     |
| `app_config.hpp`         | Contiene pines, tiempos, umbrales, parámetros del servo y constantes del sistema. |
| `messages.hpp`           | Define las estructuras de mensajes usadas en las colas.                           |
| `app_context.hpp/cpp`    | Contiene el contexto global del sistema, colas y handles de tareas.               |
| `sensor_task.hpp/cpp`    | Implementa la lectura del LDR y el filtro de mediana.                             |
| `button_task.hpp/cpp`    | Implementa la lectura de botones con antirrebote y envío de eventos.              |
| `servo_task.hpp/cpp`     | Implementa el control PWM y movimiento gradual del servomotor.                    |
| `ready_led_task.hpp/cpp` | Controla el parpadeo del LED listo.                                               |
| `task_manager.hpp/cpp`   | Coordina el sistema, procesa eventos y administra tareas.                         |

---

Salida esperada en monitor serial

Durante la ejecución se muestran mensajes de depuración por UART.
Los nombres pueden variar ligeramente dependiendo del punto exacto del código, pero la salida debe mostrar eventos del sensor, servo, botones y TaskManager.


Ejemplo de salida esperada:

```text
I (xxx) BUTTON: StartButtonTask iniciado
I (xxx) BUTTON: SpeedButtonTask iniciado
I (xxx) SENSOR: SensorTask iniciado
I (xxx) MANAGER: Sistema iniciado
I (xxx) MANAGER: STATES sensor=READY servo=READY start=BLOCKED speed=READY ready=SUSPENDED
I (xxx) SENSOR: raw=1643 filtered=1619 target=0
I (xxx) SENSOR: raw=1701 filtered=1695 target=0
I (xxx) SERVO: Nueva posicion: 0 grados
I (xxx) SERVO: Servo llego al objetivo: 0 grados
I (xxx) MANAGER: Velocidad: RAPIDA
I (xxx) SERVO: Velocidad actualizada: delay=56 ms
I (xxx) MANAGER: Velocidad: LENTA
I (xxx) SERVO: Velocidad actualizada: delay=112 ms
I (xxx) MANAGER: Operacion terminada despues de 8 segundos
I (xxx) MANAGER: STATES sensor=SUSPENDED servo=SUSPENDED start=BLOCKED speed=SUSPENDED ready=READY
```
---

Compilación y carga

Para compilar el proyecto:

```bash
pio run
```

Para cargar el programa en la ESP32:

```bash
pio run --target upload
```

Para abrir el monitor serial:

```bash
pio device monitor -b 115200
```

---

Preguntas guía

1. ¿Cuál es la diferencia entre usar una variable global y una cola para comunicar datos?

Una variable global puede ser leída o modificada por varias tareas al mismo tiempo, lo cual puede causar errores si no se protege correctamente. Además, no deja claro qué tarea produjo el dato ni cuándo fue actualizado.

Una cola FreeRTOS permite enviar mensajes entre tareas de forma ordenada. La tarea que produce información coloca un mensaje en la cola y la tarea receptora lo lee cuando esté disponible. Esto hace que la comunicación sea más segura, clara y controlada.

---

2. ¿Qué tarea queda bloqueada cuando espera datos de una cola?

La tarea que intenta recibir datos de una cola queda en estado `BLOCKED` si la cola está vacía y se le indicó un tiempo de espera.

Por ejemplo, `TaskManager` puede quedar bloqueada esperando mensajes del sensor, de los botones o del servo. También `ServoTask` puede quedar bloqueada esperando un comando de movimiento.

---

3. ¿Por qué TaskManager debe concentrar las decisiones del sistema?

El `TaskManager` debe concentrar las decisiones porque así se evita que cada tarea modifique el sistema por su cuenta.

En esta práctica, el sensor solo reporta lecturas, los botones solo reportan eventos y el servo solo ejecuta comandos. La lógica principal queda centralizada, lo que facilita entender, depurar y mantener el sistema.

---

4. ¿Por qué `pvParameters` es más flexible que crear una función distinta por tarea?

`pvParameters` permite enviar una estructura de configuración a cada tarea al momento de crearla. Así, una misma función puede reutilizarse con diferentes pines, nombres, tiempos o parámetros.

Esto reduce código repetido y permite ampliar el sistema modificando únicamente la configuración de cada tarea.

---

5. ¿Qué diferencia existe entre suspender una tarea y bloquearla esperando una cola?

Una tarea suspendida con `vTaskSuspend()` queda detenida indefinidamente y no volverá a ejecutarse hasta que otra tarea la reactive con `vTaskResume()`.

En cambio, una tarea bloqueada esperando una cola queda detenida solo mientras espera un mensaje o hasta que termina su tiempo de espera. Si llega un dato a la cola, FreeRTOS puede regresar automáticamente esa tarea al estado `READY`.

---

6. ¿Qué efecto tiene aumentar el tamaño de la ventana del filtro de mediana?

Aumentar el tamaño de la ventana del filtro de mediana hace que la lectura sea más estable y menos sensible a picos o ruido.

Sin embargo, una ventana más grande también puede hacer que el sistema responda más lento ante cambios reales de iluminación, porque necesita más muestras para calcular la nueva mediana.

---

7. ¿Por qué un filtro de mediana rechaza picos mejor que un promedio simple?

Un filtro de mediana ordena las muestras y toma el valor central. Por eso, un valor extremo muy alto o muy bajo no afecta tanto el resultado final.

En cambio, un promedio simple sí se ve afectado por valores extremos, porque suma todas las muestras antes de dividirlas. Por esa razón, la mediana rechaza mejor picos aislados.

---

8. ¿Qué ocurre si se presiona Start-operation mientras el servo está en movimiento?

Si se presiona `Start-operation` mientras el servo está en movimiento, el sistema no debe iniciar una nueva operación.

Durante la operación, la tarea del botón Start-operation queda suspendida para evitar reinicios accidentales. El sistema primero debe terminar la secuencia actual y después volver al estado listo.

---

9. ¿Cómo se garantiza que el botón de velocidad solo funcione durante la operación?

Se garantiza haciendo que el `TaskManager` controle la tarea del botón de velocidad mediante su `TaskHandle_t`.

Al inicio, cuando el sistema está en reposo, la tarea del botón de velocidad permanece suspendida. Cuando inicia una operación, el `TaskManager` la reanuda. Al terminar la operación, vuelve a suspenderla.

---

10. ¿Por qué un `constexpr` en C++ es preferible a `#define` para constantes tipadas?

Un `constexpr` es preferible porque tiene tipo de dato y el compilador puede verificarlo. Esto ayuda a evitar errores de tipo y hace el código más seguro.

En cambio, `#define` solo realiza una sustitución de texto antes de compilar, por lo que no tiene tipo ni respeta el alcance del lenguaje. En C++, `constexpr` es más claro, seguro y recomendable para constantes.

---

## Estado

Práctica validada en ESP32.
El sistema realiza la secuencia Start-operation, lectura del LDR, filtrado por mediana, movimiento gradual del servo, espera de 8 segundos y regreso al estado listo.

```
```
