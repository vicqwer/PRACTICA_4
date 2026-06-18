/**
 * CONCLUSION DEL EQUIPO
 *
 * Integrantes: Victor Hugo Barrera Garcia, Sergio Garcia Hernandez
 *
 * En esta practica se implemento un sistema de control para un servomotor usando
 * una ESP32, un sensor LDR, FreeRTOS y comunicacion entre tareas mediante colas.
 * El sistema permite iniciar una operacion con un boton, leer la iluminacion con
 * el ADC, aplicar un filtro de mediana y mover gradualmente el servo hacia 0 o
 * 180 grados dependiendo de la luz detectada.
 *
 * Se comprobo que el uso de colas es mas seguro y ordenado que comunicar datos
 * mediante variables globales, ya que cada tarea envia mensajes estructurados al
 * TaskManager sin modificar directamente el estado de otras tareas. De esta
 * manera, Task_Sensor no controla directamente al servo y los botones no cambian
 * la logica del sistema por si solos, sino que reportan eventos para que el
 * TaskManager tome las decisiones.
 *
 * El TaskManager funciono como el centro de control del sistema, ya que fue la
 * unica tarea encargada de suspender y reanudar tareas mediante TaskHandle_t,
 * procesar eventos de botones, recibir lecturas filtradas del sensor y enviar
 * comandos al servo. Esto permitio mantener una arquitectura mas clara, evitando
 * que la logica de control quedara distribuida en varias tareas.
 *
 * El uso de pvParameters permitio que cada tarea recibiera su propia estructura
 * de configuracion al ser creada, como pines, nombres, tiempos y parametros de
 * operacion. Esto hizo el codigo mas flexible que crear una funcion diferente
 * para cada tarea, ya que se pudo reutilizar la misma logica cambiando solo la
 * configuracion recibida.
 *
 * Tambien se observo la diferencia entre una tarea suspendida y una tarea
 * bloqueada esperando una cola. Una tarea bloqueada puede reanudarse
 * automaticamente cuando llega un mensaje o termina su tiempo de espera, mientras
 * que una tarea suspendida necesita que el TaskManager la reactive explicitamente.
 * Esto fue util para impedir que Start-operation reiniciara el sistema mientras
 * el servo estaba en movimiento y para habilitar el boton de velocidad solo
 * durante una operacion activa.
 *
 * Finalmente, el filtro de mediana permitio reducir picos o lecturas erroneas
 * del LDR antes de decidir el angulo objetivo. Al aumentar la ventana del filtro,
 * la senal se vuelve mas estable, aunque tambien puede responder mas lento a
 * cambios rapidos de iluminacion. Con esto se comprendio la importancia de
 * combinar filtrado, colas, tareas y administracion centralizada para construir
 * un sistema embebido mas robusto y ordenado.
 * =============================================================================
 */

#include "task_manager.hpp"

extern "C" void app_main(void)
{
    App::app_tasks_create();
}
