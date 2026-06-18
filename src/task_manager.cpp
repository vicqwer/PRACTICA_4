#include "task_manager.hpp"
#include "app_config.hpp"
#include "app_context.hpp"
#include "messages.hpp"
#include "sensor_task.hpp"
#include "button_task.hpp"
#include "servo_task.hpp"
#include "ready_led_task.hpp"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

namespace App
{
    static const char *TAG = "MANAGER";

    static SensorTaskConfig sensor_cfg 
    { 
/*Configuración para la tarea del sensor LDR: unidad ADC, canal, periodo de muestreo, tamaño del filtro y nombre*/
    AppConfig::LDR_ADC_UNIT,
    AppConfig::LDR_ADC_CHANNEL,
    AppConfig::SENSOR_PERIOD_MS,
    AppConfig::FILTER_WINDOW_SIZE,
    "SensorTask"
    };

    static ServoTaskConfig servo_cfg 
    { 
        /* // Configuración para la tarea del servo: GPIO, canal/timer PWM,
         resolución, frecuencia y microsegundos mín/máx)*/
         static_cast<uint8_t>(AppConfig::SERVO_GPIO),
    AppConfig::SERVO_PWM_CHANNEL,
    AppConfig::SERVO_PWM_TIMER,
    AppConfig::SERVO_PWM_MODE,
    AppConfig::SERVO_PWM_RES_BITS,
    AppConfig::SERVO_PWM_FREQ_HZ,
    AppConfig::SERVO_MIN_US,
    AppConfig::SERVO_MAX_US,
    "ServoTask"
    };

    static ButtonTaskConfig start_btn_cfg 
    { 
        /* Configuración para el botón de Inicio: GPIO, nombre, tipo de evento (Start),
         tiempo de sondeo y enviar solo al cambiar*/
         static_cast<uint8_t>(AppConfig::START_BUTTON_GPIO),
    "StartButtonTask",
    ButtonEventType::Start,
    AppConfig::BUTTON_POLL_MS,
    true
    };

    static ButtonTaskConfig speed_btn_cfg 
    { 
        /* Configuración para el botón de Velocidad: GPIO, nombre, tipo de evento (SpeedState), tiempo de sondeo*/
        static_cast<uint8_t>(AppConfig::SPEED_BUTTON_GPIO),
    "SpeedButtonTask",
    ButtonEventType::SpeedState,
    AppConfig::BUTTON_POLL_MS,
    true
    };

    static ReadyLedTaskConfig ready_led_cfg
    {
        /*Configuración del LED de listo: GPIO, tiempo encendido, tiempo apagado y nombre */
         static_cast<uint8_t>(AppConfig::READY_LED_GPIO),
    AppConfig::READY_LED_ON_MS,
    AppConfig::READY_LED_OFF_MS,
    "ReadyLedTask"
    };

    static ManagerTaskConfig manager_cfg 
    { 
        /*Configuración del TaskManager: tiempo de espera en objetivo (8s), tolerancia, delays lento/rápido, paso y nombre*/
         AppConfig::HOLD_TARGET_MS,
    AppConfig::SERVO_TOLERANCE_DEG,
    AppConfig::SERVO_DELAY_SLOW_MS,
    AppConfig::SERVO_DELAY_FAST_MS,
    AppConfig::SERVO_STEP_DEG,
    "ManagerTask"
    };
//Convierte el estado interno de una tarea de FreeRTOS a un string legible para imprimirlo en el monitor serial
    static const char *state_text(eTaskState st)
    {
        switch (st)
        {
            case eRunning: 
                return "RUNNING";
            case eReady: 
                return "READY";
            case eBlocked: 
                return "BLOCKED";
            case eSuspended: 
                return "SUSPENDED";
            case eDeleted: 
                return "DELETED";
            default: 
                return "UNKNOWN";
        }
    }
    // Construye un comando (ServoCmd) con el ángulo objetivo, la velocidad (rápida o lenta) y lo envía a la cola del servo
    static void send_servo_cmd(uint8_t target, bool fast, const ManagerTaskConfig *cfg)
    {
        ServoCmd cmd 
        { 
            target, 
            cfg->tolerance_deg, 
            fast ? cfg->fast_delay_ms : cfg->slow_delay_ms, 
            cfg->step_deg 
        };
        // Usamos Overwrite porque la cola solo tiene espacio para 1 comando 
        xQueueOverwrite(g_queues.servo_cmd, &cmd);
    }
   // Imprime en el puerto serie el estado actual de todas las tareas del sistema para depuración
    static void print_states()
    {
        ESP_LOGI(TAG, "STATES sensor=%s servo=%s start=%s speed=%s ready=%s",
                 state_text(eTaskGetState(g_handles.sensor)),
                 state_text(eTaskGetState(g_handles.servo)),
                 state_text(eTaskGetState(g_handles.start_button)),
                 state_text(eTaskGetState(g_handles.speed_button)),
                 state_text(eTaskGetState(g_handles.ready_led)));
    }

    void TaskManager::run(void *pvParameters)
    {
        // Recibimos la configuración a través de pvParameters (tal como exige la práctica)
        auto *cfg = static_cast<ManagerTaskConfig *>(pvParameters);

        // Variables de estado del sistema
        bool fast_mode = false;     // Indica si el servo va rápido o lento
        bool operating = false;     // Bandera que indica si una operación está en curso
        bool reached = false;       // Bandera que indica si el servo ya alcanzó el ángulo objetivo
        uint8_t current_target = AppConfig::SERVO_ANGLE_DARK; // Ángulo actual al que debe apuntar el servo
        TickType_t reached_tick = 0; // Instante de tiempo (tick) en que el servo llegó al objetivo

        // Configuración inicial: Al arrancar, el sistema está en reposo
        vTaskSuspend(g_handles.sensor);      // Sensor apagado
        vTaskSuspend(g_handles.servo);       // Servo apagado
        vTaskSuspend(g_handles.speed_button);// Botón de velocidad apagado
        vTaskResume(g_handles.ready_led);    // LED de listo encendido (parpadeando)

        while (true) // Bucle infinito de la tarea del administrador
        {
            /* -- PROCESAMIENTO DE MENSAJES DE BOTONES -- */
            ButtonMsg btn_msg;
            // Leemos todos los mensajes de botones que hayan llegado a la cola (sin bloquearnos)
            while (xQueueReceive(g_queues.buttons, &btn_msg, 0) == pdTRUE)
            {
                if (btn_msg.type == ButtonEventType::Start)
                {
                    // Solo procesamos si es una pulsación (presionado), ignoramos el evento de soltar
                    if (btn_msg.pressed == true)
                    {
                        // Verificamos que no esté ya en operación para evitar reinicios accidentales
                        if (!operating)
                        {
                            ESP_LOGI(TAG, "Sistema iniciado");

                            // Cambiamos el estado a "En operación"
                            operating = true;
                            reached = false;
                            fast_mode = false;

                            // Intercambiamos tareas: Apagamos el LED, encendemos el sensor, servo y botón de velocidad
                            vTaskSuspend(g_handles.ready_led);
                            vTaskResume(g_handles.sensor);
                            vTaskResume(g_handles.servo);
                            vTaskResume(g_handles.speed_button);

                            // Tomamos el primer dato del sensor (si ya hay alguno), o empezamos en 0° (oscuro)
                            SensorMsg sensor_msg;
                            if (xQueuePeek(g_queues.sensor, &sensor_msg, 0) == pdTRUE)
                            {
                                current_target = sensor_msg.target_angle;
                            }
                            else
                            {
                                current_target = AppConfig::SERVO_ANGLE_DARK;
                            }
                            
                            // Enviamos el comando inicial al servo
                            send_servo_cmd(current_target, fast_mode, cfg);
                            print_states();
                        }
                    }
                }

                // Si es un mensaje del botón de velocidad y el sistema está en operación, actualizamos la velocidad
                if (btn_msg.type == ButtonEventType::SpeedState && operating)
                {
                    fast_mode = btn_msg.pressed; // true = rápido, false = lento
                    ESP_LOGI(TAG, "Velocidad: %s", fast_mode ? "RAPIDA" : "LENTA");
                    // Enviamos el comando actualizado al servo (cambia el delay entre pasos)
                    send_servo_cmd(current_target, fast_mode, cfg);
                }
            }

            /*  PROCESAMIENTO DE DATOS DEL SENSOR  */
            if (operating)
            {
                SensorMsg sensor_msg;
                // Si hay un nuevo dato del LDR en la cola, lo leemos
                if (xQueueReceive(g_queues.sensor, &sensor_msg, 0) == pdTRUE)
                {
                    TickType_t now = xTaskGetTickCount();

                    // Si el ángulo que pide el sensor es diferente al actual...
                    if (sensor_msg.target_angle != current_target)
                    {
                        // Solo actualizamos el objetivo si NO hemos llegado aún, O si ya pasaron los 8 segundos de espera
                        if (!reached || ((now - reached_tick) >= pdMS_TO_TICKS(cfg->hold_target_ms)))
                        {
                            current_target = sensor_msg.target_angle;
                            reached = false; // Reseteamos la bandera de llegada porque el objetivo cambió
                            send_servo_cmd(current_target, fast_mode, cfg);
                        }
                    }
                }

                /*  PROCESAMIENTO DEL ESTADO DEL SERVO  */
                ServoStatusMsg servo_status;
                // Leemos el estado que nos envía la tarea del servo (Moving o Reached)
                while (xQueueReceive(g_queues.servo_status, &servo_status, 0) == pdTRUE)
                {
                    if (servo_status.status == ServoStatusType::Reached)
                    {
                        reached = true;
                        reached_tick = xTaskGetTickCount(); // Guardamos el tiempo exacto en que llegó
                    }
                }

                /*  CONTROL DE FINALIZACIÓN DE OPERACIÓN (8 SEGUNDOS) -- */
                if (reached)
                {
                    TickType_t now = xTaskGetTickCount();

                    // Si ya pasaron los 8 segundos (hold_target_ms) desde que el servo llegó al objetivo...
                    if ((now - reached_tick) >= pdMS_TO_TICKS(cfg->hold_target_ms))
                    {
                        ESP_LOGI(TAG, "Operacion terminada despues de 8 segundos");

                        // Volvemos al estado de reposo
                        operating = false;
                        reached = false;
                        fast_mode = false;

                        // Apagamos sensor, servo y velocidad, y encendemos el LED de listo
                        vTaskSuspend(g_handles.sensor);
                        vTaskSuspend(g_handles.servo);
                        vTaskSuspend(g_handles.speed_button);
                        vTaskResume(g_handles.ready_led);

                        print_states();
                    }
                }
            }
            
            // Pequeño delay para no saturar el procesador con el bucle while(true)
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    void app_tasks_create()
    {
        /* Creación de las colas de comunicación entre tareas */
        // La cola del sensor solo guarda el último dato (1 elemento)
        g_queues.sensor = xQueueCreate(1, sizeof(SensorMsg));
        // La cola de botones guarda varios mensajes (por si llegan rápido)
        g_queues.buttons = xQueueCreate(AppConfig::BUTTON_QUEUE_LEN, sizeof(ButtonMsg));
        g_queues.servo_cmd = xQueueCreate(1, sizeof(ServoCmd));
        g_queues.servo_status = xQueueCreate(1, sizeof(ServoStatusMsg));

        /* Creación de las tareas usando xTaskCreate y pasando pvParameters (estructuras de configuración) */
        // Prioridad 3: Tarea del sensor
        xTaskCreate(SensorTask::run, sensor_cfg.name, 4096, &sensor_cfg, 3, &g_handles.sensor);
        // Prioridad 3: Tarea del servo
        xTaskCreate(ServoTask::run, servo_cfg.name, 4096, &servo_cfg, 3, &g_handles.servo);
        // Prioridad 4: Botón de inicio (más prioridad que sensor y servo para no perder pulsaciones)
        xTaskCreate(ButtonTask::run, start_btn_cfg.name, 2048, &start_btn_cfg, 4, &g_handles.start_button);
        // Prioridad 4: Botón de velocidad
        xTaskCreate(ButtonTask::run, speed_btn_cfg.name, 2048, &speed_btn_cfg, 4, &g_handles.speed_button);
        // Prioridad 1: LED de listo (la prioridad más baja, solo parpadea)
        xTaskCreate(ReadyLedTask::run, ready_led_cfg.name, 2048, &ready_led_cfg, 1, &g_handles.ready_led);
        // Prioridad 5: TaskManager (La prioridad más alta porque es el cerebro que coordina todo)
        xTaskCreate(TaskManager::run, manager_cfg.name, 4096, &manager_cfg, 5, &g_handles.manager);
    }
}
