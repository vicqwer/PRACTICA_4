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
        /* CONFIGURACION DEL SENSOR :)*/
        AppConfig::LDR_ADC_UNIT,
    AppConfig::LDR_ADC_CHANNEL,
    AppConfig::SENSOR_PERIOD_MS,
    AppConfig::FILTER_WINDOW_SIZE,
    "SensorTask"
    };

    static ServoTaskConfig servo_cfg 
    { 
        /*CONFIGURACION DEL SERVO :)*/
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
        /* CONFIGURACION DEL BOTON DE INICIO :)*/
         static_cast<uint8_t>(AppConfig::START_BUTTON_GPIO),
    "StartButtonTask",
    ButtonEventType::Start,
    AppConfig::BUTTON_POLL_MS,
    true
    };

    static ButtonTaskConfig speed_btn_cfg 
    { 
        /* CONFIGURACION DEL BOTON DE VELOCIDAD :)*/
        static_cast<uint8_t>(AppConfig::SPEED_BUTTON_GPIO),
    "SpeedButtonTask",
    ButtonEventType::SpeedState,
    AppConfig::BUTTON_POLL_MS,
    true
    };

    static ReadyLedTaskConfig ready_led_cfg
    {
        /* CONFIGURACION DEL LED LISTO :)*/
         static_cast<uint8_t>(AppConfig::READY_LED_GPIO),
    AppConfig::READY_LED_ON_MS,
    AppConfig::READY_LED_OFF_MS,
    "ReadyLedTask"
    };

    static ManagerTaskConfig manager_cfg 
    { 
        /* CONFIGURACION DEL ADMINISTRADOR DE TAREAS :)*/
         AppConfig::HOLD_TARGET_MS,
    AppConfig::SERVO_TOLERANCE_DEG,
    AppConfig::SERVO_DELAY_SLOW_MS,
    AppConfig::SERVO_DELAY_FAST_MS,
    AppConfig::SERVO_STEP_DEG,
    "ManagerTask"
    };

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

    static void send_servo_cmd(uint8_t target, bool fast, const ManagerTaskConfig *cfg)
    {
        ServoCmd cmd 
        { 
            target, 
            cfg->tolerance_deg, 
            fast ? cfg->fast_delay_ms : cfg->slow_delay_ms, 
            cfg->step_deg 
        };
        xQueueOverwrite(g_queues.servo_cmd, &cmd);
    }

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
        auto *cfg = static_cast<ManagerTaskConfig *>(pvParameters);

        bool fast_mode = false;
        bool operating = false;
        bool reached = false;
        uint8_t current_target = AppConfig::SERVO_ANGLE_DARK;
        TickType_t reached_tick = 0;

        vTaskSuspend(g_handles.sensor);
        vTaskSuspend(g_handles.servo);
        vTaskSuspend(g_handles.speed_button);
        vTaskResume(g_handles.ready_led);

        while (true)
        {
                /* IMPLEMENTAR TAREA TASK MANAGER :)*/
                 ButtonMsg btn_msg;
while (xQueueReceive(g_queues.buttons, &btn_msg, 0) == pdTRUE)
{
        if (btn_msg.type == ButtonEventType::Start)
        {
            // Solo procesamos si es una pulsación (presionado)
            if (btn_msg.pressed == true)
            {
                // Verificamos que no esté ya en operación para evitar reinicios accidentales
                if (!operating)
                {
                    ESP_LOGI(TAG, "Sistema iniciado");

                    operating = true;
                    reached = false;
                    fast_mode = false;

                    // Suspender LED y reanudar tareas de trabajo
                    vTaskSuspend(g_handles.ready_led);
                    vTaskResume(g_handles.sensor);
                    vTaskResume(g_handles.servo);
                    vTaskResume(g_handles.speed_button);

                    // IMPORTANTE: Enviar el primer comando al servo (por defecto oscuridad o el último conocido)
                    // Si no hay dato en la cola del sensor aún, usamos el valor por defecto de oscuridad (0°)
                    SensorMsg sensor_msg;
                    if (xQueuePeek(g_queues.sensor, &sensor_msg, 0) == pdTRUE)
                    {
                        current_target = sensor_msg.target_angle;
                    }
                    else
                    {
                        current_target = AppConfig::SERVO_ANGLE_DARK;
                    }
                    
                    send_servo_cmd(current_target, fast_mode, cfg);

                    print_states();
                }
            }
        }

        if (btn_msg.type == ButtonEventType::SpeedState && operating)
        {
            fast_mode = btn_msg.pressed;

            ESP_LOGI(TAG, "Velocidad: %s", fast_mode ? "RAPIDA" : "LENTA");

            send_servo_cmd(current_target, fast_mode, cfg);
        }
    }

    if (operating)
    {
        SensorMsg sensor_msg;

        if (xQueueReceive(g_queues.sensor, &sensor_msg, 0) == pdTRUE)
        {
            TickType_t now = xTaskGetTickCount();

            if (sensor_msg.target_angle != current_target)
            {
                if (!reached || ((now - reached_tick) >= pdMS_TO_TICKS(cfg->hold_target_ms)))
                {
                    current_target = sensor_msg.target_angle;
                    reached = false;

                    send_servo_cmd(current_target, fast_mode, cfg);
                }
            }
        }

        ServoStatusMsg servo_status;

        while (xQueueReceive(g_queues.servo_status, &servo_status, 0) == pdTRUE)
        {
            if (servo_status.status == ServoStatusType::Reached)
            {
                reached = true;
                reached_tick = xTaskGetTickCount();
            }
        }
        if (reached)
        {
            TickType_t now = xTaskGetTickCount();

            if ((now - reached_tick) >= pdMS_TO_TICKS(cfg->hold_target_ms))
            {
                ESP_LOGI(TAG, "Operacion terminada despues de 8 segundos");

                operating = false; // Esto lo tenías bien
                reached = false;
                fast_mode = false;

                vTaskSuspend(g_handles.sensor);
                vTaskSuspend(g_handles.servo);
                vTaskSuspend(g_handles.speed_button);
                vTaskResume(g_handles.ready_led);

                print_states();
            }
        }
    }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    void app_tasks_create()
    {
        /* VERIFIQUEN LA PRIORIDAD DE LAS TAREAS y/o AJUSTENLAS EN FUNCION DE SUS RESULTADOS*/
        g_queues.sensor = xQueueCreate(1, sizeof(SensorMsg));
        g_queues.buttons = xQueueCreate(AppConfig::BUTTON_QUEUE_LEN, sizeof(ButtonMsg));
        g_queues.servo_cmd = xQueueCreate(1, sizeof(ServoCmd));
        g_queues.servo_status = xQueueCreate(1, sizeof(ServoStatusMsg));

xTaskCreate(SensorTask::run, sensor_cfg.name, 4096, &sensor_cfg, 3, &g_handles.sensor);
xTaskCreate(ServoTask::run, servo_cfg.name, 4096, &servo_cfg, 3, &g_handles.servo);
xTaskCreate(ButtonTask::run, start_btn_cfg.name, 2048, &start_btn_cfg, 4, &g_handles.start_button);
xTaskCreate(ButtonTask::run, speed_btn_cfg.name, 2048, &speed_btn_cfg, 4, &g_handles.speed_button);
xTaskCreate(ReadyLedTask::run, ready_led_cfg.name, 2048, &ready_led_cfg, 1, &g_handles.ready_led);
xTaskCreate(TaskManager::run, manager_cfg.name, 4096, &manager_cfg, 5, &g_handles.manager);
    }
}
