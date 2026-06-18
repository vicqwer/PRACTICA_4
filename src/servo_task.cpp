#include "servo_task.hpp"
#include "app_context.hpp"
#include "messages.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

namespace App
{
    // Etiqueta para los mensajes de log del servo
    static const char *TAG = "SERVO";

    /**
     * @brief Convierte un ángulo en grados (0-180) al valor de Duty Cycle (PWM) requerido por el ESP32.
     * 
     * @param cfg Configuración del servo (frecuencia, resolución, microsegundos min/max).
     * @param angle Ángulo objetivo en grados (0 a 180).
     * @return uint32_t El valor del duty cycle para configurar el PWM del LEDC.
     */
    static uint32_t angle_to_duty(const ServoTaskConfig *cfg, uint8_t angle)
    {
        // Calcular el valor máximo del duty cycle (depende de la resolución, ej: 16 bits = 65535)
        uint32_t max_duty = (1UL << cfg->resolution) - 1UL;

        // Mapear el ángulo (0-180) al ancho de pulso en microsegundos (500us - 2500us)
        uint32_t pulse_us =
            cfg->min_us + ((uint32_t)angle * (cfg->max_us - cfg->min_us)) / 180UL;

        // Calcular el periodo de la señal PWM en microsegundos (50Hz -> 20000us)
        uint32_t period_us = 1000000UL / cfg->freq_hz;

        // Regla de 3 para convertir microsegundos al valor entero del duty cycle
        return (pulse_us * max_duty) / period_us;
    }

    /**
     * @brief Escribe físicamente el ángulo en el servo actualizando el Duty Cycle del PWM.
     * 
     * @param cfg Configuración del servo.
     * @param angle Ángulo a establecer (0-180).
     */
    static void servo_write_angle(const ServoTaskConfig *cfg, uint8_t angle)
    {
        uint32_t duty = angle_to_duty(cfg, angle);
        ledc_set_duty(cfg->mode, cfg->channel, duty);
        ledc_update_duty(cfg->mode, cfg->channel);
    }

    /**
     * @brief Función principal de la tarea del Servomotor.
     *        Escucha comandos por cola y mueve el servo de forma gradual y suave.
     */
    void ServoTask::run(void *pvParameters)
    {
        // 1. Recibir la configuración a través del puntero pvParameters
        auto *cfg = static_cast<ServoTaskConfig *>(pvParameters);

        // 2. Configurar el hardware del Timer PWM del LEDC
        ledc_timer_config_t timer_cfg = {
            .speed_mode = cfg->mode,
            .duty_resolution = cfg->resolution,
            .timer_num = cfg->timer,
            .freq_hz = cfg->freq_hz,
            .clk_cfg = LEDC_AUTO_CLK,
            .deconfigure = false
        };
        ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

        // 3. Configurar el canal PWM que irá conectado al GPIO del servo
        ledc_channel_config_t ch_cfg = {
            .gpio_num = cfg->gpio,
            .speed_mode = cfg->mode,
            .channel = cfg->channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = cfg->timer,
            .duty = 0,
            .hpoint = 0,
            .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
            .flags = {
                .output_invert = 0
            },
            .deconfigure = false
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

        // 4. Inicializar el servo en la posición 0° al arrancar el sistema
        uint8_t current_angle = 0;
        servo_write_angle(cfg, current_angle);

        // 5. Bucle infinito de la tarea
        while (true)
        {
            ServoCmd cmd;

            // Esperar (bloqueándose) hasta que llegue un comando a la cola
            if (xQueueReceive(g_queues.servo_cmd, &cmd, portMAX_DELAY) == pdTRUE)
            {
                ESP_LOGI(TAG, "Nueva posicion: %d grados", cmd.target_angle);

                // Movimiento gradual: avanzar paso a paso hasta alcanzar el objetivo
                while (current_angle != cmd.target_angle)
                {
                    ServoCmd new_cmd;

                    // Verificar si llegó un nuevo comando de velocidad MIENTRAS nos estamos moviendo
                    if (xQueueReceive(g_queues.servo_cmd, &new_cmd, 0) == pdTRUE)
                    {
                        // Actualizar la velocidad (rápida/lenta) sobre la marcha
                        cmd.step_delay_ms = new_cmd.step_delay_ms;
                        cmd.step_deg = new_cmd.step_deg;
                        cmd.tolerance_deg = new_cmd.tolerance_deg;
                        cmd.target_angle = new_cmd.target_angle; // También actualizar destino si cambia

                        ESP_LOGI(TAG, "Velocidad actualizada: delay=%d ms", cmd.step_delay_ms);
                    }

                    // Calcular la diferencia para evitar sobrepaso (overshoot)
                    int16_t difference = cmd.target_angle - current_angle;

                    if (difference > 0)
                    {
                        // Si la diferencia es menor o igual al paso, vamos directo al objetivo exacto
                        if (difference <= cmd.step_deg)
                        {
                            current_angle = cmd.target_angle;
                        }
                        else
                        {
                            current_angle += cmd.step_deg;
                        }
                    }
                    else if (difference < 0)
                    {
                        // Si la diferencia absoluta es menor o igual al paso, vamos directo al objetivo exacto
                        if (-difference <= cmd.step_deg)
                        {
                            current_angle = cmd.target_angle;
                        }
                        else
                        {
                            current_angle -= cmd.step_deg;
                        }
                    }

                    // Escribir el nuevo ángulo en el servo
                    servo_write_angle(cfg, current_angle);

                    // Reportar estado de "En movimiento" al TaskManager
                    ServoStatusMsg moving_msg =
                    {
                        ServoStatusType::Moving,
                        current_angle,
                        cmd.target_angle,
                        static_cast<uint32_t>(xTaskGetTickCount())
                    };
                    xQueueOverwrite(g_queues.servo_status, &moving_msg);

                    // Esperar el tiempo configurado entre cada paso (determina la velocidad)
                    vTaskDelay(pdMS_TO_TICKS(cmd.step_delay_ms));
                }

                // Una vez que el bucle termina, significa que alcanzamos el objetivo exacto
                ServoStatusMsg reached_msg =
                {
                    ServoStatusType::Reached,
                    current_angle,
                    cmd.target_angle,
                    static_cast<uint32_t>(xTaskGetTickCount())
                };
                xQueueOverwrite(g_queues.servo_status, &reached_msg);

                ESP_LOGI(TAG, "Servo llego al objetivo: %d grados", current_angle);
            }
        }
    }
}