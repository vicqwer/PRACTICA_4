#include "servo_task.hpp"
#include "app_context.hpp"
#include "messages.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

namespace App
{
    static const char *TAG = "SERVO";

  static uint32_t angle_to_duty(const ServoTaskConfig *cfg, uint8_t angle)
{


    uint32_t max_duty = (1UL << cfg->resolution) - 1UL;

    uint32_t pulse_us =
        cfg->min_us + ((uint32_t)angle * (cfg->max_us - cfg->min_us)) / 180UL;

    uint32_t period_us = 1000000UL / cfg->freq_hz;

    return (pulse_us * max_duty) / period_us;
}
    static void servo_write_angle(const ServoTaskConfig *cfg, uint8_t angle)
    {
        uint32_t duty = angle_to_duty(cfg, angle);
        ledc_set_duty(cfg->mode, cfg->channel, duty);
        ledc_update_duty(cfg->mode, cfg->channel);
    }


    void ServoTask::run(void *pvParameters)
    {
        auto *cfg = static_cast<ServoTaskConfig *>(pvParameters);

        ledc_timer_config_t timer_cfg = {
            .speed_mode = cfg->mode,
            .duty_resolution = cfg->resolution,
            .timer_num = cfg->timer,
            .freq_hz = cfg->freq_hz,
            .clk_cfg = LEDC_AUTO_CLK,
            .deconfigure = false
        };
        ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

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

        uint8_t current_angle = 0;
        servo_write_angle(cfg, current_angle);

while (true)
        {
            /*implementar tarea :)*/

            ServoCmd cmd;

            if (xQueueReceive(g_queues.servo_cmd, &cmd, portMAX_DELAY) == pdTRUE)
            {
                ESP_LOGI(TAG, "Nueva posicion: %d grados", cmd.target_angle);

                while (current_angle != cmd.target_angle)
                {
                    ServoCmd new_cmd;

                    if (xQueueReceive(g_queues.servo_cmd, &new_cmd, 0) == pdTRUE)
                    {
                        cmd.step_delay_ms = new_cmd.step_delay_ms;
                        cmd.step_deg = new_cmd.step_deg;
                        cmd.tolerance_deg = new_cmd.tolerance_deg;

                        ESP_LOGI(TAG, "Velocidad actualizada: delay=%d ms", cmd.step_delay_ms);
                    }

                    if (current_angle < cmd.target_angle)
                    {
                        current_angle += cmd.step_deg;

                        if (current_angle > cmd.target_angle)
                        {
                            current_angle = cmd.target_angle;
                        }
                    }
                    else
                    {
                        if (current_angle > cmd.step_deg)
                        {
                            current_angle -= cmd.step_deg;
                        }
                        else
                        {
                            current_angle = 0;
                        }

                        if (current_angle < cmd.target_angle)
                        {
                            current_angle = cmd.target_angle;
                        }
                    }

                    servo_write_angle(cfg, current_angle);
//ESP_LOGI(TAG, "Angulo actual: %d grados", static_cast<int>(current_angle));
                    ServoStatusMsg moving_msg =
                    {
                        ServoStatusType::Moving,
                        current_angle,
                        cmd.target_angle,
                        static_cast<uint32_t>(xTaskGetTickCount())
                    };

                    xQueueOverwrite(g_queues.servo_status, &moving_msg);

                    vTaskDelay(pdMS_TO_TICKS(cmd.step_delay_ms));
                }

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
