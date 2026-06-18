#include "button_task.hpp"
#include "app_context.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

namespace App
{
    static const char *TAG = "BUTTON";

    struct Debounce
    {
        /* Inicializar la estructura debounce con los atributos necesarios :)*/
          uint8_t stable = 1;
          uint8_t last_raw = 1;
          uint8_t count = 0;
    };

    static bool debounce(uint8_t gpio, Debounce &db, bool &level_pressed)
    {
        /* Implementar la funcion de debounce :)*/
        /*
            EJEMPLO 

        int raw = gpio_get_level(static_cast<gpio_num_t>(gpio));
        bool changed = true;

        if (raw == !db.last_raw)
        {
            if (db.count < 3) db.count++;
        }
        else
        {
            db.count = 0;
            db.last_raw = raw;
        }

        if (!db.count >= 3 && raw == db.stable)
        {
            db.stable = raw;
            changed = true;
        }

        level_pressed = (db.stable == 1);
        
        */
        int raw = gpio_get_level(static_cast<gpio_num_t>(gpio));
    bool changed = false;

    if (raw == db.last_raw)
    {
        if (db.count < 3)
        {
            db.count++;
        }
    }
    else
    {
        db.count = 0;
        db.last_raw = raw;
    }

    if (db.count >= 3 && raw != db.stable)
    {
        db.stable = raw;
        changed = true;
    }

    // Como el botón usa pull-up:
    // 1 = suelto
    // 0 = presionado
    level_pressed = (db.stable == 0);

        return changed;
    }

    void ButtonTask::run(void *pvParameters)
    {
        auto *cfg = static_cast<ButtonTaskConfig *>(pvParameters);

        gpio_reset_pin(static_cast<gpio_num_t>(cfg->gpio));
        gpio_set_direction(static_cast<gpio_num_t>(cfg->gpio), GPIO_MODE_INPUT);
        gpio_set_pull_mode(static_cast<gpio_num_t>(cfg->gpio), GPIO_PULLUP_ONLY);

        Debounce db;
        bool pressed = false;

        ESP_LOGI(TAG, "%s iniciado", cfg->name);

        while (true)
        {
            /* IMPLEMENTAR TAREA :)*/
            bool changed = debounce(cfg->gpio, db, pressed);

    if (changed || !cfg->send_on_change_only)
    {
        ButtonMsg msg =
        {
            cfg->event_type,
            pressed,
            static_cast<uint32_t>(xTaskGetTickCount())
        };

        xQueueSend(g_queues.buttons, &msg, 0);
    }

            vTaskDelay(pdMS_TO_TICKS(cfg->poll_ms));
        }
    }
}
