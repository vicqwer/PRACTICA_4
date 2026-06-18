#include "ready_led_task.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

namespace App
{
    /**
     * @brief Función principal de la tarea del LED de estado.
     *        Su función es parpadear de forma continua cuando el sistema está en reposo.
     *        Cuando la tarea es suspendida por TaskManager, el parpadeo se detiene.
     */
    void ReadyLedTask::run(void *pvParameters)
    {
        // 1. Recibir la configuración a través del puntero pvParameters
        auto *cfg = static_cast<ReadyLedTaskConfig *>(pvParameters);

        // 2. Configurar el pin del LED como salida digital
        gpio_reset_pin(static_cast<gpio_num_t>(cfg->gpio));
        gpio_set_direction(static_cast<gpio_num_t>(cfg->gpio), GPIO_MODE_OUTPUT);

        // 3. Bucle infinito de parpadeo
        while (true)
        {
            // Encender el LED
            gpio_set_level(static_cast<gpio_num_t>(cfg->gpio), 1);
            
            // Esperar el tiempo configurado en estado ON (250 ms según PDF)
            vTaskDelay(pdMS_TO_TICKS(cfg->on_ms));

            // Apagar el LED
            gpio_set_level(static_cast<gpio_num_t>(cfg->gpio), 0);
            
            // Esperar el tiempo configurado en estado OFF (100 ms según PDF)
            vTaskDelay(pdMS_TO_TICKS(cfg->off_ms));
        }
    }
}