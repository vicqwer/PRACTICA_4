#include "button_task.hpp"
#include "app_context.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

namespace App
{
    // Etiqueta para los mensajes de log de esta tarea
    static const char *TAG = "BUTTON";

    // Estructura para mantener el estado del filtro de rebote (debounce) de un botón
    struct Debounce
    {
        uint8_t stable = 1;    // Estado lógico estable confirmado (1 = suelto, 0 = presionado)
        uint8_t last_raw = 1;  // Último valor leído directamente del GPIO
        uint8_t count = 0;     // Contador de veces que el valor raw se ha mantenido igual
    };

    /**
     * @brief Función de filtrado por rebote para evitar lecturas falsas debido al rebote mecánico del botón.
     * 
     * @param gpio El número del pin GPIO a leer.
     * @param db Referencia a la estructura de estado del debounce.
     * @param level_pressed Referencia booleana que se actualizará con el estado estable (true = presionado).
     * @return true Si el estado estable del botón cambió (hubo una pulsación o liberación confirmada).
     * @return false Si el estado se mantiene igual.
     */
    static bool debounce(uint8_t gpio, Debounce &db, bool &level_pressed)
    {
        // 1. Leer el estado actual del pin (0 o 1)
        int raw = gpio_get_level(static_cast<gpio_num_t>(gpio));
        bool changed = false;

        // 2. Verificar si la lectura es igual a la lectura anterior
        if (raw == db.last_raw)
        {
            // Si es igual, aumentamos el contador (la señal se está estabilizando)
            if (db.count < 3)
            {
                db.count++;
            }
        }
        else
        {
            // Si es diferente, hubo ruido; reiniciamos el contador y actualizamos el último valor conocido
            db.count = 0;
            db.last_raw = raw;
        }

        // 3. Si el contador llegó a 3 (se mantuvo estable 3 lecturas) y es diferente al estado estable...
        if (db.count >= 3 && raw != db.stable)
        {
            db.stable = raw; // Confirmamos el nuevo estado estable
            changed = true;  // Marcamos que hubo un cambio de estado real
        }

        // 4. Traducir el estado del GPIO al significado lógico (Pull-up invertido)
        //    El botón usa una resistencia PULL-UP:
        //    - Nivel ALTO (1) = Botón SUELTO
        //    - Nivel BAJO (0) = Botón PRESIONADO
        level_pressed = (db.stable == 0);

        return changed;
    }

    /**
     * @brief Función principal de la tarea del botón.
     *        Lee el GPIO periódicamente, aplica el filtro de rebote y envía el estado a la cola.
     */
    void ButtonTask::run(void *pvParameters)
    {
        // 1. Recibir la configuración a través del puntero pvParameters (tal como exige la práctica)
        auto *cfg = static_cast<ButtonTaskConfig *>(pvParameters);

        // 2. Configurar el hardware del GPIO
        gpio_reset_pin(static_cast<gpio_num_t>(cfg->gpio));
        gpio_set_direction(static_cast<gpio_num_t>(cfg->gpio), GPIO_MODE_INPUT);
        gpio_set_pull_mode(static_cast<gpio_num_t>(cfg->gpio), GPIO_PULLUP_ONLY);

        // 3. Variables locales de estado
        Debounce db;
        bool pressed = false;

        ESP_LOGI(TAG, "%s iniciado", cfg->name);

        // 4. Bucle infinito de la tarea
        while (true)
        {
            // Leer el botón con el filtro de rebote
            bool changed = debounce(cfg->gpio, db, pressed);

            // Si hubo un cambio de estado REAL, o si está configurado para enviar siempre:
            if (changed || !cfg->send_on_change_only)
            {
                // Construir el mensaje estructurado
                ButtonMsg msg =
                {
                    cfg->event_type,        // Tipo de evento (Start o SpeedState)
                    pressed,                // Estado actual (true = presionado)
                    static_cast<uint32_t>(xTaskGetTickCount()) // Marca de tiempo
                };

                // Enviar el mensaje a la cola de botones (no bloquear si la cola está llena)
                xQueueSend(g_queues.buttons, &msg, 0);
            }

            // Esperar el tiempo de sondeo configurado antes de la siguiente lectura
            vTaskDelay(pdMS_TO_TICKS(cfg->poll_ms));
        }
    }
}