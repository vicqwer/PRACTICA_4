#include "sensor_task.hpp"
#include "app_config.hpp"
#include "app_context.hpp"
#include "messages.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

namespace App
{
    // Etiqueta para los logs de esta tarea
    static const char *TAG = "SENSOR";

    /**
     * @brief Implementa el filtro de mediana para eliminar ruido y picos en las lecturas del sensor.
     * 
     * @param v Puntero al array de muestras de 16 bits.
     * @param n Cantidad de elementos en el array (tamaño de la ventana).
     * @return uint16_t El valor central (mediana) del array ordenado.
     */
    static uint16_t median_u16(uint16_t *v, uint8_t n)
    {
        // Algoritmo de ordenamiento por burbuja (bubble sort) para ordenar el array de muestras.
        // Es eficiente para ventanas pequeñas (5 muestras en esta práctica).
        for (uint8_t i = 0; i < n; i++)
        {
            for (uint8_t j = i + 1; j < n; j++)
            {
                if (v[j] < v[i])
                {
                    // Intercambiar valores si están en el orden incorrecto
                    uint16_t temp = v[i];
                    v[i] = v[j];
                    v[j] = temp;
                }
            }
        }
        // Retornar el valor que se encuentra justo a la mitad del array ordenado.
        // Ejemplo: Si hay 5 elementos, retorna el índice 2 (el 3er elemento).
        return v[n / 2];
    }

    /**
     * @brief Determina el ángulo objetivo del servo basándose en la lectura del LDR filtrada.
     * 
     * @param filtered El valor del ADC ya filtrado por la mediana.
     * @return uint8_t 0 grados (Dark) o 180 grados (Light) según la especificación.
     */
    static uint8_t target_from_ldr(uint16_t filtered)
    {
        // Un LDR tiene mayor valor ADC cuando hay poca luz (más resistencia).
        // Si el valor filtrado supera el umbral alto, significa que está oscuro -> 0°.
        if (filtered >= AppConfig::LDR_THRESHOLD_HIGH)
        {
            return AppConfig::SERVO_ANGLE_DARK;
        }
            
        // Si el valor filtrado es inferior al umbral bajo, significa que hay luz -> 180°.
        if (filtered <= AppConfig::LDR_THRESHOLD_LOW) 
        {
            return AppConfig::SERVO_ANGLE_LIGHT;
        }

        // Zona intermedia (entre umbrales). Si está más cerca del umbral alto, es oscuridad. 
        // Si está más cerca del bajo, es luz.
        return (filtered > ((AppConfig::LDR_THRESHOLD_LOW + AppConfig::LDR_THRESHOLD_HIGH) / 2)) ? AppConfig::SERVO_ANGLE_LIGHT : AppConfig::SERVO_ANGLE_DARK;
    }

    /**
     * @brief Función principal de la tarea del sensor LDR.
     *        Toma muestras del ADC, aplica el filtro de mediana y envía el resultado a TaskManager.
     */
    void SensorTask::run(void *pvParameters)
    {
        // 1. Recibir la configuración a través del puntero pvParameters
        auto *cfg = static_cast<SensorTaskConfig *>(pvParameters);

        // 2. Configurar el hardware del ADC (One-Shot mode)
        adc_oneshot_unit_handle_t adc_handle;
        
        adc_oneshot_unit_init_cfg_t unit_cfg = {}; 
        unit_cfg.unit_id = cfg->unit_id;
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

        adc_oneshot_chan_cfg_t chan_cfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, cfg->channel, &chan_cfg));

        ESP_LOGI(TAG, "%s iniciado", cfg->name);

        // 3. Bucle infinito de la tarea
        while (true)
        {
            // Array para almacenar las muestras del ADC (tamaño definido en app_config.hpp)
            uint16_t samples[AppConfig::FILTER_WINDOW_SIZE];
            int raw = 0;

            // Tomar 'n' muestras (tamaño de la ventana del filtro)
            for (uint8_t i = 0; i < cfg->filter_window; i++)
            {
                ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, cfg->channel, &raw));
                samples[i] = static_cast<uint16_t>(raw);
                
                // Pequeño retardo entre muestras para dar tiempo al ADC a estabilizarse
                vTaskDelay(pdMS_TO_TICKS(5));
            }

            // 4. Aplicar el filtro de mediana y determinar el ángulo
            uint16_t filtered = median_u16(samples, cfg->filter_window);
            uint8_t target = target_from_ldr(filtered);

            // 5. Construir el mensaje estructurado para enviar al TaskManager
            SensorMsg msg =
            {
                filtered,                 // Enviamos el valor filtrado para depuración (mejor que raw)
                filtered,                 // Repetimos el filtrado (o podrías poner el valor promedio)
                target,                   // Ángulo objetivo calculado (0° o 180°)
                static_cast<uint32_t>(xTaskGetTickCount()) // Marca de tiempo
            };

            // 6. Enviar el mensaje a la cola del sensor (sobrescribir porque solo guarda el último)
            xQueueOverwrite(g_queues.sensor, &msg);

            // Log para monitoreo en el puerto serie
            ESP_LOGI(TAG, "raw=%u filtered=%u target=%u",
                     msg.raw,
                     msg.filtered,
                     msg.target_angle);

            // 7. Esperar el periodo de muestreo configurado (300-500 ms según PDF)
            vTaskDelay(pdMS_TO_TICKS(cfg->period_ms));
        }
    }
}