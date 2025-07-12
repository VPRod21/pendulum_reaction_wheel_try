// adc_task.c

#include "adc_task.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "soc/soc_caps.h"
#include "esp_timer.h"

// Manejadores globales del ADC y calibración
static adc_continuous_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_cali_handle = NULL;

// Buffer circular para filtro de media móvil
#define FILTER_SIZE 50
static uint16_t adc_buffer[FILTER_SIZE] = {0};  // Almacena muestras crudas
static uint32_t adc_sum = 0;                    // Suma acumulada para cálculo de promedio
static uint8_t buffer_index = 0;                // Índice actual en el buffer circular
static bool buffer_filled = false;              // Marca si el buffer ya está lleno al menos una vez

// Tag para logs
static const char *TAG = "ADC_TASK";

// Tarea encargada de leer el ADC continuamente y mantener el filtro actualizado
static void adc_reading_task(void *pvParameters) {
    uint8_t result[ADC_READ_BUFFER_SIZE] = {0};  // Buffer temporal para datos del ADC
    uint32_t bytes_read = 0;

    while (1) {
        esp_err_t ret = adc_continuous_read(
            s_adc_handle,
            result,
            ADC_READ_BUFFER_SIZE,
            &bytes_read,
            portMAX_DELAY
        );

        if (ret == ESP_OK) {
            for (int i = 0; i < bytes_read; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];

                if (p->type2.channel == ADC_CHANNEL_CURRENT_SENSOR) {
                    // Reemplaza la muestra más antigua en el buffer y actualiza la suma acumulada
                    uint16_t old_value = adc_buffer[buffer_index];
                    uint16_t new_value = p->type2.data;
                    adc_sum = adc_sum - old_value + new_value;
                    adc_buffer[buffer_index] = new_value;

                    // Avanza el índice circular
                    buffer_index = (buffer_index + 1) % FILTER_SIZE;
                    if (buffer_index == 0) buffer_filled = true;  // Marca que el buffer está lleno
                }
            }
        }
    }
}

// Inicializa el ADC continuo y lanza la tarea de lectura
esp_err_t init_adc_and_task(void) {
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = 1024,
        .conv_frame_size = ADC_READ_BUFFER_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &s_adc_handle));

    // Configura el patrón de conversión (canal, atenuación, etc.)
    adc_digi_pattern_config_t adc_pattern = {
        .atten = ADC_ATTEN_DB_12,
        .channel = ADC_CHANNEL_CURRENT_SENSOR,
        .unit = ADC_UNIT_1,
        .bit_width = ADC_BITWIDTH_12,
    };

    adc_continuous_config_t adc_config = {
        .pattern_num = 1,
        .adc_pattern = &adc_pattern,
        .sample_freq_hz = 20000,  // Frecuencia de muestreo: 20 kHz
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    ESP_ERROR_CHECK(adc_continuous_config(s_adc_handle, &adc_config));

    // Inicializa la calibración por curva de ajuste
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &s_cali_handle));

    // Inicia el ADC continuo
    ESP_ERROR_CHECK(adc_continuous_start(s_adc_handle));

    // Crea la tarea de lectura ADC en el núcleo 0 con alta prioridad
    xTaskCreatePinnedToCore(adc_reading_task, "adc_reader", 4096, NULL, 15, NULL, 0);

    return ESP_OK;
}

// Obtiene el promedio filtrado en milivoltios (mV)
uint16_t get_adc_filtered_mv(void) {
    if (!buffer_filled) return 0;  // Retorna 0 si el filtro aún no se ha llenado

    uint16_t avg_raw = adc_sum / FILTER_SIZE;  // Media aritmética de muestras crudas
    int mv = 0;
    if (adc_cali_raw_to_voltage(s_cali_handle, avg_raw, &mv) == ESP_OK) {
        return mv;
    } else {
        return 0;  // Si la calibración falla, retorna 0
    }
}
