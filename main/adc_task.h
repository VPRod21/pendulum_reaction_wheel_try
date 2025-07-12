// adc_task.h

#ifndef ADC_TASK_H
#define ADC_TASK_H

#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_err.h"

// Define el canal del ADC que usarás.
// Para el ESP32-S3, es simplemente ADC_CHANNEL_X
#define ADC_CHANNEL_CURRENT_SENSOR    ADC_CHANNEL_9

// Tamaño del buffer para leer los resultados de la conversión
#define ADC_READ_BUFFER_SIZE         256

uint16_t get_adc_filtered_mv(void);
esp_err_t init_adc_and_task(void);

#endif // ADC_TASK_H