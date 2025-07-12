idf_component_register(SRCS "cnl_proyect.c" "adc_task.c"
                    INCLUDE_DIRS "."
                       REQUIRES "esp_adc" "hal" "driver" "soc" "freertos"
                       PRIV_REQUIRES esp_timer esp_psram)
