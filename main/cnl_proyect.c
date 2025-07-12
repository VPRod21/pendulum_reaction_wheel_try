// cnl_poyect.c

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "adc_task.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "driver/pulse_cnt.h"

// Definiciones de pines GPIO para el L298N (ejemplo para un motor)
#define MOTOR_A_IN1_GPIO    GPIO_NUM_18 // Pin de dirección 1 para Motor A
#define MOTOR_A_IN2_GPIO    GPIO_NUM_36 // Pin de dirección 2 para Motor A
#define MOTOR_A_ENA_GPIO    GPIO_NUM_41 // Pin PWM para habilitar Motor A (conectado a ENA del L298N)

// Parámetros del MCPWM
#define MCPWM_TIMER_RESOLUTION_HZ 1000000 // 1MHz, 1 tick = 1us
#define MCPWM_TIMER_PERIOD_TICKS  20000   // 20000 ticks = 20ms (50Hz PWM)

// Manejadores globales para MCPWM 
mcpwm_timer_handle_t motor_timer = NULL;
mcpwm_oper_handle_t motor_operator = NULL;
mcpwm_cmpr_handle_t motor_comparator = NULL;
mcpwm_gen_handle_t motor_generator = NULL;
pcnt_unit_handle_t enc1_unit = NULL;
pcnt_unit_handle_t enc2_unit = NULL;

uint8_t speed_index = 0;
bool speed_buffer_filled = false;

static const char *TAG = "MOTOR_CONTROL";

#define ENCODER_PPR      1000
#define DT_MS            10
#define DT_S             (DT_MS / 1000.0f)
#define SPEED_FILTER_LEN 10
float speed_buffer[SPEED_FILTER_LEN] = {0};


#define ENCODER1_PPR 1000        // Pulsos por revolución
#define ENCODER2_PPR 800         // Supuesto para Encoder 2
#define PI 3.14159265359
#define RAD_PER_PULSE (2 * PI / ENCODER1_PPR)
#define ADC_MV_MAX 3300
#define ADC_BITS 4095


#define COUNTS_PER_REV (4 * ENCODER_PPR)  // cuadratura: 4x

float count_to_rad(int32_t count) {
    return (2.0f * PI * count) / COUNTS_PER_REV;
}

void velocity_estimation_task(void *arg) {
    int last_count = 0;
    int current_count = 0;

    while (1) {
        pcnt_unit_get_count(enc1_unit, &current_count);
        int delta = current_count - last_count;

        float raw_speed = (2.0f * PI * delta) / (ENCODER_PPR * DT_S);

        // Actualiza buffer circular
        speed_buffer[speed_index] = raw_speed;
        speed_index = (speed_index + 1) % SPEED_FILTER_LEN;
        if (speed_index == 0) speed_buffer_filled = true;

        // Calcula promedio si ya hay suficientes muestras
        float filtered_speed = 0;
        int N = speed_buffer_filled ? SPEED_FILTER_LEN : speed_index;
        for (int i = 0; i < N; i++) {
            filtered_speed += speed_buffer[i];
        }
        filtered_speed /= N;

        //printf("Velocidad: %.2f rad/s (Filtrada)\n", filtered_speed);

        last_count = current_count;
        vTaskDelay(pdMS_TO_TICKS(DT_MS));
    }
}


void motor_init() {
    ESP_LOGI(TAG, "Configurando GPIOs para dirección...");
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << MOTOR_A_IN1_GPIO) | (1ULL << MOTOR_A_IN2_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_LOGI(TAG, "Configurando periférico MCPWM para velocidad...");

    // 1. Configuración del Temporizador MCPWM
    mcpwm_timer_config_t timer_config = {
       .group_id = 0,
       .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
       .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
       .period_ticks = MCPWM_TIMER_PERIOD_TICKS,
       .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &motor_timer));

    // 2. Configuración del Operador MCPWM
    mcpwm_operator_config_t operator_config = {
       .group_id = 0, // El operador debe estar en el mismo grupo que el temporizador
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &motor_operator));

    // Conectar temporizador y operador
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(motor_operator, motor_timer));

    // 3. Configuración del Comparador MCPWM para el ciclo de trabajo
    mcpwm_comparator_config_t comparator_config = {
       .flags.update_cmp_on_tez = true, // Actualizar valor de comparación cuando el temporizador llega a cero
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(motor_operator, &comparator_config, &motor_comparator));

    // 4. Configuración del Generador MCPWM para la señal PWM
    mcpwm_generator_config_t generator_config = {
       .gen_gpio_num = MOTOR_A_ENA_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(motor_operator, &generator_config, &motor_generator));

    // Establecer acciones del generador para crear la señal PWM
    // Poner HIGH al inicio del ciclo (cuando el temporizador es 0)
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        motor_generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
    ));
    // Poner LOW cuando el temporizador iguala el valor del comparador (fin del pulso)
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        motor_generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor_comparator, MCPWM_GEN_ACTION_LOW)
    ));

    // Habilitar y iniciar el temporizador MCPWM
    ESP_ERROR_CHECK(mcpwm_timer_enable(motor_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(motor_timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "Inicialización de motor completada.");
}

typedef enum {
    MOTOR_STOP,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
} motor_direction_t;

void set_motor_direction(motor_direction_t direction) {
    switch (direction) {
        case MOTOR_FORWARD:
            gpio_set_level(MOTOR_A_IN1_GPIO, 1);
            gpio_set_level(MOTOR_A_IN2_GPIO, 0);
            //ESP_LOGI(TAG, "Motor: Adelante");
            break;
        case MOTOR_BACKWARD:
            gpio_set_level(MOTOR_A_IN1_GPIO, 0);
            gpio_set_level(MOTOR_A_IN2_GPIO, 1);
            //ESP_LOGI(TAG, "Motor: Atrás");
            break;
        case MOTOR_STOP:
        default:
            gpio_set_level(MOTOR_A_IN1_GPIO, 0);
            gpio_set_level(MOTOR_A_IN2_GPIO, 0); // O 1,1 para frenado, según el comportamiento de parada deseado
            //ESP_LOGI(TAG, "Motor: Detenido");
            break;
    }
}
void set_motor_speed(uint32_t speed_percent) {
    if (speed_percent > 100) {
        speed_percent = 100;
    }
    // Calcular el valor de ticks para el ciclo de trabajo
    // Un 100% de velocidad equivale al periodo completo del temporizador
    uint32_t duty_ticks = (MCPWM_TIMER_PERIOD_TICKS * speed_percent) / 100;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor_comparator, duty_ticks));
    //ESP_LOGI(TAG, "Velocidad del motor: %lu%% (Duty: %lu ticks)", speed_percent, duty_ticks);
}
void encoder_init(void) {
    // Encoder 1
    pcnt_channel_handle_t enc1_chan = NULL;
    pcnt_unit_config_t unit_config1 = {
        .high_limit = 10000,
        .low_limit = -10000,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config1, &enc1_unit));

    pcnt_chan_config_t chan_config1 = {
        .edge_gpio_num = 4,
        .level_gpio_num = 5,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(enc1_unit, &chan_config1, &enc1_chan));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(enc1_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(enc1_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    ESP_ERROR_CHECK(pcnt_unit_enable(enc1_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(enc1_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(enc1_unit));

    // Encoder 2
    pcnt_channel_handle_t enc2_chan = NULL;
    pcnt_unit_config_t unit_config2 = {
        .high_limit = 10000,
        .low_limit = -10000,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config2, &enc2_unit));

    pcnt_chan_config_t chan_config2 = {
        .edge_gpio_num = 6,
        .level_gpio_num = 7,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(enc2_unit, &chan_config2, &enc2_chan));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(enc2_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(enc2_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    ESP_ERROR_CHECK(pcnt_unit_enable(enc2_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(enc2_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(enc2_unit));
}


static float q1_des = 1.0f;  // Posición deseada en rad
static const float k_p = 10.0f; // Ganancia proporcional
static const float k_pwm = 10.0f; // Ganancia PWM corriente -> torque

void control_task(void *arg) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 1;   // 100 Hz

    while (1) {
        int encoder1_count = 0;
        int encoder2_count = 0;
        pcnt_unit_get_count(enc1_unit, &encoder1_count);
        pcnt_unit_get_count(enc2_unit, &encoder2_count);

        float q1 = count_to_rad(encoder1_count);
        float q2 = encoder2_count * (2 * PI / ENCODER2_PPR);  // Aproximado
        static float prev_q1 = 0.0f, prev_q2 = 0.0f;
        float vel_q1 = (q1 - prev_q1) / 0.001f;  // 0.001 s entre ciclos
        float vel_q2 = (q2 - prev_q2) / 0.001f;
        prev_q1 = q1;
        prev_q2 = q2;

        float u = -k_p * (q1 - q1_des);  // Torque deseado

        // Leer ADC y convertir a mV
        uint16_t adc_mv = get_adc_filtered_mv();

        // Comparar con u para generar señal PWM
        // pwm = k*(u_actual - u_deseado)
        // Aqui usamos adc_mv como referencia indirecta del torque actual
        float u_actual = (adc_mv / 1000.0f); // Ejemplo: escala lineal arbitraria a Nm
        float pwm = k_pwm * (u - u_actual);

        // Limitar PWM al rango 0-100%
        if (pwm < 0) pwm = 0;
        if (pwm > 100) pwm = 100;

        if (u >= 0) {
            set_motor_direction(MOTOR_FORWARD);
        } else {
            set_motor_direction(MOTOR_BACKWARD);
            pwm = -pwm;
        }
        set_motor_speed((uint32_t)pwm);

        //printf("ENC1: %.3frad | ENC2: %.3frad | ADC: %umV | PWM: %.2f%%\n", q1, q2, adc_mv, pwm);
        static TickType_t print_timer = 0;
        print_timer += xFrequency;
        if (print_timer >= pdMS_TO_TICKS(500)) {
            printf("q1: %.3f rad, vel_q1: %.3f rad/s | q2: %.3f rad, vel_q2: %.3f rad/s | ADC: %u mV | PWM: %.2f%%\n",
                q1, vel_q1, q2, vel_q2, adc_mv, pwm);
            print_timer = 0;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void app_main(void)
{
    encoder_init();  
    ESP_ERROR_CHECK(init_adc_and_task());
    motor_init();

    // Lanzar tareas
    xTaskCreate(velocity_estimation_task, "vel_task", 2048, NULL, 5, NULL);
    xTaskCreate(control_task, "control_task", 8192, NULL, 6, NULL);
}