#include "driver/ledc.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"

// ==== HARDWARE ====
#define PWM_GPIO        4
#define LED_GPIO        2
#define ADC_CHANNEL     ADC1_CHANNEL_7
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define PWM_FREQ        10000              // Hz
#define CTRL_PERIOD_US  100                // Ts = 100 µs

// ==== PARÂMETROS DO SISTEMA ====
const float T = 0.0001f;                   // Período de amostragem (100 µs)
const float R1 = 51000.0f;
const float R2 = 18000.0f;
const float C1 = 1e-7f;
const float C2 = 6.8e-7f;

// Matriz A
const float a11 = 0.0f;
const float a12 = 1.0f / C1;
const float a21 = -1.0f / (R1 * R2 * C2);
const float a22 = -1.0f / (R1 * C2) - 1.0f / (R2 * C2);

// Matriz B
const float b1 = 0.0f;
const float b2 = 1.0f / (R1 * R2 * C2);

// Matriz C
const float c1 = 1.0f;

// ==== GANHOS DO CONTROLADOR ====
const float K1 = 36.697830815f;       // 1e6 * 0.000036697...
const float K2 = 1535.404294588f;     // 1e6 * 1.535...
const float Ki = 7945.920574390f;

// ==== GANHOS DO OBSERVADOR ====
const float Ke1 = 124.759707805f;     // 1e2 * 1.247...
const float Ke2 = 0.002470683627f;    // 1e2 * 0.00002470...

// ==== VARIÁVEIS ====
float rk = 1.0f;
float yk = 0.0f, y_til = 0.0f;
float ek = 0.0f, E = 0.0f, E_dot = 0.0f;
float uk = 0.0f;

float x1_til = 0.0f, x2_til = 0.0f;
float x1_til_dot = 0.0f, x2_til_dot = 0.0f;
bool led_state = false;

int64_t last_print_time = 0;  // tempo em microssegundos

// ==== ALTERAÇÃO DE REFERÊNCIA ====
void IRAM_ATTR ref_callback(void* arg) {
    rk = (rk == 1.0f) ? 1.5f : 1.0f;
    led_state = !led_state;
    gpio_set_level(LED_GPIO, led_state);
}

// ==== LOOP DE CONTROLE ====
void IRAM_ATTR control_callback(void* arg) {
    int raw_adc = adc1_get_raw(ADC_CHANNEL);

    // NOVO LOG DO VALOR ADC BRUTO - DEBUG
    static int adc_log_counter = 0;
    adc_log_counter++;
    if (adc_log_counter >= 5000) {
        ESP_LOGI("ADC_MONITOR", "ADC bruto: %d", raw_adc);
        adc_log_counter = 0;
    }

    yk = ((float)raw_adc + 180.0f) / 4095.0f * 3.3f;

    x1_til += T * x1_til_dot;
    x2_til += T * x2_til_dot;

    // Integrador
    E_dot = rk - yk;
    E += T * E_dot;

    y_til = c1 * x1_til;
    ek = yk - y_til;

    // Observador
    x1_til_dot = a11 * x1_til + a12 * x2_til + b1 * uk + Ke1 * ek;
    x2_til_dot = a21 * x1_til + a22 * x2_til + b2 * uk + Ke2 * ek;


    // Controle
    float v = K1 * x1_til + K2 * x2_til;
    uk = Ki * E - v;

    // Saturação
    if (uk > 3.3f) uk = 3.3f;
    if (uk < 0.0f) uk = 0.0f;

    // PWM
    uint32_t duty = (uint32_t)(uk * (8191.0f / 3.3f));
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    // === PRINT uk periodicamente === DEBUG
    static int uk_log_counter = 0;
    uk_log_counter++;
    if (uk_log_counter >= 5000) {  // a cada ~0.5 segundos (100 µs × 5000)
        ESP_LOGI("UK_MONITOR", "uk: %.4f V | duty: %u", uk, duty);
        uk_log_counter = 0;
    }

    // === PRINT A CADA 5 SEGUNDOS === DEBUG
    float vc1 = x1_til;
    float ic1 = C1 * x1_til_dot;
    int64_t now = esp_timer_get_time();

    if (now - last_print_time >= 5000000) {
        ESP_LOGI("C1_MONITOR", "Tensão C1: %.4f V | Corrente C1: %.4f mA", vc1, ic1 * 1000.0f);
        last_print_time = now;
    }
}

// ==== SETUP ====
void app_main(void) {
    // PWM
    ledc_timer_config_t pwm_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_channel = {
        .gpio_num   = PWM_GPIO,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&pwm_channel);

    // LED
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_GPIO, 0);

    // ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);

    // Timer referência
    esp_timer_create_args_t ref_args = {
        .callback = &ref_callback,
        .name     = "ref_timer"
    };
    esp_timer_handle_t ref_timer;
    esp_timer_create(&ref_args, &ref_timer);
    esp_timer_start_periodic(ref_timer, 100000); // 100ms

    // Timer controle
    esp_timer_create_args_t ctrl_args = {
        .callback = &control_callback,
        .name     = "ctrl_timer"
    };
    esp_timer_handle_t ctrl_timer;
    esp_timer_create(&ctrl_args, &ctrl_timer);
    esp_timer_start_periodic(ctrl_timer, CTRL_PERIOD_US);

    ESP_LOGI("APP", "Controle com observador e monitoramento de C1 iniciado.");
}

