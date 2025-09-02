#include "driver/ledc.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"

// ==== CONFIGURAÇÕES DE HARDWARE ====
#define PWM_GPIO        4                   // GPIO onde sai o PWM
#define LED_GPIO        2                   // GPIO de LED indicador
#define ADC_CHANNEL     ADC1_CHANNEL_6      // GPIO34 ligado à saída da planta
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define PWM_FREQ        3276                // Frequência de PWM compatível com Ts=3,054ms

// ==== COEFICIENTES DO CONTROLADOR ====
const float c0 = 4.3499f;
const float c1 = -6.9901f;
const float c2 = 3.1539f;
const float d1 = -1.31063f;
const float d2 = 0.31063f;

// ==== VARIÁVEIS DE ESTADO ====
float rk = 1.0;        // Referência alternada entre 1.0V e 1.5V
float yk = 0.0;        // Saída medida da planta
float e[3] = {0};      // Erro atual e anteriores
float u[3] = {0};      // Sinal de controle atual e anteriores

bool led_state = false;
bool controle_ativo = true; // 'true' para malha fechada, 'false' para aberta

// ==== TIMER 100ms PARA ALTERAR REFERÊNCIA ====
void IRAM_ATTR ref_callback(void* arg) {
    rk = (rk == 1.0f) ? 1.5f : 1.0f;
    led_state = !led_state;
    gpio_set_level(LED_GPIO, led_state);
}

// ==== TIMER 3.054ms PARA CONTROLE ====
void IRAM_ATTR control_callback(void* arg) {
    int raw_adc = adc1_get_raw(ADC_CHANNEL);

    // Conversão para tensão
    yk = ((float)raw_adc + 180.0f) / 4095.0f * 3.3f;

    if (!controle_ativo) {
        // MALHA ABERTA: injeta degrau baseado em rk
        u[0] = rk;
    } else {
        // MALHA FECHADA: aplica controlador recursivo
        e[0] = rk - yk;
        u[0] = c0 * e[0] + c1 * e[1] + c2 * e[2]
             - d1 * u[1] - d2 * u[2];

        // Clamping
        if (u[0] > 3.3f) u[0] = 3.3f;
        if (u[0] < 0.0f) u[0] = 0.0f;
    }

    // PWM duty (13 bits → 0 a 8191)
    uint32_t duty = (uint32_t)(u[0] * (8191.0f / 3.3f));
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    // Atualiza históricos
    e[2] = e[1]; e[1] = e[0];
    u[2] = u[1]; u[1] = u[0];
}

// ==== SETUP PRINCIPAL ====
void app_main(void) {
    // PWM timer config
    ledc_timer_config_t pwm_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    // PWM channel config
    ledc_channel_config_t pwm_channel = {
        .gpio_num   = PWM_GPIO,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&pwm_channel);

    // LED indicador
    gpio_config_t io_conf = {
        .pin_bit_mask   = (1ULL << LED_GPIO),
        .mode           = GPIO_MODE_OUTPUT,
        .pull_up_en     = GPIO_PULLUP_DISABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_GPIO, 0);

    // ADC setup
    adc1_config_width(ADC_WIDTH_BIT_12);
    
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);

    // Timer de referência (100ms)
    esp_timer_create_args_t ref_args = {
        .callback = &ref_callback,
        .name     = "ref_timer"
    };
    esp_timer_handle_t ref_timer;
    esp_timer_create(&ref_args, &ref_timer);
    esp_timer_start_periodic(ref_timer, 100000); // 100ms

    // Timer de controle (3.054ms)
    esp_timer_create_args_t ctrl_args = {
        .callback = &control_callback,
        .name     = "ctrl_timer"
    };
    esp_timer_handle_t ctrl_timer;
    esp_timer_create(&ctrl_args, &ctrl_timer);
    esp_timer_start_periodic(ctrl_timer, 3054); // Ts = 3.054ms

    ESP_LOGI("APP", "Controle digital iniciado.");
}

