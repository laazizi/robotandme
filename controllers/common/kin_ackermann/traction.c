#include "traction.h"
#include "config.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define PWM_RESOLUTION  LEDC_TIMER_10_BIT
#define PWM_DUTY_MAX    ((1 << 10) - 1)

// TIMER_0 a 20 kHz, CHANNEL_0. Le servo de direction est sur TIMER_1 a 50 Hz
// (steering.c) : un timer LEDC ne porte qu'une frequence.
#define TRACTION_TIMER    LEDC_TIMER_0
#define TRACTION_CHANNEL  LEDC_CHANNEL_0

void traction_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num       = TRACTION_TIMER,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    gpio_config_t dir = {
        .pin_bit_mask = 1ULL << PIN_TRACTION_DIR,
        .mode         = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&dir));
    gpio_set_level(PIN_TRACTION_DIR, 0);

    ledc_channel_config_t chan = {
        .gpio_num   = PIN_TRACTION_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = TRACTION_CHANNEL,
        .timer_sel  = TRACTION_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&chan));
}

void traction_set(float cmd)
{
    if (cmd > 1.0f) cmd = 1.0f;
    if (cmd < -1.0f) cmd = -1.0f;
    if (TRACTION_INVERT) cmd = -cmd;

    gpio_set_level(PIN_TRACTION_DIR, cmd >= 0.0f ? 1 : 0);

    float mag = cmd >= 0.0f ? cmd : -cmd;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, TRACTION_CHANNEL, (uint32_t)(mag * PWM_DUTY_MAX));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, TRACTION_CHANNEL);
}

void traction_stop(void)
{
    traction_set(0.0f);
}
