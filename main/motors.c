#include "motors.h"
#include "config.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define PWM_RESOLUTION  LEDC_TIMER_10_BIT
#define PWM_DUTY_MAX    ((1 << 10) - 1)

static const ledc_channel_t s_channel[2] = { LEDC_CHANNEL_0, LEDC_CHANNEL_1 };
static const int s_pwm_pin[2] = { PIN_MOTOR_L_PWM, PIN_MOTOR_R_PWM };
static const int s_dir_pin[2] = { PIN_MOTOR_L_DIR, PIN_MOTOR_R_DIR };
static const int s_invert[2]  = { MOTOR_L_INVERT, MOTOR_R_INVERT };

void motors_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int m = 0; m < 2; m++) {
        gpio_config_t dir = {
            .pin_bit_mask = 1ULL << s_dir_pin[m],
            .mode         = GPIO_MODE_OUTPUT,
        };
        ESP_ERROR_CHECK(gpio_config(&dir));
        gpio_set_level(s_dir_pin[m], 0);

        ledc_channel_config_t chan = {
            .gpio_num   = s_pwm_pin[m],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = s_channel[m],
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&chan));
    }
}

void motors_set(motor_id_t motor, float cmd)
{
    if (cmd > 1.0f) cmd = 1.0f;
    if (cmd < -1.0f) cmd = -1.0f;
    if (s_invert[motor]) cmd = -cmd;

    gpio_set_level(s_dir_pin[motor], cmd >= 0.0f ? 1 : 0);

    float mag = cmd >= 0.0f ? cmd : -cmd;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, s_channel[motor], (uint32_t)(mag * PWM_DUTY_MAX));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, s_channel[motor]);
}

void motors_stop(void)
{
    motors_set(MOTOR_LEFT, 0.0f);
    motors_set(MOTOR_RIGHT, 0.0f);
}
