#include "steering.h"
#include "config.h"

#include "driver/ledc.h"
#include "esp_err.h"

#include <math.h>
#include <stdbool.h>

// 14 bits a 50 Hz : la periode de 20 000 us est decoupee en 16 384 pas, soit
// 1,22 us par pas. La resolution angulaire qui en decoule, ~0,07 deg avec les
// placeholders, est tres au-dela du jeu mecanique d'un servo RC.
#define SERVO_RESOLUTION   LEDC_TIMER_14_BIT
#define SERVO_DUTY_MAX     ((1 << 14) - 1)
#define SERVO_PERIOD_US    (1000000 / SERVO_FREQ_HZ)

// TIMER_1 et CHANNEL_2 : le TIMER_0 est a 20 kHz pour la traction (traction.c)
// et un timer LEDC ne porte qu'UNE frequence. Le canal 1 reste libre.
#define SERVO_TIMER        LEDC_TIMER_1
#define SERVO_CHANNEL      LEDC_CHANNEL_2

static float s_delta;   // dernier angle commande [rad]
static bool s_ecrit;    // le servo a-t-il recu au moins une impulsion ?

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static void servo_write_us(int us)
{
    us = us < SERVO_MIN_US ? SERVO_MIN_US : (us > SERVO_MAX_US ? SERVO_MAX_US : us);
    uint32_t duty = (uint32_t)((int64_t)us * SERVO_DUTY_MAX / SERVO_PERIOD_US);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL);
}

void steering_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_RESOLUTION,
        .timer_num       = SERVO_TIMER,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t chan = {
        .gpio_num   = PIN_SERVO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = SERVO_CHANNEL,
        .timer_sel  = SERVO_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&chan));

    steering_center();
}

void steering_set(float delta_rad)
{
    const float cible = clampf(delta_rad, -STEER_MAX_RAD, STEER_MAX_RAD);
    // ZONE MORTE : on ignore les variations plus petites que SERVO_DEADBAND_RAD.
    // Sans elle, le servo corrige en permanence pour du bruit de commande (voir
    // robot.h). Le premier appel passe toujours, sinon le servo ne serait
    // jamais initialise a l'allumage.
    if (s_ecrit && fabsf(cible - s_delta) < SERVO_DEADBAND_RAD) {
        return;
    }
    s_delta = cible;
    s_ecrit = true;
    float d = SERVO_INVERT ? -s_delta : s_delta;
    servo_write_us(SERVO_CENTER_US + (int)(d * SERVO_US_PER_RAD));
}

void steering_center(void)
{
    steering_set(0.0f);
}

float steering_get(void)
{
    return s_delta;
}
