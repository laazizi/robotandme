#include "encoders.h"
#include "config.h"

#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define PCNT_LIMIT 32000

static pcnt_unit_handle_t s_unit[2];

static pcnt_unit_handle_t quadrature_unit_create(int gpio_a, int gpio_b, int invert)
{
    if (invert) {
        int tmp = gpio_a;
        gpio_a = gpio_b;
        gpio_b = tmp;
    }

    pcnt_unit_config_t unit_cfg = {
        .high_limit = PCNT_LIMIT,
        .low_limit  = -PCNT_LIMIT,
        .flags.accum_count = true,
    };
    pcnt_unit_handle_t unit;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &unit));

    pcnt_glitch_filter_config_t filter = { .max_glitch_ns = 1000 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter));

    pcnt_chan_config_t cfg_a = { .edge_gpio_num = gpio_a, .level_gpio_num = gpio_b };
    pcnt_chan_config_t cfg_b = { .edge_gpio_num = gpio_b, .level_gpio_num = gpio_a };
    pcnt_channel_handle_t chan_a, chan_b;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &cfg_a, &chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &cfg_b, &chan_b));

    // Pull-up internes : indispensables pour encodeurs open-collector, et
    // evitent qu'un canal debranche flotte et capte le bruit PWM 20 kHz
    // (comptage parasite). A faire APRES pcnt_new_channel qui reconfigure la GPIO.
    ESP_ERROR_CHECK(gpio_set_pull_mode(gpio_a, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode(gpio_b, GPIO_PULLUP_ONLY));

    // Décodage quadrature x4 : chaque front de A et de B compte,
    // le sens est donné par le niveau de l'autre canal.
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Watch points aux limites : avec accum_count, le compteur logiciel
    // 64 bits continue au-delà de ±PCNT_LIMIT sans perte.
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, PCNT_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, -PCNT_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));
    return unit;
}

void encoders_init(void)
{
    s_unit[ENCODER_LEFT]  = quadrature_unit_create(PIN_ENC_L_A, PIN_ENC_L_B, ENC_L_INVERT);
    s_unit[ENCODER_RIGHT] = quadrature_unit_create(PIN_ENC_R_A, PIN_ENC_R_B, ENC_R_INVERT);
}

int64_t encoder_get_ticks(encoder_id_t encoder)
{
    int count = 0;
    pcnt_unit_get_count(s_unit[encoder], &count);
    return (int64_t)count;
}
