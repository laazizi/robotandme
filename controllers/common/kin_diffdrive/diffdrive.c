// Cinematique DIFFDRIVE : deux roues motrices independantes + roues folles.
// Tourne par DIFFERENCE de vitesse entre les roues ; la rotation sur place est
// possible, et c'est ce qui la rend indispensable a la tonte en boustrophedon.
// Utilisee par mowbot_p4 et mowbot_wroom.
//
// Contenu repris A L'IDENTIQUE de l'ancien firmware mowbot -- motors.c,
// odometry.c et la partie diffdrive de main.c -- simplement range derriere
// l'interface kin.h. Le comportement de robot A, CALIBRE ET VALIDE, ne doit
// pas changer d'un iota : toute modification ici le concerne directement.

#include "kin.h"
#include "config.h"
#include "encoders.h"
#include "pid.h"

#include <math.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "diffdrive";

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// ---------------------------------------------------------------------------
// Moteurs (ex-motors.c) : MDD10A en sign-magnitude, PWM = |cmd|, DIR = signe.
// ---------------------------------------------------------------------------
typedef enum { MOTOR_LEFT = 0, MOTOR_RIGHT = 1 } motor_id_t;

#define PWM_RESOLUTION  LEDC_TIMER_10_BIT
#define PWM_DUTY_MAX    ((1 << 10) - 1)

static const ledc_channel_t s_channel[2] = { LEDC_CHANNEL_0, LEDC_CHANNEL_1 };
static const int s_pwm_pin[2] = { PIN_MOTOR_L_PWM, PIN_MOTOR_R_PWM };
static const int s_dir_pin[2] = { PIN_MOTOR_L_DIR, PIN_MOTOR_R_DIR };
static const int s_invert[2]  = { MOTOR_L_INVERT, MOTOR_R_INVERT };

static void motors_init(void)
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

static void motors_set(motor_id_t motor, float cmd)
{
    if (cmd > 1.0f) cmd = 1.0f;
    if (cmd < -1.0f) cmd = -1.0f;
    if (s_invert[motor]) cmd = -cmd;

    gpio_set_level(s_dir_pin[motor], cmd >= 0.0f ? 1 : 0);

    float mag = cmd >= 0.0f ? cmd : -cmd;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, s_channel[motor], (uint32_t)(mag * PWM_DUTY_MAX));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, s_channel[motor]);
}

static void motors_stop(void)
{
    motors_set(MOTOR_LEFT, 0.0f);
    motors_set(MOTOR_RIGHT, 0.0f);
}

// ---------------------------------------------------------------------------
// Odometrie (ex-odometry.c)
// ---------------------------------------------------------------------------
static void odometry_update(kin_odom_t *odom, float d_left_m, float d_right_m, float dt)
{
    float d_center = 0.5f * (d_left_m + d_right_m);
    float d_theta  = (d_right_m - d_left_m) / TRACK_WIDTH_M;

    // Intégration au point milieu : exacte au 2e ordre, suffisante à 50 Hz
    odom->x += d_center * cosf(odom->theta + 0.5f * d_theta);
    odom->y += d_center * sinf(odom->theta + 0.5f * d_theta);
    odom->theta += d_theta;

    while (odom->theta > (float)M_PI)  odom->theta -= 2.0f * (float)M_PI;
    while (odom->theta < -(float)M_PI) odom->theta += 2.0f * (float)M_PI;

    odom->v = (dt > 0.0f) ? d_center / dt : 0.0f;
    odom->w = (dt > 0.0f) ? d_theta / dt : 0.0f;
}

// ---------------------------------------------------------------------------
// Boucle de controle (ex-main.c)
// ---------------------------------------------------------------------------
// Consignes vitesse roue [m/s] : écrites par kin_apply_twist (callback
// cmd_vel), lues par kin_update (timer) -- même executor, pas de concurrence.
static float s_target_left;
static float s_target_right;

static pid_ctrl_t s_pid_left;
static pid_ctrl_t s_pid_right;
static int64_t s_prev_ticks_left;
static int64_t s_prev_ticks_right;

static const float METERS_PER_TICK =
    2.0f * (float)M_PI * WHEEL_RADIUS_M / TICKS_PER_WHEEL_REV;

void kin_init(void)
{
    // Meme ordre que l'ancien app_main : moteurs a zero avant tout le reste.
    motors_init();
    motors_stop();
    encoders_init();
    pid_init(&s_pid_left, PID_KP, PID_KI, PID_KD, -1.0f, 1.0f);
    pid_init(&s_pid_right, PID_KP, PID_KI, PID_KD, -1.0f, 1.0f);
}

bool kin_apply_twist(float v, float w)
{
    // Cinématique inverse diffdrive
    s_target_left = clampf(v - w * TRACK_WIDTH_M * 0.5f,
                           -MAX_WHEEL_SPEED_MPS, MAX_WHEEL_SPEED_MPS);
    s_target_right = clampf(v + w * TRACK_WIDTH_M * 0.5f,
                            -MAX_WHEEL_SPEED_MPS, MAX_WHEEL_SPEED_MPS);
    return true;   // un diffdrive realise n'importe quel (v, w) borne
}

void kin_update(float dt, kin_odom_t *odom)
{
    int64_t ticks_left = encoder_get_ticks(ENCODER_LEFT);
    int64_t ticks_right = encoder_get_ticks(ENCODER_RIGHT);
    float d_left = (float)(ticks_left - s_prev_ticks_left) * METERS_PER_TICK;
    float d_right = (float)(ticks_right - s_prev_ticks_right) * METERS_PER_TICK;
    s_prev_ticks_left = ticks_left;
    s_prev_ticks_right = ticks_right;

    odometry_update(odom, d_left, d_right, dt);

    float v_left = d_left / dt;
    float v_right = d_right / dt;

    // FEED-FORWARD + PID par roue : le FF envoie d'emblee le duty theorique
    // (cible / vitesse max), le PID ne corrige que l'ecart (charge, pente).
    // FF_GAIN=0 -> PID pur (robot 12 V, valide ainsi). Consigne nulle :
    // moteur coupe + reset PID (pas de freinage actif -> pas d'oscillation).
    if (s_target_left == 0.0f) {
        pid_reset(&s_pid_left);
        motors_set(MOTOR_LEFT, 0.0f);
    } else {
        float ff = FF_GAIN * clampf(s_target_left / MAX_WHEEL_SPEED_MPS, -1.0f, 1.0f);
        motors_set(MOTOR_LEFT, clampf(ff + pid_update(&s_pid_left, s_target_left, v_left, dt), -1.0f, 1.0f));
    }
    if (s_target_right == 0.0f) {
        pid_reset(&s_pid_right);
        motors_set(MOTOR_RIGHT, 0.0f);
    } else {
        float ff = FF_GAIN * clampf(s_target_right / MAX_WHEEL_SPEED_MPS, -1.0f, 1.0f);
        motors_set(MOTOR_RIGHT, clampf(ff + pid_update(&s_pid_right, s_target_right, v_right, dt), -1.0f, 1.0f));
    }
}

void kin_stop(void)
{
    motors_stop();
}

// ---------------------------------------------------------------------------
// Test banc (ex-boot_test_run de main.c). Fait tourner UNE roue a la fois
// (2 s avant, 2 s arriere) et logge le delta de ticks des DEUX encodeurs a
// chaque phase, puis 30 s de compteurs bruts pour un test a la main.
// ---------------------------------------------------------------------------
static void bench_phase(const char *nom, motor_id_t motor, float duty)
{
    const TickType_t DUREE = pdMS_TO_TICKS(2000);
    int64_t g0 = encoder_get_ticks(ENCODER_LEFT);
    int64_t d0 = encoder_get_ticks(ENCODER_RIGHT);
    motors_set(motor, duty);
    vTaskDelay(DUREE);
    motors_stop();
    int64_t dg = encoder_get_ticks(ENCODER_LEFT) - g0;
    int64_t dd = encoder_get_ticks(ENCODER_RIGHT) - d0;
    ESP_LOGW(TAG, "%-16s : enc_G=%+6lld  enc_D=%+6lld",
             nom, (long long)dg, (long long)dd);
    vTaskDelay(pdMS_TO_TICKS(500));
}

void kin_bench_test(void)
{
    const float DUTY = 0.25f;
    ESP_LOGW(TAG, "==== TEST BANC PAR ROUE (2 s/phase, roues en l'air !) ====");
    ESP_LOGW(TAG, "attendu : la roue testee compte FORT (+ en avant, - en arriere),");
    ESP_LOGW(TAG, "          l'autre encodeur reste ~0. Sinon : 0=muet, signe=INVERT,");
    ESP_LOGW(TAG, "          mauvais compteur=cables croises G/D.");
    bench_phase("G avant  (+)", MOTOR_LEFT,  +DUTY);
    bench_phase("G arriere(-)", MOTOR_LEFT,  -DUTY);
    bench_phase("D avant  (+)", MOTOR_RIGHT, +DUTY);
    bench_phase("D arriere(-)", MOTOR_RIGHT, -DUTY);

    ESP_LOGW(TAG, "---- TEST A LA MAIN : tournez les roues (30 s) ----");
    for (int i = 0; i < 30; i++) {
        ESP_LOGW(TAG, "t=%2ds  enc_G=%+8lld  enc_D=%+8lld", i,
                 (long long)encoder_get_ticks(ENCODER_LEFT),
                 (long long)encoder_get_ticks(ENCODER_RIGHT));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGW(TAG, "==== FIN TEST BANC ====");
}
