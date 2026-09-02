// Cinematique ACKERMANN : essieu avant directionnel (servo RC), traction sur
// l'essieu arriere (un canal MDD10A), encodeurs PCNT sur les deux roues
// arriere. PAS de rotation sur place -- voir ackermann.c pour ce que cela
// implique cote nav2. Utilisee par ackerbot_p4. N'a JAMAIS ROULE : voir le
// robot.h de ce controleur pour les placeholders.
//
// Ce fichier fait le lien entre l'interface kin.h et les modules Ackermann :
//   ackermann.c  : (v, w) -> (v, delta) et odometrie bicyclette (testes sur PC)
//   steering.c   : le servo (LEDC 50 Hz)
//   traction.c   : le moteur arriere (LEDC 20 kHz, un canal MDD10A)

#include "kin.h"
#include "config.h"
#include "ackermann.h"
#include "encoders.h"
#include "pid.h"
#include "steering.h"
#include "traction.h"

#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ackermann";

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// Consignes ecrites par kin_apply_twist (callback cmd_vel), lues par
// kin_update (timer) -- meme executor, pas de concurrence.
static float s_target_v;       // [m/s]  vitesse de l'essieu arriere
static float s_target_delta;   // [rad]  angle de braquage
static int s_spin_requests;    // rotations sur place demandees (impossibles)

static pid_ctrl_t s_pid_traction;
static ackermann_odom_t s_odom;   // garde delta en plus des champs de kin_odom_t
static int64_t s_prev_ticks_left;
static int64_t s_prev_ticks_right;

static const float METERS_PER_TICK =
    2.0f * (float)M_PI * WHEEL_RADIUS_M / TICKS_PER_WHEEL_REV;

void kin_init(void)
{
    // Traction a zero et roues droites avant tout le reste.
    traction_init();
    steering_init();
    kin_stop();
    encoders_init();
    pid_init(&s_pid_traction, PID_KP, PID_KI, PID_KD, -1.0f, 1.0f);
}

bool kin_apply_twist(float v, float w)
{
    bool ok = ackermann_twist_to_cmd(v, w, &s_target_v, &s_target_delta);
    if (!ok) {
        // Rotation sur place : impossible en Ackermann. On compte, et on le dit
        // une fois sur 50 pour ne pas noyer le journal. Si ce message revient,
        // nav2 n'est pas configure pour un Ackermann (voir ackermann.c).
        if ((++s_spin_requests % 50) == 1) {
            ESP_LOGW(TAG, "rotation sur place demandee (%d fois) : impossible en "
                          "Ackermann, nav2 est-il configure pour ce robot ?",
                     s_spin_requests);
        }
    }
    return ok;
}

void kin_update(float dt, kin_odom_t *odom)
{
    // Direction : le servo est asservi en position par construction, on lui
    // envoie l'angle et c'est tout.
    steering_set(s_target_delta);

    // Distance de l'essieu arriere = moyenne des deux roues. En virage la roue
    // interieure parcourt moins que l'exterieure ; leur moyenne est exactement
    // le chemin du milieu de l'essieu, ce que le modele bicyclette attend.
    int64_t ticks_left = encoder_get_ticks(ENCODER_LEFT);
    int64_t ticks_right = encoder_get_ticks(ENCODER_RIGHT);
    float d_left = (float)(ticks_left - s_prev_ticks_left) * METERS_PER_TICK;
    float d_right = (float)(ticks_right - s_prev_ticks_right) * METERS_PER_TICK;
    s_prev_ticks_left = ticks_left;
    s_prev_ticks_right = ticks_right;
    float d_rear = 0.5f * (d_left + d_right);

    // L'angle utilise pour l'odometrie est l'angle COMMANDE, faute de retour
    // sur le servo (voir robot.h). C'est la principale source d'erreur de
    // cette odometrie.
    ackermann_odometry_update(&s_odom, d_rear, steering_get(), dt);

    float v_rear = d_rear / dt;

    // FEED-FORWARD + PID sur la vitesse de l'essieu (schema de mowbot).
    // Consigne nulle : moteur coupe + reset PID, pas de freinage actif.
    if (s_target_v == 0.0f) {
        pid_reset(&s_pid_traction);
        traction_set(0.0f);
    } else {
        float ff = FF_GAIN * clampf(s_target_v / MAX_SPEED_MPS, -1.0f, 1.0f);
        traction_set(clampf(ff + pid_update(&s_pid_traction, s_target_v, v_rear, dt),
                            -1.0f, 1.0f));
    }

    odom->x = s_odom.x;
    odom->y = s_odom.y;
    odom->theta = s_odom.theta;
    odom->v = s_odom.v;
    odom->w = s_odom.w;
}

void kin_stop(void)
{
    // Une voiture qui perd sa liaison doit aller DROIT, pas garder son braquage.
    traction_stop();
    steering_center();
}

// ---------------------------------------------------------------------------
// Test banc, ROUES EN L'AIR et servo alimente. Verifie ce qui peut l'etre
// sans rouler : la course et le sens du servo, le sens de la traction et le
// signe des deux encodeurs. C'est la premiere chose a faire au bring-up.
// ---------------------------------------------------------------------------
void kin_bench_test(void)
{
    ESP_LOGW(TAG, "==== TEST BANC ACKERMANN (roues en l'air, servo alimente !) ====");

    ESP_LOGW(TAG, "1) direction : centre, gauche, droite, centre -- 1,5 s chacun.");
    ESP_LOGW(TAG, "   attendu : roues a GAUCHE quand delta > 0. Sinon : SERVO_INVERT.");
    ESP_LOGW(TAG, "   Les roues doivent atteindre la butee SANS forcer : sinon reduire");
    ESP_LOGW(TAG, "   STEER_MAX_RAD ou ajuster SERVO_MIN_US / SERVO_MAX_US.");
    static const struct { const char *nom; float delta; } etapes[] = {
        { "centre",        0.0f },
        { "gauche (+max)", +STEER_MAX_RAD },
        { "droite (-max)", -STEER_MAX_RAD },
        { "centre",        0.0f },
    };
    for (unsigned i = 0; i < sizeof(etapes) / sizeof(etapes[0]); i++) {
        steering_set(etapes[i].delta);
        ESP_LOGW(TAG, "   %-14s delta=%+.2f rad", etapes[i].nom, (double)etapes[i].delta);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }

    ESP_LOGW(TAG, "2) traction : 2 s avant (+), 2 s arriere (-).");
    ESP_LOGW(TAG, "   attendu : les DEUX encodeurs comptent + en avant, - en arriere.");
    ESP_LOGW(TAG, "   Sinon : 0=muet, signe d'une roue=ENC_x_INVERT, les deux=TRACTION_INVERT.");
    const float DUTY = 0.25f;
    for (int sens = +1; sens >= -1; sens -= 2) {
        int64_t g0 = encoder_get_ticks(ENCODER_LEFT);
        int64_t d0 = encoder_get_ticks(ENCODER_RIGHT);
        traction_set((float)sens * DUTY);
        vTaskDelay(pdMS_TO_TICKS(2000));
        traction_stop();
        ESP_LOGW(TAG, "   %s : enc_G=%+6lld  enc_D=%+6lld",
                 sens > 0 ? "avant  (+)" : "arriere(-)",
                 (long long)(encoder_get_ticks(ENCODER_LEFT) - g0),
                 (long long)(encoder_get_ticks(ENCODER_RIGHT) - d0));
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGW(TAG, "==== FIN TEST BANC ====");
}
