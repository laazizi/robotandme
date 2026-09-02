// Cinematique ACKERMANN : essieu avant directionnel (servo RC), traction sur
// l'essieu arriere par DEUX moteurs (les deux canaux MDD10A), encodeurs sur
// arriere. PAS de rotation sur place -- voir ackermann.c pour ce que cela
// implique cote nav2. Utilisee par ackerbot_p4. N'a JAMAIS ROULE : voir le
// robot.h de ce controleur pour les placeholders.
//
// Deux moteurs arriere SANS differentiel mecanique : ce fichier calcule le
// differentiel ELECTRONIQUE. C'est obligatoire, pas une option -- commander
// les deux roues a la meme vitesse les fait se battre en virage.
//
// Ce fichier fait le lien entre l'interface kin.h et les modules Ackermann :
//   ackermann.c  : (v, w) -> (v, delta) et odometrie bicyclette (testes sur PC)
//   steering.c   : le servo (LEDC TIMER_1, 50 Hz)
//   traction.c   : les deux moteurs arriere (LEDC TIMER_0, 20 kHz)

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

static pid_ctrl_t s_pid_left;      // une boucle de vitesse PAR ROUE arriere
static pid_ctrl_t s_pid_right;
static int s_slip_warnings;        // caps incoherents (patinage ou delta faux)
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
    pid_init(&s_pid_left, PID_KP, PID_KI, PID_KD, -1.0f, 1.0f);
    pid_init(&s_pid_right, PID_KP, PID_KI, PID_KD, -1.0f, 1.0f);
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
    const float delta = steering_get();

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
    ackermann_odometry_update(&s_odom, d_rear, delta, dt);

    const float v_rear  = d_rear / dt;
    const float v_left  = d_left / dt;
    const float v_right = d_right / dt;

    // ---- DIFFERENTIEL ELECTRONIQUE ------------------------------------------
    // Deux moteurs arriere, aucun differentiel mecanique : la roue interieure
    // DOIT tourner moins vite, sinon les deux roues se battent (ripage, appel
    // de courant, odometrie faussee).
    //   k = voie * tan(delta) / (2 L)      v_int = v (1-k)   v_ext = v (1+k)
    // Forme equivalente a (R -+ voie/2)/R avec R = L/tan(delta), mais SANS
    // division par R : aucune singularite en ligne droite (delta=0 -> k=0).
    // k reste tres inferieur a 1 (0,239 a la butee avec la geometrie actuelle),
    // donc la roue interieure ne s'inverse jamais. delta > 0 = virage a GAUCHE,
    // la roue gauche est donc l'interieure.
    const float k = TRACK_WIDTH_M * tanf(delta) / (2.0f * WHEELBASE_M);
    const float target_left  = s_target_v * (1.0f - k);
    const float target_right = s_target_v * (1.0f + k);

    // FEED-FORWARD + PID par ROUE (schema de mowbot, une boucle par moteur).
    // Consigne nulle : moteurs coupes + reset PID, pas de freinage actif.
    if (s_target_v == 0.0f) {
        pid_reset(&s_pid_left);
        pid_reset(&s_pid_right);
        traction_stop();
    } else {
        const float ff_l = FF_GAIN * clampf(target_left / MAX_SPEED_MPS, -1.0f, 1.0f);
        const float ff_r = FF_GAIN * clampf(target_right / MAX_SPEED_MPS, -1.0f, 1.0f);
        traction_set(TRACTION_LEFT,
                     clampf(ff_l + pid_update(&s_pid_left, target_left, v_left, dt),
                            -1.0f, 1.0f));
        traction_set(TRACTION_RIGHT,
                     clampf(ff_r + pid_update(&s_pid_right, target_right, v_right, dt),
                            -1.0f, 1.0f));
    }

    // ---- RECOUPEMENT DU CAP : patinage, ou servo qui ment ? ------------------
    // Avoir deux encodeurs arriere donne un SECOND cap, independant du modele :
    //   w_roues  = (v_droite - v_gauche) / voie
    //   w_modele = v * tan(delta) / L
    // Le servo etant SANS retour, une divergence durable signale soit du
    // patinage, soit un angle de braquage reel different du commande. C'est
    // gratuit, et c'est le seul controle possible sur delta. Le seuil de
    // 0,35 rad/s (~20 deg/s) est a affiner au bring-up.
    if (fabsf(v_rear) > 0.1f) {
        const float w_wheels = (v_right - v_left) / TRACK_WIDTH_M;
        const float w_model  = v_rear * tanf(delta) / WHEELBASE_M;
        if (fabsf(w_wheels - w_model) > 0.35f && (++s_slip_warnings % 50) == 1) {
            ESP_LOGW(TAG, "cap incoherent (%d fois) : roues %+.2f rad/s vs modele "
                          "%+.2f rad/s a delta=%+.2f rad -- patinage, ou braquage "
                          "reel different du commande",
                     s_slip_warnings, (double)w_wheels, (double)w_model, (double)delta);
        }
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
    ESP_LOGW(TAG, "   PIN_SERVO=%d est un CANDIDAT non valide sur cette carte. Si le",
             PIN_SERVO);
    ESP_LOGW(TAG, "   servo reste INERTE aux 4 etapes, la broche est probablement morte :");
    ESP_LOGW(TAG, "   en essayer une autre du header DROIT (robot.h). Verifier d'abord");
    ESP_LOGW(TAG, "   que le servo est alimente SEPAREMENT en 5-6 V, masse commune.");
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

    ESP_LOGW(TAG, "2) traction : chaque roue SEPAREMENT, 2 s avant (+), 2 s arriere (-).");
    ESP_LOGW(TAG, "   attendu : SEULE la roue testee tourne, et SEUL son encodeur bouge,");
    ESP_LOGW(TAG, "   en comptant + en avant.");
    ESP_LOGW(TAG, "   0 partout        -> moteur non alimente ou broche PWM muette");
    ESP_LOGW(TAG, "   signe inverse    -> ENC_x_INVERT de cette roue");
    ESP_LOGW(TAG, "   roue a l'envers  -> MOTOR_x_INVERT de cette roue");
    ESP_LOGW(TAG, "   l'AUTRE encodeur bouge -> canaux moteur ou encodeurs permutes");
    const float DUTY = 0.25f;
    static const struct { const char *nom; traction_id_t id; } roues[] = {
        { "GAUCHE", TRACTION_LEFT  },
        { "DROITE", TRACTION_RIGHT },
    };
    for (unsigned r = 0; r < sizeof(roues) / sizeof(roues[0]); r++) {
        for (int sens = +1; sens >= -1; sens -= 2) {
            int64_t g0 = encoder_get_ticks(ENCODER_LEFT);
            int64_t d0 = encoder_get_ticks(ENCODER_RIGHT);
            traction_set(roues[r].id, (float)sens * DUTY);
            vTaskDelay(pdMS_TO_TICKS(2000));
            traction_stop();
            ESP_LOGW(TAG, "   roue %s %s : enc_G=%+6lld  enc_D=%+6lld",
                     roues[r].nom, sens > 0 ? "avant  (+)" : "arriere(-)",
                     (long long)(encoder_get_ticks(ENCODER_LEFT) - g0),
                     (long long)(encoder_get_ticks(ENCODER_RIGHT) - d0));
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    // Pas d'action, juste de quoi verifier la geometrie avant de rouler : si
    // ces pourcentages semblent absurdes, TRACK_WIDTH_M / WHEELBASE_M /
    // STEER_MAX_RAD sont encore des placeholders.
    const float k_max = TRACK_WIDTH_M * tanf(STEER_MAX_RAD) / (2.0f * WHEELBASE_M);
    ESP_LOGW(TAG, "3) differentiel electronique (calcul, aucun mouvement) :");
    ESP_LOGW(TAG, "   L=%.3f m  voie=%.3f m  delta_max=%.2f rad  ->  k_max=%.3f",
             (double)WHEELBASE_M, (double)TRACK_WIDTH_M, (double)STEER_MAX_RAD,
             (double)k_max);
    ESP_LOGW(TAG, "   a la butee : roue INT a %.0f %%, roue EXT a %.0f %% de la vitesse",
             (double)(100.0f * (1.0f - k_max)), (double)(100.0f * (1.0f + k_max)));
    ESP_LOGW(TAG, "   rayon de braquage min = %.3f m (a donner a nav2)",
             (double)MIN_TURNING_RADIUS_M);

    ESP_LOGW(TAG, "==== FIN TEST BANC ====");
}
