#include "ackermann.h"
#include "config.h"

#include <math.h>

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

bool ackermann_twist_to_cmd(float v, float w, float *v_out, float *delta_out)
{
    if (fabsf(v) < V_EPS_MPS) {
        // ROTATION SUR PLACE DEMANDEE. Un Ackermann ne sait pas faire : sans
        // avance, tan(delta) ne produit aucune rotation. C'est exactement ce que
        // produisent les comportements Spin et RotateToGoal de nav2, ecrits pour
        // un diffdrive. On ne bouge pas, on pre-braque dans le sens voulu, et on
        // renvoie false pour que main.c puisse le journaliser : si ce cas revient
        // en boucle, c'est que nav2 n'est pas configure en Ackermann (controleur
        // RPP avec use_rotate_to_heading: false, ou MPPI en motion_model
        // Ackermann, et un planificateur qui respecte MIN_TURNING_RADIUS_M).
        *v_out = 0.0f;
        if (w > W_EPS_RADPS)       *delta_out =  STEER_MAX_RAD;
        else if (w < -W_EPS_RADPS) *delta_out = -STEER_MAX_RAD;
        else                       *delta_out =  0.0f;
        return fabsf(w) < W_EPS_RADPS;
    }

    // Modele bicyclette inverse : w = v tan(delta) / L  =>  delta = atan(w L / v).
    //
    // MARCHE ARRIERE : la formule reste JUSTE telle quelle. Avec v < 0 et w > 0
    // (nav2 demande de tourner dans le sens trigo en reculant), atan donne
    // delta < 0 : roues braquees a droite. Or en reculant roues a droite, le nez
    // part bien a gauche -- c'est ce que fait une voiture en creneau. Ne pas
    // "corriger" le signe pour la marche arriere, ce serait faux.
    float delta = atanf(w * WHEELBASE_M / v);

    // Au-dela du braquage max on GARDE v et on sature delta : la trajectoire
    // sera plus large que demandee. Nav2 ne doit jamais demander une courbure
    // superieure a 1/MIN_TURNING_RADIUS_M ; si cela arrive, c'est un probleme de
    // configuration cote SBC, pas de firmware.
    *delta_out = clampf(delta, -STEER_MAX_RAD, STEER_MAX_RAD);
    *v_out = clampf(v, -MAX_SPEED_MPS, MAX_SPEED_MPS);
    return true;
}

void ackermann_odometry_update(ackermann_odom_t *o, float d_rear_m,
                               float delta, float dt)
{
    // Rotation du pas : d * tan(delta) / L. Le signe de d porte la marche
    // arriere, celui de delta le cote de braquage ; leur produit donne le bon
    // sens de rotation dans tous les cas (voir ackermann_twist_to_cmd).
    float d_theta = d_rear_m * tanf(delta) / WHEELBASE_M;

    // Integration au point milieu : exacte au 2e ordre, suffisante a 50 Hz.
    // Meme schema que l'odometrie diffdrive de mowbot.
    o->x += d_rear_m * cosf(o->theta + 0.5f * d_theta);
    o->y += d_rear_m * sinf(o->theta + 0.5f * d_theta);
    o->theta += d_theta;

    while (o->theta > (float)M_PI)  o->theta -= 2.0f * (float)M_PI;
    while (o->theta < -(float)M_PI) o->theta += 2.0f * (float)M_PI;

    o->v = (dt > 0.0f) ? d_rear_m / dt : 0.0f;
    o->w = (dt > 0.0f) ? d_theta / dt : 0.0f;
    o->delta = delta;
}
