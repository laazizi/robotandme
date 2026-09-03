#include "ackermann.h"
#include "config.h"

#include <math.h>

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

bool ackermann_twist_to_cmd(float v, float w, float delta_courant,
                            float *v_out, float *delta_out)
{
    if (fabsf(v) < V_EPS_MPS) {
        // ROTATION SUR PLACE DEMANDEE. Un Ackermann ne sait pas faire : sans
        // avance, tan(delta) ne produit aucune rotation. C'est exactement ce que
        // produisent les comportements Spin et RotateToGoal de nav2, ecrits pour
        // un diffdrive. On ne bouge pas, on pre-braque dans le sens voulu, et on
        // renvoie false pour que main.c puisse le journaliser : si ce cas revient
        // en boucle, c'est que nav2 n'est pas configure en Ackermann (controleur
        // RPP avec use_rotate_to_heading: false, ou MPPI en motion_model
        // Ackermann, et un planificateur qui respecte ackermann_min_turning_radius()).
        // ON GARDE L'ANGLE COURANT, on ne braque PAS a fond.
        // Le pre-braquage a fond etait la depuis l'origine, pour "etre pret a
        // partir". C'est une mauvaise idee sur ce robot, et l'utilisateur l'a
        // vu le 04/09/2026 : "il n'arrete pas de faire tourner le servomoteur".
        // MPPI, quand il hesite, emet des consignes a vitesse quasi nulle avec
        // une rotation : chacune faisait claquer le servo a +-45 degres A
        // L'ARRET, en boucle. C'est le pire cas de couple pour un servo RC de
        // 10 kg.cm, et cela ne sert a RIEN puisque ce tricycle ne peut de toute
        // facon pas pivoter sur place.
        // Reconduire l'angle courant supprime le battement sans rien coter :
        // des que la vitesse repasse au-dessus de V_EPS_MPS, la formule
        // bicyclette reprend la main au cycle suivant.
        *v_out = 0.0f;
        *delta_out = delta_courant;
        return fabsf(w) < W_EPS_RADPS;
    }

    // Modele bicyclette inverse : w = v tan(delta)/x_s => delta = atan(w x_s/v).
    // x_s est SIGNE (STEER_X_M) : une roue directrice a l'arriere donne un
    // braquage de signe oppose, ce qui est physiquement juste.
    //
    // MARCHE ARRIERE : la formule reste JUSTE telle quelle. Avec v < 0 et w > 0
    // (nav2 demande de tourner dans le sens trigo en reculant), atan donne
    // delta < 0 : roues braquees a droite. Or en reculant roues a droite, le nez
    // part bien a gauche -- c'est ce que fait une voiture en creneau. Ne pas
    // "corriger" le signe pour la marche arriere, ce serait faux.
    float delta = atanf(w * STEER_X_M / v);

    // Au-dela du braquage max on GARDE v et on sature delta : la trajectoire
    // sera plus large que demandee. Nav2 ne doit jamais demander une courbure
    // superieure a 1/ackermann_min_turning_radius() ; si cela arrive, c'est un probleme de
    // configuration cote SBC, pas de firmware.
    *delta_out = clampf(delta, -STEER_MAX_RAD, STEER_MAX_RAD);
    *v_out = clampf(v, -MAX_SPEED_MPS, MAX_SPEED_MPS);
    return true;
}

void ackermann_wheel_targets(float v, float delta, float *v_left, float *v_right)
{
    // k = voie*tan(delta)/(2*x_s). Forme equivalente a (R -+ voie/2)/R avec
    // R = x_s/tan(delta), mais sans division par R : pas de singularite en
    // ligne droite. x_s SIGNE, donc la roue interieure change de cote selon
    // que la roue directrice est devant ou derriere.
    const float k = TRACK_WIDTH_M * tanf(delta) / (2.0f * STEER_X_M);
    float g = v * (1.0f - k);
    float d = v * (1.0f + k);

    // Saturation a courbure constante : on divise les deux par le meme facteur.
    const float pire = fabsf(g) > fabsf(d) ? fabsf(g) : fabsf(d);
    if (pire > MAX_SPEED_MPS) {
        const float f = MAX_SPEED_MPS / pire;
        g *= f;
        d *= f;
    }

    *v_left = g;
    *v_right = d;
}

void ackermann_odometry_update(ackermann_odom_t *o, float d_rear_m,
                               float delta, float dt)
{
    // Rotation du pas : d * tan(delta) / x_s. Le signe de d porte la marche
    // arriere, celui de delta le cote de braquage, celui de x_s la position de
    // la roue directrice ; leur produit donne le bon sens dans tous les cas.
    float d_theta = d_rear_m * tanf(delta) / STEER_X_M;

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
