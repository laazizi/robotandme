#pragma once

#include <stdbool.h>

// ============================================================
// INTERFACE DE CINEMATIQUE : la frontiere entre ce qui est commun a tous les
// robots (main.c : micro-ROS, deadman, publication /odom et /imu) et ce qui
// change avec la MECANIQUE (comment un Twist devient un mouvement de roues,
// et comment un mouvement de roues redevient une pose).
//
// UNE SEULE implementation est compilee par controleur, choisie par le
// main/CMakeLists.txt du dossier du controleur (SRC_DIRS) :
//   common/kin_diffdrive/  -> mowbot_p4, mowbot_wroom
//   common/kin_ackermann/  -> ackerbot_p4
// Aucun #if dans ces fichiers : chacun ignore que l'autre existe.
// ============================================================

// Pose et vitesses dans le repere odom, telles que main.c les publie.
typedef struct {
    float x;      // [m]
    float y;      // [m]
    float theta;  // [rad] normalise dans [-pi, pi]
    float v;      // [m/s]
    float w;      // [rad/s]
} kin_odom_t;

// Materiel de la cinematique : moteurs, servo, encodeurs, regulateurs. Doit
// laisser le robot A L'ARRET (sortie moteur nulle, direction au centre).
void kin_init(void);

// Nouvelle consigne (v, w) d'un geometry_msgs/Twist. Ne fait que MEMORISER
// la consigne : c'est kin_update() qui agit, au rythme de la boucle de
// controle. Renvoie false si la demande est physiquement impossible pour ce
// robot (ex. rotation sur place en Ackermann) -- la consigne memorisee est
// alors la plus proche realisable. (0, 0) veut toujours dire "arret".
bool kin_apply_twist(float v, float w);

// Un pas de la boucle de controle a CONTROL_PERIOD_MS : lit les capteurs,
// integre l'odometrie dans *odom, applique la regulation aux actionneurs.
void kin_update(float dt, kin_odom_t *odom);

// Arret SUR, immediat, sans regulation : erreur micro-ROS.
void kin_stop(void);

// Test au banc, ROUES EN L'AIR, appele au boot si BOOT_BENCH_TEST vaut 1.
// Chaque cinematique sait ce qui vaut la peine d'etre verifie sur sa
// mecanique (sens des moteurs et des encodeurs, course du servo...).
void kin_bench_test(void);
