#pragma once

#include <stdbool.h>
#include <math.h>

#include "config.h"

// Cinematique et odometrie d'un vehicule Ackermann, MODELE BICYCLETTE :
// les deux roues avant sont ramenees a une roue virtuelle d'angle delta au
// milieu de l'essieu, les deux roues arriere a une roue virtuelle qui avance
// de d. Suffisant pour la navigation ; l'ecart entre roue interieure et
// exterieure (geometrie d'Ackermann proprement dite) est l'affaire de la
// timonerie mecanique, pas du firmware.
//
//   theta_dot = v * tan(delta) / x_s      (x_s = STEER_X_M, SIGNE)
//
// x_s est la position SIGNEE de la roue directrice depuis l'essieu moteur, +x
// vers l'avant. Positif = roue devant (tricycle classique) ; negatif = roue
// derriere (direction arriere), et le robot tourne alors du cote OPPOSE pour
// un meme braquage. Un seul parametre signe couvre les deux cas, plutot qu'un
// signe cache dans les formules.
//
// Convention ROS partout : x vers l'avant, y vers la gauche, theta et delta
// positifs dans le sens trigonometrique (braquer a GAUCHE = delta > 0).

typedef struct {
    float x;      // [m]   repere odom
    float y;      // [m]
    float theta;  // [rad] normalise dans [-pi, pi]
    float v;      // [m/s] vitesse lineaire courante
    float w;      // [rad/s] vitesse angulaire courante
    float delta;  // [rad] angle de braquage utilise pour ce pas
} ackermann_odom_t;

// Rayon de braquage MINIMAL, reellement DERIVE de la geometrie : R = L/tan(dmax).
// C'est la valeur a fournir a nav2 (SmacPlannerHybrid minimum_turning_radius,
// MPPI min_turning_r) et il ne faut JAMAIS la saisir a la main cote SBC.
//
// C'etait une macro qui codait tan(0,52) EN DUR : elle ne suivait donc pas
// STEER_MAX_RAD. A 45 deg elle annoncait un rayon 75 % trop grand. Une fonction
// ne peut pas se desynchroniser.
static inline float ackermann_min_turning_radius(void)
{
    return fabsf(STEER_X_M) / tanf(STEER_MAX_RAD);
}

// (v, w) d'un geometry_msgs/Twist -> (v, delta) pour la traction et le servo.
// Renvoie false quand la demande est physiquement IMPOSSIBLE : une rotation
// sur place (|v| ~ 0 et w != 0). Dans ce cas v_out = 0 et delta_out est braque
// a fond dans le sens demande, pour etre pret a partir.
bool ackermann_twist_to_cmd(float v, float w, float *v_out, float *delta_out);

// DIFFERENTIEL ELECTRONIQUE : vitesse d'essieu + braquage -> consigne de
// CHAQUE roue motrice. Obligatoire faute de differentiel mecanique : commander
// les deux roues a la meme vitesse les ferait se battre en virage.
//
// Sature en PRESERVANT LA COURBURE : si une roue depasse MAX_SPEED_MPS, les
// DEUX consignes sont reduites du meme facteur. Laisser une seule roue saturer
// changerait le rapport entre elles, donc le rayon : le robot ne tournerait
// plus assez. Avec la voie de robot A et 45 deg, la roue exterieure demande
// 174 % de la vitesse d'essieu et sature des 57 % de MAX_SPEED_MPS -- ce n'est
// donc pas un cas rare : a 45 deg la roue exterieure demande 164 % de la
// vitesse d'essieu, donc elle sature des 61 % de MAX_SPEED_MPS.
void ackermann_wheel_targets(float v, float delta, float *v_left, float *v_right);

// Integration d'un pas : d_rear_m = distance parcourue par l'essieu arriere,
// delta = angle de braquage pendant ce pas, dt = duree du pas.
void ackermann_odometry_update(ackermann_odom_t *o, float d_rear_m,
                               float delta, float dt);
