#pragma once

#include <stdbool.h>

// Cinematique et odometrie d'un vehicule Ackermann, MODELE BICYCLETTE :
// les deux roues avant sont ramenees a une roue virtuelle d'angle delta au
// milieu de l'essieu, les deux roues arriere a une roue virtuelle qui avance
// de d. Suffisant pour la navigation ; l'ecart entre roue interieure et
// exterieure (geometrie d'Ackermann proprement dite) est l'affaire de la
// timonerie mecanique, pas du firmware.
//
//   theta_dot = v * tan(delta) / L        (L = WHEELBASE_M)
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

// (v, w) d'un geometry_msgs/Twist -> (v, delta) pour la traction et le servo.
// Renvoie false quand la demande est physiquement IMPOSSIBLE : une rotation
// sur place (|v| ~ 0 et w != 0). Dans ce cas v_out = 0 et delta_out est braque
// a fond dans le sens demande, pour etre pret a partir.
bool ackermann_twist_to_cmd(float v, float w, float *v_out, float *delta_out);

// Integration d'un pas : d_rear_m = distance parcourue par l'essieu arriere,
// delta = angle de braquage pendant ce pas, dt = duree du pas.
void ackermann_odometry_update(ackermann_odom_t *o, float d_rear_m,
                               float delta, float dt);
