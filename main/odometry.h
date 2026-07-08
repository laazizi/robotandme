#pragma once

// Odométrie diffdrive intégrée sur le MCU. Sur une tondeuse le patinage
// rend cette odométrie dérivante : elle sert de source SECONDAIRE dans
// l'EKF côté SBC (robot_localization), fusionnée avec IMU puis RTK GPS.

typedef struct {
    float x;      // [m] repère odom
    float y;      // [m]
    float theta;  // [rad] normalisé dans [-pi, pi]
    float v;      // [m/s] vitesse linéaire courante
    float w;      // [rad/s] vitesse angulaire courante
} odom_state_t;

void odometry_update(odom_state_t *odom, float d_left_m, float d_right_m, float dt);
