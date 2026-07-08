#pragma once

// Pilotage MDD10A rev 2.0 en sign-magnitude : PWM = |vitesse|, DIR = sens.

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1,
} motor_id_t;

void motors_init(void);

// cmd dans [-1.0, 1.0] ; clampé en interne.
void motors_set(motor_id_t motor, float cmd);

void motors_stop(void);
