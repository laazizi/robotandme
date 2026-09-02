#pragma once

// PID de vitesse roue avec anti-windup par clamping de l'intégrale.
// (pid_ctrl_t et non pid_t : pid_t est pris par <sys/types.h>)

typedef struct {
    float kp, ki, kd;
    float out_min, out_max;
    float integral;
    float prev_error;
} pid_ctrl_t;

void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd,
              float out_min, float out_max);

float pid_update(pid_ctrl_t *pid, float setpoint, float measure, float dt);

void pid_reset(pid_ctrl_t *pid);
