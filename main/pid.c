#include "pid.h"

void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid_reset(pid);
}

float pid_update(pid_ctrl_t *pid, float setpoint, float measure, float dt)
{
    float error = setpoint - measure;

    pid->integral += error * dt;
    if (pid->ki > 0.0f) {
        float integral_max = pid->out_max / pid->ki;
        if (pid->integral > integral_max) pid->integral = integral_max;
        if (pid->integral < -integral_max) pid->integral = -integral_max;
    }

    float derivative = (dt > 0.0f) ? (error - pid->prev_error) / dt : 0.0f;
    pid->prev_error = error;

    float out = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;
    return out;
}

void pid_reset(pid_ctrl_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
