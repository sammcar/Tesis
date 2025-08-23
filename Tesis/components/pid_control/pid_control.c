#include "pid_control.h"

/*--------------- helpers -------------*/
static inline float clamp(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi ? hi : v);
}

/*--------------- API -----------------*/

void pid_init(PID_t *pid, float kp, float ki, float kd,
              float setpoint, float out_min, float out_max,
              float Ts, float alpha){
    pid->kp = kp;
    pid->ki = ki * Ts;
    pid->kd = kd / Ts;
    pid->Ts = Ts;
    pid->setpoint = setpoint;
    pid->out_min = out_min;
    pid->out_max = out_max;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;

    pid->alpha = alpha;         // ← guardar α
    pid->d_filtered = 0.0f;     // ← inicializar filtro
}


void pid_set_gains(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki * pid->Ts;
    pid->kd = kd / pid->Ts;
}

void pid_set_setpoint(PID_t *pid, float setpoint) { pid->setpoint = setpoint; }

void pid_reset_state(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

float pid_compute(PID_t *pid, float y)
{
    float error = pid->setpoint - y;

    // proporcional
    float P = pid->kp * error;

    // integral con anti-wind-up
    pid->integral += pid->ki * error;
    pid->integral  = clamp(pid->integral, pid->out_min, pid->out_max);

    // derivativa con filtro EMA (si alpha < 1)
    float d_raw = error - pid->prev_error;
    pid->d_filtered = pid->alpha * pid->d_filtered + (1.0f - pid->alpha) * d_raw;
    float D = pid->kd * pid->d_filtered;
    pid->prev_error = error;

    float out = P + pid->integral + D;
    return clamp(out, pid->out_min, pid->out_max);
}

