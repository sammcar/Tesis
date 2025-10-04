#pragma once
#include <stdint.h>

typedef struct {
    /* Ganancias */
    float kp;
    float ki;
    float kd;

    /* Límites y estado */
    float setpoint;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
    float Ts;             /* periodo de muestreo [s] */

     // NUEVOS: para filtro derivativo
    float alpha;           // α ∈ [0, 1]
    float d_filtered;      // valor filtrado anterior
} PID_t;

/* ---------- API pública ---------- */
void pid_init(PID_t *pid,
              float kp, float ki, float kd,
              float setpoint,
              float out_min, float out_max,
              float Ts,
              float alpha);   // ← AGREGA ESTE PARÁMETRO


void  pid_set_gains     (PID_t *pid, float kp, float ki, float kd);
void  pid_set_setpoint  (PID_t *pid, float setpoint);
float pid_compute       (PID_t *pid, float measurement);
void  pid_reset_state   (PID_t *pid);
