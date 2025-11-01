#include "ServoSG90.h"
#include <stdio.h>

static inline uint32_t duty_from_us(const ServoSG90_Handle_t* s, uint16_t pulse_us){
    const uint32_t period_us = 1000000UL / s->freq_hz;              // 20,000 us @50Hz
    const uint32_t max_duty  = (1UL << s->duty_resolution) - 1UL;   // p.ej. 65535
    if (pulse_us < s->us_min) pulse_us = s->us_min;
    if (pulse_us > s->us_max) pulse_us = s->us_max;
    // duty = (pulse / periodo) * max_duty
    return (uint32_t)((uint64_t)pulse_us * max_duty / period_us);
}

bool ServoSG90_Init(ServoSG90_Handle_t* s, char* msg, size_t n){
    if (!s) return false;
    if (s->freq_hz == 0) s->freq_hz = 50;
    if (s->duty_resolution == 0) s->duty_resolution = LEDC_TIMER_16_BIT;
    if (s->speed_mode != LEDC_LOW_SPEED_MODE && s->speed_mode != LEDC_HIGH_SPEED_MODE)
        s->speed_mode = LEDC_LOW_SPEED_MODE;
    if (s->us_min == 0) s->us_min = 500;
    if (s->us_max == 0) s->us_max = 2500;
    if (s->deg_max == 0) { s->deg_min = 0; s->deg_max = 180; }
    if (s->timer < LEDC_TIMER_0 || s->timer > LEDC_TIMER_3) s->timer = LEDC_TIMER_0;
    if (s->channel < LEDC_CHANNEL_0 || s->channel > LEDC_CHANNEL_7) s->channel = LEDC_CHANNEL_0;

    ledc_timer_config_t tcfg = {
        .speed_mode       = s->speed_mode,
        .duty_resolution  = s->duty_resolution,
        .timer_num        = s->timer,
        .freq_hz          = s->freq_hz,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&tcfg);
    if (err != ESP_OK){ if (msg&&n) snprintf(msg,n,"ledc_timer_config: %s", esp_err_to_name(err)); return false; }

    ledc_channel_config_t ccfg = {
        .gpio_num   = s->pin,
        .speed_mode = s->speed_mode,
        .channel    = s->channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = s->timer,
        .duty       = 0,
        .hpoint     = 0
    };
    err = ledc_channel_config(&ccfg);
    if (err != ESP_OK){ if (msg&&n) snprintf(msg,n,"ledc_channel_config: %s", esp_err_to_name(err)); return false; }

    s->inited = true;
    if (msg && n) snprintf(msg, n, "ServoSG90 listo en GPIO %d @%lu Hz", (int)s->pin, (unsigned long)s->freq_hz);
    return true;
}

bool ServoSG90_SetPulseUS(ServoSG90_Handle_t* s, uint16_t pulse_us, char* msg, size_t n){
    if (!s || !s->inited){ if (msg&&n) snprintf(msg,n,"Servo no inicializado"); return false; }
    uint32_t duty = duty_from_us(s, pulse_us);
    esp_err_t err = ledc_set_duty(s->speed_mode, s->channel, duty);
    if (err == ESP_OK) err = ledc_update_duty(s->speed_mode, s->channel);
    if (err != ESP_OK){ if (msg&&n) snprintf(msg,n,"SetPulse %uus: %s", pulse_us, esp_err_to_name(err)); return false; }
    if (msg && n) snprintf(msg, n, "Pulso %u us (duty=%lu)", pulse_us, (unsigned long)duty);
    return true;
}

bool ServoSG90_SetAngleDeg(ServoSG90_Handle_t* s, float degrees, char* msg, size_t n){
    if (!s || !s->inited){ if (msg&&n) snprintf(msg,n,"Servo no inicializado"); return false; }
    if (degrees < s->deg_min) degrees = s->deg_min;
    if (degrees > s->deg_max) degrees = s->deg_max;
    float frac = (degrees - s->deg_min) / (float)(s->deg_max - s->deg_min); // 0..1
    uint16_t pulse = (uint16_t)(s->us_min + frac * (s->us_max - s->us_min));
    return ServoSG90_SetPulseUS(s, pulse, msg, n);
}

bool ServoSG90_Stop(ServoSG90_Handle_t* s, char* msg, size_t n){
    if (!s || !s->inited){ if (msg&&n) snprintf(msg,n,"Servo no inicializado"); return false; }
    esp_err_t err = ledc_set_duty(s->speed_mode, s->channel, 0);
    if (err == ESP_OK) err = ledc_update_duty(s->speed_mode, s->channel);
    if (msg && n) snprintf(msg, n, (err==ESP_OK) ? "PWM detenido" : "Error al detener: %s", esp_err_to_name(err));
    return (err == ESP_OK);
}

bool ServoSG90_Deinit(ServoSG90_Handle_t* s, char* msg, size_t n){
    if (!s) return false;
    // No hay una API "uninstall" por canal; detener PWM es suficiente.
    s->inited = false;
    if (msg && n) snprintf(msg, n, "ServoSG90 deinit");
    return true;
}

// --- utilidades de conversión ---
float ServoSG90_MmToDeg(const ServoSG90_Handle_t* s, float mm){
    float m = mm;
    if (m < 0) m = 0;
    if (s->stroke_mm_max > 0 && m > s->stroke_mm_max) m = s->stroke_mm_max;

    float deg_per_mm = 360.0f / (s->mm_per_rev > 0 ? s->mm_per_rev : 52.5f);
    float sign = s->invert_dir ? -1.0f : +1.0f;
    return s->zero_deg + sign * (m * deg_per_mm);
}

float ServoSG90_DegToMm(const ServoSG90_Handle_t* s, float deg){
    float deg_per_mm = 360.0f / (s->mm_per_rev > 0 ? s->mm_per_rev : 52.5f);
    return (deg - s->zero_deg) / deg_per_mm;
}

void ServoSG90_SetZeroDeg(ServoSG90_Handle_t* s, float zero_deg){
    if (!s) return;
    s->zero_deg = zero_deg;
}

// --- mando abierto en mm ---
bool ServoSG90_SetByMillimeters(ServoSG90_Handle_t* s, float target_mm, char* msg, size_t n){
    if (!s || !s->inited){ if (msg&&n) snprintf(msg,n,"Servo no inicializado"); return false; }
    float mm = target_mm;
    if (mm < 0) mm = 0;
    if (s->stroke_mm_max > 0 && mm > s->stroke_mm_max) mm = s->stroke_mm_max;

    float deg = ServoSG90_MmToDeg(s, mm);

    // clamp a límites mecánicos del servo
    if (deg < s->deg_min) deg = s->deg_min;
    if (deg > s->deg_max) deg = s->deg_max;

    return ServoSG90_SetAngleDeg(s, deg, msg, n);
}

// --- mando abierto + corrección fina (PID en grados pequeños) ---
bool ServoSG90_SetByMillimetersWithFine(ServoSG90_Handle_t* s, float target_mm,
                                        float fine_deg, float fine_deg_limit,
                                        char* msg, size_t n)
{
    if (!s || !s->inited){ if (msg&&n) snprintf(msg,n,"Servo no inicializado"); return false; }
    // base en mm
    float mm = target_mm;
    if (mm < 0) mm = 0;
    if (s->stroke_mm_max > 0 && mm > s->stroke_mm_max) mm = s->stroke_mm_max;

    float base_deg = ServoSG90_MmToDeg(s, mm);

    // limitar corrección fina
    float lim = (fine_deg_limit > 0 ? fine_deg_limit : 10.0f); // por defecto ±10°
    if (fine_deg >  lim) fine_deg =  lim;
    if (fine_deg < -lim) fine_deg = -lim;

    float cmd_deg = base_deg + fine_deg;

    // clamp a límites del servo
    if (cmd_deg < s->deg_min) cmd_deg = s->deg_min;
    if (cmd_deg > s->deg_max) cmd_deg = s->deg_max;

    return ServoSG90_SetAngleDeg(s, cmd_deg, msg, n);
}
