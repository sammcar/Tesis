#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "driver/ledc.h"
#include "driver/gpio.h"

// ... (tu header actual arriba)
typedef struct {
    gpio_num_t        pin;
    ledc_channel_t    channel;
    ledc_timer_t      timer;
    ledc_mode_t       speed_mode;
    ledc_timer_bit_t  duty_resolution;
    uint32_t          freq_hz;
    uint16_t          us_min, us_max;
    uint16_t          deg_min, deg_max;
    bool invert_dir; // false por defecto
    bool              inited;

    // --- NUEVO: mapeo lineal y calibración ---
    float             mm_per_rev;      // 52.5 mm por 360°
    float             stroke_mm_max;   // 95 mm
    float             zero_deg;        // offset mecánico (deg del "0 mm")
} ServoSG90_Handle_t;

// Conversión directa mm->deg con clamp de carrera y salida
bool ServoSG90_SetByMillimeters(ServoSG90_Handle_t* s, float target_mm, char* msg, size_t n);

// Igual que arriba, pero con corrección fina en grados (p.ej. salida PID pequeña)
bool ServoSG90_SetByMillimetersWithFine(ServoSG90_Handle_t* s, float target_mm,
                                        float fine_deg, float fine_deg_limit,
                                        char* msg, size_t n);

// Utilidades de conversión (por si las quieres en tu main/PID)
float ServoSG90_MmToDeg(const ServoSG90_Handle_t* s, float mm);
float ServoSG90_DegToMm(const ServoSG90_Handle_t* s, float deg);

// Setear/ajustar el cero mecánico (home)
void  ServoSG90_SetZeroDeg(ServoSG90_Handle_t* s, float zero_deg);


/** Inicializa LEDC y deja el servo listo */
bool ServoSG90_Init(ServoSG90_Handle_t* s, char* msg, size_t n);

/** Set ángulo en grados (clamp entre deg_min y deg_max) */
bool ServoSG90_SetAngleDeg(ServoSG90_Handle_t* s, float degrees, char* msg, size_t n);

/** Set pulso directo en microsegundos (clamp entre us_min y us_max) */
bool ServoSG90_SetPulseUS(ServoSG90_Handle_t* s, uint16_t pulse_us, char* msg, size_t n);

/** Apaga PWM (mantiene configuración) */
bool ServoSG90_Stop(ServoSG90_Handle_t* s, char* msg, size_t n);

/** Libera LEDC (opcional) */
bool ServoSG90_Deinit(ServoSG90_Handle_t* s, char* msg, size_t n);
