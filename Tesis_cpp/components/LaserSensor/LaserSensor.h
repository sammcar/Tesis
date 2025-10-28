#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c.h"
#include "driver/gpio.h"

// Dirección I2C por defecto del VL53L0X
#define VL53L0X_I2C_ADDR  0x29

typedef struct {
    i2c_port_t   i2c_num;       // I2C_NUM_0 o I2C_NUM_1 (usas I2C_NUM_0)
    gpio_num_t   pin_sda;       // por defecto GPIO_NUM_21
    gpio_num_t   pin_scl;       // por defecto GPIO_NUM_22
    uint32_t     i2c_clk_hz;    // p. ej. 100000 o 400000
    gpio_num_t   pin_xshut;     // opcional, GPIO_NUM_NC si no se usa
    bool         inited;        // estado del init
} VL53L0X_Handle_t;

/* ===== Driver VL53L0X (capa cruda) ===== */
bool VL53L0X_Init          (VL53L0X_Handle_t* h, char* msg, size_t n);
bool VL53L0X_Reset         (VL53L0X_Handle_t* h, char* msg, size_t n);
bool VL53L0X_ReadDistanceMM(VL53L0X_Handle_t* h, uint16_t* mm, char* msg, size_t n);
bool VL53L0X_Deinit        (VL53L0X_Handle_t* h, char* msg, size_t n);

/* ===== Capa filtrada (task + filtros, API pública) ===== */
bool     Laser_init(VL53L0X_Handle_t* h);
bool     Laser_is_inited(void);

float    Laser_get_mm_filtrado(void);  // mediana(5) + EMA
float    Laser_get_mm_raw(void);       // crudo (última lectura válida)
uint16_t Laser_get_raw(void);          // raw auxiliar (mm truncado)

void     Laser_set_ema_alpha(float alpha);     // 0<alpha<=1
void     Laser_set_sample_period_ms(uint32_t ts_ms); // Ts>=10 ms
