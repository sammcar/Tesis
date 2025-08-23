#pragma once
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    gpio_num_t step_pin, dir_pin, en_pin;
    rmt_channel_handle_t ch;
    rmt_encoder_handle_t enc;

    size_t   steps_left;
    uint32_t tick_high, tick_low;

    uint32_t resolution_hz;        //  <──  NUEVO
    bool     busy;
} stepper_motor_t;


esp_err_t stepper_init(stepper_motor_t *m,
                       gpio_num_t step, gpio_num_t dir, gpio_num_t en,
                       uint32_t resolution_hz);             // 0 → 10 MHz por defecto

esp_err_t stepper_move(stepper_motor_t *m,
                       int32_t steps,           // signo = dirección
                       uint32_t freq_hz);       // 1 Hz … 30 kHz aprox.

bool  stepper_is_busy(const stepper_motor_t *m);
void  stepper_stop(stepper_motor_t *m);
void  stepper_enable(stepper_motor_t *m, bool enable_low_active);
