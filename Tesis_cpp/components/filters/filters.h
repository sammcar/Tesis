/* filters.h --------------------------------------------------*/
#pragma once
#include <stdint.h>
typedef struct {
    float alpha;
    float y_prev;
} filt_ema_t;

void  ema_init(filt_ema_t *f, float alpha);
float ema_apply(filt_ema_t *f, float x);

typedef struct {
    float buf[5];
    uint8_t idx;
} filt_med5_t;

void  med5_init(filt_med5_t *f);
float med5_apply(filt_med5_t *f, float x);
