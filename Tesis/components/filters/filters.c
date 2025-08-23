/* filters.c --------------------------------------------------*/
#include "filters.h"
#include <string.h>

/* -------- EMA -------- */
void  ema_init(filt_ema_t *f, float alpha){ f->alpha = alpha; f->y_prev = 0; }
float ema_apply(filt_ema_t *f, float x){
    float y = f->alpha * x + (1 - f->alpha) * f->y_prev;
    f->y_prev = y;
    return y;
}

/* -------- Mediana 5 --- */
static inline void swapf(float *a, float *b){ float t=*a; *a=*b; *b=t; }
static float median5(float *v){           // ordenación parcial
    /* Algoritmo de selección rápido (codigo compacto) */
    #define MINMAX(a,b) if(v[a]>v[b]) swapf(&v[a],&v[b])
    MINMAX(0,1); MINMAX(3,4); MINMAX(0,3);
    MINMAX(1,4); MINMAX(1,2); MINMAX(2,3);
    return v[2];
}
void  med5_init(filt_med5_t *f){ memset(f,0,sizeof(*f)); }
float med5_apply(filt_med5_t *f, float x){
    f->buf[f->idx++] = x;
    if(f->idx==5) f->idx = 0;
    float v[5]; memcpy(v,f->buf,sizeof v);
    return median5(v);
}
