#ifndef POTENCIOMETRO_H
#define POTENCIOMETRO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Potenciometro_init(void);
float Potenciometro_get_vueltas(void);
float Potenciometro_get_grados(void);
float Potenciometro_get_grados_filtrado(void);
void Potenciometro_reset_calib(void);

uint32_t Potenciometro_get_mV(void);
uint16_t Potenciometro_get_raw(void);

#ifdef __cplusplus
}
#endif

#endif // POTENCIOMETRO_H
