#ifndef VARIAC_RS485_H
#define VARIAC_RS485_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void Variac_Init(void);
bool Variac_VerificarConexion(void);
bool Variac_Encender(void);
bool Variac_Apagar(void);
bool Variac_EstablecerVoltaje(float voltaje);  // Ej: 80.0 para 80V
float Variac_LeerVoltaje(void);

#ifdef __cplusplus
}
#endif

#endif  // VARIAC_RS485_H
