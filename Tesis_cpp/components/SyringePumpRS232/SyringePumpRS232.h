#ifndef SYRINGE_PUMP_RS232_H
#define SYRINGE_PUMP_RS232_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// UART y configuración de pines
#define SYRINGE_UART       UART_NUM_2
#define SYRINGE_TX_PIN     16
#define SYRINGE_RX_PIN     17
#define SYRINGE_BUF_SIZE   256

// Inicialización y dirección
void Syringe_Init(void);
void Syringe_SetAddress(uint8_t addr);

// Comandos básicos
bool Syringe_ExitSafeMode(void);
bool Syringe_VerifyConnection(void);
bool Syringe_SendCommand(const char *core);
int  Syringe_ReadResponse(char *out, int maxlen, int timeout_ms);

// Funciones específicas
bool Syringe_SelectPhase(uint8_t phase);
bool Syringe_QueryPhase(uint8_t *out_phase);
bool Syringe_SetPhaseFunction(const char *func_core);
bool Syringe_QueryPhaseFunction(char *out_func, int maxlen);
bool Syringe_SetRate(float rate, const char *units);
bool Syringe_QueryRate(float *out_rate, char *out_units, int units_len);
bool Syringe_SetDiameter(float diameter);
bool Syringe_QueryDiameter(float *out_diameter);
bool Syringe_Run(uint8_t start_phase);
bool Syringe_Stop(void);
bool Syringe_SetDirection(const char *dir);
bool Syringe_QueryDirection(char *out_dir, int maxlen);
bool Syringe_QueryDispensed(float *out_infuse, float *out_withdraw, char *out_units, int units_len);
bool Syringe_ClearDispensed(const char *which);

// Puedes añadir aquí más funciones específicas según las vayas implementando
// Ej: bool Syringe_SetVolume(...);

#ifdef __cplusplus
}
#endif

#endif // SYRINGE_PUMP_RS232_H
