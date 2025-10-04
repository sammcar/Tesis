#ifndef UART_CONFIG_H
#define UART_CONFIG_H

#include <stdio.h>

/**
 * @brief Lee una línea desde UART, procesa pares Clave:Valor y actualiza variables.
 */
void uart_leer_y_procesar(void);

/** @brief Obtiene el valor de Pot. */
int uart_getPot(void);
/** @brief Obtiene el valor de Variac. */
int uart_getVariac(void);
/** @brief Obtiene el valor de Rango. */
int uart_getRango(void);
/** @brief Obtiene el valor de FloatVal. */
float uart_getFloatVal(void);
/** @brief Obtiene la cadena de Modo. */
const char* uart_getModo(void);

#endif // UART_CONFIG_H