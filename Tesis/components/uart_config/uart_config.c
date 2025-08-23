// uart_config.c
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ------ Variables globales a actualizar ------
static int    Pot      = 0;
static int    Variac   = 0;
static int    Rango    = 0;
static float  FloatVal = 0.0f;
static char   Modo[16] = "Automatico";

// ------ Prototipos internos ------
static void trim(char *s);
static void procesarLinea(char *buffer);
void uart_leer_y_procesar(void);

// ------------------------------------------------
// Recibe una línea por UART, la parsea y actualiza
// ------------------------------------------------
void uart_leer_y_procesar(void)
{
    char buffer[128];
    int  index = 0;
    int  c;

    // Limpia buffer
    memset(buffer, 0, sizeof(buffer));

    // Leer hasta '\n'
    while (1) {
        c = getchar();
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (c == '\r') continue;   // ignora CR
        if (c == '\n') break;
        if (index < (int)sizeof(buffer) - 1)
            buffer[index++] = (char)c;
    }
    buffer[index] = '\0';

    // Procesa los pares Clave:Valor
    procesarLinea(buffer);
}

// -----------------------
// Quita espacios delante/detrás
// -----------------------
static void trim(char *s)
{
    // trim al inicio
    char *p = s;
    while (*p == ' ' || *p == '\t') p++;
    if (p != s) memmove(s, p, strlen(p) + 1);

    // trim al final
    int len = strlen(s);
    while (len > 0 && (s[len-1] == ' ' || s[len-1] == '\t')) {
        s[--len] = '\0';
    }
}

// -------------------------------------------
// Parsea buffer con tokens "Clave:Valor", coma
// -------------------------------------------
static void procesarLinea(char *buffer)
{
    char *token = strtok(buffer, ",");
    while (token) {
        char *sep = strchr(token, ':');
        if (sep) {
            *sep = '\0';
            char *clave = token;
            char *valor = sep + 1;
            trim(clave);
            trim(valor);

            // ------ Comparaciones de claves -------
            if (strcasecmp(clave, "Pot") == 0) {
                Pot = atoi(valor);
                printf("[UART] Pot = %d\n", Pot);
            }
            else if (strcasecmp(clave, "Variac") == 0) {
                Variac = atoi(valor);
                printf("[UART] Variac = %d\n", Variac);
            }
            else if (strcasecmp(clave, "Rango") == 0) {
                Rango = atoi(valor);
                printf("[UART] Rango = %d\n", Rango);
            }
            else if (strcasecmp(clave, "FloatVal") == 0) {
                FloatVal = strtof(valor, NULL);
                printf("[UART] FloatVal = %.3f\n", FloatVal);
            }
            else if (strcasecmp(clave, "Modo") == 0) {
                strncpy(Modo, valor, sizeof(Modo)-1);
                Modo[sizeof(Modo)-1] = '\0';
                printf("[UART] Modo = %s\n", Modo);
            }
            else {
                // clave desconocida
                printf("[UART] Ignorado: %s\n", clave);
            }
        }
        token = strtok(NULL, ",");
    }
}

// -------------------------------------------------
// Funciones de acceso para otras partes del sistema
// -------------------------------------------------
int    uart_getPot(void)      { return Pot; }
int    uart_getVariac(void)   { return Variac; }
int    uart_getRango(void)    { return Rango; }
float  uart_getFloatVal(void) { return FloatVal; }
const char* uart_getModo(void){ return Modo; }
