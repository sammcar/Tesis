#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"  
#include "driver/uart.h"

#define NUM_ETAPAS 10
#define NUM_SENSORES 6
#define NUM_SALIDAS 8
#define BUF_SIZE 128
#define TIMEOUT_MANUAL 10

typedef enum { MODO_AUTOMATICO, MODO_CICLO, MODO_ETAPA, MODO_MANUAL } Modo;

const char* SENSORES[NUM_SENSORES] = {"p1e", "p1r", "p2e", "p2r", "p3e", "p3r"};
const char* SALIDAS[NUM_SALIDAS] = {"P1EXT", "P1RET", "P2EXT", "P2RET", "P3EXT", "P3RET", "5 SEG", "3SEG"};

const char* ETAPA_SALIDAS_MAP[NUM_ETAPAS][4] = {
    {NULL},
    {"P1EXT", NULL},
    {"P1RET", NULL},
    {"P2EXT", NULL},
    {"P2RET", NULL},
    {"P1EXT", "P2EXT", NULL},
    {"timer_5", NULL},
    {"P1RET", "P2RET", NULL},
    {"P3EXT", "timer_3", NULL},
    {"P3RET", NULL}
};

typedef struct {
    const char* nombre;
    int duracion;
    time_t inicio;
    bool terminado;
} Timer;

Timer timers[] = {
    {"timer_5", 5, 0, false},
    {"timer_3", 3, 0, false}
};

int TIEMPO_FALLO_SENSOR[NUM_SENSORES] = {10,10,10,10,6,6};

bool etapas[NUM_ETAPAS] = {true};
bool sensores[NUM_SENSORES] = {false};
bool latch[NUM_SENSORES] = {false};
bool salidas[NUM_SALIDAS] = {false};
bool pausa = false;
bool fallo = false;
bool start = false, clc = false, se = false;
int etapa_actual = 0;
Modo modo = MODO_AUTOMATICO;
time_t tiempo_fallo = 0;
int etapa_fallo = -1;

// Modo manual
const char* salida_manual = NULL;
time_t inicio_manual = 0;

void prender_led_verde() {
    printf("[FUNC] LED VERDE encendido\n");
}

void (*SALIDA_FUNCIONES[NUM_SALIDAS])() = {
    prender_led_verde, NULL, NULL, NULL, NULL, NULL, NULL, NULL
};

int idx_salida(const char* nombre) {
    for (int i = 0; i < NUM_SALIDAS; i++) if (strcmp(SALIDAS[i], nombre) == 0) return i;
    return -1;
}

int idx_sensor(const char* nombre) {
    for (int i = 0; i < NUM_SENSORES; i++) if (strcmp(SENSORES[i], nombre) == 0) return i;
    return -1;
}

void activar_salida(const char* salida) {
    int idx = idx_salida(salida);
    if (idx >= 0) {
        salidas[idx] = true;
        printf("⚙️ Activando salida: %s\n", salida);
        if (SALIDA_FUNCIONES[idx]) SALIDA_FUNCIONES[idx]();
    }
}

void evaluar_timer() {
    const char** salidas_etapa = ETAPA_SALIDAS_MAP[etapa_actual];
    for (int t = 0; t < sizeof(timers)/sizeof(timers[0]); t++) {
        bool encontrada = false;
        for (int i = 0; salidas_etapa[i]; i++)
            if (strcmp(salidas_etapa[i], timers[t].nombre) == 0) encontrada = true;
        if (encontrada) {
            if (!timers[t].inicio) timers[t].inicio = time(NULL);
            if (difftime(time(NULL), timers[t].inicio) >= timers[t].duracion) timers[t].terminado = true;
        } else {
            timers[t].inicio = 0;
            timers[t].terminado = false;
        }
    }
}

void verificar_fallos() {
    if (etapa_fallo != etapa_actual) {
        etapa_fallo = etapa_actual;
        tiempo_fallo = time(NULL);
    }
    int sensores_por_etapa[NUM_ETAPAS][3] = {
        {},{0},{1},{2},{3},{0,2},{},{1,3},{4},{5}
    };
    for (int i = 0; i < 3; i++) {
        int idx = sensores_por_etapa[etapa_actual][i];
        if (idx >= 0 && idx < NUM_SENSORES && !latch[idx]) {
            if (difftime(time(NULL), tiempo_fallo) > TIEMPO_FALLO_SENSOR[idx]) {
                printf("❌ Falla detectada en sensor: %s\n", SENSORES[idx]);
                fallo = true;
            }
        }
    }
}

void modo_manual_func() {
    for (int i = 0; i < NUM_SALIDAS; i++) salidas[i] = false;
    if (salida_manual) {
        if (strcmp(salida_manual, "RFALLA_MANUAL") == 0) {
            int idx1 = idx_salida("P1RET");
            int idx2 = idx_salida("P2RET");
            int idx3 = idx_salida("P3RET");
            salidas[idx1] = salidas[idx2] = salidas[idx3] = true;
            if (inicio_manual == 0) {
                inicio_manual = time(NULL);
                printf("⚙️ [Manual] Activando P1RET, P2RET y P3RET para prueba de falla.\n");
            }
            if (latch[1] && latch[3] && latch[5]) {
                salida_manual = NULL;
                inicio_manual = 0;
            }
        } else {
            int idx = idx_salida(salida_manual);
            if (idx >= 0) {
                salidas[idx] = true;
                if (inicio_manual == 0) {
                    inicio_manual = time(NULL);
                    printf("⚙️ [Manual] Activando salida: %s\n", salida_manual);
                }
                if (latch[idx]) {
                    salida_manual = NULL;
                    inicio_manual = 0;
                } else if (difftime(time(NULL), inicio_manual) > TIMEOUT_MANUAL) {
                    printf("❌ Falla en modo manual: sensor no activado a tiempo.\n");
                    fallo = true;
                }
            }
        }
    }
}

void graficar_etapa() {
    static int anterior = -1;
    if (etapa_actual != anterior) {
        printf("\n🔁 Etapa activa: %d\n", etapa_actual);
        anterior = etapa_actual;
        for (int i = 0; i < NUM_SALIDAS; i++) salidas[i] = false;
        for (int i = 0; ETAPA_SALIDAS_MAP[etapa_actual][i]; i++)
            activar_salida(ETAPA_SALIDAS_MAP[etapa_actual][i]);
    }
}

void evo() {
    bool s[NUM_SENSORES];
    memcpy(s, latch, sizeof(s));
    bool auto_m = (modo == MODO_AUTOMATICO);
    bool ciclo_m = (modo == MODO_CICLO);
    bool etapa_m = (modo == MODO_ETAPA);

    struct { int actual; bool cond; int siguiente; } trans[] = {
        {0, (auto_m && start) || (ciclo_m && clc) || (etapa_m && se), 1},
        {1, (auto_m || ciclo_m || (etapa_m && se)) && s[0], 2},
        {2, (auto_m || ciclo_m || (etapa_m && se)) && s[1], 3},
        {3, (auto_m || ciclo_m || (etapa_m && se)) && s[2], 4},
        {4, ((auto_m || (etapa_m && se)) && s[3]) || (ciclo_m && s[3] && clc), 5},
        {5, (auto_m || ciclo_m || (etapa_m && se)) && s[0] && s[2], 6},
        {6, (auto_m || ciclo_m || (etapa_m && se)) && timers[0].terminado, 7},
        {7, ((auto_m || (etapa_m && se)) && s[1] && s[3]) || (ciclo_m && clc && s[1] && s[3]), 8},
        {8, (auto_m || ciclo_m || (etapa_m && se)) && timers[1].terminado && s[4], 9},
        {9, (auto_m || ciclo_m || (etapa_m && se)) && s[5], 0}
    };

    for (int i = 0; i < 10; i++) {
        if (etapas[trans[i].actual] && trans[i].cond) {
            memset(etapas, 0, sizeof(etapas));
            etapas[trans[i].siguiente] = true;
            etapa_actual = trans[i].siguiente;
            break;
        }
    }

    start = clc = se = false;
}

void grafcet_task(void* arg) {
    printf("⚙️ Simulador GRAFCET listo.\n");
    printf("\n🔁 Etapa activa: %d\n", etapa_actual);
    printf("Comandos: start, clc, se, modo, paro, ok, reset, rfalla, sensores, salidas...\n");
    while (1) {
        if (!pausa && !fallo) {
            if (modo == MODO_MANUAL) modo_manual_func();
            else {
                evaluar_timer();
                evo();
                verificar_fallos();
                graficar_etapa();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void comando_task(void* arg) {
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
    char buf[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_NUM_0, (uint8_t*)buf, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            buf[len] = '\0';
            if (strstr(buf, "start")) start = true;
            else if (strstr(buf, "clc")) clc = true;
            else if (strstr(buf, "se")) se = true;
            else if (strstr(buf, "paro")) { pausa = true; printf("🟥 PARO DE EMERGENCIA activado.\n"); }
            else if (strstr(buf, "ok")) { pausa = false; printf("▶️ Proceso reanudado.\n"); }
            else if (strstr(buf, "reset")) {
                printf("🔄 Sistema reiniciado.\n");
                esp_restart();
            }
            else if (strstr(buf, "modo")) {
                if (strstr(buf, "automatico")) modo = MODO_AUTOMATICO;
                else if (strstr(buf, "ciclo")) modo = MODO_CICLO;
                else if (strstr(buf, "etapa")) modo = MODO_ETAPA;
                else if (strstr(buf, "manual")) modo = MODO_MANUAL;
                start = clc = se = false;
                salida_manual = NULL;
                inicio_manual = 0;
                printf("✅ Modo cambiado a %s\n", 
                    modo == MODO_AUTOMATICO ? "automatico" :
                    modo == MODO_CICLO ? "ciclo" :
                    modo == MODO_ETAPA ? "etapa" : "manual");
                printf("\nComandos: start, clc, se, modo, paro, ok, reset, rfalla, sensores, salidas...\nComando:\n");
            }
            else if (strstr(buf, "rfalla")) {
                if (modo == MODO_MANUAL) {
                    salida_manual = "RFALLA_MANUAL";
                    inicio_manual = 0;
                } else {
                    printf("✅ Falla reseteada.\n");
                    esp_restart();
                }
            } else {
                for (int i = 0; i < NUM_SENSORES; i++) {
                    if (strstr(buf, SENSORES[i]) && salidas[i]) {
                        latch[i] = true;
                    }
                }
                for (int i = 0; i < NUM_SALIDAS; i++) {
                    if (modo == MODO_MANUAL && strstr(buf, SALIDAS[i])) {
                        salida_manual = SALIDAS[i];
                        inicio_manual = 0;
                        printf("⚙️ [Manual] Activando salida: %s\n", SALIDAS[i]);
                    }
                }
            }
        }
    }
}

void app_main() {
    xTaskCreate(grafcet_task, "grafcet", 4096, NULL, 5, NULL);
    xTaskCreate(comando_task, "comando", 4096, NULL, 5, NULL);
}
