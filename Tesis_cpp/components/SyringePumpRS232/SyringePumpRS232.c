#include "SyringePumpRS232.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>    // ← **añade esta línea**

/* --- Internal utilities --- */

uint8_t Syringe_Address = 0;  // dirección “00” por defecto

/**
 * @brief  Devuelve un puntero a buf+3 si el buff tiene al menos 3 bytes,
 *         o al propio buf en caso contrario.
 */
static char* strip_prefix(char *buf) {
    if (buf && strlen(buf) > 3) {
        return buf + 3;
    }
    return buf;
}


// Packet to turn Safe Mode OFF: [STX][LEN][SAFO][CRC16 hi][CRC16 lo][ETX]
static const uint8_t SAFE_OFF_PKT[] = {
    0x02, 0x08,
    'S','A','F','O',
    0x55, 0x43,
    0x03
};

static esp_err_t syringe_send_raw(const uint8_t *data, size_t len) {
    int w = uart_write_bytes(SYRINGE_UART, (const char*)data, len);
    if (w != (int)len) return ESP_FAIL;
    return uart_wait_tx_done(SYRINGE_UART, pdMS_TO_TICKS(200));
}

static bool syringe_send_cmd(const char *core) {
    char buf[64];
    int len;
    if (Syringe_Address == 0) {
        // un solo comando en Basic Mode, sin prefijo
        len = snprintf(buf, sizeof(buf), "%s\r", core);
    } else {
        // bombas adicionales: dirección + colon
        len = snprintf(buf, sizeof(buf), "%02u%s\r", Syringe_Address, core);
    }
    if (len < 0 || len >= (int)sizeof(buf)) return false;
    if (uart_write_bytes(SYRINGE_UART, buf, len) != len) return false;
    return uart_wait_tx_done(SYRINGE_UART, pdMS_TO_TICKS(200)) == ESP_OK;
}



static int syringe_read_response(char *out, int maxlen, int timeout_ms) {
    uint8_t b;
    int idx = 0;
    TickType_t start = xTaskGetTickCount();

    // Skip until STX or start collecting if no STX arrives
    while (1) {
        if (uart_read_bytes(SYRINGE_UART, &b, 1, pdMS_TO_TICKS(10)) == 1) {
            if (b == 0x02) break;
            // If it's printable ASCII, we may start
            if (b >= 0x20 && b <= 0x7E) {
                out[idx++] = b;
                break;
            }
        }
        if ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS > timeout_ms) {
            return 0;
        }
    }
    // Read until ETX or buffer full
    while (idx < maxlen - 1) {
        if (uart_read_bytes(SYRINGE_UART, &b, 1, pdMS_TO_TICKS(10)) == 1) {
            if (b == 0x03) break;
            out[idx++] = b;
        } else {
            if ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS > timeout_ms) break;
        }
    }
    out[idx] = '\0';
    return idx;
}

/* --- Public API --- */

void Syringe_Init(void) {
    const uart_config_t cfg = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(SYRINGE_UART, SYRINGE_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(SYRINGE_UART, &cfg);
    uart_set_pin(SYRINGE_UART, SYRINGE_TX_PIN, SYRINGE_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void Syringe_SetAddress(uint8_t addr) {
    if (addr <= 99) {
        Syringe_Address = addr;
    }
}


bool Syringe_ExitSafeMode(void) {
    // 1) Paquete binario SAFO
    syringe_send_raw(SAFE_OFF_PKT, sizeof(SAFE_OFF_PKT));
    vTaskDelay(pdMS_TO_TICKS(100));
    // 2) Comando ASCII para guardar Basic Mode
    syringe_send_cmd("SAF 0");
    vTaskDelay(pdMS_TO_TICKS(100));
    return true;
}

bool Syringe_VerifyConnection(void) {
    Syringe_ExitSafeMode();

    // 1) Pregunta versión
    syringe_send_cmd("VER");
    char buf[64];
    int len = syringe_read_response(buf, sizeof(buf), 1000);
    if (len <= 0) {
        printf("⚠️ Sin respuesta a VER\n");
        return false;
    }
    printf("📥 Firmware raw: %s\n", buf);

    // 2) Si vino alarma de Reset, limpiarla
    if (strstr(buf, "A?R")) {
        printf("⚠️ Alarma RESET detectada, limpiando alarma...\n");
        // envía cualquier consulta válida para ack de alarma:
        syringe_send_cmd("DIA");        
        // lee y descarta su respuesta
        syringe_read_response(buf, sizeof(buf), 500);
        printf("   Alarma limpiada, respondiendo: %s\n", buf);
    }

    // 3) Vuelve a preguntar versión ya sin alarma
    syringe_send_cmd("VER");
    len = syringe_read_response(buf, sizeof(buf), 500);
    if (len <= 0) {
        printf("⚠️ Sin respuesta final a VER\n");
        return false;
    }
    printf("📥 Firmware: %s\n", buf);
    return true;
}

bool Syringe_SendCommand(const char *core) {
    return syringe_send_cmd(core);
}

int Syringe_ReadResponse(char *out, int maxlen, int timeout_ms) {
    return syringe_read_response(out, maxlen, timeout_ms);
}

/* --- Faltantes: implementaciones de funciones declaradas en el .h --- */

bool Syringe_SelectPhase(uint8_t phase) {
    char cmd[16];
    // PHN <n>
    snprintf(cmd, sizeof(cmd), "PHN %u", phase);
    return syringe_send_cmd(cmd);
}

bool Syringe_QueryPhase(uint8_t *out_phase) {
    if (!syringe_send_cmd("PHN")) return false;
    char buf[16];
    if (syringe_read_response(buf, sizeof(buf), 200) <= 0) return false;
    char *p = strip_prefix(buf);
    *out_phase = (uint8_t)atoi(p);
    return true;
}

bool Syringe_SetPhaseFunction(const char *func_core) {
    // FUN <func_core>
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "FUN %s", func_core);
    return syringe_send_cmd(cmd);
}

bool Syringe_QueryPhaseFunction(char *out_func, int maxlen) {
    if (!syringe_send_cmd("FUN")) return false;
    char buf[32];
    int len = syringe_read_response(buf, sizeof(buf), 200);
    if (len <= 0) return false;
    char *p = strip_prefix(buf);
    strncpy(out_func, p, maxlen);
    return true;
}

bool Syringe_SetRate(float rate, const char *units) {
    // RAT <rate><units>
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "RAT %.2f%s", rate, units);
    return syringe_send_cmd(cmd);
}

bool Syringe_QueryRate(float *out_rate, char *out_units, int units_len) {
    if (!syringe_send_cmd("RAT")) return false;
    char buf[32];
    if (syringe_read_response(buf, sizeof(buf), 200) <= 0) return false;
    char *p = strip_prefix(buf);
    // p ahora es algo como "5.00UL"
    // separa número y unidades
    char *u = p;
    while (*u && (isdigit((unsigned char)*u) || *u == '.')) u++;
    *u = '\0';
    *out_rate = atof(p);
    strncpy(out_units, u+1, units_len); // u apunta a '\0', u+1 primeras letras de unidad
    return true;
}

/* --- High-level helpers (examples) --- */

bool Syringe_QueryDiameter(float *out_diameter) {
    if (!syringe_send_cmd("DIA")) return false;
    char buf[32];
    int len = syringe_read_response(buf, sizeof(buf), 500);
    if (len <= 0) {
        printf("⚠️ DIA: sin respuesta\n");
        return false;
    }
    // DEBUG: imprime en ASCII y hex
    printf("🛠️ DIA raw: '%s'  [", buf);
    for (int i = 0; i < len; ++i) printf("%02X ", (uint8_t)buf[i]);
    printf("]\n");
    char *p = strip_prefix(buf);
    *out_diameter = atof(p);
    return true;
}


bool Syringe_SetDiameter(float diameter) {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "DIA %.1f", diameter);
    return syringe_send_cmd(cmd);
}

bool Syringe_Run(uint8_t start_phase) {
    if (start_phase == 0) {
        return syringe_send_cmd("RUN");
    } else {
        char cmd[16];
        snprintf(cmd, sizeof(cmd), "RUN %d", start_phase);
        return syringe_send_cmd(cmd);
    }
}

bool Syringe_Stop(void) {
    return syringe_send_cmd("STP");
}

/**
 * @brief Cambia la dirección de bombeo.
 * @param dir  "INF", "WDR" o "REV"
 */
bool Syringe_SetDirection(const char *dir) {
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "DIR %s", dir);
    return syringe_send_cmd(cmd);
}

/**
 * @brief Consulta la dirección actual de bombeo.
 * @param out_dir  Buffer para recibir "INF" o "WDR"
 * @param maxlen   Tamaño de out_dir
 * @return true si se leyó correctamente
 */

bool Syringe_QueryDirection(char *out_dir, int maxlen) {
    if (!syringe_send_cmd("DIR")) return false;
    char buf[16];
    int len = syringe_read_response(buf, sizeof(buf), 200);
    if (len <= 0) return false;
    char *p = strip_prefix(buf);
    strncpy(out_dir, p, maxlen);
    return true;
}

bool Syringe_QueryDispensed(float *out_infuse, float *out_withdraw, char *out_units, int units_len) {
    if (!syringe_send_cmd("DIS")) return false;
    char buf[64];
    if (syringe_read_response(buf, sizeof(buf), 500) <= 0) return false;
    // buf = "00SI 1.00W 0.00UL"
    char *p = strip_prefix(buf);
    // ahora p = "I 1.00 W 0.00 UL"
    if (sscanf(p, "I %f W %f %7s", out_infuse, out_withdraw, out_units) < 3) {
        return false;
    }
    return true;
}

bool Syringe_ClearDispensed(const char *which) {
    // which: "INF" or "WDR"
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "CLD %s", which);
    return syringe_send_cmd(cmd);
}

/* Añade aquí más funciones específicas: PHN, FUN, RAT, VOL, DIR, etc. */

