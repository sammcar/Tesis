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


// Reemplaza Syringe_ExitSafeMode por esta versión con reintentos mínimos
bool Syringe_ExitSafeMode(void) {
    const int RETRIES = 5;
    for (int r = 0; r < RETRIES; ++r) {
        // limpiar RX antes de intentar
        uart_flush_input(SYRINGE_UART);

        // 1) Paquete binario SAFO
        esp_err_t es = syringe_send_raw(SAFE_OFF_PKT, sizeof(SAFE_OFF_PKT));
        if (es != ESP_OK) {
            printf("⚠️ SAFO write failed (attempt %d)\n", r+1);
        } else {
            vTaskDelay(pdMS_TO_TICKS(150)); // dejar tiempo a la bomba
        }

        // 2) Comando ASCII para persistir Basic Mode
        syringe_send_cmd("SAF 0");
        vTaskDelay(pdMS_TO_TICKS(200));

        // lee lo que la bomba devuelva por un breve periodo para ver si respondio
        char dbg[64] = {0};
        int n = syringe_read_response(dbg, sizeof(dbg), 300);
        if (n > 0) {
            // imprimimos la posible respuesta cruda (útil en debug)
            printf("ℹ️ Resp tras SAFO/SAF0: '%s' (len=%d)\n", dbg, n);
            return true; // aceptamos que SAFO pudo tener efecto
        }

        // si no hubo respuesta, reintentamos
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    // último intento: enviar una vez más pero continuar
    uart_flush_input(SYRINGE_UART);
    syringe_send_raw(SAFE_OFF_PKT, sizeof(SAFE_OFF_PKT));
    vTaskDelay(pdMS_TO_TICKS(150));
    syringe_send_cmd("SAF 0");
    vTaskDelay(pdMS_TO_TICKS(200));
    uart_flush_input(SYRINGE_UART);
    return true; // devolvemos true porque el envío fue ejecutado; la verificación real la hará Syringe_VerifyConnection
}

/*
 * Syringe_VerifyConnection: asegura salida de Safe Mode y obtiene VER (firmware).
 * - Reintenta SAFO/SAF0 hasta MAX_ATTEMPTS.
 * - Limpia A?R con PF 0 si aparece.
 * - Imprime diagnóstico y copia firmware en pantalla.
 */
bool Syringe_VerifyConnection(void) {
    const int MAX_ATTEMPTS = 10;
    const int VER_TOTAL_TIMEOUT_MS = 1500; // tiempo total por intento para esperar VER
    const int READ_WINDOW_MS = 250;        // ventana de lectura en cada iteración

    char rxbuf[128];
    char fwbuf[64];

    for (int attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
        printf("🔁 VerifyConnection: intento %d/%d\n", attempt+1, MAX_ATTEMPTS);

        // 1) Intentar salir de safe mode (envía SAFO + SAF 0)
        uart_flush_input(SYRINGE_UART);
        syringe_send_raw(SAFE_OFF_PKT, sizeof(SAFE_OFF_PKT));
        vTaskDelay(pdMS_TO_TICKS(150));
        syringe_send_cmd("SAF 0");
        vTaskDelay(pdMS_TO_TICKS(200));
        uart_flush_input(SYRINGE_UART);

        // 2) Preguntar VER y esperar respuesta (ventanas pequeñas)
        syringe_send_cmd("VER");

        int elapsed = 0;
        bool got_fw = false;
        TickType_t t0 = xTaskGetTickCount();
        while (elapsed < VER_TOTAL_TIMEOUT_MS) {
            int n = syringe_read_response(rxbuf, sizeof(rxbuf), READ_WINDOW_MS);
            if (n > 0) {
                // recorte básico: quitar CR/LF de final
                int len = n;
                while (len > 0 && (rxbuf[len-1] == '\r' || rxbuf[len-1] == '\n')) rxbuf[--len] = '\0';
                rxbuf[len] = '\0';

                // Si viene la alarma A?R -> limpiar con PF 0 y reintentar VER
                if (strstr(rxbuf, "A?R")) {
                    printf("⚠️ Alarma A?R detectada en respuesta: '%s'\n", rxbuf);
                    // mandar PF 0 para ack y limpiar la alarma
                    syringe_send_cmd("PF 0");
                    vTaskDelay(pdMS_TO_TICKS(150));
                    // leer/descartar posible ACK
                    char dump[64] = {0};
                    syringe_read_response(dump, sizeof(dump), 250);
                    // reenvía VER y continúa esperando
                    syringe_send_cmd("VER");
                    // reiniciar ventana de espera sin consumir intento completo
                    t0 = xTaskGetTickCount();
                    elapsed = 0;
                    continue;
                }

                // Normalizar posible prefijo (ej: '00S' o similar)
                char *p = strip_prefix(rxbuf);
                // quitar espacios al inicio
                while (*p && (unsigned char)*p <= ' ') ++p;
                // quitar CR/LF al final otra vez por si acaso
                int plen = strlen(p);
                while (plen > 0 && (p[plen-1] == '\r' || p[plen-1] == '\n')) p[--plen] = '\0';

                if (plen > 0) {
                    // Suponemos que esto es el firmware
                    strncpy(fwbuf, p, sizeof(fwbuf) - 1);
                    fwbuf[sizeof(fwbuf) - 1] = '\0';
                    printf("📥 Firmware: %s\n", fwbuf);
                    got_fw = true;
                    break;
                } else {
                    // si el buffer quedó vacío tras strip, continuar leyendo
                    printf("ℹ️ Respuesta recibida pero vacía tras strip: '%s'\n", rxbuf);
                }
            }
            // actualizar tiempo transcurrido
            elapsed = (int)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
        } // while espera VER

        if (got_fw) {
            // Éxito: asegúrate de limpiar RX y devolver true
            uart_flush_input(SYRINGE_UART);
            return true;
        }

        // si llegamos aquí no obtuvimos firmware en este intento
        printf("⚠️ No se obtuvo VER en intento %d. Reintentando...\n", attempt+1);
        vTaskDelay(pdMS_TO_TICKS(200));
    } // attempts

    // todos los intentos fallaron
    printf("❌ Syringe_VerifyConnection: no se pudo obtener VER tras %d intentos\n", MAX_ATTEMPTS);
    return false;
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
static void trim_ascii_inline(char* s){
    if(!s) return;
    size_t n = strlen(s);
    while(n>0 && (s[n-1]=='\r'||s[n-1]=='\n'||s[n-1]==' '||s[n-1]=='\t')) s[--n] = '\0';
    char* p = s; while(*p==' '||*p=='\t') p++;
    if(p!=s) memmove(s,p,strlen(p)+1);
}
static char* strip_prefix3_inline(char* s){
    if(!s) return s;
    if(strlen(s)>=3 && isprint((unsigned char)s[0]) && isprint((unsigned char)s[1]) && isprint((unsigned char)s[2])){
        return s+3;
    }
    return s;
}

bool Syringe_TestVER(char *out_fw, int fw_len, char *out_diag, int diag_len, int timeout_ms) {
    if (out_fw)  out_fw[0]  = '\0';
    if (out_diag) out_diag[0] = '\0';
    char diag[256]; diag[0]='\0';
    #define APPEND(msg) do{ if((int)strlen(diag)<(int)sizeof(diag)-4){ strncat(diag, msg, sizeof(diag)-1); strncat(diag, ";", sizeof(diag)-1);} }while(0)

    // 1) Salir de Safe Mode con margen y limpiar RX
    APPEND("flush_rx");
    uart_flush_input(SYRINGE_UART);
    APPEND("send_SAFO");
    syringe_send_raw(SAFE_OFF_PKT, sizeof(SAFE_OFF_PKT));
    vTaskDelay(pdMS_TO_TICKS(150));
    APPEND("send_SAF0");
    syringe_send_cmd("SAF 0");
    vTaskDelay(pdMS_TO_TICKS(250));
    APPEND("flush_rx2");
    uart_flush_input(SYRINGE_UART);

    // 2) Pedir VER y leer con timeout total (robusto)
    APPEND("send_VER");
    syringe_send_cmd("VER");
    TickType_t t0 = xTaskGetTickCount();
    char buf[96] = {0};
    bool got = false;
    while (!got) {
        int n = syringe_read_response(buf, sizeof(buf), 200); // ventanas de 200 ms
        if (n > 0) {
            trim_ascii_inline(buf);
            // ¿Alarma de reset?
            if (strstr(buf, "A?R")) {
                APPEND("A?R_detected");
                // → usa PF 0 para limpiar reset (más efectivo que DIA en algunos modelos)
                syringe_send_cmd("PF 0");
                vTaskDelay(pdMS_TO_TICKS(150));
                char dump[64]; syringe_read_response(dump, sizeof(dump), 300);
                uart_flush_input(SYRINGE_UART);
                APPEND("retry_VER");
                syringe_send_cmd("VER");
                continue;
}
            // Normalizar y devolver firmware
            char *p = strip_prefix3_inline(buf);
            trim_ascii_inline(p);
            if (out_fw && p && *p) {
                strncpy(out_fw, p, fw_len);
                out_fw[fw_len-1] = '\0';
            }
            APPEND("VER_ok");
            got = true;
            break;
        }
        // timeout total
        int elapsed = (int)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
        if (elapsed > (timeout_ms>0?timeout_ms:800)) break;
    }

    if (out_diag){
        strncpy(out_diag, diag, diag_len);
        out_diag[diag_len-1] = '\0';
    }
    return got && out_fw && out_fw[0]!='\0';
}

