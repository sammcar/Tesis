#include "VariacRS485.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>
#include <stdio.h>

#define VARIAC_UART      UART_NUM_1
#define VARIAC_TXD       4
#define VARIAC_RXD       5
#define VARIAC_DE_RE     33  // Control del transceiver RS485
#define VARIAC_BUF_SIZE  128

void Variac_Init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,  // Según manual WEG
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(VARIAC_UART, VARIAC_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(VARIAC_UART, &uart_config);
    uart_set_pin(VARIAC_UART, VARIAC_TXD, VARIAC_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << VARIAC_DE_RE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    gpio_set_level(VARIAC_DE_RE, 0);  // Iniciar en modo recepción
}

static bool variac_send_command(const uint8_t *cmd, size_t len) {
    gpio_set_level(VARIAC_DE_RE, 1);  // Activar transmisión
    printf("🟢 Transmitiendo RS485...\n");

    //esp_rom_delay_us(50);  // Breve espera opcional

    if (uart_write_bytes(VARIAC_UART, (const char *)cmd, len) != len) {
        gpio_set_level(VARIAC_DE_RE, 0);  // Volver a recepción
        return false;
    }

    uart_wait_tx_done(VARIAC_UART, pdMS_TO_TICKS(100));
    gpio_set_level(VARIAC_DE_RE, 0);  // Volver a recepción
    return true;
}

bool Variac_VerificarConexion(void) {
    const uint8_t ping_cmd[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
    uint8_t resp[32] = {0};

    printf("📤 Enviando comando Variac: ");
    for (int i = 0; i < sizeof(ping_cmd); i++) {
        printf("%02X ", ping_cmd[i]);
    }
    printf("\n");

    if (!variac_send_command(ping_cmd, sizeof(ping_cmd))) {
        printf("❌ Fallo al enviar comando al Variac.\n");
        return false;
    }

    int len = uart_read_bytes(VARIAC_UART, resp, sizeof(resp), pdMS_TO_TICKS(300));
    if (len <= 0) {
        printf("⚠️ Variac: Sin respuesta\n");
        return false;
    }

    printf("📥 Variac respondió (%d bytes): ", len);
    for (int i = 0; i < len; i++) {
        printf("%02X ", resp[i]);
    }
    printf("\n");

    return true;
}



bool Variac_Encender(void) {
    const uint8_t cmd[] = {0x01, 0x06, 0x00, 0x02, 0x00, 0x01, 0x48, 0x0B};  // Escribir 0x0001 a registro 0x0002
    return variac_send_command(cmd, sizeof(cmd));
}

bool Variac_Apagar(void) {
    const uint8_t cmd[] = {0x01, 0x06, 0x00, 0x02, 0x00, 0x00, 0x89, 0xCB};  // Escribir 0x0000 a registro 0x0002
    return variac_send_command(cmd, sizeof(cmd));
}

bool Variac_EstablecerVoltaje(float voltaje) {
    // TODO: Implementar conversión según fórmula y protocolo WEG (ej: 0–100% Vout)
    return false;
}

float Variac_LeerVoltaje(void) {
    // TODO: Implementar decodificación del registro de voltaje de salida
    return -1.0;
}
