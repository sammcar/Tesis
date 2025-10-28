#include "LaserSensor.h"
#include "filters.h"

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us
#include "esp_check.h"     // opcional, si usas ESP_RETURN*


/* =================== LOG =================== */
static const char* TAG = "VL53L0X";

/* =================== REGISTROS VL53L0X (mínimos) =================== */
#define REG_SYSRANGE_START          0x00
#define REG_SYSTEM_INTERRUPT_CLEAR  0x0B
#define REG_RESULT_INTERRUPT_STATUS 0x13
#define REG_RESULT_RANGE_STATUS     0x14
// Distancia mm: 16 bits en (RESULT_RANGE_STATUS + 10) = 0x1E
#define REG_RESULT_RANGE_MM_MSB     (REG_RESULT_RANGE_STATUS + 10)  // 0x1E

/* =================== PARÁMETROS FILTRO/TAREA =================== */
#define LASER_TS_MS_DEFAULT      50U
#define LASER_EMA_ALPHA_DEFAULT  0.20f
#define LASER_FAILS_BEFORE_WARN  8

/* =================== ESTADO CAPA FILTRADA =================== */
static VL53L0X_Handle_t* s_h = NULL;
static TaskHandle_t       s_task = NULL;
static SemaphoreHandle_t  s_mutex = NULL;

static filt_med5_t s_med5;
static filt_ema_t  s_ema;

static volatile uint32_t s_ts_ms = LASER_TS_MS_DEFAULT;
static volatile float    s_alpha = LASER_EMA_ALPHA_DEFAULT;
static float             s_last_alpha = -1.0f;

static float    s_last_mm_f = 0.0f;  // filtrado
static float    s_last_mm   = 0.0f;  // crudo
static uint16_t s_last_raw  = 0;     // auxiliar (igual a mm truncado)
static bool     s_ready     = false;

/* =================== I2C UTILS =================== */
static void i2c_bus_recover(gpio_num_t sda, gpio_num_t scl) {
  gpio_config_t io = {
    .pin_bit_mask = (1ULL<<sda) | (1ULL<<scl),
    .mode = GPIO_MODE_OUTPUT_OD,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&io);
  gpio_set_level(sda, 1);
  for (int i=0;i<9;i++){ gpio_set_level(scl,1); esp_rom_delay_us(5); gpio_set_level(scl,0); esp_rom_delay_us(5); }
  // STOP
  gpio_set_level(sda,0); esp_rom_delay_us(5);
  gpio_set_level(scl,1); esp_rom_delay_us(5);
  gpio_set_level(sda,1); esp_rom_delay_us(5);
}

static esp_err_t write_reg8(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, uint8_t val, TickType_t to_ticks)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(i2c_num, addr, buf, 2, to_ticks);
}

static esp_err_t read_reg8(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, uint8_t* val, TickType_t to_ticks)
{
    esp_err_t err = i2c_master_write_to_device(i2c_num, addr, &reg, 1, to_ticks);
    if (err != ESP_OK) return err;
    return i2c_master_read_from_device(i2c_num, addr, val, 1, to_ticks);
}

static esp_err_t read_reg16_be(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, uint16_t* out, TickType_t to_ticks)
{
    esp_err_t err = i2c_master_write_to_device(i2c_num, addr, &reg, 1, to_ticks);
    if (err != ESP_OK) return err;
    uint8_t buf[2] = {0};
    err = i2c_master_read_from_device(i2c_num, addr, buf, 2, to_ticks);
    if (err != ESP_OK) return err;
    *out = ((uint16_t)buf[0] << 8) | buf[1];  // big-endian
    return ESP_OK;
}

static esp_err_t i2c_probe_addr(i2c_port_t i2c_num, uint8_t addr, TickType_t to_ticks)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err = i2c_master_start(cmd);
    if (err == ESP_OK) err = i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true); // expect ACK
    if (err == ESP_OK) err = i2c_master_stop(cmd);
    if (err == ESP_OK) err = i2c_master_cmd_begin(i2c_num, cmd, to_ticks);
    i2c_cmd_link_delete(cmd);
    return err; // ESP_OK => presente
}

/* =================== DRIVER VL53L0X (capa cruda) =================== */
bool VL53L0X_Reset(VL53L0X_Handle_t* h, char* msg, size_t n)
{
    if (!h) return false;

    if (h->pin_xshut == GPIO_NUM_NC) {
        if (msg && n) snprintf(msg, n, "XSHUT no conectado; se omite reset.");
        return true; // no es error, solo se omite
    }

    // XSHUT activo en bajo
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << h->pin_xshut),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    // Apagar
    gpio_set_level(h->pin_xshut, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    // Encender
    gpio_set_level(h->pin_xshut, 1);
    vTaskDelay(pdMS_TO_TICKS(5));

    if (msg && n) snprintf(msg, n, "XSHUT toggled correctamente.");
    return true;
}

bool VL53L0X_Init(VL53L0X_Handle_t* h, char* msg, size_t n)
{
  if (!h) return false;

  // Defaults
  if (h->i2c_clk_hz == 0) h->i2c_clk_hz = 50000; // 50 kHz, más tolerante
  if (h->i2c_num != I2C_NUM_0 && h->i2c_num != I2C_NUM_1) h->i2c_num = I2C_NUM_0;
  if (h->pin_sda == GPIO_NUM_NC) h->pin_sda = GPIO_NUM_21;
  if (h->pin_scl == GPIO_NUM_NC) h->pin_scl = GPIO_NUM_22;

  // XSHUT opcional
  VL53L0X_Reset(h, NULL, 0);

  // Si SCL está LOW, intenta recuperar bus
  gpio_set_direction(h->pin_sda, GPIO_MODE_INPUT);
  gpio_set_direction(h->pin_scl, GPIO_MODE_INPUT);
  gpio_pullup_en(h->pin_sda);
  gpio_pullup_en(h->pin_scl);
  esp_rom_delay_us(50);
  if (gpio_get_level(h->pin_scl) == 0) {
    i2c_bus_recover(h->pin_sda, h->pin_scl);
    esp_rom_delay_us(50);
  }
  // Devolver a INPUT+PU antes de configurar I2C
  gpio_set_direction(h->pin_sda, GPIO_MODE_INPUT);
  gpio_set_direction(h->pin_scl, GPIO_MODE_INPUT);
  gpio_pullup_en(h->pin_sda);
  gpio_pullup_en(h->pin_scl);

  // Config + (re)instalación limpia del driver
  i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = h->pin_sda,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = h->pin_scl,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = h->i2c_clk_hz,
    .clk_flags = 0
  };

  esp_err_t err;  // <--- SOLO UNA DECLARACIÓN

  err = i2c_param_config(h->i2c_num, &conf);
  if (err != ESP_OK) {
    if (msg&&n) snprintf(msg,n,"i2c_param_config: %s", esp_err_to_name(err));
    return false;
  }

  // (Re)instala el driver (ignora error al borrar si no estaba)
  (void)i2c_driver_delete(h->i2c_num);
  vTaskDelay(pdMS_TO_TICKS(2));
  err = i2c_driver_install(h->i2c_num, I2C_MODE_MASTER, 0, 0, 0);
  if (err == ESP_ERR_INVALID_STATE) {
    (void)i2c_driver_delete(h->i2c_num);
    vTaskDelay(pdMS_TO_TICKS(2));
    err = i2c_driver_install(h->i2c_num, I2C_MODE_MASTER, 0, 0, 0);
  }
  if (err != ESP_OK) {
    if (msg&&n) snprintf(msg,n,"i2c_driver_install: %s", esp_err_to_name(err));
    return false;
  }

  // Timeout grande por posibles clock-stretches
  i2c_set_timeout(h->i2c_num, 0xFFFFF);

  // Espera tras XSHUT/power-up
  vTaskDelay(pdMS_TO_TICKS(25));

  // Probe robusto (tres intentos, start+write+stop)
  for (int k=0;k<3;k++) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_I2C_ADDR<<1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(h->i2c_num, cmd, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK) {
      h->inited = true;
      if (msg&&n) snprintf(msg,n,"VL53L0X ok en 0x29 @%lu Hz",(unsigned long)h->i2c_clk_hz);
      ESP_LOGI(TAG, "VL53L0X detectado (0x29) @%lu Hz", (unsigned long)h->i2c_clk_hz);
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(30));
  }

  if (msg && n) snprintf(msg, n, "VL53L0X (0x29) NO responde: ESP_ERR_TIMEOUT");
  ESP_LOGW(TAG, "Sensor no encontrado en 0x29: ESP_ERR_TIMEOUT");
  return false;
}

bool VL53L0X_ReadDistanceMM(VL53L0X_Handle_t* h, uint16_t* mm, char* msg, size_t n)
{
    if (!h || !h->inited) { if (msg && n) snprintf(msg, n, "Sensor no inicializado."); return false; }
    esp_err_t err;
    uint8_t istat = 0;
    uint16_t dist = 0;

    // Limpia posibles flags previos
    (void)write_reg8(h->i2c_num, VL53L0X_I2C_ADDR, REG_SYSTEM_INTERRUPT_CLEAR, 0x01, pdMS_TO_TICKS(20));

    // -------- INTENTO A: single-shot --------
    err = write_reg8(h->i2c_num, VL53L0X_I2C_ADDR, REG_SYSRANGE_START, 0x01, pdMS_TO_TICKS(50)); // single
    if (err == ESP_OK){
        TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(200);  // 200 ms
        do {
            err = read_reg8(h->i2c_num, VL53L0X_I2C_ADDR, REG_RESULT_INTERRUPT_STATUS, &istat, pdMS_TO_TICKS(20));
            if (err != ESP_OK) break;
            if ((istat & 0x07) != 0) { // data ready
                if (read_reg16_be(h->i2c_num, VL53L0X_I2C_ADDR, REG_RESULT_RANGE_MM_MSB, &dist, pdMS_TO_TICKS(20)) == ESP_OK){
                    (void)write_reg8(h->i2c_num, VL53L0X_I2C_ADDR, REG_SYSTEM_INTERRUPT_CLEAR, 0x01, pdMS_TO_TICKS(20));
                    if (mm) *mm = dist;
                    if (msg && n) snprintf(msg, n, "OK(single): %u mm", dist);
                    return true;
                }
                break; // si falló la lectura, salimos a intento B
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        } while (xTaskGetTickCount() < deadline);
    }

    // -------- INTENTO B: continuous back-to-back --------
    (void)write_reg8(h->i2c_num, VL53L0X_I2C_ADDR, REG_SYSTEM_INTERRUPT_CLEAR, 0x01, pdMS_TO_TICKS(20));
    err = write_reg8(h->i2c_num, VL53L0X_I2C_ADDR, REG_SYSRANGE_START, 0x02, pdMS_TO_TICKS(50)); // continuous back-to-back
    if (err == ESP_OK){
        TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);  // hasta 0.5 s la primera vez
        do {
            err = read_reg8(h->i2c_num, VL53L0X_I2C_ADDR, REG_RESULT_INTERRUPT_STATUS, &istat, pdMS_TO_TICKS(20));
            if (err != ESP_OK) break;
            if ((istat & 0x07) != 0) {
                if (read_reg16_be(h->i2c_num, VL53L0X_I2C_ADDR, REG_RESULT_RANGE_MM_MSB, &dist, pdMS_TO_TICKS(20)) == ESP_OK){
                    (void)write_reg8(h->i2c_num, VL53L0X_I2C_ADDR, REG_SYSTEM_INTERRUPT_CLEAR, 0x01, pdMS_TO_TICKS(20));
                    if (mm) *mm = dist;
                    if (msg && n) snprintf(msg, n, "OK(continuous): %u mm", dist);
                    return true;
                }
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        } while (xTaskGetTickCount() < deadline);

        // Paramos el continuo si no funcionó
        (void)write_reg8(h->i2c_num, VL53L0X_I2C_ADDR, REG_SYSRANGE_START, 0x00, pdMS_TO_TICKS(20));
    }

    // -------- INTENTO C: fallback por delay fijo --------
    vTaskDelay(pdMS_TO_TICKS(50)); // presupuesto típico ~33 ms
    if (read_reg16_be(h->i2c_num, VL53L0X_I2C_ADDR, REG_RESULT_RANGE_MM_MSB, &dist, pdMS_TO_TICKS(20)) == ESP_OK){
        (void)write_reg8(h->i2c_num, VL53L0X_I2C_ADDR, REG_SYSTEM_INTERRUPT_CLEAR, 0x01, pdMS_TO_TICKS(20));
        if (mm) *mm = dist;
        if (msg && n) snprintf(msg, n, "OK(fallback): %u mm", dist);
        return true;
    }

    if (msg && n) snprintf(msg, n, "Timeout de medición (no ready).");
    return false;
}

bool VL53L0X_Deinit(VL53L0X_Handle_t* h, char* msg, size_t n)
{
    if (!h) return false;
    i2c_driver_delete(h->i2c_num);
    h->inited = false;
    if (msg && n) snprintf(msg, n, "I2C desinicializado.");
    return true;
}

/* =================== CAPA FILTRADA (LECTURA + FILTROS) =================== */
static bool Laser_read_raw_mm_unsafe(uint16_t* raw, float* mm)
{
    if (!s_h || !s_h->inited) return false;

    uint16_t dist_mm = 0;
    if (!VL53L0X_ReadDistanceMM(s_h, &dist_mm, NULL, 0)) {
        return false;
    }
    if (mm)  *mm  = (float)dist_mm;
    if (raw) *raw = dist_mm;
    return true;
}

static void Laser_task(void *arg)
{
    uint32_t fails = 0;

    med5_init(&s_med5);
    ema_init(&s_ema, s_alpha);
    s_last_alpha = s_alpha;

    for (;;)
    {
        uint16_t raw;
        float mm;
        if (Laser_read_raw_mm_unsafe(&raw, &mm))
        {
            float mm_med = med5_apply(&s_med5, mm);

            // Actualizar α si cambió (sin ema_set_alpha: re-init)
            float cur_alpha = s_alpha;
            if (fabsf(cur_alpha - s_last_alpha) > 1e-6f) {
                ema_init(&s_ema, cur_alpha);
                s_last_alpha = cur_alpha;
            }

            float mm_flt = ema_apply(&s_ema, mm_med);

            if (xSemaphoreTake(s_mutex, portMAX_DELAY))
            {
                s_last_raw  = raw;
                s_last_mm   = mm;
                s_last_mm_f = mm_flt;
                xSemaphoreGive(s_mutex);
            }
            fails = 0;
        }
        else
        {
            if (++fails == LASER_FAILS_BEFORE_WARN) {
                ESP_LOGW(TAG, "Lecturas fallidas consecutivas: %u", fails);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(s_ts_ms));
    }
}

/* =================== API PÚBLICA (capa filtrada) =================== */
bool Laser_init(VL53L0X_Handle_t* h)
{
    if (!h) {
        ESP_LOGE(TAG, "Handle nulo");
        return false;
    }
    s_h = h;

    // Si el driver aún no está iniciado, intenta iniciarlo aquí
    if (!s_h->inited) {
        char msg[64];
        if (!VL53L0X_Init(s_h, msg, sizeof(msg))) {
            ESP_LOGE(TAG, "VL53L0X_Init falló: %s", msg);
            s_ready = false;
            return false;
        } else {
            ESP_LOGI(TAG, "VL53L0X listo");
        }
    }

    if (!s_mutex) {
        s_mutex = xSemaphoreCreateMutex();
        if (!s_mutex) {
            ESP_LOGE(TAG, "No se pudo crear mutex");
            s_ready = false;
            return false;
        }
    }

    if (!s_task) {
        BaseType_t ok = xTaskCreate(Laser_task, "LaserTask", 3072, NULL, 5, &s_task);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "No se pudo crear tarea LaserTask");
            s_ready = false;
            return false;
        }
    }

    s_ready = true;
    ESP_LOGI(TAG, "Laser filtrado inicializado. Ts=%ums, EMA α=%.2f", s_ts_ms, s_alpha);
    return true;
}

bool Laser_is_inited(void)
{
    return s_ready && s_h && s_h->inited && s_task != NULL && s_mutex != NULL;
}

float Laser_get_mm_filtrado(void)
{
    if (!Laser_is_inited()) return 0.0f;
    float v = 0.0f;
    if (xSemaphoreTake(s_mutex, portMAX_DELAY)) {
        v = s_last_mm_f;
        xSemaphoreGive(s_mutex);
    }
    return v;
}

float Laser_get_mm_raw(void)
{
    if (!Laser_is_inited()) return 0.0f;
    float v = 0.0f;
    if (xSemaphoreTake(s_mutex, portMAX_DELAY)) {
        v = s_last_mm;
        xSemaphoreGive(s_mutex);
    }
    return v;
}

uint16_t Laser_get_raw(void)
{
    if (!Laser_is_inited()) return 0;
    uint16_t v = 0;
    if (xSemaphoreTake(s_mutex, portMAX_DELAY)) {
        v = s_last_raw;
        xSemaphoreGive(s_mutex);
    }
    return v;
}

void Laser_set_ema_alpha(float alpha)
{
    if (alpha <= 0.0f) alpha = 0.01f;
    if (alpha > 1.0f)  alpha = 1.0f;
    s_alpha = alpha;
}

void Laser_set_sample_period_ms(uint32_t ts_ms)
{
    if (ts_ms < 10) ts_ms = 10;
    s_ts_ms = ts_ms;
}
