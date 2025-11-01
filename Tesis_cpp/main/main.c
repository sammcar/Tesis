// INCLUDES

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_system.h"
#include <ctype.h>
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include <inttypes.h> 
#include <stdarg.h>
#include <math.h>
#include "esp_timer.h"
#include "cJSON.h"
#include "nvs.h"
#include "SyringePumpRS232.h"
#include "pid_control.h"
#include "Potenciometro.h"
#include "motor_stepper.h"
#include "filters.h"
#include "driver/gpio.h"
#include "LaserSensor.h"
#include "driver/i2c.h"
#include "esp_crt_bundle.h"
#include "ServoSG90.h"
#include "esp_log.h"
#include "freertos/semphr.h"

// DEFINICIONES

#define WIFI_SSID   "Sam"
#define WIFI_PASS   "Mina2025"
#define MQTT_URI   "mqtts://bbd56c9d1bd5405196effdfbcee53114.s1.eu.hivemq.cloud:8883"
#define MQTT_USER "esp32"                        // estamos en allow_anonymous
#define MQTT_PASS "Aa12345*"
#define NET_BIT_IP (1<<0)
#define DEVICE_ID   "esp32-demo"                // Debe coincidir con la GUI (tesis/<DEVICE_ID>/...)

#define NUM_ETAPAS    15
#define NUM_SENSORES  12
#define NUM_SALIDAS   12

#define SPD_DEADBAND_MM      2.0f      // banda para considerar "en target"
#define SPD_HOLD_MS          500       // tiempo estable para confirmar
#define SPD_MANUAL_MM_DEFAULT 30.0f    // valor por defecto en modo MANUAL (dentro de 0..stroke)
#define SPD_FINE_DEG_LIMIT   2.0f      // micro-ajuste final (límite en grados)

#define KV_PER_DEG      0.00831869f   // a
#define KV_INTERCEPT    0.862752f     // b
#define KV_MIN_CLAMP    0.0f          // por seguridad en UI/telemetría
#define KV_MAX_CLAMP    30.0f         // 10 vueltas = 30 kV (tu fuente)

#define BUF_SIZE      256
#define TICK_MS       100    // igual a tu sleep(0.5) del Python
#define TIMEOUT_MANUAL_RESCATE 15

#define STEP_PIN   GPIO_NUM_18
#define DIR_PIN    GPIO_NUM_5
#define EN_PIN     GPIO_NUM_23

#define PID_LOOP_HZ       20.0f
#define PID_DEADBAND_DEG  5.0f
#define PID_MAX_RATE_HZ   3000.0f

#define HOME_DEG_OFFSET   30.0f   // ← tu “cero” físico: 25°
#define DEG_FULL_SCALE    3600.0f
#define KV_MIN  10.0f
#define KV_MAX  50.0f
#define MH_BAND_DEG    10.0f   // banda para considerar "en home" (ángulo)

#define CAL_A  1.0f
#define CAL_B (-114.4f)
#define NEEDLE_MIN_MM 130.0f
#define NEEDLE_MAX_MM 150.0f

#define GAP_RING_N          5
#define GAP_STALE_MS        300    // si pasan >300 ms sin refresco, trátalo como “stale”
#define OPENLOOP_STEP_DEG   2.0f   // avance en grados cuando la lectura está “stale”/no válida
#define VL53_REINIT_AFTER_N 20

#define VL53_FAIL_REINIT_N   20     // re-init del VL53 tras N fallos seguidos
#define OPENLOOP_STEP_DEG     2.0f  // paso abierto cuando no hay muestra
#define OPENLOOP_EPS_DEG   1.5f

#ifndef OPENLOOP_EPS_DEG
#define OPENLOOP_EPS_DEG   1.5f          // error angular permitido para "en banda" (open-loop)
#endif

#ifndef OPENLOOP_HOLD_MS
#define OPENLOOP_HOLD_MS   SPD_HOLD_MS    // usa el mismo hold que en control cerrado
#endif



// STRUCTS
  VL53L0X_Handle_t laser = {
    .i2c_num    = I2C_NUM_0,
    .pin_sda    = GPIO_NUM_21,
    .pin_scl    = GPIO_NUM_22,
    .i2c_clk_hz = 100000,
    .pin_xshut  = GPIO_NUM_NC  // XSHUT ya a 3V3 en HW
  };
  
  ServoSG90_Handle_t servo = {
    .pin             = GPIO_NUM_19,
    .channel         = LEDC_CHANNEL_0,
    .timer           = LEDC_TIMER_0,
    .speed_mode      = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_16_BIT,
    .freq_hz         = 50,
    .us_min          = 600,
    .us_max          = 2400,
    .deg_min         = 0,
    .deg_max         = 180,
    .mm_per_rev      = 50.0f,  // 50 mm por 360°
    .stroke_mm_max   = 25.0f,  // carrera útil mecánica (≈ 0°→180°)
    .zero_deg        = 0.0f
  };
  
typedef struct {
  bool  collectorOn;
  float rpm;        // rpm del variador (si collectorOn = true)
  float kv;         // kV de la fuente HV
  float mlmin;      // caudal jeringa
  float syringeD;   // diámetro jeringa (mm)
  float gap;        // distancia (mm)
  float kvFS;       // Conversión
} Params;

typedef enum { MODO_AUTOMATICO, MODO_CICLO, MODO_ETAPA, MODO_MANUAL } Modo;

typedef bool (*SensorHandler)(int s_idx);   // devuelve true si el sensor confirmó
    
typedef enum { ST_IDLE, ST_SETPOINT, ST_TRACK, ST_HOLD, ST_DONE } fsm_t;

static Params g_params = {
  .collectorOn = false,
  .rpm = 0,
  .kv = 0,
  .mlmin = 0,
  .syringeD = 0,
  .gap = 0,
  .kvFS = 30.0f,
};

typedef struct {
  float    sensor_mm;   // distancia cruda VL53
  float    gap_mm;      // sensor_mm - offset
  float    gap_filt;    // promedio móvil
  int64_t  last_us;     // timestamp de la última actualización (esp_timer_get_time)
  bool     valid;       // true cuando hay al menos 1 muestra válida
} gap_shared_t;

// VARIABLES AZULES

static volatile bool   g_pid_enabled = false;
static volatile float  g_target_deg  = 0.0f;
static volatile bool g_motor_moving = false;
char smsg[96];
char lmsg[96];
char m1[96];
static volatile bool g_servo_moving = false;
static bool g_spd_moved_once = false;
static int SENSOR_TO_SALIDA[NUM_SENSORES];
static int ETAPA_SENSORES_FALLO[NUM_ETAPAS][6];
static bool etapas[NUM_ETAPAS] = { 0 };              // etapa 0 activa
static bool sensores[NUM_SENSORES] = { 0 };          // lectura instante
static bool latch[NUM_SENSORES]    = { 0 };          // “memoria” de sensor
static bool salidas[NUM_SALIDAS]   = { 0 };          // salidas activas
static bool   sensor_active[NUM_SENSORES]  = { 0 };  // armado en etapa
static bool pausa=false, fallo=false;
static bool start_flag=false, clc=false, se_flag=false;  // start es “palanca” (toggle)
static int  etapa_actual = 0;
static bool colectorEN = false;
static int    etapa_fallo_idx = -1;
static const char* salida_manual = NULL;
static bool salidas_prev[NUM_SALIDAS] = { 0 };

const float GAP_0_DEG_MM   = 120.0f;
const float GAP_90_DEG_MM  = 137.0f;
const float GAP_180_DEG_MM = 150.0f;
const float LASER_TO_TIP_OFFSET_MM = 113.0f; // Offset láser→punta (promedio de (sensor - real) en los 3 puntos):
const float SLOPE_0_90   = (GAP_90_DEG_MM  - GAP_0_DEG_MM)   / 90.0f; // ≈ 0.1889 mm/deg
const float SLOPE_90_180 = (GAP_180_DEG_MM - GAP_90_DEG_MM)  / 90.0f; // ≈ 0.1444 mm/deg
float target_gap_mm = 140.0f;
const float LOOP_HZ        = 20.0f;
const TickType_t LOOP_TICK = pdMS_TO_TICKS((int)(1000.0f/LOOP_HZ));
const float DEADBAND_MM    = 2.0f;    // no mover si |error| ≤ deadband
const float STEP_DEG_MAX   = 2.0f;    // límite de cambio por ciclo (suaviza)
float servo_deg_cmd        = 90.0f;   // arranca al centro
  // Pequeño promedio móvil (5) para estabilizar lectura sin EMA
float ring[5] = {0};
int   rpos    = 0;
int   rcount  = 0;

static volatile float g_needle_target_mm = 140.0f;  // objetivo actual
static volatile float g_needle_gap_mm    = 0.0f;    // último gap filtrado
static volatile bool  g_needle_moving    = false;   // true mientras esté fuera de banda
static volatile bool mover_servo_task = false;

// Snapshots publicados por needle_task (últimos valores válidos)
static volatile float g_sensor_mm_raw  = NAN;   // lectura cruda del VL53L0X
static volatile float g_gap_mm_filt    = NAN;   // gap filtrado (mm)
static volatile float g_servo_deg_last = NAN;   // último ángulo comandado
static volatile float g_last_gap_mm = NAN;
static volatile bool g_spd_reached = false;


// VARIABLES VERDES

static EventGroupHandle_t net_eg;
static stepper_motor_t g_motor;
static PID_t           g_pid;
static time_t inicio_manual = 0;
static time_t deadline_manual = 0;
static time_t tiempo_fallo_etapa = 0;
static Modo modo = MODO_AUTOMATICO;
static time_t sensor_deadline[NUM_SENSORES] = { 0 }; // fin de tiempo para confirmar
static SensorHandler SENSOR_FN[NUM_SENSORES] = { 0 }; // callbacks por sensor
static time_t sensor_armed_since[NUM_SENSORES] = {0};
static esp_mqtt_client_handle_t mqtt = NULL;
static gap_shared_t g_gap = { NAN, NAN, NAN, 0, false };
static SemaphoreHandle_t g_gap_mutex = NULL;

// PROTOTIPOS

static bool sensor_mock_timed_hold_2s(int s_idx);
static bool sensor_mock_timed_hold_1s(int s_idx);
static void mqtt_start(void);
static bool spd_handler(int s_idx);    // prototipo
static void process_command_line(char* line);
static void log_line(const char* fmt, ...);
static void publish_fault(const char* code, const char* message);
static void clear_fault(void);
static void aplicar_salidas_de_etapa(void);
static void modo_manual_tick(void);
static void sensores_tick_en_manual(void);
static void evo(void);
static void sensores_tick_en_etapa(void);
static void verificar_fallos(void);
static void liberar_latches_si_corresponde(void);
static void publish_sensor_float(const char* name, float value);
static bool sensor_mock_timed_hold_generic(int s_idx, int hold_s);

// CONSTANTES GRAFCET

static const char* SENSORES[NUM_SENSORES] = {
  "sh","mh","ok232","ok485","fcl","vd","spd","mpd","son","okmon","soff","voff"
};

static const char* SALIDAS[NUM_SALIDAS] = {
  "SERVOHOME","MOTORHOME","P232","P485","OKCL","VACON","SERVOMOVE","MOTORMOVE","SYON","MON","SYOFF","VACOFF"
};

static const char* ETAPA_SALIDAS_MAP[NUM_ETAPAS][6] = {
  /* 0  */ { NULL },
  /* 1  */ { "SERVOHOME","MOTORHOME","SYOFF","VACOFF", NULL },
  /* 2  */ { "P232", NULL },
  /* 3  */ { NULL },
  /* 4  */ { "P485", NULL },
  /* 5  */ { "OKCL", NULL },
  /* 6  */ { NULL },
  /* 7  */ { "VACON", NULL },
  /* 8  */ { "SERVOMOVE","MOTORMOVE", NULL },
  /* 9  */ { "SYON","MON", NULL },
  /* 10 */ { "OKCL", NULL },
  /* 11 */ { "SERVOHOME","MOTORHOME","SYOFF", NULL },
  /* 12 */ { NULL },
  /* 13 */ { "VACOFF", NULL },
  /* 14 */ { "OKCL", NULL }
};

static int TIEMPO_FALLA_SENSOR[NUM_SENSORES] = {
  /* sh   */ 20,  /* mh    */ 60,  /* ok232 */ 10,   /* ok485 */ 5,
  /* fcl  */ 9999,/* vd    */ 20,  /* spd   */ 20,  /* mpd   */ 60,
  /* son  */ 15,  /* okmon */ 9999,/* soff  */ 15,  /* voff  */ 20
};

// FUNCIONES AUX

static inline float clampf(float x, float lo, float hi){return x<lo?lo:(x>hi?hi:x);}
static inline float needle_get_last_gap_mm(void)      { return g_gap_mm_filt; }
static inline float needle_get_last_sensor_mm(void)   { return g_sensor_mm_raw; }
static inline float needle_get_last_servo_deg(void)   { return g_servo_deg_last; }
static inline void needle_set_last_gap_mm(float v) { g_last_gap_mm = v; }

static inline float kv_to_deg(float kv){
  // si quieres permitir pedir <0 o >30, no los “clampes” aquí;
  // yo los limito a lo físico de la fuente:
  kv = clampf(kv, KV_MIN_CLAMP, KV_MAX_CLAMP);
  float deg = (kv - KV_INTERCEPT) / KV_PER_DEG;
  return clampf(deg, 0.0f, DEG_FULL_SCALE);   // 0..3600°
}

static inline float deg_to_kv(float deg){
  float kv = KV_PER_DEG * deg + KV_INTERCEPT;
  return clampf(kv, KV_MIN_CLAMP, KV_MAX_CLAMP);
}

static inline float kv_to_deg_off(float kv, float kvFS, float off){
  float kv_min = KV_MIN;
  float kv_max = KV_MAX;
  if (kv < kv_min) kv = kv_min;
  if (kv > kv_max) kv = kv_max;
  float span_deg = DEG_FULL_SCALE - off;
  float span_kv  = kv_max - kv_min;
  float th = off + ((kv - kv_min) / span_kv) * span_deg;
  return clampf(th, off, DEG_FULL_SCALE);
}

static inline float deg_to_kv_off(float deg, float kvFS, float off){
  float kv_min = KV_MIN;
  float kv_max = KV_MAX;

  if (deg < off) deg = off;
  float span_deg = DEG_FULL_SCALE - off;
  float span_kv  = kv_max - kv_min;
  float kv = kv_min + ((deg - off) / span_deg) * span_kv;
  return clampf(kv, kv_min, kv_max);
}

static inline float clamp_mm_target(float mm, const ServoSG90_Handle_t* s)
{
    float lo = 0.0f;
    float hi = (s && s->stroke_mm_max > 0.0f) ? s->stroke_mm_max : 95.0f;
    if (mm < lo) mm = lo;
    if (mm > hi) mm = hi;
    return mm;
}

static int idx_salida(const char* nombre){
  for(int i=0;i<NUM_SALIDAS;i++) if(SALIDAS[i] && strcmp(SALIDAS[i],nombre)==0) return i;
  return -1;
}

static int idx_sensor(const char* nombre){
  for(int i=0;i<NUM_SENSORES;i++) if(SENSORES[i] && strcmp(SENSORES[i],nombre)==0) return i;
  return -1;
}

static void pin_sanity_test(void) {
  const gpio_num_t SDA = GPIO_NUM_21;
  const gpio_num_t SCL = GPIO_NUM_22;

  gpio_reset_pin(SDA);
  gpio_reset_pin(SCL);

  gpio_set_direction(SDA, GPIO_MODE_INPUT);
  gpio_set_direction(SCL, GPIO_MODE_INPUT);
  gpio_pullup_en(SDA);
  gpio_pullup_en(SCL);
  vTaskDelay(pdMS_TO_TICKS(2));
  printf("PINTEST Step1 INPUT+PU: SDA=%d SCL=%d\n",
         gpio_get_level(SDA), gpio_get_level(SCL));

  gpio_set_direction(SDA, GPIO_MODE_OUTPUT);
  gpio_set_direction(SCL, GPIO_MODE_OUTPUT);
  gpio_set_level(SDA, 1); gpio_set_level(SCL, 1); vTaskDelay(pdMS_TO_TICKS(1));
  printf("PINTEST Step2 OUT=1:    SDA=%d SCL=%d\n",
         gpio_get_level(SDA), gpio_get_level(SCL));
  gpio_set_level(SDA, 0); gpio_set_level(SCL, 0); vTaskDelay(pdMS_TO_TICKS(1));
  printf("PINTEST Step2 OUT=0:    SDA=%d SCL=%d\n",
         gpio_get_level(SDA), gpio_get_level(SCL));
  gpio_set_level(SDA, 1); gpio_set_level(SCL, 1); vTaskDelay(pdMS_TO_TICKS(1));
  printf("PINTEST Step2 OUT=1b:   SDA=%d SCL=%d\n",
         gpio_get_level(SDA), gpio_get_level(SCL));

  gpio_set_direction(SDA, GPIO_MODE_INPUT);
  gpio_set_direction(SCL, GPIO_MODE_INPUT);
  gpio_pullup_en(SDA);
  gpio_pullup_en(SCL);
  vTaskDelay(pdMS_TO_TICKS(2));
  printf("PINTEST Step3 INPUT+PU: SDA=%d SCL=%d\n",
         gpio_get_level(SDA), gpio_get_level(SCL));
}

static void trim_line(char* s) {
  // quita \r y \n y espacios extremos
  size_t n = strlen(s);
  while (n>0 && (s[n-1]=='\r' || s[n-1]=='\n' || s[n-1]==' ' || s[n-1]=='\t')) s[--n]='\0';
  char* p = s;
  while (*p==' ' || *p=='\t') p++;
  if (p!=s) memmove(s, p, strlen(p)+1);
}

static void str_tolower(char* s){
  for(; *s; ++s) *s = (char)tolower((unsigned char)*s);
}

void Needle_Control_Loop(float target_gap_mm)
{
  TickType_t last = xTaskGetTickCount();
  static bool was_moving = false;  // para habilitar/deshabilitar logs

  for (;;)
  {
    // 1) Leer láser (mm al punto de impacto del láser)
    uint16_t sensor_mm_u16 = 0;
    char rmsg[64];
    if (!VL53L0X_ReadDistanceMM(&laser, &sensor_mm_u16, rmsg, sizeof(rmsg))) {
      vTaskDelayUntil(&last, LOOP_TICK);
      continue;
    }

    float sensor_mm = (float)sensor_mm_u16;

    // 2) Convertir a gap real (punta→colector) usando el offset calibrado
    float gap_mm = sensor_mm - LASER_TO_TIP_OFFSET_MM;

    // 2b) Promedio móvil de 5 muestras (suave, sin EMA)
    ring[rpos] = gap_mm;
    rpos = (rpos + 1) % 5;
    if (rcount < 5) rcount++;
    float gap_filt = 0.0f;
    for (int i = 0; i < rcount; ++i) gap_filt += ring[i];
    gap_filt /= (float)rcount;

    // 3) Error y estado de movimiento
    float err = target_gap_mm - gap_filt;
    bool moving = (fabsf(err) > DEADBAND_MM);

    // 4) Cálculo feed-forward: target_gap_mm → grados deseados (piecewise)
    float desired_deg = servo_deg_cmd; // default: mantener
    float tgt = target_gap_mm;

    if (tgt <= GAP_0_DEG_MM) {
      desired_deg = 0.0f;
    } else if (tgt <= GAP_90_DEG_MM) {
      desired_deg = (tgt - GAP_0_DEG_MM) / SLOPE_0_90; // 0→90°
    } else if (tgt <= GAP_180_DEG_MM) {
      desired_deg = 90.0f + (tgt - GAP_90_DEG_MM) / SLOPE_90_180; // 90→180°
    } else {
      desired_deg = 180.0f;
    }

    // 5) Si fuera de deadband, acércate a desired_deg con rampa limitada
    if (moving) {
      float delta = desired_deg - servo_deg_cmd;
      if (delta >  STEP_DEG_MAX) delta =  STEP_DEG_MAX;
      if (delta < -STEP_DEG_MAX) delta = -STEP_DEG_MAX;
      servo_deg_cmd += delta;
      servo_deg_cmd = fmaxf(0.0f, fminf(servo_deg_cmd, 180.0f));
      (void)ServoSG90_SetAngleDeg(&servo, servo_deg_cmd, smsg, sizeof(smsg));
    }

    // 6) Telemetría (MQTT siempre) + logs sólo durante movimiento
    publish_sensor_float("gap_mm", gap_filt);

    if (moving) {
      if (!was_moving) {
        log_line("[CTRL] Moving towards target...");
      }
      log_line("[VL53L0X] sensor=%.1f mm | gap=%.1f mm | tgt=%.1f | servo=%.1f° | err=%.1f",
               sensor_mm, gap_filt, target_gap_mm, servo_deg_cmd, err);
    } else if (was_moving) {
      log_line("[CTRL] Target reached: gap=%.1f mm (tgt=%.1f)", gap_filt, target_gap_mm);
    }

    was_moving = moving;

    vTaskDelayUntil(&last, LOOP_TICK);
  }
}

void needle_set_target_mm(float mm){
  float tgt = clampf(mm, NEEDLE_MIN_MM, NEEDLE_MAX_MM);
  g_needle_target_mm = tgt;
  log_line("[needle] new target=%.1f mm", tgt);
}


// TASKS

static void pid_control_task(void *arg){
  const float Kp=0.35f, Ki=0.005f, Kd=0.10f, alpha=1.0f;
  pid_init(&g_pid, Kp, Ki, Kd, 0.0f, -PID_MAX_RATE_HZ, PID_MAX_RATE_HZ, 1.0f/PID_LOOP_HZ, alpha);

  bool moving_prev = false;
  float prev_err = 0.0f;

  for(;;){
    float theta = Potenciometro_get_grados_filtrado();
    theta = clampf(theta, 0.0f, 3600.0f);

    float sp = g_pid_enabled ? g_target_deg : theta;
    pid_set_setpoint(&g_pid, sp);

    float err = sp - theta;

    // --- Early stop si entro en banda PID ---
    if (!g_pid_enabled || fabsf(err) <= PID_DEADBAND_DEG){
      stepper_stop(&g_motor);
      g_motor_moving = false;
      prev_err = err;
      vTaskDelay(pdMS_TO_TICKS((int)roundf(1000.0f / PID_LOOP_HZ)));
      continue;
    }

    // --- Soft-landing: baja la velocidad al acercarte ---
    float u = pid_compute(&g_pid, theta);         // Hz (signed)
    float rate = fabsf(u);
    if (rate > PID_MAX_RATE_HZ) rate = PID_MAX_RATE_HZ;

    float approach = clampf(fabsf(err) / (2.0f*PID_DEADBAND_DEG), 0.10f, 1.0f);
    rate *= approach;  // reduce rate cerca del setpoint

    // --- Clamp por cruce de setpoint (evita “largarse” de largo) ---
    if ((prev_err > 0.0f && err < 0.0f) || (prev_err < 0.0f && err > 0.0f)) {
      if (fabsf(err) <= (1.5f * PID_DEADBAND_DEG)) {
        stepper_stop(&g_motor);
        g_motor_moving = false;
        prev_err = err;
        vTaskDelay(pdMS_TO_TICKS((int)roundf(1000.0f / PID_LOOP_HZ)));
        continue;
      }
    }

    int dir = (u >= 0.0f);
    int steps = (int)roundf(rate / PID_LOOP_HZ);
    if (steps > 0) {
      stepper_move(&g_motor, dir ? steps : -steps, rate);
      g_motor_moving = true;
    } else {
      stepper_stop(&g_motor);
      g_motor_moving = false;
    }

    // Log de inicio de movimiento (usa conversión CON offset)
    if (g_motor_moving && !moving_prev) {
      log_line("[pot] start θ=%.2f° | V=%.3f kV",
               theta, deg_to_kv_off(theta, g_params.kvFS, HOME_DEG_OFFSET));
    }
    moving_prev = g_motor_moving;
    prev_err = err;

    vTaskDelay(pdMS_TO_TICKS((int)roundf(1000.0f / PID_LOOP_HZ)));
  }
}

static void net_task(void *arg){
  for(;;){
    xEventGroupWaitBits(net_eg, NET_BIT_IP, pdTRUE, pdFALSE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(100));   // deja respirar al loop de eventos
    mqtt_start();                     // ahora en contexto propio, no en el handler
  }
}

static void telemetry_task(void *arg) {
  const int period_ms = 200;   // como tu fragmento de ejemplo

  // Acumuladores de logs (para no spamear)
  int acc_ms_laser = 0;
  int acc_ms_pot   = 0;

  for (;;) {
    // ====== LÁSER: lectura directa ======
    uint16_t mm = 0;
    char rmsg[64];
    bool rok = VL53L0X_ReadDistanceMM(&laser, &mm, rmsg, sizeof(rmsg));

    if (rok) {
      float sensor_mm = (float)mm;
      float gap_mm    = sensor_mm - LASER_TO_TIP_OFFSET_MM;

      // Actualiza buffer compartido como VÁLIDO y fresco
      int64_t now_us = esp_timer_get_time();
      if (xSemaphoreTake(g_gap_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_gap.sensor_mm = sensor_mm;
        g_gap.gap_mm    = gap_mm;
        g_gap.gap_filt  = gap_mm;     // sin filtros: usa el mismo valor
        g_gap.last_us   = now_us;
        g_gap.valid     = true;
        xSemaphoreGive(g_gap_mutex);
      }

      // MQTT: publica SOLO valores REALES + flags
      //publish_sensor_float("laser_ok", 1.0f);
      //publish_sensor_float("gap_valid", 1.0f);
      publish_sensor_float("laser_mm", gap_mm);
      //publish_sensor_float("gap_mm",   gap_mm);

      // LOG del láser SOLO si el SERVO se está moviendo
      if (g_needle_moving) {
        acc_ms_laser += period_ms;
        if (acc_ms_laser >= 1000) {
          acc_ms_laser = 0;
          log_line("[laser] gap=%.1f mm | sensor=%.1f mm", gap_mm, sensor_mm);
        }
      } else {
        acc_ms_laser = 0;
      }

    } else {
      // Marca buffer como INVÁLIDO (sin tocar valores)
      if (xSemaphoreTake(g_gap_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_gap.valid = false;
        xSemaphoreGive(g_gap_mutex);
      }
      // Sin logs de láser aquí (a menos que quieras uno condicional):
      // if (g_needle_moving) log_line("[laser] sin muestra: %s", rmsg);
      acc_ms_laser = 0;
    }

    // ====== POTENCIÓMETRO / KV: SIEMPRE MQTT, logs solo con stepper en movimiento ======
    float theta = Potenciometro_get_grados_filtrado();
    theta = clampf(theta, 0.0f, 3600.0f);
    float kv_now     = deg_to_kv_off(theta, g_params.kvFS, HOME_DEG_OFFSET);
    float target_deg = kv_to_deg_off(g_params.kv, g_params.kvFS, HOME_DEG_OFFSET);

    publish_sensor_float("angle_deg",  theta);
    publish_sensor_float("voltage_kv", kv_now);
    publish_sensor_float("target_deg", target_deg);
    publish_sensor_float("kv_target",  g_params.kv);
    publish_sensor_float("kv_fs",      g_params.kvFS);

    if (g_motor_moving) {
      acc_ms_pot += period_ms;
      if (acc_ms_pot >= 1000) {
        acc_ms_pot = 0;
        log_line("[pot] θ=%.2f° | KV=%.3f", theta, kv_now);
      }
    } else {
      acc_ms_pot = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(period_ms));
  }
}


static void grafcet_task(void*){
  log_line("⚙️ Simulador GRAFCET listo.\n");
  aplicar_salidas_de_etapa(); // imprime etapa 0
  for(;;){
    if(!pausa && !fallo){
      if(modo==MODO_MANUAL){
        modo_manual_tick();
        sensores_tick_en_manual();
      } else {
        evo();
        sensores_tick_en_etapa(); // Verifica/ejecuta funciones de sensores para la etapa actual
        verificar_fallos();
        aplicar_salidas_de_etapa();
        liberar_latches_si_corresponde();
      }
    } else {
      g_pid_enabled = false;
      stepper_stop(&g_motor);
      tiempo_fallo_etapa = time(NULL); // “congela” contadores de fallo mientras está pausado
    }
    vTaskDelay(pdMS_TO_TICKS(TICK_MS));
  }
}

static void needle_task(void *arg)
{
  TickType_t last = xTaskGetTickCount();
  static bool    was_moving         = false;
  static int64_t inband_start_us    = 0;   // hold para lazo CERRADO (igual que antes)
  static int64_t ol_inband_start_us = 0;   // *** NUEVO: hold para lazo ABIERTO ***

  for (;;) {
    // ⛔️ Task apagada (sin movimiento)
    if (!mover_servo_task) {
      was_moving         = false;
      inband_start_us    = 0;
      ol_inband_start_us = 0;   // *** NUEVO ***
      vTaskDelayUntil(&last, LOOP_TICK);
      continue;
    }

    // ====== Snapshot del buffer compartido del gap ======
    float   gap_filt = NAN;
    int64_t last_us  = 0;
    bool    valid    = false;

    if (xSemaphoreTake(g_gap_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      gap_filt = g_gap.gap_filt;
      last_us  = g_gap.last_us;
      valid    = g_gap.valid;
      xSemaphoreGive(g_gap_mutex);
    }

    // ¿datos viejos?
    bool stale = true;
    if (valid) {
      int64_t age_us = esp_timer_get_time() - last_us;
      stale = (age_us > (int64_t)GAP_STALE_MS * 1000LL);
    }

    // target actual (lo fija spd/mh/mpd/grafcet)
    float tgt = g_params.gap;

    // mapeo gap→deg (igual al tuyo)
    float desired_deg;
    if      (tgt <= GAP_0_DEG_MM)        desired_deg = 0.0f;
    else if (tgt <= GAP_90_DEG_MM)       desired_deg = (tgt - GAP_0_DEG_MM) / SLOPE_0_90;
    else if (tgt <= GAP_180_DEG_MM)      desired_deg = 90.0f + (tgt - GAP_90_DEG_MM) / SLOPE_90_180;
    else                                  desired_deg = 180.0f;

    bool moving = false;

    // ====== SIN feedback válido → lazo ABIERTO ======
    if (!valid || stale) {
      // Paso angulado limitado (pre-posición en open-loop)
      float delta = desired_deg - servo_deg_cmd;
      if (delta >  OPENLOOP_STEP_DEG) delta =  OPENLOOP_STEP_DEG;
      if (delta < -OPENLOOP_STEP_DEG) delta = -OPENLOOP_STEP_DEG;

      bool changed = false;
      if (fabsf(delta) > 0.001f) {
        servo_deg_cmd += delta;
        if (servo_deg_cmd < 0.0f)   servo_deg_cmd = 0.0f;
        if (servo_deg_cmd > 180.0f) servo_deg_cmd = 180.0f;
        (void)ServoSG90_SetAngleDeg(&servo, servo_deg_cmd, smsg, sizeof(smsg));
        changed = true;
      }

      moving = changed;

      // *** NUEVO: criterio de finalización en lazo abierto ***
      // Si estamos "encima" del target en términos angulares y nos mantenemos
      // dentro de la banda durante OPENLOOP_HOLD_MS, cerramos el SPD.
      float   err_deg = fabsf(desired_deg - servo_deg_cmd);
      int64_t now    = esp_timer_get_time();

      if (err_deg <= OPENLOOP_EPS_DEG) {
        if (ol_inband_start_us == 0) {
          ol_inband_start_us = now;
        } else if ((now - ol_inband_start_us) >= (int64_t)OPENLOOP_HOLD_MS * 1000LL) {
          mover_servo_task   = false;   // fin del ciclo SPD
          g_spd_reached      = true;    // marca OK para spd_handler
          log_line("[CTRL-OL] Target reached (open-loop): servo=%.1f° (tgt=%.1f°)",
                   servo_deg_cmd, desired_deg);
          ol_inband_start_us = 0;
        }
      } else {
        ol_inband_start_us = 0;         // aún no dentro de banda angular
      }

      // Mantén limpio el hold del lazo CERRADO
      inband_start_us = 0;
    }
    // ====== CON feedback válido → control NORMAL (cerrado) ======
    else {
      float err = tgt - gap_filt;
      moving = (fabsf(err) > DEADBAND_MM);

      if (moving) {
        float delta = desired_deg - servo_deg_cmd;
        if (delta >  STEP_DEG_MAX) delta =  STEP_DEG_MAX;
        if (delta < -STEP_DEG_MAX) delta = -STEP_DEG_MAX;

        servo_deg_cmd += delta;
        if (servo_deg_cmd < 0.0f)   servo_deg_cmd = 0.0f;
        if (servo_deg_cmd > 180.0f) servo_deg_cmd = 180.0f;
        (void)ServoSG90_SetAngleDeg(&servo, servo_deg_cmd, smsg, sizeof(smsg));

        // Reinicia holds
        inband_start_us    = 0;
        ol_inband_start_us = 0;

        // Log opcional (lo tenías comentado)
        // log_line("[VL53] gap=%.1f tgt=%.1f servo=%.1f err=%.1f",
        //          gap_filt, tgt, servo_deg_cmd, err);
      } else {
        // En banda (cerrado): mantener hold y cerrar SPD cuando corresponda
        int64_t now = esp_timer_get_time();
        if (inband_start_us == 0) {
          inband_start_us = now;
        } else if ((now - inband_start_us) >= (int64_t)SPD_HOLD_MS * 1000LL) {
          mover_servo_task = false;     // fin del ciclo SPD
          g_spd_reached    = true;      // marca OK para spd_handler
          log_line("[CTRL] Target reached: gap=%.1f mm (tgt=%.1f)", gap_filt, tgt);
          inband_start_us  = 0;
        }
        // No toques ol_inband_start_us en cerrado
      }
    }

    // Exporta flags/estado visibles para telemetría
    g_servo_deg_last = servo_deg_cmd;
    g_needle_moving  = moving;

    vTaskDelayUntil(&last, LOOP_TICK);
  }
}

// Handlers

static bool mh_handler(int s_idx){
  (void)s_idx;
  static fsm_t  st = ST_IDLE;
  static int64_t t_inband_us = 0;

  float theta = Potenciometro_get_grados_filtrado();
  theta = clampf(theta, 0.0f, 3600.0f);

  switch (st) {

    case ST_IDLE: {
      // Target SIEMPRE = HOME_DEG_OFFSET (en manual y en auto)
      g_target_deg  = HOME_DEG_OFFSET;
      g_pid_enabled = true;
      t_inband_us   = 0;
      log_line("[mh] init → θ_obj=%.2f° (kv=%.3f kvFS=%.3f)",
               g_target_deg, g_params.kv, g_params.kvFS);
      st = ST_TRACK;
      return false;
    }

    case ST_TRACK: {
      // Reafirma el setpoint por si cambió el offset
      float desired = HOME_DEG_OFFSET;
      if (fabsf(desired - g_target_deg) > 0.05f) {
        g_target_deg = desired;
      }

      // Usa la banda específica de home (MH_BAND_DEG), no la del PID
      if (fabsf(theta - g_target_deg) <= MH_BAND_DEG){
        t_inband_us = esp_timer_get_time();
        log_line("[mh] hold start");
        st = ST_HOLD;
      }
      return false;
    }

    case ST_HOLD: {
      // Si sale de banda, vuelve a trackear
      if (fabsf(theta - g_target_deg) > MH_BAND_DEG){
        st = ST_TRACK; 
        return false;
      }
      int64_t now = esp_timer_get_time();
      if ((now - t_inband_us) >= 1000LL*1000LL) { // 1.0 s estable
        g_pid_enabled = false;
        stepper_stop(&g_motor);
        log_line("✅ [mh] OK @ θ=%.2f° (V=%.3f kV)", theta, deg_to_kv(theta));
        st = ST_DONE;
        return true;
      }
      return false;
    }

    case ST_SETPOINT:   // para silenciar -Wswitch y futuro-proof
      st = ST_TRACK;
      return false;

    case ST_DONE:
      st = ST_IDLE;
      return true;

    default:
      st = ST_IDLE;
      return false;
  }
}

static bool mpd_handler(int s_idx){
  (void)s_idx;
  static fsm_t  st = ST_IDLE;
  static int64_t t_inband_us = 0;

  // lectura actual
  float theta = Potenciometro_get_grados_filtrado();
  theta = clampf(theta, 0.0f, 3600.0f);

  const bool modo_simple = (modo != MODO_AUTOMATICO);  // MANUAL, CICLO, ETAPA

  switch (st) {
    case ST_IDLE: {
      float target = modo_simple
        ? 1440.0f  // ← comportamiento “manual” para MANUAL/CICLO/ETAPA
        : kv_to_deg_off(g_params.kv, g_params.kvFS, HOME_DEG_OFFSET);

      g_target_deg  = target;
      g_pid_enabled = true;
      t_inband_us   = 0;

      log_line("[mpd] init %s → θ_obj=%.2f° (kv=%.3f kvFS=%.3f)",
               (modo_simple ? "SIMPLE" : "AUTO"),
               g_target_deg, g_params.kv, g_params.kvFS);
      st = ST_TRACK;
      return false;
    }

    case ST_TRACK: {
      float desired = modo_simple
        ? 1440.0f
        : kv_to_deg_off(g_params.kv, g_params.kvFS, HOME_DEG_OFFSET);

      if (fabsf(desired - g_target_deg) > 0.05f) {
        g_target_deg = desired;  // retarget si cambió kv en AUTO
      }

      if (fabsf(theta - g_target_deg) <= PID_DEADBAND_DEG){
        t_inband_us = esp_timer_get_time();
        log_line("[mpd] hold start");
        st = ST_HOLD;
      }
      return false;
    }

    case ST_HOLD: {
      if (fabsf(theta - g_target_deg) > PID_DEADBAND_DEG){
        st = ST_TRACK; 
        return false;
      }
      int64_t now = esp_timer_get_time();
      if ((now - t_inband_us) >= 1000LL*1000LL) { // 1 s estable
        g_pid_enabled = false;
        stepper_stop(&g_motor);
        log_line("✅ [mpd] OK @ θ=%.2f° (V=%.3f kV)", theta, deg_to_kv(theta));
        st = ST_DONE;
        return true;
      }
      return false;
    }

    case ST_SETPOINT:
      st = ST_TRACK; return false;

    case ST_DONE:
      st = ST_IDLE;  return true;

    default:
      st = ST_IDLE;  return false;
  }
}

static bool sh_handler(int s_idx){
  (void)s_idx;
  mover_servo_task = false;

  if (!ServoSG90_SetAngleDeg(&servo, 180.0f, smsg, sizeof(smsg))) {
    log_line("❌ [sh] Servo cmd falló: %s", smsg);
    return false;
  }

  log_line("✅ [sh] OK → 0.0° (~150 mm)");
  return true;
}

static bool spd_handler(int s_idx)
{
  (void)s_idx;

  // 1) Decide target según modo (y clámpealo)
  float target_mm;
  if (modo == MODO_AUTOMATICO) {
    target_mm = clampf(g_params.gap, NEEDLE_MIN_MM, NEEDLE_MAX_MM);
    if (target_mm != g_params.gap) g_params.gap = target_mm;
  } else {
    // MANUAL / CICLO / ETAPA: ir a home (0°)
    mover_servo_task = false;
    if (!ServoSG90_SetAngleDeg(&servo, 0.0f, smsg, sizeof(smsg))) {
      log_line("❌ [spd] set servo 0°: %s", smsg);
      return false;
    }
    log_line("✅ [spd] MAN/CICLO/ETAPA → servo=0° (≈150 mm)");
    return true;
  }

  // ===== ORDEN CORRECTO DE CHEQUEOS =====
  // A) Si la task está en curso, seguimos esperando
  if (mover_servo_task) {
    return false;
  }

  // B) Si la task YA terminó y marcó llegada (incluye lazo abierto)
  if (g_spd_reached) {
    g_spd_reached = false;   // listo para el próximo SPD
    log_line("✅ [spd] OK (target %.1f mm)", target_mm);
    return true;
  }

  // C) Si no está en curso ni marcada como terminada → iniciar nueva corrida
  g_spd_reached   = false;
  mover_servo_task = true;   // needle_task comienza a moverse hacia g_params.gap
  log_line("[spd] start → target=%.1f mm (task ON)", target_mm);
  return false;
}

static bool sensor_mock_handler(int s_idx){
  (void)s_idx;
  return false;    // hoy: basta con que la salida esté activa; luego reemplazas por IO real
}

static void registrar_handler_sensor(const char* sensor_nombre, SensorHandler fn){
  int s = idx_sensor(sensor_nombre);
  if(s>=0) SENSOR_FN[s] = fn ? fn : sensor_mock_handler;
}

static bool sensor_mock_timed_hold_2s(int s_idx) {
  return sensor_mock_timed_hold_generic(s_idx, 2);
}

static bool llamar_ok232_handler(int s_idx){
  (void)s_idx;

  static bool started = false;
  static bool bootstrapped = false;

  if (!bootstrapped) {
    log_line("[ok232] Bootstrap (Init/Addr/SafeOff)...");
    Syringe_Init();
    Syringe_SetAddress(0);
    Syringe_ExitSafeMode();
    vTaskDelay(pdMS_TO_TICKS(150));
    uart_flush_input(SYRINGE_UART);
    bootstrapped = true;
  }

  if (!started){
    started = true;
    log_line("[ok232] Test iniciado (SAFO + SAF0 + VER)");
  }

  char fw[64] = {0};
  char diag[256] = {0};

  // ↑ sube timeout para mayor robustez bajo carga Wi-Fi/MQTT:
  bool ok = Syringe_TestVER(fw, sizeof(fw), diag, sizeof(diag), 4000);

  // imprime tokens de diag (pero sin “flood”)
  if (diag[0]) {
    char *p = diag, *tok;
    int count = 0;
    while ((tok = strsep(&p, ";")) != NULL) {
      if (*tok) {
        log_line("[ok232] %s", tok);
        if (++count > 12) break; // corta por si hay spam
      }
    }
  }

  if (ok) {
    log_line("✅ [ok232] Firmware: %s", fw);
    started = false;
    return true;
  } else {
    log_line("❌ [ok232] No me he podido comunicar (esperando timeout de etapa).");
  }
  return false;
}

// NVS

static void params_load_from_nvs(void) {
  nvs_handle_t h;
  if (nvs_open("params", NVS_READONLY, &h) != ESP_OK) return;
  uint8_t on; size_t len;
  if (nvs_get_u8(h, "collectorOn", &on) == ESP_OK) g_params.collectorOn = (on != 0);
  nvs_get_blob(h, "rpm",       &g_params.rpm,       (len = sizeof(float), &len) ? &len : &len);
  nvs_get_blob(h, "kv",        &g_params.kv,        (len = sizeof(float), &len) ? &len : &len);
  nvs_get_blob(h, "mlmin",     &g_params.mlmin,     (len = sizeof(float), &len) ? &len : &len);
  nvs_get_blob(h, "syringeD",  &g_params.syringeD,  (len = sizeof(float), &len) ? &len : &len);
  nvs_get_blob(h, "gap",       &g_params.gap,       (len = sizeof(float), &len) ? &len : &len);
  if (nvs_get_blob(h, "kvFS",  &g_params.kvFS,      (len = sizeof(float), &len) ? &len : &len) != ESP_OK) {
    g_params.kvFS = 30.0f; // default si aún no existe
  }
  nvs_close(h);
}

static void params_save_to_nvs(const Params* p) {
  if (!p) return;
  nvs_handle_t h;
  if (nvs_open("params", NVS_READWRITE, &h) != ESP_OK) return;
  nvs_set_u8(h, "collectorOn", p->collectorOn ? 1 : 0);
  nvs_set_blob(h, "rpm",      &p->rpm,      sizeof(float));
  nvs_set_blob(h, "kv",       &p->kv,       sizeof(float));
  nvs_set_blob(h, "mlmin",    &p->mlmin,    sizeof(float));
  nvs_set_blob(h, "syringeD", &p->syringeD, sizeof(float));
  nvs_set_blob(h, "gap",      &p->gap,      sizeof(float));
  nvs_set_blob(h, "kvFS",     &p->kvFS,     sizeof(float));
  nvs_commit(h);
  nvs_close(h);
}

// GRAFCET

static const char* salida_de_sensor(int s_idx){
  int out = (s_idx>=0 && s_idx<NUM_SENSORES) ? SENSOR_TO_SALIDA[s_idx] : -1;
  return (out>=0) ? SALIDAS[out] : NULL;
}

static void construir_mapas(void){
    for (int e = 0; e < NUM_ETAPAS; e++)
    for (int k = 0; k < 6; k++)
    ETAPA_SENSORES_FALLO[e][k] = -1;
  memset(SENSOR_TO_SALIDA, -1, sizeof(SENSOR_TO_SALIDA));
  const struct { const char* sensor; const char* salida; } pairs[] = {
    {"sh","SERVOHOME"},{"ok485","P485"},{"spd","SERVOMOVE"},{"okmon","MON"},
    {"mh","MOTORHOME"},{"fcl","OKCL"},{"mpd","MOTORMOVE"},{"soff","SYOFF"},
    {"ok232","P232"},{"vd","VACON"},{"son","SYON"},{"voff","VACOFF"}
  };
  for(size_t i=0;i<sizeof(pairs)/sizeof(pairs[0]);i++){
    int s = idx_sensor(pairs[i].sensor);
    int o = idx_salida(pairs[i].salida);
    if(s>=0 && o>=0) SENSOR_TO_SALIDA[s]=o;
  }

  const struct { int etapa; const char* lista[6]; } fallo_map[] = {
    {1,  {"sh","mh","soff","voff",NULL}},
    {2,  {"ok232",NULL}},
    {4,  {"ok485",NULL}},
    {5,  {"fcl",NULL}},
    {7,  {"vd",NULL}},
    {8,  {"spd","mpd",NULL}},
    {9,  {"son","okmon",NULL}},
    {10, {"fcl",NULL}},
    {11, {"soff","mh","sh",NULL}},
    {13, {"voff",NULL}},
    {14, {"fcl",NULL}},
  };
  for(int e=0;e<NUM_ETAPAS;e++){
    for(int k=0;k<6;k++) ETAPA_SENSORES_FALLO[e][k]=-1;
  }
  for(size_t j=0;j<sizeof(fallo_map)/sizeof(fallo_map[0]);j++){
    int e = fallo_map[j].etapa;
    int k = 0;
    for(int i=0; fallo_map[j].lista[i]; i++){
      int s = idx_sensor(fallo_map[j].lista[i]);
      if(s>=0) ETAPA_SENSORES_FALLO[e][k++] = s;
    }
  }
  for(int s=0; s<NUM_SENSORES; s++) SENSOR_FN[s] = sensor_mock_handler;
}

static bool salida_presente_en_etapa(const char* s){
  for(int i=0; ETAPA_SALIDAS_MAP[etapa_actual][i]; i++)
    if(strcmp(ETAPA_SALIDAS_MAP[etapa_actual][i], s)==0) return true;
  return false;
}

static void sensores_tick_en_etapa(void){
  for(int k=0; ETAPA_SENSORES_FALLO[etapa_actual][k]!=-1; k++){
    int s = ETAPA_SENSORES_FALLO[etapa_actual][k];
    if(!sensor_active[s] || latch[s]) continue; // ya confirmado o no armado

    const char* out_name = salida_de_sensor(s);
    bool salida_ok = false;
    if(out_name){
      // Debe existir la salida en esta etapa (tu requisito)
      salida_ok = salida_presente_en_etapa(out_name);
    } else {
      // sensores sin salida asociada: permitir verificación directa
      salida_ok = true;
    }

    if(!salida_ok) continue;
    // Llama el handler específico (mock hoy)
    SensorHandler fn = SENSOR_FN[s] ? SENSOR_FN[s] : sensor_mock_handler;
    bool ok = fn(s);
    if(ok){
      latch[s] = true;
      sensor_active[s] = false;
      log_line("✅ Sensor '%s' confirmado.\n", SENSORES[s]);
    }
  }
}

static void activar_salida(const char* s){
  int i = idx_salida(s);
  if(i>=0){
    salidas[i]=true;
    log_line("⚙️ Activando salida: %s\n", s);
    if(strcmp(s,"OKCL")==0){
      // Equivalente a tu SALIDA_FUNCIONES["OKCL"]: prender_led("verde")
      log_line("[FUNC] LED VERDE encendido\n");
    }
  }
}

static void reset_to_stage0(void){
  memset(etapas,0,sizeof(etapas));
  etapas[0]=true; etapa_actual=0;
  fallo=false; // pausa se preserva, start (palanca) se preserva

  g_pid_enabled = false;
  stepper_stop(&g_motor);

  memset(sensores,0,sizeof(sensores));
  memset(latch,0,sizeof(latch));
  memset(salidas,0,sizeof(salidas));
  memset(salidas_prev,0,sizeof(salidas_prev));
  memset(sensor_active,0,sizeof(sensor_active));
  memset(sensor_deadline,0,sizeof(sensor_deadline));

  tiempo_fallo_etapa = time(NULL);
  etapa_fallo_idx = 0;

  // Fuerza impresión de etapa 0
  log_line("\n🔁 Etapa activa: %d\n", etapa_actual);
}

static void verificar_fallos(void){
  if(etapa_fallo_idx != etapa_actual){
    etapa_fallo_idx = etapa_actual;
    tiempo_fallo_etapa = time(NULL);
  }
  for(int k=0; ETAPA_SENSORES_FALLO[etapa_actual][k]!=-1; k++){
    int s = ETAPA_SENSORES_FALLO[etapa_actual][k];
    if(s>=0 && !latch[s]){
      int tf = TIEMPO_FALLA_SENSOR[s];
      if(tf>0 && difftime(time(NULL), tiempo_fallo_etapa) > tf){
        log_line("❌ Falla detectada en sensor: %s\n", SENSORES[s]);
        fallo=true;
        publish_fault("SENSOR_TIMEOUT", SENSORES[s]); 
        break;
      }
    }
  }

  // Deadlines por sensor armado en la etapa:
  time_t now = time(NULL);
  for(int k=0; ETAPA_SENSORES_FALLO[etapa_actual][k]!=-1; k++){
    int s = ETAPA_SENSORES_FALLO[etapa_actual][k];
    if(s<0) continue;
    if(sensor_active[s] && !latch[s] && sensor_deadline[s] > 0 && now > sensor_deadline[s]){
      log_line("❌ Timeout de sensor '%s' en etapa %d\n", SENSORES[s], etapa_actual);
      fallo = true;
      publish_fault("SENSOR_TIMEOUT", SENSORES[s]);
      break;
    }
  }
}

static void sensores_tick_en_manual(void){
  if (!salida_manual) return;

  // Encuentra la salida activa en manual y su sensor asociado (si lo hay)
  int out = idx_salida(salida_manual);
  if (out < 0) return;

  int s_idx = -1;
  for (int s = 0; s < NUM_SENSORES; s++){
    if (SENSOR_TO_SALIDA[s] == out) { s_idx = s; break; }
  }
  if (s_idx < 0) return;         // no hay sensor asociado a esta salida
  if (latch[s_idx]) return;      // ya confirmado

  // Llama el handler real (o mock) una y otra vez sin bloquear
  SensorHandler fn = SENSOR_FN[s_idx] ? SENSOR_FN[s_idx] : sensor_mock_handler;
  if (fn(s_idx)) {
    latch[s_idx] = true;
    log_line("✅ [Manual] Sensor '%s' confirmado.\n", SENSORES[s_idx]);
  }
}

static void aplicar_salidas_de_etapa(void){
  static int etapa_anterior=-1;

  // Construye set de salidas de la etapa
  bool current[NUM_SALIDAS]={0};
  for(int i=0; ETAPA_SALIDAS_MAP[etapa_actual][i]; i++){
    int idx = idx_salida(ETAPA_SALIDAS_MAP[etapa_actual][i]);
    if(idx>=0) current[idx]=true;
  }

  // Cambios
  if(etapa_actual != etapa_anterior){
    log_line("\n🔁 Etapa activa: %d\n", etapa_actual);
    log_line("[ESTADO] START=%s | MODO=%s | COLECTOR=%s\n",
      start_flag? "ON":"OFF",
      (modo==MODO_AUTOMATICO? "automatico": modo==MODO_CICLO? "ciclo": modo==MODO_ETAPA? "etapa":"manual"),
      colectorEN? "ON":"OFF");
    etapa_anterior = etapa_actual;

    // Reset y enciende sólo las de la etapa, imprimiendo una vez
    memset(salidas,0,sizeof(salidas));
    for(int i=0;i<NUM_SALIDAS;i++){
      if(current[i]){
        salidas[i]=true;
        activar_salida(SALIDAS[i]);
      }
    }

    // 🔔 Armado de sensores esperados en esta etapa:
    time_t now = time(NULL);
    for (int k = 0; ETAPA_SENSORES_FALLO[etapa_actual][k] != -1; k++) {
      int s = ETAPA_SENSORES_FALLO[etapa_actual][k];
      latch[s]            = false;     // debe reconfirmarse en esta etapa
      sensor_active[s]    = true;      // sensor “pendiente”
      sensor_armed_since[s] = now;     // ⬅️ para el mock de “tiempo en ON”
      int tf              = TIEMPO_FALLA_SENSOR[s] > 0 ? TIEMPO_FALLA_SENSOR[s] : 10;
      sensor_deadline[s]  = now + tf;  // deadline de confirmación
      const char* out_name = salida_de_sensor(s);

      if (out_name) {
        log_line("⏱️ Armando sensor '%s' ligado a salida '%s' (tmax=%ds)\n",
                SENSORES[s], out_name, tf);
      } else {
        log_line("⏱️ Armando sensor '%s' (sin salida asociada) (tmax=%ds)\n",
                SENSORES[s], tf);
      }
    }

  } else {
    // Misma etapa: imprime flancos de OFF→ON
    for(int i=0;i<NUM_SALIDAS;i++){
      bool want = current[i];
      if(want && !salidas_prev[i]){
        salidas[i]=true;
        activar_salida(SALIDAS[i]);
      } else {
        salidas[i]=want;
      }
    }
  }

  // Guardar estado para flancos
  memcpy(salidas_prev, salidas, sizeof(salidas_prev));
}

static void liberar_latches_si_corresponde(void){
  for(int s=0;s<NUM_SENSORES;s++){
    int out = SENSOR_TO_SALIDA[s];
    bool salida_activa = (out>=0) ? salidas[out] : false;
    bool manual_salida_es_esta = (modo==MODO_MANUAL && salida_manual && out>=0 && strstr(salida_manual, SALIDAS[out])!=NULL);
    if(!salida_activa && !manual_salida_es_esta){
      latch[s]=false;
    }
  }
}

static void modo_manual_tick(void){
  // Si hay falla: bloquear manual
  if(fallo){
    memset(salidas,0,sizeof(salidas));
    salida_manual=NULL; inicio_manual=0; deadline_manual=0;
    return;
  }

  // Apaga todo; luego enciende sólo lo pedido
  memset(salidas,0,sizeof(salidas));

  if(!salida_manual){
    inicio_manual=0; deadline_manual=0;
    return;
  }

  if(strcmp(salida_manual,"RFALLA_MANUAL")==0){
    int syoff = idx_salida("SYOFF");
    int mh    = idx_salida("MOTORHOME");
    int sh    = idx_salida("SERVOHOME");
    if(syoff>=0) salidas[syoff]=true;
    if(mh>=0)    salidas[mh]=true;
    if(sh>=0)    salidas[sh]=true;

    if(!inicio_manual){
      inicio_manual = time(NULL);
      deadline_manual = inicio_manual + TIMEOUT_MANUAL_RESCATE;
      log_line("⚙️ [Manual] Rescate: SYOFF + MOTORHOME + SERVOHOME\n");
    }

    // ¿Sensores confirmados?
    int ok=0;
    int s_syoff = idx_sensor("soff");
    int s_mh    = idx_sensor("mh");
    int s_sh    = idx_sensor("sh");
    if(s_syoff>=0 && latch[s_syoff]) ok++;
    if(s_mh>=0   && latch[s_mh])     ok++;
    if(s_sh>=0   && latch[s_sh])     ok++;

    if(ok==3){
      log_line("✅ [Manual] Rescate completado.\n");
      salida_manual=NULL; inicio_manual=0; deadline_manual=0;
      return;
    }

    if(time(NULL) > deadline_manual){
      log_line("❌ [Manual] Rescate: sensores no confirmados a tiempo.\n");
      fallo=true;
    }
    return;
  }

  // Salida normal
  int out = idx_salida(salida_manual);
  if(out<0){
    log_line("⚠️ [Manual] Salida desconocida: %s\n", salida_manual);
    salida_manual=NULL; return;
  }
  salidas[out]=true;

  if(!inicio_manual){
    inicio_manual = time(NULL);
    int s_idx = -1; // sensor asociado si existe
    // busca sensor cuyo mapeo apunte a esta salida
    for(int s=0;s<NUM_SENSORES;s++) if(SENSOR_TO_SALIDA[s]==out){ s_idx=s; break; }
    for (int s=0; s<NUM_SENSORES; s++) latch[s] = false; 
    if(s_idx>=0){
      latch[s_idx] = false;
      sensor_active[s_idx] = true;
      sensor_armed_since[s_idx] = inicio_manual;
      int timeout = TIEMPO_FALLA_SENSOR[s_idx] > 0 ? TIEMPO_FALLA_SENSOR[s_idx] : 10;
      deadline_manual = inicio_manual + timeout;
      log_line("⚙️ [Manual] %s: esperando sensor '%s' ≤ %ds\n", SALIDAS[out], SENSORES[s_idx], timeout);
    } else {
      deadline_manual = inicio_manual + 1; // “operación breve” sin sensor
      log_line("⚙️ [Manual] %s: sin sensor asociado (operación breve)\n", SALIDAS[out]);
    }
  }

  // ¿Sensor confirmado?
  int s_idx=-1;
  for(int s=0;s<NUM_SENSORES;s++) if(SENSOR_TO_SALIDA[s]==out){ s_idx=s; break; }
  if(s_idx>=0 && latch[s_idx]){
    log_line("✅ [Manual] %s: sensor '%s' confirmado.\n", SALIDAS[out], SENSORES[s_idx]);
    salida_manual=NULL; inicio_manual=0; deadline_manual=0; return;
  }

  if(time(NULL) > deadline_manual){
    if(s_idx>=0){
      fallo=true;
      log_line("❌ [Manual] %s: sensor '%s' no confirmado a tiempo.", SALIDAS[out], SENSORES[s_idx]);
      publish_fault("MAN_TIMEOUT", SENSORES[s_idx]);
    } else {
      log_line("ℹ️ [Manual] %s: operación breve completada.", SALIDAS[out]);
    }
    salida_manual=NULL; inicio_manual=0; deadline_manual=0;
  }
}

static void evo(void){
  bool E[NUM_ETAPAS]; memcpy(E, etapas, sizeof(E));
  bool s[NUM_SENSORES]; memcpy(s, latch, sizeof(s));

  bool automatico = (modo==MODO_AUTOMATICO);
  bool ciclo      = (modo==MODO_CICLO);
  bool etapa_m    = (modo==MODO_ETAPA);

  bool condiCL = (automatico || (ciclo && clc) || (etapa_m && se_flag));
  bool condi   = (automatico ||  ciclo         || (etapa_m && se_flag));

  struct { int a; bool c; int n; } T[] = {
    {0,  (start_flag && condiCL), 1},
    {1,  (s[idx_sensor("sh")] && s[idx_sensor("mh")] && s[idx_sensor("soff")] && s[idx_sensor("voff")] && condi), 2},
    {2,  (s[idx_sensor("ok232")] && (!colectorEN) && condi), 3},
    {2,  (s[idx_sensor("ok232")] &&  colectorEN  && condi), 4},
    {3,  ((!colectorEN) && condi), 5},
    {4,  (s[idx_sensor("ok485")] && colectorEN && condi), 5},
    {5,  ((!colectorEN) && condiCL), 6},
    {5,  (colectorEN && condiCL), 7},
    {6,  ((!colectorEN) && condi), 8},
    {7,  (s[idx_sensor("vd")] && colectorEN && condi), 8},
    {8,  (s[idx_sensor("mpd")] && s[idx_sensor("spd")] && condi), 9},
    {9,  (s[idx_sensor("son")] && s[idx_sensor("okmon")] && condi), 10},
    {10, (condiCL), 11},
    {11, (s[idx_sensor("soff")] && s[idx_sensor("mh")] && s[idx_sensor("sh")] && (!colectorEN) && condi), 12},
    {11, (s[idx_sensor("soff")] && s[idx_sensor("mh")] && s[idx_sensor("sh")] &&  colectorEN  && condi), 13},
    {12, ((!colectorEN) && condi), 14},
    {13, (s[idx_sensor("voff")] && colectorEN && condi), 14},
    {14, (start_flag && condi), 0},
  };

  for(size_t i=0;i<sizeof(T)/sizeof(T[0]);i++){
    if(E[T[i].a] && T[i].c){
      memset(etapas,0,sizeof(etapas));
      etapas[T[i].n]=true;
      etapa_actual=T[i].n;
      break;
    }
  }

  // En tu Python: CLC y SE se limpian al final del ciclo
  clc=false; se_flag=false;
}

static void process_command_line(char* line){
  char buf[BUF_SIZE];
  strncpy(buf, line, sizeof(buf)-1);
  buf[sizeof(buf)-1]='\0';
  trim_line(buf);
  str_tolower(buf);  // ahora comparaciones con substrings minúsculas

  if (buf[0]=='\0') return;

  if (strstr(buf,"start")) {
    start_flag = !start_flag;
    log_line("▶️  START = %s\n", start_flag? "on":"off");
    return;
  }
  if (strstr(buf,"clc")) { clc=true; return; }
  if (strstr(buf," se")) { se_flag=true; return; }  // evita confundir con "sensor"
  if (!strncmp(buf,"se",2) && strlen(buf)==2) { se_flag=true; return; }

  if (strstr(buf,"modo")) {
    if (strstr(buf,"automatico")) modo=MODO_AUTOMATICO;
    else if (strstr(buf,"ciclo"))  modo=MODO_CICLO;
    else if (strstr(buf,"etapa"))  modo=MODO_ETAPA;
    else if (strstr(buf,"manual")) modo=MODO_MANUAL;
    else { log_line("ℹ️ Usa: modo automatico|ciclo|etapa|manual\n"); return; }
    clc=false; se_flag=false; salida_manual=NULL; inicio_manual=0; deadline_manual=0;
    reset_to_stage0();
    log_line("✅ Modo cambiado a %s y proceso reiniciado en etapa 0\n",
      (modo==MODO_AUTOMATICO? "automatico": modo==MODO_CICLO? "ciclo": modo==MODO_ETAPA? "etapa":"manual"));
    return;
  }

  if (strstr(buf,"paro")) { pausa=true;  log_line("🟥 PARO DE EMERGENCIA activado.\n"); return; }
  if (!strcmp(buf,"ok")) { pausa=false; log_line("▶️ Proceso reanudado.\n"); return; }
  if (strstr(buf,"reset")) { reset_to_stage0(); log_line("🔄 Sistema reiniciado.\n"); return; }

  if (strstr(buf,"rfalla")) {
    fallo=false; salida_manual=NULL; inicio_manual=0; deadline_manual=0;
    reset_to_stage0();
    clear_fault();
    log_line("✅ Falla reseteada y proceso en etapa 0.\n");
    return;
  }

  if (strstr(buf,"colector")) {
    if (strstr(buf,"on"))  colectorEN=true;
    if (strstr(buf,"off")) colectorEN=false;
    log_line("⚙️ colectorEN = %s\n", colectorEN? "ON":"OFF");
    return;
  }

  // Sensores (sh, mh, ok232, ...)
  for(int s=0;s<NUM_SENSORES;s++){
    if (SENSORES[s] && strstr(buf, SENSORES[s])) {
      int out = SENSOR_TO_SALIDA[s];
      bool activa = (out>=0) ? salidas[out] : false;
      bool manual_match = (modo==MODO_MANUAL && salida_manual && out>=0 && strstr(salida_manual, SALIDAS[out])!=NULL);
      if(!fallo && (activa || manual_match)){
        latch[s]=true;
        log_line("🔸 Latch sensor: %s\n", SENSORES[s]);
      } else {
        log_line("⛔ Sensor %s ignorado: salida no activa o en FALLA.\n", SENSORES[s]);
      }
      return;
    }
  }

  // Modo manual: activar salidas por nombre o RFALLA_MANUAL
  if (modo==MODO_MANUAL) {
    if (strstr(buf,"rfalla_manual")) {
      if (fallo) { log_line("⛔ [Manual] Bloqueado por FALLA. Usa 'rfalla'.\n"); return; }
      salida_manual="RFALLA_MANUAL"; inicio_manual=0; deadline_manual=0;
      log_line("⚙️ [Manual] Rescate solicitado.\n");
      return;
    }
    for(int i=0;i<NUM_SALIDAS;i++){
      char low[32]; snprintf(low, sizeof(low), "%s", SALIDAS[i]); str_tolower(low);
      if (strstr(buf, low)) {
        if (fallo) { log_line("⛔ [Manual] Bloqueado por FALLA. Usa 'rfalla'.\n"); return; }
        salida_manual = SALIDAS[i]; inicio_manual=0; deadline_manual=0;
        // Limpia latch del sensor ligado a esta salida, por si quedó "pegado"
        int out = idx_salida(salida_manual);
        for(int s=0;s<NUM_SENSORES;s++){
          if (SENSOR_TO_SALIDA[s]==out){
            latch[s] = false;
            sensor_active[s] = true;
            sensor_armed_since[s] = time(NULL);
            break;
          }
        }
        log_line("⚙️ [Manual] Activando salida: %s\n", SALIDAS[i]);
        return;
      }
    }
  }

  log_line("❓ Comando no reconocido.\n");
}

static void comando_task(void*){
  // Desactiva buffering para stdin/stdout para ver y leer al instante
  setvbuf(stdin,  NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);

  printf("\nComandos: start, clc, se, modo automatico|ciclo|etapa|manual, paro, ok, reset, rfalla, colector on|off,\n"
         "sensores (sh,mh,ok232,...) y en manual salidas (SERVOHOME, P485, ...) o RFALLA_MANUAL\n> ");

  char line[BUF_SIZE];
  int  idx = 0;

  for(;;){
    int c = getchar();
    if (c == EOF) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

    if (c == '\r') continue;      // ignoramos CR (PlatformIO suele mandar CRLF)
    if (c == '\b' || c == 127) {  // backspace
      if (idx > 0) { idx--; line[idx]='\0'; }
      continue;
    }

    if (c == '\n') {
      line[idx] = '\0';
      idx = 0;                    // resetea para la siguiente línea
      process_command_line(line);
      printf("> ");
      continue;
    }

    if (idx < (int)sizeof(line)-1) {
      line[idx++] = (char)c;
      line[idx] = '\0';
    }
  }
}

static bool sensor_mock_timed_hold_generic(int s_idx, int hold_s) {
  if (s_idx < 0 || s_idx >= NUM_SENSORES) return true;   // trivial si índice inválido

  int out = SENSOR_TO_SALIDA[s_idx];
  // Si no hay salida asociada, lo consideramos trivialmente OK a los hold_s
  bool salida_asociada = (out >= 0);
  time_t now = time(NULL);

  if (salida_asociada) {
    const char* out_name = SALIDAS[out];
    // Requisito: que esa salida exista en la etapa y esté activa
    if (!salida_presente_en_etapa(out_name)) return false;
    if (!salidas[out]) return false;  // aún no encendió físicamente
  }

  // ¿Ya cumplió el tiempo mínimo de "sostenido"?
  double held = difftime(now, sensor_armed_since[s_idx]);
  return (held >= hold_s);
}

// MQTT

static void publish_params_active(const char* reason) {
  if (!mqtt) return;
  cJSON* root = cJSON_CreateObject();
  cJSON_AddBoolToObject(root, "collectorOn", g_params.collectorOn);
  cJSON_AddNumberToObject(root, "rpm",       g_params.rpm);
  cJSON_AddNumberToObject(root, "kv",        g_params.kv);
  cJSON_AddNumberToObject(root, "mlmin",     g_params.mlmin);
  cJSON_AddNumberToObject(root, "syringeD",  g_params.syringeD);
  cJSON_AddNumberToObject(root, "gap",       g_params.gap);
  cJSON_AddNumberToObject(root, "kvFS",      g_params.kvFS);  // ⬅️ nuevo
  if (reason) cJSON_AddStringToObject(root, "reason", reason);
  char *json = cJSON_PrintUnformatted(root);
  if (json) {
    esp_mqtt_client_publish(mqtt, "tesis/" DEVICE_ID "/params/active", json, 0, 1, 1);
    free(json);
  }
  cJSON_Delete(root);
}

static void publish_state(const char* reason){
  if (!mqtt) return;

  const char* m =
    (modo==MODO_AUTOMATICO? "automatico" :
    modo==MODO_CICLO? "ciclo" :
    modo==MODO_ETAPA? "etapa" : "manual");

  char json[256];
  snprintf(json, sizeof(json),
    "{\"start\":%s,\"paro\":%s,\"fallo\":%s,"
    "\"modo\":\"%s\",\"etapa\":%d,\"colector\":%s}",
    start_flag?"true":"false",
    pausa?"true":"false",
    fallo?"true":"false",
    m, etapa_actual,
    colectorEN?"true":"false");

  // retain = 1 para que la GUI siempre reciba el último estado al conectarse
  esp_mqtt_client_publish(mqtt, "tesis/" DEVICE_ID "/state/json", json, 0, 0, 1);

  if (reason){
    char line[64];
    snprintf(line, sizeof(line), "[state:%s]", reason);
    esp_mqtt_client_publish(mqtt, "tesis/" DEVICE_ID "/log/text", line, 0, 0, 0);
  }
}

static void mqtt_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data){

  switch (event_id) {
    case MQTT_EVENT_CONNECTED: {
      log_line("[MQTT] conectado\n");
      // suscripción a comandos de texto
      esp_mqtt_client_subscribe(mqtt, "tesis/" DEVICE_ID "/cmd/text", 0);
      esp_mqtt_client_subscribe(mqtt, "tesis/" DEVICE_ID "/cmd/parametros", 1);


      // avisa presencia y estado inicial (retain=1)
      esp_mqtt_client_publish(mqtt, "tesis/" DEVICE_ID "/ack/text", "online", 0, 0, 1);
      publish_state("boot");  // función que agregamos abajo
      publish_params_active("boot");  // ⬅️ nuevo
      break;
    }

    case MQTT_EVENT_DATA: {
      esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)event_data;

      char topic[128];
      int tlen = ev->topic_len < (int)sizeof(topic)-1 ? ev->topic_len : (int)sizeof(topic)-1;
      memcpy(topic, ev->topic, tlen); topic[tlen] = 0;

      char msg[256];
      int mlen = ev->data_len < (int)sizeof(msg)-1 ? ev->data_len : (int)sizeof(msg)-1;
      memcpy(msg, ev->data, mlen); msg[mlen] = 0;

      if (strcmp(topic, "tesis/" DEVICE_ID "/cmd/text") == 0) {
        // eco/ack
        esp_mqtt_client_publish(mqtt, "tesis/" DEVICE_ID "/ack/text", msg, 0, 0, 0);

        // re-usa tu parser serial
        process_command_line(msg);

        // publica nuevo estado tras procesar
        publish_state("cmd");
      }

      if (strcmp(topic, "tesis/" DEVICE_ID "/cmd/parametros") == 0) {
        // payload JSON parcial: { collectorOn?:bool, rpm?:number, kv?:number, mlmin?:number, syringeD?:number, gap?:number, apply?:bool }
        cJSON* root = cJSON_Parse(msg);
        if (!root) {
          log_line("[PARAMS] JSON inválido\n");
          break;
        }

        Params newp = g_params; // partimos del actual
        bool changed = false;

        const cJSON* j_on  = cJSON_GetObjectItemCaseSensitive(root, "collectorOn");
        const cJSON* j_rpm = cJSON_GetObjectItemCaseSensitive(root, "rpm");
        const cJSON* j_kv  = cJSON_GetObjectItemCaseSensitive(root, "kv");
        const cJSON* j_ml  = cJSON_GetObjectItemCaseSensitive(root, "mlmin");
        const cJSON* j_sd  = cJSON_GetObjectItemCaseSensitive(root, "syringeD");
        const cJSON* j_gap = cJSON_GetObjectItemCaseSensitive(root, "gap");
        const cJSON* j_apply = cJSON_GetObjectItemCaseSensitive(root, "apply");
        const cJSON* j_kvfs = cJSON_GetObjectItemCaseSensitive(root, "kvFS");

        if (cJSON_IsBool(j_on)) {
          newp.collectorOn = cJSON_IsTrue(j_on);
          changed = true;
        }
        if (cJSON_IsNumber(j_rpm)) {
          newp.rpm = (float)j_rpm->valuedouble;
          changed = true;
        }
        if (cJSON_IsNumber(j_kv)) {
          newp.kv = (float)j_kv->valuedouble;
          changed = true;
        }
        if (cJSON_IsNumber(j_ml)) {
          newp.mlmin = (float)j_ml->valuedouble;
          changed = true;
        }
        if (cJSON_IsNumber(j_sd)) {
          newp.syringeD = (float)j_sd->valuedouble;
          changed = true;
        }
        if (cJSON_IsNumber(j_gap)) {
          newp.gap = (float)j_gap->valuedouble;
          changed = true;
        }
        if (cJSON_IsNumber(j_kvfs)) { 
          newp.kvFS = (float)j_kvfs->valuedouble; 
          changed = true; }

        bool apply = cJSON_IsBool(j_apply) ? cJSON_IsTrue(j_apply) : false;

        cJSON_Delete(root);

        if (changed) {
          g_params = newp;

          // 🔒 Clamp/validación segura de kvFS y kv
          if (g_params.kvFS < 0.1f) g_params.kvFS = 30.0f;
          if (g_params.kv   < 0.0f) g_params.kv   = 0.0f;
          if (g_params.kv   > g_params.kvFS) g_params.kv = g_params.kvFS;

          // 🔗 Enlaza parámetros con tu lógica actual (opcionales)
          colectorEN = g_params.collectorOn;

          log_line("[PARAMS] collector=%s rpm=%.2f kv=%.3f/%.3fFS mlmin=%.3f syringeD=%.3f gap=%.3f",
                  g_params.collectorOn ? "ON" : "OFF",
                  g_params.rpm, g_params.kv, g_params.kvFS,
                  g_params.mlmin, g_params.syringeD, g_params.gap);

          if (apply) {
            params_save_to_nvs(&g_params);
            log_line("[PARAMS] guardados en NVS");
          }

          publish_params_active(apply ? "apply" : "patch");
        }
      }

      break;
    }

    case MQTT_EVENT_DISCONNECTED:
      log_line("[MQTT] desconectado\n");
      break;
    case MQTT_EVENT_ERROR:
      printf("[MQTT] error\n");
      break;
    default:
      break;
  }
}

static void mqtt_start(void){
  if (mqtt) return;

  const esp_mqtt_client_config_t cfg = {
    .broker = {
      .address = {
        .uri = MQTT_URI, // "mqtts://<id>.s1.eu.hivemq.cloud:8883"
      },
      .verification = {
        .crt_bundle_attach = esp_crt_bundle_attach, // << usa bundle
      },
    },
    .credentials = {
      .username = MQTT_USER,
      .authentication = { .password = MQTT_PASS },
    },
    .session = {
      .protocol_ver = MQTT_PROTOCOL_V_3_1_1,
      .keepalive = 30,
    },
    .network = { .timeout_ms = 10000 },
    .task = { .stack_size = 8192 },
  };

  mqtt = esp_mqtt_client_init(&cfg);
  esp_mqtt_client_register_event(mqtt, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
  printf("[MQTT] Conectando a %s\n", MQTT_URI);
  ESP_ERROR_CHECK( esp_mqtt_client_start(mqtt) );
}

static void log_line(const char* fmt, ...) {
  char buf[256];

  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  // ¿el mensaje que llegó ya termina en '\n'?
  size_t n = strlen(buf);
  bool has_nl = (n > 0 && buf[n-1] == '\n');

  // --- Serial: respeta lo que venga, pero garantiza salto al final
  if (has_nl) {
    // ya trae '\n' → imprime tal cual
    printf("%s", buf);
  } else {
    // no trae '\n' → añade uno solo para serial
    printf("%s\n", buf);
  }

  // --- MQTT: publica SIN salto final (si traía, lo quitamos)
  if (mqtt) {
    if (has_nl) buf[n-1] = '\0';  // quitar '\n' solo para publicar
    esp_mqtt_client_publish(mqtt, "tesis/" DEVICE_ID "/log/text", buf, 0, 0, 0);
  }
}

static void publish_sensor_float(const char* name, float value) {
  if (!mqtt || !name) return;
  char topic[128], payload[64];
  snprintf(topic, sizeof(topic), "tesis/" DEVICE_ID "/sensor/%s", name);
  snprintf(payload, sizeof(payload), "%.3f", value);
  esp_mqtt_client_publish(mqtt, topic, payload, 0, 0, 0); // qos0, no retain
}

static void publish_fault(const char* code, const char* message) {
  if (!mqtt) return;
  char json[256];
  snprintf(json, sizeof(json),
           "{\"code\":\"%s\",\"message\":\"%s\",\"etapa\":%d}",
           code ? code : "FALLA", message ? message : "Error detectado", etapa_actual);
  esp_mqtt_client_publish(mqtt, "tesis/" DEVICE_ID "/fault/json", json, 0, 1, 1);
}

static void clear_fault(void) {
  if (!mqtt) return;
  // Publica un objeto “empty” con retain para que el último estado no sea una falla
  esp_mqtt_client_publish(mqtt, "tesis/" DEVICE_ID "/fault/json", "{}", 0, 1, 1);
}

// WIFI

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START){
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED){
    wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)event_data;
    printf("[WIFI] desconectado, reason=%d (%s) → reintentando...\n",
           e ? e->reason : -1,
           e ? (e->reason==201?"AUTH_FAIL":"") : "");
    esp_wifi_connect();
  }else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP){
    printf("[WIFI] IP obtenida\n");
    xEventGroupSetBits(net_eg, NET_BIT_IP);
  }
}

static void wifi_init_sta(void){
  // NVS + netif + loop
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

  wifi_config_t wifi_config = {0};
  strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
  strncpy((char*)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));
  wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  esp_wifi_set_ps(WIFI_PS_NONE); 

  log_line("[WIFI] iniciando STA a SSID='%s'\n", WIFI_SSID);
}

// i2c

static esp_err_t i2c_probe_addr(i2c_port_t i2c_num, uint8_t addr, TickType_t to_ticks)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err = i2c_master_start(cmd);
    if (err == ESP_OK) err = i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true /*expect ACK*/);
    if (err == ESP_OK) err = i2c_master_stop(cmd);
    if (err == ESP_OK) err = i2c_master_cmd_begin(i2c_num, cmd, to_ticks);
    i2c_cmd_link_delete(cmd);
    return err; // ESP_OK => dispositivo presente
}

static void i2c_bus_health_once(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint32_t clk_hz)
{
  printf("=== I2C health on port %d ===\n", i2c_num);

  // Lectura de niveles con pull-ups habilitados (diagnóstico real)
  gpio_set_pull_mode(sda, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(scl, GPIO_PULLUP_ONLY);
  gpio_set_direction(sda, GPIO_MODE_INPUT);
  gpio_set_direction(scl, GPIO_MODE_INPUT);
  vTaskDelay(pdMS_TO_TICKS(1));
  printf("Idle levels (PU enabled): SDA=%d, SCL=%d (1=alto)\n",
         gpio_get_level(sda), gpio_get_level(scl));

  // Driver temporal para escanear
  i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = sda,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = scl,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = clk_hz ? clk_hz : 100000,
    .clk_flags = 0
  };
  esp_err_t err = i2c_param_config(i2c_num, &conf);
  if (err != ESP_OK) { printf("i2c_param_config: %s\n", esp_err_to_name(err)); return; }

  err = i2c_driver_install(i2c_num, conf.mode, 0, 0, 0);
  if (err != ESP_OK) { printf("i2c_driver_install: %s\n", esp_err_to_name(err)); return; }

  // Timeout ancho por si hay clock stretching
  i2c_set_timeout(i2c_num, 0xFFFFF);

  // Scan
  printf("Scan addresses:\n");
  for (uint8_t addr = 1; addr < 127; addr++){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    if (err == ESP_OK) printf("  - found @0x%02X\n", addr);
  }

  // Quita el driver temporal
  i2c_driver_delete(i2c_num);
  printf("=== I2C health done ===\n");
}

static void i2c_force_release(gpio_num_t sda, gpio_num_t scl) {
  // Suelta cualquier función previa
  gpio_reset_pin(sda);
  gpio_reset_pin(scl);

  // INPUT + PU
  gpio_set_direction(sda, GPIO_MODE_INPUT);
  gpio_set_direction(scl, GPIO_MODE_INPUT);
  gpio_pullup_en(sda);
  gpio_pullup_en(scl);
  vTaskDelay(pdMS_TO_TICKS(2));

  // Si SCL está bajo, intenta recovery con pulsos
  if (gpio_get_level(scl) == 0) {
    // OD + PU para clocking manual
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

  // Vuelve a INPUT+PU (estado ocioso I²C)
  gpio_set_direction(sda, GPIO_MODE_INPUT);
  gpio_set_direction(scl, GPIO_MODE_INPUT);
  gpio_pullup_en(sda);
  gpio_pullup_en(scl);
  vTaskDelay(pdMS_TO_TICKS(2));
}

// MAIN

void app_main(void)
{
  // --- INIT ---

  bool lok = VL53L0X_Init(&laser, lmsg, sizeof(lmsg));
  printf("Laser Init: %s (ok=%d)\n", lmsg, lok);
  if (!lok) { printf("⚠️  Verifica VCC/GND/SDA/SCL/XSHUT.\n"); while(1) vTaskDelay(pdMS_TO_TICKS(1000)); }
  char rr[32];
  uint16_t u = 0;
  bool ok = VL53L0X_ReadDistanceMM(&laser, &u, rr, sizeof(rr));
  printf("[probe] ok=%d dist=%u mm msg=%s\n", ok ? 1 : 0, (unsigned)u, rr);

  g_gap_mutex = xSemaphoreCreateMutex();
  if (!g_gap_mutex) {
    printf("❌ No se pudo crear g_gap_mutex\n");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  if (!ServoSG90_Init(&servo, smsg, sizeof(smsg))) {
    printf("❌ Servo Init: %s\n", smsg);
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  printf("Servo Init: %s\n", smsg);
  (void)ServoSG90_SetAngleDeg(&servo, 90.0f, smsg, sizeof(smsg));

  Potenciometro_init();
  stepper_init(&g_motor, STEP_PIN, DIR_PIN, EN_PIN, 10*1000000);
  stepper_enable(&g_motor, true);

  // // --- Red + MQTT ---
  net_eg = xEventGroupCreate();
  xTaskCreatePinnedToCore(net_task, "net", 4096, NULL, 5, NULL, 0);
  wifi_init_sta();
  vTaskDelay(pdMS_TO_TICKS(5000));  // pequeña pausa

  xTaskCreatePinnedToCore(needle_task, "needle", 6144, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL, 6, NULL, 0);
  xTaskCreatePinnedToCore(pid_control_task,"pid",     6144, NULL, 5, NULL, 1);
  // // xTaskCreatePinnedToCore(comando_task,  "cmd",       4096, NULL, 3, NULL, 1);
  // // xTaskCreatePinnedToCore(grafcet_task,  "grafcet",   6144, NULL, 4, NULL, 1);

  // registrar_handler_sensor("sh", sh_handler);
  // registrar_handler_sensor("spd", spd_handler);
  // registrar_handler_sensor("mh", mh_handler);
  // registrar_handler_sensor("mpd", mpd_handler);

  // // ================================
  // // PRUEBAS DE HANDLERS SIN GRAFCET
  // // ================================
  modo = MODO_MANUAL;

  log_line("=== TEST SH_HANDLER ===");
  mover_servo_task = false;   // desactivar control automático
  bool ok_sh = sh_handler(-1);
  log_line("sh_handler() → %s", ok_sh ? "OK" : "FAIL");
  vTaskDelay(pdMS_TO_TICKS(3000));  // espera 3 s

  log_line("=== TEST SPD MANUAL ===");
  mover_servo_task = true;   // desactivar control automático
  modo = MODO_MANUAL;
  while (!spd_handler(-1)) vTaskDelay(pdMS_TO_TICKS(200));

  log_line("=== SH_HANDLER ===");
  vTaskDelay(pdMS_TO_TICKS(3000));  // espera 3 s
  mover_servo_task = false;   // desactivar control automático
  bool ok_sh2 = sh_handler(-1);
  log_line("sh_handler() → %s", ok_sh2 ? "OK" : "FAIL");

  // vTaskDelay(pdMS_TO_TICKS(3000));  // espera 3 s
  // mover_servo_task = false;   // desactivar control automático
  // bool ok_sh3 = sh_handler(-1);
  // log_line("sh_handler() → %s", ok_sh3 ? "OK" : "FAIL");

  log_line("=== TEST SPD_HANDLER AUTO ===");
  bool oksss = VL53L0X_ReadDistanceMM(&laser, &u, rr, sizeof(rr));
  printf("[probe 2] ok=%d dist=%u mm msg=%s\n", oksss ? 1 : 0, (unsigned)u, rr);
  vTaskDelay(pdMS_TO_TICKS(3000));  // espera 3 s
  modo = MODO_AUTOMATICO;
  bool ok_spd = false;
  g_params.gap = 130.0f;       // <- fija aquí tu setpoint
  while (!(ok_spd=spd_handler(-1))) {
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  log_line("spd_handler() → %s", ok_spd ? "OK" : "FAIL");

  bool oks2 = VL53L0X_ReadDistanceMM(&laser, &u, rr, sizeof(rr));
  printf("[probe 2] ok=%d dist=%u mm msg=%s\n", oks2 ? 1 : 0, (unsigned)u, rr);
  vTaskDelay(pdMS_TO_TICKS(3000));  // espera 3 s

  log_line("=== TEST MPD ===");
  vTaskDelay(pdMS_TO_TICKS(3000));  // pequeña pausa
  modo = MODO_MANUAL;
  bool ok_mpd = false;
  while (!(ok_mpd = mpd_handler(-1))) {
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  log_line("mpd_handler() → %s", ok_mpd ? "OK" : "FAIL");

  // log_line("=== TEST MH ===");
  // vTaskDelay(pdMS_TO_TICKS(3000));  // pequeña pausa
  // modo = MODO_MANUAL;
  // bool ok_mh = false;
  // while (!(ok_mh = mh_handler(-1))) {
  //   vTaskDelay(pdMS_TO_TICKS(200));
  // }
  // log_line("mh_handler() → %s", ok_mh ? "OK" : "FAIL");

}


// void app_main(void)
// {
//   // --- INIT ---
//   bool lok = VL53L0X_Init(&laser, lmsg, sizeof(lmsg));
//   printf("Laser Init: %s (ok=%d)\n", lmsg, lok);
//   if (!lok) { printf("⚠️  Verifica VCC/GND/SDA/SCL/XSHUT.\n"); while(1) vTaskDelay(pdMS_TO_TICKS(1000)); }

//   if (!ServoSG90_Init(&servo, smsg, sizeof(smsg))) {
//     printf("❌ Servo Init: %s\n", smsg);
//     while (1) vTaskDelay(pdMS_TO_TICKS(1000));
//   }
//   printf("Servo Init: %s\n", smsg);
//   (void)ServoSG90_SetAngleDeg(&servo, servo_deg_cmd, smsg, sizeof(smsg));

//   // --- Red + MQTT ---
//   net_eg = xEventGroupCreate();
//   xTaskCreatePinnedToCore(net_task, "net", 4096, NULL, 5, NULL, 0);
//   wifi_init_sta();

//   // --- Ejecutar lazo principal (distancia objetivo definida por usuario) ---
//   float target_gap_mm = 145.0f;  // este valor luego vendrá del usuario (NVS/GUI)
//   Needle_Control_Loop(target_gap_mm);
// }


// void app_main(void)
// {

//   // INIT
//   bool lok = VL53L0X_Init(&laser, lmsg, sizeof(lmsg));
//   printf("Laser Init: %s (ok=%d)\n", lmsg, lok);
//   if (!lok) { printf("⚠️  Verifica VCC/GND/SDA/SCL/XSHUT.\n"); while(1) vTaskDelay(pdMS_TO_TICKS(1000)); }
  
//   if (!ServoSG90_Init(&servo, smsg, sizeof(smsg))) {
//     printf("❌ Servo Init: %s\n", smsg);
//     while (1) vTaskDelay(pdMS_TO_TICKS(1000));
//   }
//   printf("Servo Init: %s\n", smsg);
//   (void)ServoSG90_SetAngleDeg(&servo, servo_deg_cmd, smsg, sizeof(smsg));

//   // --- Red + MQTT---
//   net_eg = xEventGroupCreate();
//   xTaskCreatePinnedToCore(net_task, "net", 4096, NULL, 5, NULL, 0);
//   wifi_init_sta();

//   // FUNCION
//   TickType_t last = xTaskGetTickCount();
//   for (;;)
//   {
//     // 1) Leer láser (mm al punto de impacto del láser)
//     uint16_t sensor_mm_u16 = 0;
//     char rmsg[64];
//     if (!VL53L0X_ReadDistanceMM(&laser, &sensor_mm_u16, rmsg, sizeof(rmsg))) {
//       // Si falla, solo espera y sigue
//       vTaskDelayUntil(&last, LOOP_TICK);
//       continue;
//     }
//     float sensor_mm = (float)sensor_mm_u16;
//     // 2) Convertir a gap real (punta→colector) usando el offset calibrado
//     float gap_mm = sensor_mm - LASER_TO_TIP_OFFSET_MM;
//     // 2b) Promedio móvil de 5 muestras (suave, sin EMA)
//     ring[rpos] = gap_mm;
//     rpos = (rpos + 1) % 5;
//     if (rcount < 5) rcount++;
//     float gap_filt = 0.0f;
//     for (int i = 0; i < rcount; ++i) gap_filt += ring[i];
//     gap_filt /= (float)rcount;
//     // 3) Deadband: si ya estamos cerca, no muevas
//     float err = target_gap_mm - gap_filt;

//     // 4) Cálculo feed-forward: target_gap_mm → grados deseados (piecewise)
//     float desired_deg = servo_deg_cmd; // default: mantener
//     float tgt = target_gap_mm;
//     if (tgt <= GAP_0_DEG_MM) {
//       desired_deg = 0.0f;
//     } else if (tgt <= GAP_90_DEG_MM) {
//       desired_deg = (tgt - GAP_0_DEG_MM) / SLOPE_0_90; // 0→90°
//     } else if (tgt <= GAP_180_DEG_MM) {
//       desired_deg = 90.0f + (tgt - GAP_90_DEG_MM) / SLOPE_90_180; // 90→180°
//     } else {
//       desired_deg = 180.0f;
//     }
//     // 5) Si fuera de deadband, acércate a desired_deg con rampa limitada
//     if (fabsf(err) > DEADBAND_MM) {
//       float delta = desired_deg - servo_deg_cmd;
//       if (delta >  STEP_DEG_MAX) delta =  STEP_DEG_MAX;
//       if (delta < -STEP_DEG_MAX) delta = -STEP_DEG_MAX;
//       servo_deg_cmd += delta;
//       if (servo_deg_cmd < 0.0f)   servo_deg_cmd = 0.0f;
//       if (servo_deg_cmd > 180.0f) servo_deg_cmd = 180.0f;
//       (void)ServoSG90_SetAngleDeg(&servo, servo_deg_cmd, smsg, sizeof(smsg));
//     }
//     // 6) Telemetría (MQTT + Serial)
//     publish_sensor_float("gap_mm",   gap_filt);

//     printf("[VL53L0X] sensor=%.1f mm | gap=%.1f mm | tgt=%.1f | servo=%.1f° | err=%.1f\n",
//            sensor_mm, gap_filt, target_gap_mm, servo_deg_cmd, err);

//     vTaskDelayUntil(&last, LOOP_TICK);
//   }
// }


// void app_main(void)
// {
//     /* ========= CONFIGURACIÓN INICIAL ========= */
//     const float GAP_TARGET_MM = 140.0f;   // referencia del usuario (distancia real)
//     const float LOOP_HZ = 20.0f;          // frecuencia PID
//     const float Ts = 1.0f / LOOP_HZ;
//     TickType_t last = xTaskGetTickCount();

//     // Calibración lineal entre sensor y distancia real:
//     // gap_mm = A * sensor_mm + B
//     // sensor_mm = (gap_mm - B) / A
//     #define CAL_A   0.82335f
//     #define CAL_B  (-73.58f)
//     #define GAP_FROM_SENSOR(mm_sensor) ((CAL_A * (mm_sensor)) + CAL_B)
//     #define SENSOR_FROM_GAP(mm_gap)    (((mm_gap) - CAL_B) / CAL_A)

//     const float SERVO_MM_PER_REV  = 50.0f;    // carrera mecánica
//     const float SERVO_STROKE_MM   = 25.0f;    // 0°→180° ≈ 25 mm
//     const float ZERO_SENSOR_MM    = 275.0f;   // lectura aprox. en servo=180°
//     const float FINE_DEG_LIMIT    = 2.0f;     // límite microajuste
//     const float DEADBAND_MM       = 0.8f;     // tolerancia en mm
//     const float GAP_MIN_MM        = 120.0f;
//     const float GAP_MAX_MM        = 150.0f;

//     /* ========= DIAGNÓSTICO I2C ========= */
//     pin_sanity_test();
//     i2c_force_release(GPIO_NUM_21, GPIO_NUM_22);
//     i2c_bus_health_once(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 100000);

//     /* ========= SENSOR VL53L0X ========= */
//     VL53L0X_Handle_t laser = {
//         .i2c_num    = I2C_NUM_0,
//         .pin_sda    = GPIO_NUM_21,
//         .pin_scl    = GPIO_NUM_22,
//         .i2c_clk_hz = 100000,
//         .pin_xshut  = GPIO_NUM_NC  // XSHUT a 3.3V en HW
//     };
//     char lmsg[96];
//     bool lok = VL53L0X_Init(&laser, lmsg, sizeof(lmsg));
//     printf("Laser Init: %s (ok=%d)\n", lmsg, lok);
//     if (!lok) { printf("⚠️ Verifica XSHUT a 3V3.\n"); while(1) vTaskDelay(pdMS_TO_TICKS(1000)); }

//     /* ========= SERVO SG90 ========= */
//     ServoSG90_Handle_t servo = {
//         .pin             = GPIO_NUM_19,
//         .channel         = LEDC_CHANNEL_0,
//         .timer           = LEDC_TIMER_0,
//         .speed_mode      = LEDC_LOW_SPEED_MODE,
//         .duty_resolution = LEDC_TIMER_16_BIT,
//         .freq_hz         = 50,
//         .us_min          = 600,
//         .us_max          = 2400,
//         .deg_min         = 0,
//         .deg_max         = 180,
//         .mm_per_rev      = SERVO_MM_PER_REV,
//         .stroke_mm_max   = SERVO_STROKE_MM,
//         .zero_deg        = 0.0f
//     };
//     char smsg[96];
//     if (!ServoSG90_Init(&servo, smsg, sizeof(smsg))) {
//         printf("❌ Servo Init: %s\n", smsg);
//         while (1) vTaskDelay(pdMS_TO_TICKS(1000));
//     }
//     printf("Servo Init: %s\n", smsg);

//     /* ========= RED + MQTT ========= */
//     net_eg = xEventGroupCreate();
//     xTaskCreatePinnedToCore(net_task, "net", 4096, NULL, 5, NULL, 0);
//     wifi_init_sta();

//     /* ========= PID GAP REAL ========= */
//     PID_t pid_gap;
//     const float Kp = 0.5f;
//     const float Ki = 0.02f;
//     const float Kd = 0.00f;
//     pid_init(&pid_gap, Kp, Ki, Kd, 0.0f, -1000.0f, 1000.0f, Ts, 1.0f);

//     // Clamp inicial de setpoint
//     float target_gap_mm = GAP_TARGET_MM;
//     if (target_gap_mm < GAP_MIN_MM) target_gap_mm = GAP_MIN_MM;
//     if (target_gap_mm > GAP_MAX_MM) target_gap_mm = GAP_MAX_MM;

//     publish_sensor_float("gap_target_mm", target_gap_mm);

//     /* ========= BUCLE PRINCIPAL ========= */
//     for (;;)
//     {
//         uint16_t mm_raw = 0;
//         char rmsg[64];
//         bool ok = VL53L0X_ReadDistanceMM(&laser, &mm_raw, rmsg, sizeof(rmsg));
//         if (!ok) {
//             printf("⚠️  VL53L0X error: %s\n", rmsg);
//             vTaskDelay(pdMS_TO_TICKS(200));
//             continue;
//         }

//         float sensor_mm = (float)mm_raw;
//         float gap_mm = GAP_FROM_SENSOR(sensor_mm);

//         // Publicar mediciones
//         publish_sensor_float("laser_mm_raw", sensor_mm);
//         publish_sensor_float("gap_mm", gap_mm);
//         printf("[VL53L0X] sensor=%.1f mm | gap=%.1f mm\n", sensor_mm, gap_mm);

//         // Deadband (ya está suficientemente cerca)
//         if (fabsf(gap_mm - target_gap_mm) <= DEADBAND_MM) {
//             vTaskDelayUntil(&last, pdMS_TO_TICKS((int)roundf(1000.0f / LOOP_HZ)));
//             continue;
//         }

//         // --- PID ---
//         pid_set_setpoint(&pid_gap, target_gap_mm);
//         float u_gap = pid_compute(&pid_gap, gap_mm);   // salida = corrección en mm

//         // Convertir mm → grados de servo (25 mm = 180° ⇒ 1 mm ≈ 7.2°)
//         float u_deg = u_gap * (180.0f / SERVO_STROKE_MM);
//         if (u_deg >  FINE_DEG_LIMIT) u_deg =  FINE_DEG_LIMIT;
//         if (u_deg < -FINE_DEG_LIMIT) u_deg = -FINE_DEG_LIMIT;

//         // Comando: “llevar gap a target” con micro-ajuste angular
//         float target_act_mm = (ZERO_SENSOR_MM - sensor_mm);
//         if (!ServoSG90_SetByMillimetersWithFine(&servo, target_act_mm, u_deg, FINE_DEG_LIMIT, smsg, sizeof(smsg))) {
//             printf("❌ Servo cmd: %s\n", smsg);
//         }

//         publish_sensor_float("pid_output_deg", u_deg);
//         publish_sensor_float("servo_cmd_mm", target_act_mm);

//         vTaskDelayUntil(&last, pdMS_TO_TICKS((int)roundf(1000.0f / LOOP_HZ)));
//     }
// }

// void app_main(void){
//   /* ========= Config rápida para pruebas ========= */
//   const float TEST_DEG = 90.0f;         // ← cambia aquí: 0.0, 90.0, 180.0, etc.
//   const float LOOP_HZ  = 10.0f;        // frecuencia de muestreo a imprimir/publicar
//   TickType_t last      = xTaskGetTickCount();

//   /* ========= Salud del bus I2C ========= */
//   pin_sanity_test();
//   i2c_force_release(GPIO_NUM_21, GPIO_NUM_22);
//   i2c_bus_health_once(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 100000);

//   /* ========= Inicializar VL53L0X (sin capa filtrada) ========= */
//   VL53L0X_Handle_t laser = {
//     .i2c_num    = I2C_NUM_0,
//     .pin_sda    = GPIO_NUM_21,
//     .pin_scl    = GPIO_NUM_22,
//     .i2c_clk_hz = 100000,
//     .pin_xshut  = GPIO_NUM_NC  // XSHUT alto en HW
//   };
//   char lmsg[96];
//   bool lok = VL53L0X_Init(&laser, lmsg, sizeof(lmsg));
//   printf("Laser Init: %s (ok=%d)\n", lmsg, lok);
//   if (!lok) { printf("⚠️  Verifica XSHUT a 3V3.\n"); while(1) vTaskDelay(pdMS_TO_TICKS(1000)); }

//   /* ========= Inicializar Servo SG90 (GPIO19) ========= */
//   ServoSG90_Handle_t servo = {
//     .pin             = GPIO_NUM_19,
//     .channel         = LEDC_CHANNEL_0,
//     .timer           = LEDC_TIMER_0,
//     .speed_mode      = LEDC_LOW_SPEED_MODE,
//     .duty_resolution = LEDC_TIMER_16_BIT,
//     .freq_hz         = 50,
//     .us_min          = 600,
//     .us_max          = 2400,
//     .deg_min         = 0,
//     .deg_max         = 180,
//     .mm_per_rev      = 50.0f,    // tu medición: 50 mm / 360°
//     .stroke_mm_max   = 25.0f,    // carrera útil con 0°→180°
//     .zero_deg        = 0.0f
//   };
//   char smsg[96];
//   if (!ServoSG90_Init(&servo, smsg, sizeof(smsg))) {
//     printf("❌ Servo Init: %s\n", smsg);
//     while (1) vTaskDelay(pdMS_TO_TICKS(1000));
//   }
//   printf("Servo Init: %s\n", smsg);

//   /* ========= Red + MQTT (publicaremos laser_mm y servo_deg) ========= */
//   net_eg = xEventGroupCreate();
//   xTaskCreatePinnedToCore(net_task, "net", 4096, NULL, 5, NULL, 0);
//   wifi_init_sta();

//   /* ========= Mover a un ángulo fijo (por grados) ========= */
//   // Si tu librería NO tiene SetDegrees, convertimos grados→mm de carrera:
//   float cmd_mm = (TEST_DEG / 180.0f) * servo.stroke_mm_max;
//   if (cmd_mm < 0.0f) cmd_mm = 0.0f;
//   if (cmd_mm > servo.stroke_mm_max) cmd_mm = servo.stroke_mm_max;

//   if (!ServoSG90_SetByMillimeters(&servo, cmd_mm, smsg, sizeof(smsg))) {
//     printf("❌ Servo cmd: %s\n", smsg);
//   } else {
//     printf("[SERVO] target_deg=%.1f° (cmd_mm=%.2f)\n", TEST_DEG, cmd_mm);
//   }

//   /* ========= Bucle: leer láser y publicar ========= */
//   for (;;) {
//     uint16_t mm_raw = 0;
//     char     rmsg[64];
//     bool rok = VL53L0X_ReadDistanceMM(&laser, &mm_raw, rmsg, sizeof(rmsg));
//     if (rok) {
//       float d_sensor = (float)mm_raw;

//       // Publicación MQTT
//       publish_sensor_float("laser_mm", d_sensor);
//       publish_sensor_float("servo_deg", TEST_DEG);
//       publish_sensor_float("servo_cmd_mm", cmd_mm);

//       // Log por serial (para tu tabla de calibración)
//       printf("[VL53L0X] sensor=%.1f mm | servo=%.1f° | cmd_mm=%.2f\n",
//              d_sensor, TEST_DEG, cmd_mm);
//     } else {
//       printf("⚠️  VL53L0X_ReadDistanceMM falló: %s\n", rmsg);
//     }

//     vTaskDelayUntil(&last, pdMS_TO_TICKS((int)lrintf(1000.0f/LOOP_HZ)));
//   }
// }


// void app_main(void){
//   // --- Salud de I2C (tus utilidades) ---
//   pin_sanity_test();
//   i2c_force_release(GPIO_NUM_21, GPIO_NUM_22);
//   i2c_bus_health_once(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 100000);

//   // --- Láser: solo driver crudo VL53L0X_* ---
//   VL53L0X_Handle_t laser = {
//     .i2c_num    = I2C_NUM_0,
//     .pin_sda    = GPIO_NUM_21,
//     .pin_scl    = GPIO_NUM_22,
//     .i2c_clk_hz = 100000,
//     .pin_xshut  = GPIO_NUM_NC   // XSHUT alto en HW
//   };
//   char lmsg[96];
//   bool lok = VL53L0X_Init(&laser, lmsg, sizeof(lmsg));
//   printf("Laser Init: %s (ok=%d)\n", lmsg, lok);
//   if (!lok) { printf("⚠️  Verifica XSHUT a 3V3.\n"); while(1) vTaskDelay(pdMS_TO_TICKS(1000)); }

//   // --- Servo SG90 en GPIO19 (tu mapeo) ---
//   ServoSG90_Handle_t servo = {
//     .pin             = GPIO_NUM_19,
//     .channel         = LEDC_CHANNEL_0,
//     .timer           = LEDC_TIMER_0,
//     .speed_mode      = LEDC_LOW_SPEED_MODE,
//     .duty_resolution = LEDC_TIMER_16_BIT,
//     .freq_hz         = 50,
//     .us_min          = 600,
//     .us_max          = 2400,
//     .deg_min         = 0,
//     .deg_max         = 180,
//     .mm_per_rev      = 50.0f,    // 50 mm por 360°
//     .stroke_mm_max   = 25.0f,    // 0°=250mm → 180°≈225mm (sensor)
//     .zero_deg        = 0.0f
//   };
//   char smsg[96];
//   if (!ServoSG90_Init(&servo, smsg, sizeof(smsg))) {
//     printf("❌ Servo Init: %s\n", smsg);
//     while (1) vTaskDelay(pdMS_TO_TICKS(1000));
//   }
//   printf("Servo Init: %s\n", smsg);

//   // --- Red + MQTT como ya lo tenías ---
//   net_eg = xEventGroupCreate();
//   xTaskCreatePinnedToCore(net_task, "net", 4096, NULL, 5, NULL, 0);
//   wifi_init_sta();

//   // --- PID en mm del ACTUADOR (mm_act = ZERO_SENSOR_MM − mm_sensor) ---
//   PID_t pid_mm;
//   const float loop_hz = 20.0f;             // 20 Hz
//   const float Ts      = 1.0f / loop_hz;
//   const float Kp = 0.20f, Ki = 0.01f, Kd = 0.00f, alpha = 1.0f;
//   pid_init(&pid_mm, Kp, Ki, Kd, 0.0f, -1000.0f, 1000.0f, Ts, alpha);

//   // --- Geometría/constantes (láser a 100 mm de la punta) ---
// const float SENSOR_MIN_MM = (NEEDLE_MIN_MM - CAL_B) / CAL_A;
// const float SENSOR_MAX_MM = (NEEDLE_MAX_MM - CAL_B) / CAL_A;
//   const float ZERO_SENSOR_MM = 250.0f;  // servo=0° ⇒ lectura máx (punta más lejos)
//   const float ACT_MAX_MM     = 25.0f;   // carrera mecánica
//   const float DEADBAND_MM    = 1.0f;    // banda muerta
//   const float FINE_DEG_LIM   = 2.0f;    // microajuste ±2°

//   // Setpoint por defecto (centro del rango 230–250 mm)
//   float target_needle_mm = 150.0f;
//   float target_sensor_mm = (target_needle_mm - CAL_B) / CAL_A;
//   if (target_sensor_mm < SENSOR_MIN_MM) target_sensor_mm = SENSOR_MIN_MM;
//   if (target_sensor_mm > SENSOR_MAX_MM) target_sensor_mm = SENSOR_MAX_MM;

//   float target_act_mm = ZERO_SENSOR_MM - target_sensor_mm; // 0..25 mm
//   if (target_act_mm < 0.0f) target_act_mm = 0.0f;
//   if (target_act_mm > ACT_MAX_MM) target_act_mm = ACT_MAX_MM;

//   publish_sensor_float("sensor_target_mm",   target_sensor_mm);
//   publish_sensor_float("actuator_target_mm", target_act_mm);

//   // --- Loop de control: LÁSER CRUDO + PID + MQTT ---
//   TickType_t last = xTaskGetTickCount();
//   uint16_t mm = 0;
//   char rmsg[64];

//   while (1) {
//     // 1) Leer láser (crudo, sin filtros)
//     bool rok = VL53L0X_ReadDistanceMM(&laser, &mm, rmsg, sizeof(rmsg));
//     if (!rok) {
//       // si falla una muestra, solo espera el siguiente tick
//       vTaskDelayUntil(&last, pdMS_TO_TICKS((int)roundf(1000.0f/loop_hz)));
//       continue;
//     }
//     float d_sensor = (float)mm;
//     float needle_mm = CAL_A * d_sensor + CAL_B;
//     publish_sensor_float("gap_mm", needle_mm);
//     printf("[VL53L0X] sensor=%.1f mm | gap=%.1f mm\n", d_sensor, needle_mm);


//     // 2) (mantener setpoint dentro de 230–250 por seguridad)
//     if (target_sensor_mm < SENSOR_MIN_MM) target_sensor_mm = SENSOR_MIN_MM;
//     if (target_sensor_mm > SENSOR_MAX_MM) target_sensor_mm = SENSOR_MAX_MM;

//     // 3) Convertir a mm del actuador (0…25 mm)
//     float meas_act_mm = ZERO_SENSOR_MM - d_sensor;          // medida actual 0..25
//     target_act_mm     = ZERO_SENSOR_MM - target_sensor_mm;  // objetivo 0..25

//     if (target_act_mm < 0.0f) target_act_mm = 0.0f;
//     if (target_act_mm > ACT_MAX_MM) target_act_mm = ACT_MAX_MM;

//     publish_sensor_float("sensor_target_mm",   target_sensor_mm);
//     publish_sensor_float("actuator_target_mm", target_act_mm);

//     // 4) Deadband en sensor (equivalente a actuador)
//     if (fabsf(d_sensor - target_sensor_mm) <= DEADBAND_MM) {
//       vTaskDelayUntil(&last, pdMS_TO_TICKS((int)roundf(1000.0f/loop_hz)));
//       continue;
//     }

//     // 5) PID en mm del ACTUADOR → microajuste en grados
//     pid_set_setpoint(&pid_mm, target_act_mm);
//     float u_mm  = pid_compute(&pid_mm, meas_act_mm);
//     float u_deg = ServoSG90_MmToDeg(&servo, u_mm);

//     if (u_deg >  FINE_DEG_LIM) u_deg =  FINE_DEG_LIM;
//     if (u_deg < -FINE_DEG_LIM) u_deg = -FINE_DEG_LIM;

//     // 6) “apunta” al target_act_mm + microajuste fino
//     if (!ServoSG90_SetByMillimetersWithFine(&servo, target_act_mm, u_deg, FINE_DEG_LIM, smsg, sizeof(smsg))) {
//       printf("❌ Servo cmd: %s\n", smsg);
//     }

//     vTaskDelayUntil(&last, pdMS_TO_TICKS((int)roundf(1000.0f/loop_hz)));
//   }
// }



// void app_main(void){
//   // --- Diagnóstico/health del bus I2C con tus utilidades ---
//   pin_sanity_test();
//   i2c_force_release(GPIO_NUM_21, GPIO_NUM_22);
//   i2c_bus_health_once(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 100000);

//   // --- Láser (VL53L0X) ---
//   VL53L0X_Handle_t laser = {
//     .i2c_num    = I2C_NUM_0,
//     .pin_sda    = GPIO_NUM_21,
//     .pin_scl    = GPIO_NUM_22,
//     .i2c_clk_hz = 100000,
//     .pin_xshut  = GPIO_NUM_NC   // XSHUT amarrado a 3V3 en hardware
//   };
//   char lmsg[96];
//   bool lok = VL53L0X_Init(&laser, lmsg, sizeof(lmsg));
//   printf("Laser Init: %s (ok=%d)\n", lmsg, lok);
//   if (!lok) {
//     printf("⚠️  El láser no inicializó; verifica XSHUT a 3V3.\n");
//     while (1) vTaskDelay(pdMS_TO_TICKS(1000));
//   }

//   // --- Servo SG90 en GPIO19 (tus parámetros) ---
//   ServoSG90_Handle_t servo = {
//     .pin             = GPIO_NUM_19,
//     .channel         = LEDC_CHANNEL_0,
//     .timer           = LEDC_TIMER_0,
//     .speed_mode      = LEDC_LOW_SPEED_MODE,
//     .duty_resolution = LEDC_TIMER_16_BIT,
//     .freq_hz         = 50,
//     .us_min          = 600,
//     .us_max          = 2400,
//     .deg_min         = 0,
//     .deg_max         = 180,
//     .mm_per_rev      = 50.0f,     // ⬅️ tu nueva relación real: 50 mm por 360°
//     .stroke_mm_max   = 25.0f,     // ⬅️ carrera útil actual (confirmaste ~25 mm con 180°)
//     .zero_deg        = 0.0f
//   };
//   char smsg[96];
//   if (!ServoSG90_Init(&servo, smsg, sizeof(smsg))) {
//     printf("❌ Servo Init: %s\n", smsg);
//     while (1) vTaskDelay(pdMS_TO_TICKS(1000));
//   }
//   printf("Servo Init: %s\n", smsg);

//   // --- Red + MQTT (tus utilidades existentes) ---
//   net_eg = xEventGroupCreate();
//   xTaskCreatePinnedToCore(net_task, "net", 4096, NULL, 5, NULL, 0);
//   wifi_init_sta();

//   // --- PID en mm (usando tu librería PID) ---
//   PID_t pid_mm;
//   const float loop_hz = 20.0f;             // 20 Hz
//   const float Ts      = 1.0f / loop_hz;
//   // Ganancias prudentes en mm (ajustaremos si hace falta):
//   const float Kp = 0.20f, Ki = 0.01f, Kd = 0.00f, alpha = 1.0f;
//   pid_init(&pid_mm, Kp, Ki, Kd, 0.0f, -1000.0f, 1000.0f, Ts, alpha); // límites amplios (mm/s “virtuales”)

//   // Setpoint fijo por ahora (luego: NVS/GUI y modo manual)
//   float target_mm = 250.0f;

//   // Publica el target inicial (para tu GUI)
//   publish_sensor_float("servo_target_mm", target_mm);

//   // --- Bucle de control a 20 Hz ---
//   const float DEADBAND_MM = 1.0f;          // banda muerta de ±1 mm
//   const float FINE_DEG_LIM = 2.0f;         // micro-ajuste máximo ±2°
//   TickType_t last = xTaskGetTickCount();

//   while (1) {
//     // 1) Leer láser
//     uint16_t raw_mm = 0;
//     char rmsg[64];
//     bool rok = VL53L0X_ReadDistanceMM(&laser, &raw_mm, rmsg, sizeof(rmsg));
//     float d_mm = rok ? (float)raw_mm : NAN;

//     if (rok) {
//       printf("[VL53L0X] %.1f mm\n", d_mm);
//       publish_sensor_float("laser_mm", d_mm);
//     } else {
//       printf("[VL53L0X] %s\n", rmsg);
//       // si falla lectura, solo saltamos el control este ciclo
//       vTaskDelayUntil(&last, pdMS_TO_TICKS((int)roundf(1000.0f/loop_hz)));
//       continue;
//     }

//     // 2) Clampear/validar setpoint en la carrera disponible
//     if (target_mm < 0.0f) target_mm = 0.0f;
//     if (target_mm > servo.stroke_mm_max) target_mm = servo.stroke_mm_max;

//     // (re)publicar target por si luego lo enlazamos a GUI/NVS
//     publish_sensor_float("servo_target_mm", target_mm);

//     // 3) Control: si ya estamos dentro de la banda, no corrijas
//     float err_mm = target_mm - d_mm;
//     if (fabsf(err_mm) <= DEADBAND_MM) {
//       // puedes hacer un “settling” si quieres: fine muy pequeño hacia target
//       vTaskDelayUntil(&last, pdMS_TO_TICKS((int)roundf(1000.0f/loop_hz)));
//       continue;
//     }

//     // 4) PID en mm → salida en “mm-equivalente”, luego convierto a grados
//     pid_set_setpoint(&pid_mm, target_mm);
//     float u_mm = pid_compute(&pid_mm, d_mm);            // salida en mm (conceptual)
//     float u_deg = ServoSG90_MmToDeg(&servo, u_mm);      // convierto tu salida a grados

//     // 5) Limitar micro-ajuste para no golpear el mecanismo
//     if (u_deg >  FINE_DEG_LIM) u_deg =  FINE_DEG_LIM;
//     if (u_deg < -FINE_DEG_LIM) u_deg = -FINE_DEG_LIM;

//     // 6) Aplicar al servo: objetivo absoluto = target_mm, con fine = u_deg
//     if (!ServoSG90_SetByMillimetersWithFine(&servo, target_mm, u_deg, FINE_DEG_LIM, smsg, sizeof(smsg))) {
//       printf("❌ Servo cmd: %s\n", smsg);
//     }

//     // 7) Siguiente iteración a 20 Hz
//     vTaskDelayUntil(&last, pdMS_TO_TICKS((int)roundf(1000.0f/loop_hz)));
//   }
// }

// void app_main(void){
//   // --- Diagnóstico rápido I2C con tus utilidades ---
//   pin_sanity_test();
//   i2c_force_release(GPIO_NUM_21, GPIO_NUM_22);
//   i2c_bus_health_once(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 100000);

//   // --- Handle del láser (mismo que ya venías usando) ---
//   VL53L0X_Handle_t laser = {
//     .i2c_num    = I2C_NUM_0,
//     .pin_sda    = GPIO_NUM_21,
//     .pin_scl    = GPIO_NUM_22,
//     .i2c_clk_hz = 100000,
//     .pin_xshut  = GPIO_NUM_NC   // XSHUT amarrado a 3V3 en hardware
//   };

//   // --- Inicializa VL53L0X ---
//   char init_msg[96];
//   bool ok = VL53L0X_Init(&laser, init_msg, sizeof(init_msg));
//   printf("Laser Init: %s (ok=%d)\n", init_msg, ok);
//   if (!ok){
//     printf("⚠️  El láser no inicializó. Revisa VCC/GND/SDA/SCL/XSHUT.\n");
//     while (1) vTaskDelay(pdMS_TO_TICKS(1000));
//   }

//   // (Opcional) inicializa tu capa de filtrado si ya la tienes
//   // if (!Laser_init(&laser)) {
//   //   printf("⚠️  Laser_init (filtrado) falló; publicaré sin filtro.\n");
//   // }

//   // --- Red + MQTT con tus utilidades ---
//   net_eg = xEventGroupCreate();              // ← ya lo usas en net_task
//   xTaskCreatePinnedToCore(net_task, "net", 4096, NULL, 5, NULL, 0);  // arranca el task que hace mqtt_start()
//   wifi_init_sta();                           // conecta WiFi; al obtener IP, net_task llama mqtt_start()

//   // --- Bucle de lectura + publicación MQTT cada 200 ms ---
//   while (1){
//     uint16_t mm = 0;
//     char rmsg[64];
//     bool rok = VL53L0X_ReadDistanceMM(&laser, &mm, rmsg, sizeof(rmsg));
//     if (rok) {
//       printf("[VL53L0X] %u mm\n", mm);
//       publish_sensor_float("laser_mm", (float)mm);   // ← usa tu función existente
//     } else {
//       printf("[VL53L0X] %s\n", rmsg);
//     }
//     vTaskDelay(pdMS_TO_TICKS(200));
//   }
// }


// void app_main(void){
//   // ——— Diagnóstico rápido de pines I2C (tuyas) ———
//   pin_sanity_test();                                      // imprime niveles con/ sin PU
//   i2c_force_release(GPIO_NUM_21, GPIO_NUM_22);            // suelta el bus si quedó colgado
//   i2c_bus_health_once(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 100000); // scan one-shot @100k

//   // ——— Config VL53L0X (tu handle/struct) ———
//   VL53L0X_Handle_t laser = {
//     .i2c_num    = I2C_NUM_0,
//     .pin_sda    = GPIO_NUM_21,
//     .pin_scl    = GPIO_NUM_22,
//     .i2c_clk_hz = 100000,
//     .pin_xshut  = GPIO_NUM_NC    // XSHUT amarrado a 3V3 en tu PCB
//   };

//   // ——— Init del láser ———
//   char init_msg[96];
//   bool ok = VL53L0X_Init(&laser, init_msg, sizeof(init_msg));
//   printf("Laser Init: %s (ok=%d)\n", init_msg, ok);
//   if (!ok){
//     printf("⚠️  El láser no inicializó. Revisa VCC/GND/SDA/SCL/XSHUT.\n");
//     // si quieres: quedar en lazo de error con parpadeo o solo dormir
//     while (1) vTaskDelay(pdMS_TO_TICKS(1000));
//   }

//   // ——— Bucle de lectura cada 200 ms ———
//   while (1){
//     uint16_t mm = 0;
//     char rmsg[64];
//     bool rok = VL53L0X_ReadDistanceMM(&laser, &mm, rmsg, sizeof(rmsg));
//     if (rok) {
//       printf("[VL53L0X] %u mm\n", mm);
//       // (Siguiente paso) Para publicar por MQTT:
//       // publish_sensor_float("laser_mm", (float)mm);
//     } else {
//       printf("[VL53L0X] %s\n", rmsg);  // e.g. timeout/ no ready
//     }
//     vTaskDelay(pdMS_TO_TICKS(200));
//   }
// }


// void app_main(void) {
//   // --- Config del láser (I2C0 en GPIO21/22, 100 kHz, XSHUT sin usar) ---
//   VL53L0X_Handle_t laser = {
//     .i2c_num    = I2C_NUM_0,
//     .pin_sda    = GPIO_NUM_21,
//     .pin_scl    = GPIO_NUM_22,
//     .i2c_clk_hz = 100000,          // 100 kHz (estable para empezar)
//     .pin_xshut  = GPIO_NUM_NC      // XSHUT amarrado a 3V3 externamente
//   };

//   char msg[96];
//   bool ok = VL53L0X_Init(&laser, msg, sizeof(msg));
//   printf("Laser Init: %s (ok=%d)\n", msg, ok);
//   if (!ok) {
//     printf("⚠️  No se pudo inicializar el VL53L0X. Verifica SDA/SCL, VCC y XSHUT.\n");
//     while (1) vTaskDelay(pdMS_TO_TICKS(1000));
//   }

//   // --- Bucle de lectura cada 200 ms ---
//   while (1) {
//     uint16_t mm = 0;
//     char rmsg[64];
//     bool rok = VL53L0X_ReadDistanceMM(&laser, &mm, rmsg, sizeof(rmsg));
//     if (rok) {
//       printf("[VL53L0X] %u mm\n", mm);
//     } else {
//       // Mensaje de la librería (timeout, etc.)
//       printf("[VL53L0X] %s\n", rmsg);
//     }
//     vTaskDelay(pdMS_TO_TICKS(200));
//   }
// }

// void app_main(void) {
//   // Configura el servo (GPIO19 como hablamos)
//   char smsg[96];
//   ServoSG90_Handle_t servo = {
//     .pin             = GPIO_NUM_19,
//     .channel         = LEDC_CHANNEL_0,
//     .timer           = LEDC_TIMER_0,
//     .speed_mode      = LEDC_LOW_SPEED_MODE,
//     .duty_resolution = LEDC_TIMER_16_BIT,
//     .freq_hz         = 50,
//     .us_min          = 600,   // ajusta si tu SG90 lo pide (típico 500–2500)
//     .us_max          = 2400,
//     .deg_min         = 0,
//     .deg_max         = 180,

//     // Estos 3 ya sirven para los pasos siguientes, pero hoy no afectan:
//     .mm_per_rev      = 50.0f,
//     .stroke_mm_max   = 80.0f,
//     .zero_deg        = 0.0f
//   };

//   bool ok = ServoSG90_Init(&servo, smsg, sizeof(smsg));
//   printf("Servo Init: %s (ok=%d)\n", smsg, ok);
//   if (!ok) return;

//     ServoSG90_SetAngleDeg(&servo, 0.0f, smsg, sizeof(smsg));
//     printf("→ %s\n", smsg);
//     vTaskDelay(pdMS_TO_TICKS(1000));

//     ServoSG90_SetZeroDeg(&servo, 0);
//     vTaskDelay(pdMS_TO_TICKS(1000));

//     ServoSG90_SetByMillimeters(&servo, 160.0f, smsg, sizeof(smsg));
//     printf("→ %s\n", smsg);
//     vTaskDelay(pdMS_TO_TICKS(1000));

//   // Mantén la tarea viva (opcional: evitar que el RTOS termine el main)
//   while (1) vTaskDelay(pdMS_TO_TICKS(1000));
//}

// void app_main(void){
//   pin_sanity_test();
//   log_line("[BOOT] stage A: Peripherals (pot/stepper/servo)");
//   Potenciometro_init();
//   stepper_init(&g_motor, STEP_PIN, DIR_PIN, EN_PIN, 10*1000000);
//   stepper_enable(&g_motor, true);
//   net_eg = xEventGroupCreate();
//   ServoSG90_Init(&servo, smsg, sizeof(smsg));
//   log_line("Servo Init: %s", smsg);

//   // log_line("[BOOT] stage B: Syringe bootstrap");
//   // Syringe_Init();
//   // Syringe_SetAddress(0);
//   // Syringe_ExitSafeMode();
//   // vTaskDelay(pdMS_TO_TICKS(120));
//   // uart_flush_input(SYRINGE_UART);
  
//   int sda = -1, scl = -1;
//   gpio_set_direction(GPIO_NUM_21, GPIO_MODE_INPUT);
//   gpio_set_direction(GPIO_NUM_22, GPIO_MODE_INPUT);
//   gpio_pullup_en(GPIO_NUM_21); gpio_pullup_en(GPIO_NUM_22);
//   vTaskDelay(pdMS_TO_TICKS(1));
//   sda = gpio_get_level(GPIO_NUM_21);
//   scl = gpio_get_level(GPIO_NUM_22);
//   ESP_LOGI("PRE_HEALTH", "Idle pre-health: SDA=%d SCL=%d", sda, scl);

//   i2c_force_release(GPIO_NUM_21, GPIO_NUM_22);
//   log_line("[BOOT] stage C: I2C health (one-shot, no driver kept)");
//   i2c_bus_health_once(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 100000);
//   i2c_force_release(GPIO_NUM_21, GPIO_NUM_22);
//   log_line("[BOOT] stage D: Laser init");
//   bool ok = VL53L0X_Init(&laser, m1, sizeof(m1));
//   log_line("Laser Init 0: %s (ok=%d)", m1, ok);

//   // Si quieres filtro + tarea del láser:
//   if (!Laser_init(&laser)) {
//     log_line("❌ Laser_init (filtrado) falló; SPD no disponible.");
//   }

//     log_line("[BOOT] stage G: Net/MQTT task");
//   xTaskCreatePinnedToCore(net_task, "net", 4096, NULL, 5, NULL, 0);

//   log_line("[BOOT] stage E: WiFi init");
//   wifi_init_sta();

//   log_line("[BOOT] stage F: Mapas + params + servo demo");
//   construir_mapas();
//   params_load_from_nvs();

//   ServoSG90_SetByMillimeters(&servo, 40.0f, smsg, sizeof(smsg));
//   printf("CMD mm->servo: %s\n", smsg);
//   ServoSG90_SetByMillimetersWithFine(&servo, 40.0f, +2.0f, 10.0f, smsg, sizeof(smsg));
//   printf("CMD mm+fine: %s\n", smsg);

//   // Handlers
//   registrar_handler_sensor("soff", sensor_mock_timed_hold_2s);
//   registrar_handler_sensor("voff", sensor_mock_timed_hold_2s);
//   registrar_handler_sensor("sh",   sensor_mock_timed_hold_2s);
//   registrar_handler_sensor("mh",   mh_handler);
//   registrar_handler_sensor("mpd",  mpd_handler);
//   registrar_handler_sensor("ok232", llamar_ok232_handler);

//   reset_to_stage0();

//   // Tareas
//   xTaskCreate(grafcet_task,    "grafcet",   4096, NULL, 3, NULL);
//   xTaskCreate(comando_task,    "comando",   4096, NULL, 5, NULL);
//   xTaskCreate(telemetry_task,  "telemetry", 3072, NULL, 3, NULL);
//   xTaskCreate(pid_control_task,"pid_ctrl",  4096, NULL, 6, NULL);

//   log_line("[BOOT] stage H: app_main done");





