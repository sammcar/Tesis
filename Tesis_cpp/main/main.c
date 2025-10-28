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

#define SPD_DEADBAND_MM      1.0f      // banda para considerar "en target"
#define SPD_HOLD_MS          800       // tiempo estable para confirmar
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

// STRUCTS

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

ServoSG90_Handle_t servo = {
  .pin            = GPIO_NUM_19,
  .channel        = LEDC_CHANNEL_0,
  .timer          = LEDC_TIMER_0,
  .speed_mode     = LEDC_LOW_SPEED_MODE,
  .duty_resolution= LEDC_TIMER_16_BIT,
  .freq_hz        = 50,
  .us_min         = 600,    // ajusta si tu SG90 lo pide (500–2500 típico)
  .us_max         = 2400,
  .deg_min        = 0,
  .deg_max        = 180,    // OJO: si tu servo NO es 360°, no podrás cubrir 95 mm
  .mm_per_rev     = 52.5f,
  .stroke_mm_max  = 95.0f,
  .zero_deg       = 0.0f     // calibra tu “home” mecánico aquí
};

VL53L0X_Handle_t laser = {
  .i2c_num   = I2C_NUM_0,
  .pin_sda   = GPIO_NUM_21,
  .pin_scl   = GPIO_NUM_22,
  .i2c_clk_hz= 100000,      // baja a 100 kHz para diagnóstico
  .pin_xshut = GPIO_NUM_NC
    };
    
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

// VARIABLES AZULES

static volatile bool   g_pid_enabled = false;
static volatile float  g_target_deg  = 0.0f;
static volatile bool g_motor_moving = false;
char smsg[96];
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
  const int period_ms = 1000;
  int acc_ms_pot = 0;
  int acc_ms_lsr = 0;

  for (;;) {
    // -------- Potenciómetro / Stepper --------
    float theta = Potenciometro_get_grados_filtrado();
    theta = clampf(theta, 0.0f, 3600.0f);

    float kv_now     = deg_to_kv_off(theta, g_params.kvFS, HOME_DEG_OFFSET);
    float target_deg = kv_to_deg_off(g_params.kv, g_params.kvFS, HOME_DEG_OFFSET);

    // MQTT siempre
    publish_sensor_float("angle_deg",  theta);
    publish_sensor_float("voltage_kv", kv_now);
    publish_sensor_float("target_deg", target_deg);
    publish_sensor_float("kv_target",  g_params.kv);
    publish_sensor_float("kv_fs",      g_params.kvFS);

    // LOG solo si el motor (stepper) se mueve
    if (g_motor_moving) {
      acc_ms_pot += period_ms;
      if (acc_ms_pot >= 1000) {
        acc_ms_pot = 0;
        log_line("[pot] θ=%.2f° | KV=%.3f", theta, kv_now);
      }
    } else {
      acc_ms_pot = 0;
    }

    // -------- Láser / Servo --------
    float d_mm_now = Laser_get_mm_filtrado();
    float servo_mm_sp = (modo==MODO_MANUAL) ? SPD_MANUAL_MM_DEFAULT : g_params.gap;

    // MQTT siempre (para la gráfica)
    publish_sensor_float("laser_mm",        d_mm_now);
    publish_sensor_float("servo_target_mm", clamp_mm_target(servo_mm_sp, &servo));

    // LOG solo si el servo “está moviéndose”
    if (g_servo_moving) {
      acc_ms_lsr += period_ms;
      if (acc_ms_lsr >= 1000) {
        acc_ms_lsr = 0;
        log_line("[laser] d=%.2f mm | d_obj=%.2f", d_mm_now, servo_mm_sp);
      }
    } else {
      acc_ms_lsr = 0;
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
  static fsm_t st = ST_IDLE;
  static int64_t t_inband_us = 0;

  float theta = Potenciometro_get_grados_filtrado();
  theta = clampf(theta, 0.0f, 3600.0f);

  switch (st) {
    case ST_IDLE: {
      float target = (modo==MODO_MANUAL)
               ? 1440.0f
               : kv_to_deg_off(g_params.kv, g_params.kvFS, HOME_DEG_OFFSET);
      g_target_deg  = target;
      g_pid_enabled = true;
      t_inband_us   = 0;
      log_line("[mpd] init %s → θ_obj=%.2f° (kv=%.3f kvFS=%.3f)",
               (modo==MODO_MANUAL? "MANUAL":"AUTO"),
               g_target_deg, g_params.kv, g_params.kvFS);
      st = ST_TRACK;
      return false;
    }

    case ST_TRACK: {
      // En manual mantenemos SIEMPRE 360°, ignorando kv
      float desired = (modo==MODO_MANUAL)
                ? 1440.0f
                : kv_to_deg_off(g_params.kv, g_params.kvFS, HOME_DEG_OFFSET);

      if (fabsf(desired - g_target_deg) > 0.05f) {
        g_target_deg = desired;
      }
      if (fabsf(theta - g_target_deg) <= PID_DEADBAND_DEG){
        t_inband_us = esp_timer_get_time();
        log_line("[mpd] hold start");
        st = ST_HOLD;
      }
      return false;
    }

    case ST_DONE:
      st = ST_IDLE;
      return true;

    case ST_SETPOINT: {            // ← evita -Wswitch
      st = ST_TRACK;
      return false;
    }

    default:                     // ← defensa futura
      st = ST_IDLE;
      return false;

    case ST_HOLD: {
      if (fabsf(theta - g_target_deg) > PID_DEADBAND_DEG){
        st = ST_TRACK; return false;
      }
      int64_t now = esp_timer_get_time();
      if ((now - t_inband_us) >= 1000LL*1000LL) { // 1.0 s
        g_pid_enabled = false;
        stepper_stop(&g_motor);
        log_line("✅ [mpd] OK @ θ=%.2f° (V=%.3f kV)", theta, deg_to_kv(theta));
        st = ST_DONE;
        return true;
      }
      return false;
    }
  }
  return false;
}

static bool spd_handler(int s_idx){
  (void)s_idx;
  static fsm_t   st = ST_IDLE;
  static int64_t t_hold_us = 0;
  static float   last_cmd  = -1.0f;   // recuerda última consigna enviada
  float d_mm = Laser_get_mm_filtrado();

  switch (st) {

    case ST_IDLE: {
      float target_mm = (modo == MODO_MANUAL)
          ? SPD_MANUAL_MM_DEFAULT
          : g_params.gap;

      target_mm = clamp_mm_target(target_mm, &servo);

      // Early-exit: ya en banda → HOLD
      if (fabsf(d_mm - target_mm) <= SPD_DEADBAND_MM){
        g_servo_moving = false;                // no hubo movimiento
        log_line("[spd] ya en target (%.2f mm) → HOLD\n", d_mm);
        t_hold_us = esp_timer_get_time();
        st = ST_HOLD;
        return false;
      }

      // Comando principal
      if (!ServoSG90_SetByMillimeters(&servo, target_mm, smsg, sizeof(smsg))) {
        log_line("❌ [spd] servo fail: %s\n", smsg);
        st = ST_IDLE; // reintento próximo tick
        g_servo_moving = false;
        return false;
      }

      last_cmd = target_mm;
      g_servo_moving = true;           // ← ¡ahora sí está “moviendo”!
      g_spd_moved_once = true;         // ← bandera para SH (se pidió movimiento)
      log_line("[spd] init %s → d_obj=%.2f mm (med=%.2f mm)",
               (modo==MODO_MANUAL? "MANUAL":"AUTO"), target_mm, d_mm);
      st = ST_TRACK;
      return false;
    }

    case ST_TRACK: {
      float target_mm = (modo == MODO_MANUAL)
          ? SPD_MANUAL_MM_DEFAULT
          : g_params.gap;
      target_mm = clamp_mm_target(target_mm, &servo);

      // Si cambió la consigna por GUI/NVS, re-ordenamos
      if (fabsf(target_mm - last_cmd) > 0.05f) {
        if (!ServoSG90_SetByMillimeters(&servo, target_mm, smsg, sizeof(smsg))) {
          log_line("❌ [spd] servo fail: %s\n", smsg);
          g_servo_moving = false;
          return false;
        }
        last_cmd = target_mm;
        g_servo_moving = true;
      }

      d_mm = Laser_get_mm_filtrado();
      if (fabsf(d_mm - target_mm) <= SPD_DEADBAND_MM){
        // Micro-ajuste fino
        float err_mm = target_mm - d_mm;
        float fine_deg = ServoSG90_MmToDeg(&servo, err_mm);
        if (fabsf(fine_deg) > 0.1f) {
          if (fabsf(fine_deg) > SPD_FINE_DEG_LIMIT)
            fine_deg = (fine_deg > 0 ? SPD_FINE_DEG_LIMIT : -SPD_FINE_DEG_LIMIT);
          ServoSG90_SetByMillimetersWithFine(&servo, target_mm, fine_deg, SPD_FINE_DEG_LIMIT, smsg, sizeof(smsg));
        }

        t_hold_us = esp_timer_get_time();
        log_line("[spd] hold start @ d=%.2f mm (obj=%.2f)\n", d_mm, target_mm);
        st = ST_HOLD;
      }
      return false;
    }

    case ST_HOLD: {
      float target_mm = (modo == MODO_MANUAL)
          ? SPD_MANUAL_MM_DEFAULT
          : g_params.gap;
      target_mm = clamp_mm_target(target_mm, &servo);

      d_mm = Laser_get_mm_filtrado();
      if (fabsf(d_mm - target_mm) > SPD_DEADBAND_MM){
        g_servo_moving = true;    // vuelve a corregir
        st = ST_TRACK; 
        return false;
      }

      int64_t now = esp_timer_get_time();
      if ((now - t_hold_us) >= (int64_t)SPD_HOLD_MS * 1000LL) {
        g_servo_moving = false;   // llegó y quedó estable
        log_line("✅ [spd] OK @ d=%.2f mm\n", d_mm);
        st = ST_DONE;
        return true;
      }
      return false;
    }

    case ST_SETPOINT:
      st = ST_TRACK; return false;

    case ST_DONE:
      st = ST_IDLE;
      return true;

    default:
      st = ST_IDLE;
      g_servo_moving = false;
      return false;
  }
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


void app_main(void){
  pin_sanity_test();
  log_line("[BOOT] stage A: Peripherals (pot/stepper/servo)");
  Potenciometro_init();
  stepper_init(&g_motor, STEP_PIN, DIR_PIN, EN_PIN, 10*1000000);
  stepper_enable(&g_motor, true);
  net_eg = xEventGroupCreate();
  ServoSG90_Init(&servo, smsg, sizeof(smsg));
  log_line("Servo Init: %s", smsg);

  // log_line("[BOOT] stage B: Syringe bootstrap");
  // Syringe_Init();
  // Syringe_SetAddress(0);
  // Syringe_ExitSafeMode();
  // vTaskDelay(pdMS_TO_TICKS(120));
  // uart_flush_input(SYRINGE_UART);
  
  int sda = -1, scl = -1;
  gpio_set_direction(GPIO_NUM_21, GPIO_MODE_INPUT);
  gpio_set_direction(GPIO_NUM_22, GPIO_MODE_INPUT);
  gpio_pullup_en(GPIO_NUM_21); gpio_pullup_en(GPIO_NUM_22);
  vTaskDelay(pdMS_TO_TICKS(1));
  sda = gpio_get_level(GPIO_NUM_21);
  scl = gpio_get_level(GPIO_NUM_22);
  ESP_LOGI("PRE_HEALTH", "Idle pre-health: SDA=%d SCL=%d", sda, scl);

  i2c_force_release(GPIO_NUM_21, GPIO_NUM_22);
  log_line("[BOOT] stage C: I2C health (one-shot, no driver kept)");
  i2c_bus_health_once(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 100000);
  i2c_force_release(GPIO_NUM_21, GPIO_NUM_22);
  log_line("[BOOT] stage D: Laser init");
  bool ok = VL53L0X_Init(&laser, m1, sizeof(m1));
  log_line("Laser Init 0: %s (ok=%d)", m1, ok);

  // Si quieres filtro + tarea del láser:
  if (!Laser_init(&laser)) {
    log_line("❌ Laser_init (filtrado) falló; SPD no disponible.");
  }

    log_line("[BOOT] stage G: Net/MQTT task");
  xTaskCreatePinnedToCore(net_task, "net", 4096, NULL, 5, NULL, 0);

  log_line("[BOOT] stage E: WiFi init");
  wifi_init_sta();

  log_line("[BOOT] stage F: Mapas + params + servo demo");
  construir_mapas();
  params_load_from_nvs();

  ServoSG90_SetByMillimeters(&servo, 40.0f, smsg, sizeof(smsg));
  printf("CMD mm->servo: %s\n", smsg);
  ServoSG90_SetByMillimetersWithFine(&servo, 40.0f, +2.0f, 10.0f, smsg, sizeof(smsg));
  printf("CMD mm+fine: %s\n", smsg);

  // Handlers
  registrar_handler_sensor("soff", sensor_mock_timed_hold_2s);
  registrar_handler_sensor("voff", sensor_mock_timed_hold_2s);
  registrar_handler_sensor("sh",   sensor_mock_timed_hold_2s);
  registrar_handler_sensor("mh",   mh_handler);
  registrar_handler_sensor("mpd",  mpd_handler);
  registrar_handler_sensor("ok232", llamar_ok232_handler);

  reset_to_stage0();

  // Tareas
  xTaskCreate(grafcet_task,    "grafcet",   4096, NULL, 3, NULL);
  xTaskCreate(comando_task,    "comando",   4096, NULL, 5, NULL);
  xTaskCreate(telemetry_task,  "telemetry", 3072, NULL, 3, NULL);
  xTaskCreate(pid_control_task,"pid_ctrl",  4096, NULL, 6, NULL);

  log_line("[BOOT] stage H: app_main done");
}





