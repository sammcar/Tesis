// grafcet.c — Port de tu Python a C (ESP-IDF + FreeRTOS)
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
#include <inttypes.h>  // arriba del archivo
// --- Agrega arriba ---
#include <stdarg.h>
#include <math.h>
#include "esp_timer.h"
#include "cJSON.h"         // viene con ESP-IDF
#include "nvs.h"


// === AJUSTA ESTOS VALORES ===
#define WIFI_SSID   "SAMT"
#define WIFI_PASS   "MINAIOT1"
#define MQTT_URI  "mqtt://192.168.1.6:1883"   // OJO: mqtt:// (no mqtts://)
#define MQTT_USER NULL                        // estamos en allow_anonymous
#define MQTT_PASS NULL
#define NET_BIT_IP (1<<0)
#define DEVICE_ID   "esp32-demo"                // Debe coincidir con la GUI (tesis/<DEVICE_ID>/...)

#define NUM_ETAPAS    15
#define NUM_SENSORES  12
#define NUM_SALIDAS   12

#define BUF_SIZE      256
#define TICK_MS       500     // igual a tu sleep(0.5) del Python
#define TIMEOUT_MANUAL_RESCATE 15

typedef struct {
  bool  collectorOn;
  float rpm;        // rpm del variador (si collectorOn = true)
  float kv;         // kV de la fuente HV
  float mlmin;      // caudal jeringa
  float syringeD;   // diámetro jeringa (mm)
  float gap;        // distancia (mm)
} Params;

static Params g_params = {
  .collectorOn = false,
  .rpm = 0,
  .kv = 0,
  .mlmin = 0,
  .syringeD = 0,
  .gap = 0,
};


static void mqtt_start(void);
static void process_command_line(char* line);
static void log_line(const char* fmt, ...);
static void publish_fault(const char* code, const char* message);
static void clear_fault(void);

typedef enum { MODO_AUTOMATICO, MODO_CICLO, MODO_ETAPA, MODO_MANUAL } Modo;

// --- Sensores “reales” (bool + timer + handler) ---
typedef bool (*SensorHandler)(int s_idx);   // devuelve true si el sensor confirmó

/* === Listas (idénticas a tu Python) === */
static const char* SENSORES[NUM_SENSORES] = {
  "sh","mh","ok232","ok485","fcl","vd","spd","mpd","son","okmon","soff","voff"
};

static const char* SALIDAS[NUM_SALIDAS] = {
  "SERVOHOME","MOTORHOME","P232","P485","OKCL","VACON","SERVOMOVE","MOTORMOVE","SYON","MON","SYOFF","VACOFF"
};

/* SENSOR -> SALIDA (índices, -1 si no aplica) */
static int SENSOR_TO_SALIDA[NUM_SENSORES];
static EventGroupHandle_t net_eg;

// Tiempo desde que el sensor fue "armado" en la etapa
static time_t sensor_armed_since[NUM_SENSORES] = {0};

// Prototipos de mocks
static bool sensor_mock_timed_hold_2s(int s_idx);
static bool sensor_mock_timed_hold_1s(int s_idx);

/* ETAPA -> SALIDAS (terminar cada fila con NULL) */
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

/* Tiempos de falla por SENSOR (segundos) */
static int TIEMPO_FALLA_SENSOR[NUM_SENSORES] = {
  /* sh   */ 20,  /* mh    */ 20,  /* ok232 */ 5,   /* ok485 */ 5,
  /* fcl  */ 9999,/* vd    */ 20,  /* spd   */ 20,  /* mpd   */ 20,
  /* son  */ 15,  /* okmon */ 9999,/* soff  */ 15,  /* voff  */ 20
};

/* ETAPA -> sensores esperados para fallo (índices, terminar con -1) */
static int ETAPA_SENSORES_FALLO[NUM_ETAPAS][6];

/* === Estado === */
static bool etapas[NUM_ETAPAS] = { 0 };              // etapa 0 activa
static bool sensores[NUM_SENSORES] = { 0 };          // lectura instante
static bool latch[NUM_SENSORES]    = { 0 };          // “memoria” de sensor
static bool salidas[NUM_SALIDAS]   = { 0 };          // salidas activas
static bool   sensor_active[NUM_SENSORES]  = { 0 };  // armado en etapa
static time_t sensor_deadline[NUM_SENSORES] = { 0 }; // fin de tiempo para confirmar
static SensorHandler SENSOR_FN[NUM_SENSORES] = { 0 }; // callbacks por sensor

static bool pausa=false, fallo=false;
static bool start_flag=false, clc=false, se_flag=false;  // start es “palanca” (toggle)
static int  etapa_actual = 0;
static Modo modo = MODO_AUTOMATICO;
static bool colectorEN = false;

/* Fallo por etapa */
static time_t tiempo_fallo_etapa = 0;
static int    etapa_fallo_idx = -1;

/* Manual */
static const char* salida_manual = NULL;
static time_t inicio_manual = 0;
static time_t deadline_manual = 0;

/* Para imprimir sólo flancos en salidas cuando no cambia de etapa */
static bool salidas_prev[NUM_SALIDAS] = { 0 };

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
  nvs_commit(h);
  nvs_close(h);
}

static void net_task(void *arg){
  for(;;){
    xEventGroupWaitBits(net_eg, NET_BIT_IP, pdTRUE, pdFALSE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(100));   // deja respirar al loop de eventos
    mqtt_start();                     // ahora en contexto propio, no en el handler
  }
}

/* === Utilidades de índices === */
static int idx_salida(const char* nombre){
  for(int i=0;i<NUM_SALIDAS;i++) if(SALIDAS[i] && strcmp(SALIDAS[i],nombre)==0) return i;
  return -1;
}
static int idx_sensor(const char* nombre){
  for(int i=0;i<NUM_SENSORES;i++) if(SENSORES[i] && strcmp(SENSORES[i],nombre)==0) return i;
  return -1;
}

// Devuelve la salida asociada a un sensor (o NULL si no hay)
static const char* salida_de_sensor(int s_idx){
  int out = (s_idx>=0 && s_idx<NUM_SENSORES) ? SENSOR_TO_SALIDA[s_idx] : -1;
  return (out>=0) ? SALIDAS[out] : NULL;
}

// Mock por defecto: si la salida asociada está presente en la etapa y activa, “confirma” el sensor.
static bool sensor_mock_handler(int s_idx){
  (void)s_idx;
  return false;    // hoy: basta con que la salida esté activa; luego reemplazas por IO real
}

// Permite registrar un handler real por nombre de sensor
static void registrar_handler_sensor(const char* sensor_nombre, SensorHandler fn){
  int s = idx_sensor(sensor_nombre);
  if(s>=0) SENSOR_FN[s] = fn ? fn : sensor_mock_handler;
}

/* === Inicialización de mapas desde nombres (sin errores de “dedo”) === */
static void construir_mapas(void){
  // SENSOR -> SALIDA
  // {
  //   "sh":"SERVOHOME","ok485":"P485","spd":"SERVOMOVE","okmon":"MON",
  //   "mh":"MOTORHOME","fcl":"OKCL","mpd":"MOTORMOVE","soff":"SYOFF",
  //   "ok232":"P232","vd":"VACON","son":"SYON","voff":"VACOFF"
  // }
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

  // ETAPA -> sensores de fallo (según tu dict)
  // Reemplazamos por índices (terminar cada fila con -1)
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

/* === Helpers de etapa/salidas === */
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

/* === reset_to_stage0 === */
static void reset_to_stage0(void){
  memset(etapas,0,sizeof(etapas));
  etapas[0]=true; etapa_actual=0;
  fallo=false; // pausa se preserva, start (palanca) se preserva

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

/* === Fallos por sensor (por etapa) === */
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

/* === Salida (con impresión sólo en cambio de etapa o flancos) === */
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

/* === Lógica de liberación de latches (como tu Python) === */
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

/* === Modo manual (incluye RFALLA_MANUAL) === */
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
    if(s_idx>=0){
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

/* === Transiciones (evo): copia de tu Python === */
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

// ---------- WIFI + MQTT ----------
static esp_mqtt_client_handle_t mqtt = NULL;

static void publish_params_active(const char* reason) {
  if (!mqtt) return;
  cJSON* root = cJSON_CreateObject();
  cJSON_AddBoolToObject(root, "collectorOn", g_params.collectorOn);
  cJSON_AddNumberToObject(root, "rpm",       g_params.rpm);
  cJSON_AddNumberToObject(root, "kv",        g_params.kv);
  cJSON_AddNumberToObject(root, "mlmin",     g_params.mlmin);
  cJSON_AddNumberToObject(root, "syringeD",  g_params.syringeD);
  cJSON_AddNumberToObject(root, "gap",       g_params.gap);
  if (reason) cJSON_AddStringToObject(root, "reason", reason);

  char *json = cJSON_PrintUnformatted(root);
  if (json) {
    esp_mqtt_client_publish(mqtt, "tesis/" DEVICE_ID "/params/active", json, 0, 1, 1); // qos=1, retain=1
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

        bool apply = cJSON_IsBool(j_apply) ? cJSON_IsTrue(j_apply) : false;

        cJSON_Delete(root);

        if (changed) {
          g_params = newp;

          // 🔗 Enlaza parámetros con tu lógica actual (opcionales)
          // - Colector: sincroniza con tu flag 'colectorEN'
          colectorEN = g_params.collectorOn;

          // - Aquí podrías escribir setpoints a drivers reales (variador/HV/bomba)
          //   por ahora sólo log:
          log_line("[PARAMS] collector=%s rpm=%.2f kv=%.2f mlmin=%.3f syringeD=%.3f gap=%.3f\n",
            g_params.collectorOn ? "ON" : "OFF",
            g_params.rpm, g_params.kv, g_params.mlmin, g_params.syringeD, g_params.gap);

          // Guarda en NVS si 'apply=true'
          if (apply) {
            params_save_to_nvs(&g_params);
            log_line("[PARAMS] guardados en NVS\n");
          }

          // Publica snapshot para la GUI
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

// Reemplaza la parte de mqtt_start() donde armaras 'cfg' por esto:
static void mqtt_start(void){
if (mqtt) return;

const esp_mqtt_client_config_t cfg = {
  .broker.address.uri = MQTT_URI,
  .session.protocol_ver = MQTT_PROTOCOL_V_3_1_1,  // v3.1.1
};

mqtt = esp_mqtt_client_init(&cfg);
esp_mqtt_client_register_event(mqtt, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
printf("[MQTT] Conectando a mqtt://192.168.1.6:1883 (tcp)\n");
ESP_ERROR_CHECK( esp_mqtt_client_start(mqtt) );
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START){
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED){
    printf("[WIFI] desconectado, reintentando...\n");
    esp_wifi_connect();
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP){
    printf("[WIFI] IP obtenida\n");
    xEventGroupSetBits(net_eg, NET_BIT_IP);
  }
}

// Publica una línea en log/text y también la imprime por serial
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

// Publica una falla en JSON (retain=1 para que la GUI la vea aunque se conecte tarde)
static void publish_fault(const char* code, const char* message) {
  if (!mqtt) return;
  char json[256];
  snprintf(json, sizeof(json),
           "{\"code\":\"%s\",\"message\":\"%s\",\"etapa\":%d}",
           code ? code : "FALLA", message ? message : "Error detectado", etapa_actual);
  esp_mqtt_client_publish(mqtt, "tesis/" DEVICE_ID "/fault/json", json, 0, 1, 1);
}

static void telemetry_task(void *arg) {
  const int period_ms = 150;  // 0.15 s → hace match con tu dtSec=0.15 en la GUI
  for (;;) {
    double t = esp_timer_get_time() / 1000000.0;

    // Mock: voltaje 20±10 kV, distancia 150±50 mm
    float voltage_kv  = 20.0f + 10.0f * sinf((float)(t / 0.8));
    float distance_mm = 150.0f + 50.0f * cosf((float)(t / 0.9));

    publish_sensor_float("voltage_kv",  voltage_kv);
    publish_sensor_float("distance_mm", distance_mm);

    vTaskDelay(pdMS_TO_TICKS(period_ms));
  }
}

// Limpia la falla publicada (por si quieres “borrar” el popup para futuros clientes)
static void clear_fault(void) {
  if (!mqtt) return;
  // Publica un objeto “empty” con retain para que el último estado no sea una falla
  esp_mqtt_client_publish(mqtt, "tesis/" DEVICE_ID "/fault/json", "{}", 0, 1, 1);
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

  log_line("[WIFI] iniciando STA a SSID='%s'\n", WIFI_SSID);
}

/* === Tareas === */
static void grafcet_task(void*){
  log_line("⚙️ Simulador GRAFCET listo.\n");
  aplicar_salidas_de_etapa(); // imprime etapa 0
  for(;;){
    if(!pausa && !fallo){
      if(modo==MODO_MANUAL){
        modo_manual_tick();
      } else {
        evo();
        sensores_tick_en_etapa(); // Verifica/ejecuta funciones de sensores para la etapa actual
        verificar_fallos();
        aplicar_salidas_de_etapa();
        liberar_latches_si_corresponde();
      }
    } else {
      tiempo_fallo_etapa = time(NULL); // “congela” contadores de fallo mientras está pausado
    }
    vTaskDelay(pdMS_TO_TICKS(TICK_MS));
  }
}

/* Comandos por UART0 (monitor serie):
   - start (toggle)
   - clc
   - se
   - modo automatico|ciclo|etapa|manual
   - paro / ok
   - reset
   - rfalla  (limpia falla y reset_to_stage0)
   - colector on | colector off
   - nombres de sensores (sh, mh, ok232, ...) → hacen latch si la salida asociada está activa
   - en modo manual: nombres de salidas (SERVOHOME, P485, ...) o RFALLA_MANUAL
*/

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

// Helper genérico: confirma si la salida asociada está activa durante hold_s segundos
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

// Wrappers concretos (puedes crear más con distintos tiempos)
static bool sensor_mock_timed_hold_2s(int s_idx) {
  return sensor_mock_timed_hold_generic(s_idx, 2);
}
static bool sensor_mock_timed_hold_1s(int s_idx) {
  return sensor_mock_timed_hold_generic(s_idx, 1);
}


/* === app_main === */
void app_main(void){
  net_eg = xEventGroupCreate();
  xTaskCreatePinnedToCore(net_task, "net", 4096, NULL, 5, NULL, 0);

  wifi_init_sta();
  construir_mapas();

  registrar_handler_sensor("soff", sensor_mock_timed_hold_2s);
  registrar_handler_sensor("voff", sensor_mock_timed_hold_2s);
  registrar_handler_sensor("mh", sensor_mock_timed_hold_2s);
  registrar_handler_sensor("sh", sensor_mock_timed_hold_2s);

  // ⬅️ NUEVO: carga params persistidos (si existen)
  params_load_from_nvs();

  reset_to_stage0();
  xTaskCreate(grafcet_task,  "grafcet",  4096, NULL, 5, NULL);
  xTaskCreate(comando_task,  "comando",  4096, NULL, 5, NULL);
  xTaskCreate(telemetry_task,"telemetry",3072, NULL, 4, NULL);
}



