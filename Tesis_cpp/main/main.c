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

#define NUM_ETAPAS    15
#define NUM_SENSORES  12
#define NUM_SALIDAS   12

#define BUF_SIZE      256
#define TICK_MS       500     // igual a tu sleep(0.5) del Python
#define TIMEOUT_MANUAL_RESCATE 15

typedef enum { MODO_AUTOMATICO, MODO_CICLO, MODO_ETAPA, MODO_MANUAL } Modo;

/* === Listas (idénticas a tu Python) === */
static const char* SENSORES[NUM_SENSORES] = {
  "sh","mh","ok232","ok485","fcl","vd","spd","mpd","son","okmon","soff","voff"
};

static const char* SALIDAS[NUM_SALIDAS] = {
  "SERVOHOME","MOTORHOME","P232","P485","OKCL","VACON","SERVOMOVE","MOTORMOVE","SYON","MON","SYOFF","VACOFF"
};

/* SENSOR -> SALIDA (índices, -1 si no aplica) */
static int SENSOR_TO_SALIDA[NUM_SENSORES];

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


/* — Nota: rellenamos ETAPA_SENSORES_FALLO de forma programática abajo
      para que los nombres queden 1:1 y no nos equivoquemos con índices. */

/* === Estado === */
static bool etapas[NUM_ETAPAS] = { 0 };              // etapa 0 activa
static bool sensores[NUM_SENSORES] = { 0 };          // lectura instante
static bool latch[NUM_SENSORES]    = { 0 };          // “memoria” de sensor
static bool salidas[NUM_SALIDAS]   = { 0 };          // salidas activas

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

/* === Utilidades de índices === */
static int idx_salida(const char* nombre){
  for(int i=0;i<NUM_SALIDAS;i++) if(SALIDAS[i] && strcmp(SALIDAS[i],nombre)==0) return i;
  return -1;
}
static int idx_sensor(const char* nombre){
  for(int i=0;i<NUM_SENSORES;i++) if(SENSORES[i] && strcmp(SENSORES[i],nombre)==0) return i;
  return -1;
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
}

/* === Helpers de etapa/salidas === */
static bool salida_presente_en_etapa(const char* s){
  for(int i=0; ETAPA_SALIDAS_MAP[etapa_actual][i]; i++)
    if(strcmp(ETAPA_SALIDAS_MAP[etapa_actual][i], s)==0) return true;
  return false;
}
static void activar_salida(const char* s){
  int i = idx_salida(s);
  if(i>=0){
    salidas[i]=true;
    printf("⚙️ Activando salida: %s\n", s);
    if(strcmp(s,"OKCL")==0){
      // Equivalente a tu SALIDA_FUNCIONES["OKCL"]: prender_led("verde")
      printf("[FUNC] LED VERDE encendido\n");
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

  tiempo_fallo_etapa = time(NULL);
  etapa_fallo_idx = 0;

  // Fuerza impresión de etapa 0
  printf("\n🔁 Etapa activa: %d\n", etapa_actual);
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
        printf("❌ Falla detectada en sensor: %s\n", SENSORES[s]);
        fallo=true;
        break;
      }
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
    printf("\n🔁 Etapa activa: %d\n", etapa_actual);
    printf("[ESTADO] START=%s | MODO=%s | COLECTOR=%s\n",
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
      printf("⚙️ [Manual] Rescate: SYOFF + MOTORHOME + SERVOHOME\n");
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
      printf("✅ [Manual] Rescate completado.\n");
      salida_manual=NULL; inicio_manual=0; deadline_manual=0;
      return;
    }

    if(time(NULL) > deadline_manual){
      printf("❌ [Manual] Rescate: sensores no confirmados a tiempo.\n");
      fallo=true;
    }
    return;
  }

  // Salida normal
  int out = idx_salida(salida_manual);
  if(out<0){
    printf("⚠️ [Manual] Salida desconocida: %s\n", salida_manual);
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
      printf("⚙️ [Manual] %s: esperando sensor '%s' ≤ %ds\n", SALIDAS[out], SENSORES[s_idx], timeout);
    } else {
      deadline_manual = inicio_manual + 1; // “operación breve” sin sensor
      printf("⚙️ [Manual] %s: sin sensor asociado (operación breve)\n", SALIDAS[out]);
    }
  }

  // ¿Sensor confirmado?
  int s_idx=-1;
  for(int s=0;s<NUM_SENSORES;s++) if(SENSOR_TO_SALIDA[s]==out){ s_idx=s; break; }
  if(s_idx>=0 && latch[s_idx]){
    printf("✅ [Manual] %s: sensor '%s' confirmado.\n", SALIDAS[out], SENSORES[s_idx]);
    salida_manual=NULL; inicio_manual=0; deadline_manual=0; return;
  }

  if(time(NULL)>deadline_manual){
    if(s_idx>=0){
      printf("❌ [Manual] %s: sensor '%s' no confirmado a tiempo.\n", SALIDAS[out], SENSORES[s_idx]);
      fallo=true;
    } else {
      printf("ℹ️ [Manual] %s: operación breve completada.\n", SALIDAS[out]);
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

/* === Tareas === */
static void grafcet_task(void*){
  printf("⚙️ Simulador GRAFCET listo.\n");
  aplicar_salidas_de_etapa(); // imprime etapa 0
  for(;;){
    if(!pausa && !fallo){
      if(modo==MODO_MANUAL){
        modo_manual_tick();
      } else {
        evo();
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
    printf("▶️  START = %s\n", start_flag? "on":"off");
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
    else { printf("ℹ️ Usa: modo automatico|ciclo|etapa|manual\n"); return; }
    clc=false; se_flag=false; salida_manual=NULL; inicio_manual=0; deadline_manual=0;
    reset_to_stage0();
    printf("✅ Modo cambiado a %s y proceso reiniciado en etapa 0\n",
      (modo==MODO_AUTOMATICO? "automatico": modo==MODO_CICLO? "ciclo": modo==MODO_ETAPA? "etapa":"manual"));
    return;
  }

  if (strstr(buf,"paro")) { pausa=true;  printf("🟥 PARO DE EMERGENCIA activado.\n"); return; }
  if (!strcmp(buf,"ok")) { pausa=false; printf("▶️ Proceso reanudado.\n"); return; }
  if (strstr(buf,"reset")) { reset_to_stage0(); printf("🔄 Sistema reiniciado.\n"); return; }

  if (strstr(buf,"rfalla")) {
    fallo=false; salida_manual=NULL; inicio_manual=0; deadline_manual=0;
    reset_to_stage0();
    printf("✅ Falla reseteada y proceso en etapa 0.\n");
    return;
  }

  if (strstr(buf,"colector")) {
    if (strstr(buf,"on"))  colectorEN=true;
    if (strstr(buf,"off")) colectorEN=false;
    printf("⚙️ colectorEN = %s\n", colectorEN? "ON":"OFF");
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
        printf("🔸 Latch sensor: %s\n", SENSORES[s]);
      } else {
        printf("⛔ Sensor %s ignorado: salida no activa o en FALLA.\n", SENSORES[s]);
      }
      return;
    }
  }

  // Modo manual: activar salidas por nombre o RFALLA_MANUAL
  if (modo==MODO_MANUAL) {
    if (strstr(buf,"rfalla_manual")) {
      if (fallo) { printf("⛔ [Manual] Bloqueado por FALLA. Usa 'rfalla'.\n"); return; }
      salida_manual="RFALLA_MANUAL"; inicio_manual=0; deadline_manual=0;
      printf("⚙️ [Manual] Rescate solicitado.\n");
      return;
    }
    for(int i=0;i<NUM_SALIDAS;i++){
      char low[32]; snprintf(low, sizeof(low), "%s", SALIDAS[i]); str_tolower(low);
      if (strstr(buf, low)) {
        if (fallo) { printf("⛔ [Manual] Bloqueado por FALLA. Usa 'rfalla'.\n"); return; }
        salida_manual = SALIDAS[i]; inicio_manual=0; deadline_manual=0;
        printf("⚙️ [Manual] Activando salida: %s\n", SALIDAS[i]);
        return;
      }
    }
  }

  printf("❓ Comando no reconocido.\n");
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

/* === app_main === */
void app_main(void){
  construir_mapas();
  reset_to_stage0();
  xTaskCreate(grafcet_task,  "grafcet",  4096, NULL, 5, NULL);
  xTaskCreate(comando_task,  "comando",  4096, NULL, 5, NULL);
}
