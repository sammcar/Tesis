/* -------------- includes del proyecto -------------- */
#include "pid_control.h"        /* nuestra librería recién creada */
#include "Potenciometro.h"      /* sensor de tensión/posición   */
#include "VariacRS485.h"        /* actuador 1 : variac           */
#include "SyringePumpRS232.h"   /* actuador 2 : bomba de jeringa */
#include "filters.h"
#include "motor_stepper.h"
#include "uart_config.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "driver/uart.h"

/* ---------- pines del motor (ajusta a tu HW) ---------- */
#define STEP_PIN   GPIO_NUM_18
#define DIR_PIN    GPIO_NUM_5
#define EN_PIN     GPIO_NUM_23

/* ---------- micro-paso y escala del potenciómetro ----- */
#define MICROSTEPPING         8                 // 1/8  → 1600 µ-steps/vuelta
#define DEG_PER_STEP_POT      (3600.0f / (200.0f * MICROSTEPPING))
                                              // = 2.25 ° pot / µ-step

/* ---------- parámetros de la prueba “bloque sencillo” -- */
#define TEST_BLOCK_USTEPS     100              // µ-steps por bloque
#define TEST_VEL_HZ           100.0f           // velocidad (µ-step/s)
#define N_BLOCKS              10               // cuántas veces repetir

/* ---------- RTOS y temporización ----------------------- */
#define LOOP_HZ               20              
#define SETPOINT_DEG       00.0f               
#define SAMPLE_MS 50
#define N_SAMPLES 200
#define FILTER_SETTLE_MS    600          // 3·τ del EMA (α=0.2)

/* ---------- ajustes de seguridad opcionales ---------- */
#define DEADBAND_DEG      0.5f     // zona muerta real ±0.5°
#define MAX_RATE_HZ       3000.0f  // límite físico
#define MIN_RATE_HZ       20.0f    // evita errores por frecuencias muy bajas

/* ---------- parámetros steps ---------- */
#define TEST_STEPS          1000            // 1 k µ-steps
#define TEST_VEL_HZ         100.0f          // 100 µ-step/s
#define POST_WAIT_MS        300             // espera después del movimiento

static stepper_motor_t motor;
static PID_t pid;

void pruebaVariac(void) {
    printf("⚡ Iniciando prueba de VariacRS485...\n");

    Variac_Init();

    if (Variac_VerificarConexion()) {
        printf("✅ Comunicación con el Variac exitosa.\n");
    } else {
        printf("❌ No se pudo comunicar con el Variac.\n");
    }
    
}

void pruebaSyringe(void) {
    printf("⚡ Iniciando prueba de SyringeRS232...\n");

    Syringe_Init();

    if (Syringe_VerifyConnection()) {
        printf("✅ Comunicación con el Syringe exitosa.\n");
    } else {
        printf("❌ No se pudo comunicar con el Syringe.\n");
    }
}

static void test_ruido_pot(void *arg)
{

    vTaskDelay(pdMS_TO_TICKS(2000));   // 2 s de gracia

    double sum = 0.0, sum2 = 0.0;

    for (int i = 0; i < N_SAMPLES; ++i) {
        double g = (double)Potenciometro_get_grados_filtrado();
        sum  += g;
        sum2 += g * g;
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));
    }

    double media = sum / N_SAMPLES;
    double sigma = sqrt( (sum2 / N_SAMPLES) - media * media );

    printf("\n---- Test de ruido ----\n");
    printf("Media  = %.2f °\n", (float)media);
    printf("σ      = %.3f °\n", (float)sigma);
    printf("-----------------------\n\n");

    vTaskDelete(NULL);   // tarea terminada
}

static void control_task(void *arg)
{
    /* 1. ganancias */
    const float Kp = 0.35f, Ki = 0.005f, Kd = 0.1f, alpha = 1.0f;

    pid_init(&pid, Kp, Ki, Kd,
             SETPOINT_DEG,
             -MAX_RATE_HZ, MAX_RATE_HZ,
             1.0f/LOOP_HZ,
             alpha);

    /* 2. sembramos prev_sign con el error inicial */
    float y = Potenciometro_get_grados_filtrado();
    float error = pid.setpoint - y;
    int prev_sign = (error > 0) - (error < 0);

    int init_pot = uart_getPot();
    if (init_pot < 0) init_pot = 0;
    if (init_pot > 3600) init_pot = 3600;  // p. ej. 10 vueltas = 3600°

    if (init_pot != (int)pid.setpoint) {
        pid_set_setpoint(&pid, (float)init_pot);
        pid_reset_state(&pid);
        printf("[UART] Setpoint inicial desde UART: %d°\n", init_pot);
        // resembrar prev_sign:
        y         = Potenciometro_get_grados_filtrado();
        error     = pid.setpoint - y;
        prev_sign = (error>0) - (error<0);
    }

    for (;;)
    {
        y     = Potenciometro_get_grados_filtrado();
        error = pid.setpoint - y;
        int sign = (error > 0) - (error < 0);

        /* 1) Detectar cruce del setpoint */
        if (prev_sign != 0 && sign != prev_sign)
        {
            // cruce
            stepper_stop(&motor);
            printf("→ Setpoint cruzado: θ=%.2f°, err=%.2f°\n", y, error);

            // idle hasta nuevo Pot (UART)
            float old_sp = pid.setpoint;
            for (;;)
            {
                int pot_deg = uart_getPot();

                if (pot_deg < 0)     pot_deg = 0;
                if (pot_deg > 3600)  pot_deg = 3600;
                
                if (pot_deg != (int)old_sp)
                {
                    pid_set_setpoint(&pid, (float)pot_deg);
                    pid_reset_state(&pid);
                    printf("[UART] Nuevo setpoint: %d°\n", pot_deg);
                    error     = pid.setpoint - y;
                    prev_sign = (error>0) - (error<0);
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            continue;
        }
        prev_sign = sign;

        /* 2) Zona muerta */
        if (fabsf(error) < DEADBAND_DEG)
        {
            stepper_stop(&motor);
            printf("→ Dentro de dead‑band ±%.2f°: θ=%.2f°\n", DEADBAND_DEG, y);

            // idle igual que antes
            float old_sp = pid.setpoint;
            for (;;)
            {
                int pot_deg = uart_getPot();
                if (pot_deg < 0)     pot_deg = 0;
                if (pot_deg > 3600)  pot_deg = 3600;
                if (pot_deg != (int)old_sp)
                {
                    pid_set_setpoint(&pid, (float)pot_deg);
                    pid_reset_state(&pid);
                    printf("[UART] Nuevo setpoint: %d°\n", pot_deg);
                    error     = pid.setpoint - y;
                    prev_sign = (error>0) - (error<0);
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            continue;
        }

        /* PID normal */
        float u = pid_compute(&pid, y);
        int   dir   = (u >= 0);
        float rate  = fabsf(u);
        if (rate > MAX_RATE_HZ) rate = MAX_RATE_HZ;

        int steps = (int)roundf(rate / LOOP_HZ);
        if (steps > 0)
            stepper_move(&motor, dir ? steps : -steps, rate);

        /* debug */
        if (fabsf(error) > 2.0f)
            printf("θ=%.2f° err=%.2f° u=%.2f rate=%.2fHz steps=%d\n",
                   y, fabsf(error), u, rate, steps);

        vTaskDelay(pdMS_TO_TICKS(1000 / LOOP_HZ));
    }
}

/* ===================== helper bloqueante ========================= */
static void stepper_move_wait(stepper_motor_t *m, int steps, float vel_hz)
{
    stepper_move(m, steps, vel_hz);
    while (stepper_is_busy(m))
        vTaskDelay(pdMS_TO_TICKS(2));
}

/* ---------- helper sin filtro (simple) ---------- */
static inline float pot_now_deg(void)
{
    return Potenciometro_get_grados();   // conversión directa cruda
}

static void one_block_test(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1000));                    // gracia inicial
    printf("\nµ-steps: %d  |  Vel: %.0f µ-step/s\n", TEST_STEPS, TEST_VEL_HZ);

    float th_ini = pot_now_deg();

    stepper_move_wait(&motor, TEST_STEPS, TEST_VEL_HZ);
    vTaskDelay(pdMS_TO_TICKS(POST_WAIT_MS));            // deja que termine

    float th_fin = pot_now_deg();
    float delta  = th_fin - th_ini;

    printf("θ_ini = %.1f °   θ_fin = %.1f °   Δθ = %.1f °\n", th_ini, th_fin, delta);
    printf("°/µ-step = %.3f\n", delta / TEST_STEPS);

    vTaskDelete(NULL);
}

static void uart_task(void *arg)
{
    (void)arg;
    for (;;) {
        uart_leer_y_procesar();
        vTaskDelay(pdMS_TO_TICKS(100));  // pequeño pausado
    }
}

void example_syringe_sequence(void) {
    char buf[64];
    int len;

    // 1) Init + Basic Mode persistente
    Syringe_Init();
    Syringe_SetAddress(0);
    Syringe_ExitSafeMode();      // SAFO + "SAF 0"
    vTaskDelay(pdMS_TO_TICKS(100));

    // 2) VER + limpiar alarma Reset si aparece
    uart_flush_input(SYRINGE_UART);
    Syringe_SendCommand("VER");
    len = Syringe_ReadResponse(buf, sizeof(buf), 1000);
    if (len <= 0) { printf("❌ No responde VER\n"); return; }
    printf("📥 Firmware raw: %s\n", buf);

    if (strstr(buf, "A?R")) {
        printf("⚠️ RESET alarm, doing PF 0...\n");
        uart_flush_input(SYRINGE_UART);
        Syringe_SendCommand("PF 0");
        vTaskDelay(pdMS_TO_TICKS(100));
        len = Syringe_ReadResponse(buf, sizeof(buf), 500);
        printf("   PF0 resp: %s\n", buf);
    }

    uart_flush_input(SYRINGE_UART);
    Syringe_SendCommand("VER");
    len = Syringe_ReadResponse(buf, sizeof(buf), 500);
    if (len <= 0) { printf("❌ No responde VER (2)\n"); return; }
    printf("📥 Firmware: %s\n\n", buf);

    // --- Ya en Basic Mode y sin alarmas ---

    // 3) DIA 10.0
    float dia;
    uart_flush_input(SYRINGE_UART);
    Syringe_QueryDiameter(&dia);
    printf("🔍 DIA antes: %.1f mm\n", dia);

    uart_flush_input(SYRINGE_UART);
    Syringe_SetDiameter(10.0f);
    vTaskDelay(pdMS_TO_TICKS(100));

    uart_flush_input(SYRINGE_UART);
    Syringe_QueryDiameter(&dia);
    printf("⚙️ DIA ahora: %.1f mm\n\n", dia);

    // 4) PHN 1
    uint8_t phase;
    uart_flush_input(SYRINGE_UART);
    Syringe_SelectPhase(1);
    vTaskDelay(pdMS_TO_TICKS(100));

    uart_flush_input(SYRINGE_UART);
    Syringe_QueryPhase(&phase);
    printf("🔍 PHN: %u\n\n", phase);

    // 5) FUN RAT
    char func[16];
    uart_flush_input(SYRINGE_UART);
    Syringe_SetPhaseFunction("RAT");
    vTaskDelay(pdMS_TO_TICKS(100));

    uart_flush_input(SYRINGE_UART);
    Syringe_QueryPhaseFunction(func, sizeof(func));
    printf("🔍 FUN: %s\n\n", func);

    // 6) Rate: 5 µL/min (Unidad y espacio exactos)
    uart_flush_input(SYRINGE_UART);
    Syringe_SendCommand("RAT 10.00 UM");
    vTaskDelay(pdMS_TO_TICKS(100));

    // Confirmación inmediata
    uart_flush_input(SYRINGE_UART);
    Syringe_SendCommand("RAT");
    len = Syringe_ReadResponse(buf, sizeof(buf), 200);
    if (len > 3) {
        char *p = buf + 3;
        float rate;
        char runits[4];
        sscanf(p, "%f %3s", &rate, runits);
        printf("🔍 RAT: %.2f %s\n\n", rate, runits);
    }

    // 7) VOL 1.00 (use units defined by DIA)
    uart_flush_input(SYRINGE_UART);
    Syringe_SendCommand("VOL 100.00");  // o "VOL 1.00ULM" si el manual lo exige
    vTaskDelay(pdMS_TO_TICKS(100));

    uart_flush_input(SYRINGE_UART);
    Syringe_SendCommand("VOL");         // Query según modelo
    len = Syringe_ReadResponse(buf, sizeof(buf), 200);
    printf("🔍 VOL = %s\n", buf+3);

    // 8) DIR INF
    char dbuf[8];
    uart_flush_input(SYRINGE_UART);
    Syringe_SetDirection("INF");
    vTaskDelay(pdMS_TO_TICKS(100));

    uart_flush_input(SYRINGE_UART);
    Syringe_QueryDirection(dbuf, sizeof(dbuf));
    printf("🔍 DIR: %s\n\n", dbuf);

    // 9) CLD INF, CLD WDR + RUN
    uart_flush_input(SYRINGE_UART);
    Syringe_ClearDispensed("INF");
    Syringe_ClearDispensed("WDR");
    vTaskDelay(pdMS_TO_TICKS(100));

    uart_flush_input(SYRINGE_UART);
    Syringe_Run(1);  // Start phase 1
    printf("▶️ RUN iniciado\n\n");

    // 10) Monitor breve
    for (int i = 0; i < 5; ++i) {
        float inf, wdr;
        char units[8];
        uart_flush_input(SYRINGE_UART);
        Syringe_QueryDispensed(&inf, &wdr, units, sizeof(units));
        printf("📊 DIS: I=%.2f %s | W=%.2f %s\n",
               inf, units, wdr, units);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // 11) STP
    uart_flush_input(SYRINGE_UART);
    Syringe_Stop();
    printf("\n⏹️ STOP\n");
}

void app_main(void)
{
    // Inicializaciones básicas
    Potenciometro_init();
    stepper_init(&motor, STEP_PIN, DIR_PIN, EN_PIN, 10*1000000);
    stepper_enable(&motor, true);

    // // 1) Lanzar la tarea UART
    // xTaskCreatePinnedToCore(uart_task,"UART_Task", 2048, NULL, 5, NULL, tskNO_AFFINITY);

    // // 2) Lanzar la tarea de control PID
    // printf("\n==== PID: Seguimiento del setpoint ====\n");
    // xTaskCreate(control_task,"PID_Task",4096,NULL,5,NULL);

    example_syringe_sequence();
}





