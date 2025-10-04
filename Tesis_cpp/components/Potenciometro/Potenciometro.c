#include "Potenciometro.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "filters.h"


#define ADC_UNIT        ADC_UNIT_1
#define ADC_CHANNEL     ADC_CHANNEL_0  // GPIO36
#define ADC_ATTEN       ADC_ATTEN_DB_12

#define POT_MIN_MV   142
#define POT_MAX_MV   3176
#define POT_RANGE_MV (POT_MAX_MV - POT_MIN_MV)  // = 3034

static filt_med5_t med5;
static filt_ema_t  ema;
static float last_deg_f = 0;     /* valor filtrado en grados */

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle;
static SemaphoreHandle_t mutex_pot;
static uint32_t last_mv = 0;
static uint16_t last_raw = 0;

static void Potenciometro_task(void *arg)
{
    for (;;)
    {
        int raw;
        if (adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw) == ESP_OK)
        {
            int mv = 0;
            adc_cali_raw_to_voltage(cali_handle, raw, &mv);

            /* --- conversión a grados sin filtrar ------------------*/
            float vueltas = ((float)(POT_MAX_MV - mv) / POT_RANGE_MV) * 10.0f;
            float grados  = vueltas * 360.0f;

            /* --- filtro mediana + EMA -----------------------------*/
            float g_med = med5_apply(&med5, grados);
            float g_flt = ema_apply(&ema, g_med);

            if (xSemaphoreTake(mutex_pot, portMAX_DELAY))
            {
                last_raw    = raw;
                last_mv     = (mv >= 0) ? (uint32_t)mv : 0;
                last_deg_f  = g_flt;
                xSemaphoreGive(mutex_pot);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

float Potenciometro_get_vueltas(void) {
    uint32_t mv = Potenciometro_get_mV();
    if (mv < POT_MIN_MV) mv = POT_MIN_MV;
    if (mv > POT_MAX_MV) mv = POT_MAX_MV;
    return ((float)(POT_MAX_MV - mv) / POT_RANGE_MV) * 10.0f;
}

float Potenciometro_get_grados_filtrado(void)
{
    float g = 0;
    if (xSemaphoreTake(mutex_pot, portMAX_DELAY)) {
        g = last_deg_f;
        xSemaphoreGive(mutex_pot);
    }
    return g;
}

float Potenciometro_get_grados(void) {
    return Potenciometro_get_vueltas() * 360.0f;
}


void Potenciometro_init(void) {
    mutex_pot = xSemaphoreCreateMutex();
    med5_init(&med5);
    ema_init(&ema, 0.2f);   // α ≈ 0.2   (τ≈200 ms con Ts=50 ms)


    adc_oneshot_unit_init_cfg_t cfg = {
        .unit_id = ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    adc_oneshot_new_unit(&cfg, &adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &chan_cfg);

    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    adc_cali_create_scheme_line_fitting(&cali_cfg, &cali_handle);

    xTaskCreate(Potenciometro_task, "PotTask", 2048, NULL, 5, NULL);
}

uint32_t Potenciometro_get_mV(void) {
    uint32_t mv = 0;
    if (xSemaphoreTake(mutex_pot, portMAX_DELAY)) {
        mv = last_mv;
        xSemaphoreGive(mutex_pot);
    }
    return mv;
}

uint16_t Potenciometro_get_raw(void) {
    uint16_t val = 0;
    if (xSemaphoreTake(mutex_pot, portMAX_DELAY)) {
        val = last_raw;
        xSemaphoreGive(mutex_pot);
    }
    return val;
}
