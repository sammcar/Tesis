#include "motor_stepper.h"
#include "esp_check.h"
#include "esp_log.h"
#include <string.h>

#define TAG              "STEPPER_RMT"
#define MEM_BLOCKS       2                           // 128 símbolos
#define ITEMS_PER_BUF    (64 * MEM_BLOCKS)

static bool tx_done_cb(rmt_channel_handle_t ch,
                       const rmt_tx_done_event_data_t *edata,
                       void *ctx);

static esp_err_t load_batch(stepper_motor_t *m, size_t n_sym);

/* ----------------------------- INIT -------------------------------- */

esp_err_t stepper_init(stepper_motor_t *m,
                       gpio_num_t step, gpio_num_t dir, gpio_num_t en,
                       uint32_t resolution_hz)
{
    ESP_RETURN_ON_FALSE(m, ESP_ERR_INVALID_ARG, TAG, "null");

    memset(m, 0, sizeof(*m));
    m->step_pin = step; m->dir_pin = dir; m->en_pin = en;

    if (resolution_hz == 0) resolution_hz = 10 * 1000 * 1000; // 10 MHz
    m->resolution_hz = resolution_hz;          // <── NUEVO

    /* DIR & EN como salida */
    gpio_config_t io = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << dir) | (1ULL << en)
    };
    gpio_config(&io);
    gpio_set_level(en, 1);                     // driver deshabilitado

    if (resolution_hz == 0) resolution_hz = 10 * 1000 * 1000;   // 10 MHz

    /* --- canal TX --- */
    rmt_tx_channel_config_t cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .gpio_num          = step,
        .mem_block_symbols = ITEMS_PER_BUF,
        .resolution_hz     = resolution_hz,
        .trans_queue_depth = 4,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&cfg, &m->ch), TAG, "new ch");

    /* --- encoder “copy” --- */
    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&copy_cfg, &m->enc),
                        TAG, "copy enc");

    rmt_tx_event_callbacks_t cbs = { .on_trans_done = tx_done_cb };
    rmt_tx_register_event_callbacks(m->ch, &cbs, m);

    ESP_RETURN_ON_ERROR(rmt_enable(m->ch), TAG, "enable ch");
    return ESP_OK;
}

/* ------------------------ movimiento ------------------------------- */

esp_err_t stepper_move(stepper_motor_t *m, int32_t steps, uint32_t freq_hz)
{
    ESP_RETURN_ON_FALSE(!m->busy, ESP_ERR_INVALID_STATE, TAG, "busy");
    ESP_RETURN_ON_FALSE(steps != 0 && freq_hz > 0,
                        ESP_ERR_INVALID_ARG, TAG, "steps/freq");

    bool dir = steps > 0;
    m->steps_left = (size_t) llabs(steps);

    uint32_t period_ticks = m->resolution_hz / freq_hz;
    m->tick_high = m->tick_low = period_ticks / 2;


    gpio_set_level(m->dir_pin, dir);
    gpio_set_level(m->en_pin, 0);          // habilita driver
    m->busy = true;

    size_t first = m->steps_left > ITEMS_PER_BUF ? ITEMS_PER_BUF : m->steps_left;
    m->steps_left -= first;
    return load_batch(m, first);
}

bool stepper_is_busy(const stepper_motor_t *m) { return m->busy; }

void stepper_stop(stepper_motor_t *m)
{
    if (!m->busy) return;
    rmt_disable(m->ch);
    gpio_set_level(m->en_pin, 1);
    m->busy = false;
}

void stepper_enable(stepper_motor_t *m, bool en_low_active)
{
    gpio_set_level(m->en_pin, en_low_active ? 0 : 1);
}

/* ----------------------- internals --------------------------------- */

static esp_err_t load_batch(stepper_motor_t *m, size_t n_sym)
{
    /* buffer de símbolos RAM */
    rmt_symbol_word_t *buf = calloc(n_sym, sizeof(rmt_symbol_word_t));
    ESP_RETURN_ON_FALSE(buf, ESP_ERR_NO_MEM, TAG, "no mem");

    for (size_t i = 0; i < n_sym; ++i) {
        buf[i].level0     = 1;
        buf[i].duration0  = m->tick_high;
        buf[i].level1     = 0;
        buf[i].duration1  = m->tick_low;
    }

    rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
    esp_err_t ret = rmt_transmit(m->ch, m->enc,
                                 buf, n_sym * sizeof(rmt_symbol_word_t),
                                 &tx_cfg);
    free(buf);
    return ret;
}

static bool tx_done_cb(rmt_channel_handle_t ch,
                       const rmt_tx_done_event_data_t *edata,
                       void *ctx)
{
    stepper_motor_t *m = (stepper_motor_t *)ctx;

    if (m->steps_left == 0) {          // fin del movimiento
        gpio_set_level(m->en_pin, 1);  // deshabilita driver
        m->busy = false;
        return true;                   // se procesó la ISR
    }

    size_t n = m->steps_left > ITEMS_PER_BUF ? ITEMS_PER_BUF : m->steps_left;
    m->steps_left -= n;
    load_batch(m, n);
    return true;
}
