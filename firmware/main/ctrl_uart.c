#include "ctrl_uart.h"
#include "trip_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// JC-ESP32P4-M3-DEV: GPIO16/17 заняты SDIO к C6 — UART-мост к S3 не используется.
// Пины оставлены для совместимости; при наличии внешнего S3 переназначить на свободные.
#define CTRL_UART   UART_NUM_1
#define CTRL_TX     GPIO_NUM_25     // к RX ESP32-C6/S3 (если внешний)
#define CTRL_RX     GPIO_NUM_26     // от TX ESP32-C6/S3
#define FRAME_SOF   0xC7            // команды от BLE-чипа к P4
#define STAT_SOF    0xC8            // статус от P4 к BLE-чипу

static const char *TAG = "ctrl";

// Периодически отдаём телеметрию BLE-чипу: [0xC8][len][payload...].
// payload(7): rpm_lo, rpm_hi, throttle(0..255), state, preset, tempC, err
static void status_task(void *arg) {
    engine_t *e = (engine_t *)arg;
    while (1) {
        uint16_t rpm = (uint16_t)(e->rpm < 0 ? 0 : e->rpm);
        uint8_t thr = (uint8_t)(e->throttle * 255.0f);
        uint8_t state = (e->throttle > 0.5f ? 1 : 0) | (e->bfLeft > 0 ? 2 : 0);
        uint8_t f[9] = { STAT_SOF, 7,
            (uint8_t)(rpm & 0xFF), (uint8_t)(rpm >> 8), thr, state,
            e->preset, e->coolantC, e->errFlags };
        uart_write_bytes(CTRL_UART, (const char*)f, sizeof(f));
        vTaskDelay(pdMS_TO_TICKS(100));   // 10 Гц
    }
}

static void ctrl_task(void *arg) {
    void **ctx = (void **)arg;
    engine_t *e = (engine_t *)ctx[0];
    ctrl_preset_cb on_preset = (ctrl_preset_cb)ctx[1];
    uint8_t b;
    while (1) {
        if (uart_read_bytes(CTRL_UART, &b, 1, portMAX_DELAY) != 1 || b != FRAME_SOF) continue;
        uint8_t hdr[2];
        if (uart_read_bytes(CTRL_UART, hdr, 2, pdMS_TO_TICKS(50)) != 2) continue;
        uint8_t cmd = hdr[0], len = hdr[1];
        uint8_t payload[16];
        if (len > sizeof(payload)) len = sizeof(payload);
        if (len && uart_read_bytes(CTRL_UART, payload, len, pdMS_TO_TICKS(50)) != len) continue;

        switch (cmd) {
            case 0x01: if (len>=1) { e->masterGain = payload[0] / 255.0f;
                                     ESP_LOGI(TAG,"volume=%.2f", e->masterGain); } break;
            case 0x02: if (len>=1 && on_preset) { ESP_LOGI(TAG,"preset=%d", payload[0]);
                                     on_preset(payload[0]); } break;
            case 0x03: if (len>=1) engine_trigger_event(e, payload[0]); break;
            case 0x04: if (len>=1) trip_log_set(payload[0] != 0); break;  // лог поездки вкл/выкл
            case 0x05: if (len>=2) { e->rpm_ovr = (float)(payload[0] | (payload[1] << 8));
                                     e->rpm_ovr_ms = 3000;  // ~3 c оверрайда, продлеваем новыми командами
                                     ESP_LOGI(TAG,"rpm_override=%.0f", e->rpm_ovr); } break;
            default: ESP_LOGW(TAG,"unknown cmd 0x%02x", cmd);
        }
    }
}

static void *s_ctx[2];

void ctrl_uart_start(engine_t *engine, ctrl_preset_cb on_preset) {
    uart_config_t cfg = {
        .baud_rate = 115200, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(CTRL_UART, 512, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CTRL_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(CTRL_UART, CTRL_TX, CTRL_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    s_ctx[0] = engine; s_ctx[1] = (void*)on_preset;
    xTaskCreatePinnedToCore(ctrl_task, "ctrl", 4096, s_ctx, 5, NULL, 0);
    xTaskCreatePinnedToCore(status_task, "ctrlstat", 3072, engine, 4, NULL, 0);
}
