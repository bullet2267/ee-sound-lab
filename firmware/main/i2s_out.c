#include "i2s_out.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

// --- аудио JC-ESP32-P4-M3-DEV: ES8311 (I2C 0x18) + NS4150B (PA=GPIO11) ---
#define I2S_MCLK  GPIO_NUM_13
#define I2S_BCLK  GPIO_NUM_12
#define I2S_LRCK  GPIO_NUM_10
#define I2S_DOUT  GPIO_NUM_9
#define I2C_SDA   GPIO_NUM_7
#define I2C_SCL   GPIO_NUM_8
#define PA_PIN    GPIO_NUM_11          // NS4150B enable (по пинауту борды AMP_EN=11)

#define FRAMES_PER_BLOCK  128              // 128×4 ≈ 23мс буфера — отклик на газ резче

static const char *TAG = "i2s_out";
static i2s_chan_handle_t s_tx = NULL;

static void audio_task(void *arg) {
    engine_t *e = (engine_t *)arg;
    static int16_t buf[FRAMES_PER_BLOCK * 2];
    size_t wrote;
    uint32_t silBlk = 0; int paOn = 1;
    while (1) {
        engine_render(e, buf, FRAMES_PER_BLOCK);
        // гейт усилителя: кадр полных нулей >~400мс -> PA off (нет фона в простое);
        // первый ненулевой сэмпл -> PA on. Небольшой pop при включении допустим.
        int act = 0;
        for (int i = 0; i < FRAMES_PER_BLOCK * 2; i++) if (buf[i]) { act = 1; break; }
        if (act) { silBlk = 0; if (!paOn) { gpio_set_level(PA_PIN, 1); paOn = 1; } }
        else if (++silBlk > 70 && paOn) { gpio_set_level(PA_PIN, 0); paOn = 0; }
        if (i2s_channel_write(s_tx, buf, sizeof(buf), &wrote, portMAX_DELAY) != ESP_OK) {
            static unsigned underrun;
            if (++underrun % 50 == 1)
                ESP_LOGW("i2s", "write timeout x%u — underrun (SD-нагрузка?)", underrun);
            vTaskDelay(1);                              // канал в реконфиге — не жечь CPU
        }
    }
}

static i2c_master_dev_handle_t s_i2cdev;

static void es_reg(uint8_t reg, uint8_t val, int delay_ms) {
    uint8_t w[2] = {reg, val};
    esp_err_t r = i2c_master_transmit(s_i2cdev, w, 2, 50);
    if (r) ESP_LOGW(TAG, "es8311 w%02x=%02x rc=%d", reg, val, r);
    if (delay_ms) vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

static void codec_init(uint32_t sr) {
    (void)sr;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = 0, .sda_io_num = I2C_SDA, .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus;
    esp_err_t rc = i2c_new_master_bus(&bus_cfg, &i2c_bus);
    if (rc) { ESP_LOGE(TAG, "i2c bus err %d", rc); return; }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = 0x18,
        .scl_speed_hz = 100000,
    };
    rc = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &s_i2cdev);
    if (rc) { ESP_LOGE(TAG, "i2c dev add err %d", rc); return; }
    if (i2c_master_probe(i2c_bus, 0x18, 50) != ESP_OK) {
        ESP_LOGE(TAG, "es8311 no ack @0x18"); return;
    }
    // ES8311 init — точная последовательность из esp_codec_dev (open+start+fmt+sample@44.1k)
    // coeff row: mclk=11289600 fs=44100 pre=1 mult=1 adc=1 dac=1 fsm=0 lrch=0 lrcl=255 bck=4 osr=0x10
    es_reg(0x0D, 0xFA, 0);           // REG0D power seq
    es_reg(0x44, 0x08, 0);           // GPIO (noise immunity)
    es_reg(0x44, 0x08, 0);
    es_reg(0x01, 0x30, 0);           // CLK_MANAGER1
    es_reg(0x02, 0x00, 0);
    es_reg(0x03, 0x10, 0);
    es_reg(0x16, 0x24, 0);
    es_reg(0x04, 0x10, 0);
    es_reg(0x05, 0x00, 0);
    es_reg(0x0B, 0x00, 0);
    es_reg(0x0C, 0x00, 0);
    es_reg(0x10, 0x1F, 0);
    es_reg(0x11, 0x7F, 0);
    es_reg(0x00, 0x80, 20);          // reset
    es_reg(0x00, 0x80, 20);          // slave mode (driver пишет 0x80&0xBF=0x80)
    es_reg(0x01, 0x3F, 0);           // clk src = MCLK pin
    es_reg(0x06, 0x03, 0);           // bclk_div-1=3 (sclk no invert)
    es_reg(0x13, 0x10, 0);
    es_reg(0x1B, 0x0A, 0);
    es_reg(0x1C, 0x6A, 0);
    es_reg(0x44, 0x58, 0);           // internal ref ADCL+DACR
    // es8311_start: SDP enable + I2S fmt 16bit (0x0C), iface bit6=0
    es_reg(0x09, 0x0C, 0);           // SDP_IN
    es_reg(0x0A, 0x0C, 0);           // SDP_OUT
    es_reg(0x17, 0xBF, 0);           // DAC/ADC vol path
    es_reg(0x0E, 0x02, 0);           // SYSTEM pwr up
    es_reg(0x12, 0x00, 0);           // DAC enable
    es_reg(0x14, 0x1A, 0);           // analog out enable
    es_reg(0x0D, 0x01, 0);           // system enable
    es_reg(0x15, 0x40, 0);
    es_reg(0x37, 0x08, 0);
    es_reg(0x45, 0x00, 0);
    // clock coefficients @44.1k mclk=11.2896M
    es_reg(0x02, 0x00, 0);           // pre_div-1=0, mult=0
    es_reg(0x05, 0x00, 0);           // adc_div-1|dac_div-1
    es_reg(0x03, 0x10, 0);           // fs_mode<<6 | adc_osr 0x10
    es_reg(0x04, 0x10, 0);           // dac_osr 0x10
    es_reg(0x07, 0x00, 0);           // lrck_h
    es_reg(0x08, 0xFF, 0);           // lrck_l=255
    es_reg(0x06, 0x03, 0);           // bclk_div-1=3
    es_reg(0x32, 0xBF, 0);           // DAC volume ~unmute
    // PA: AMP_EN GPIO11 -> NS4150B enable (по пинауту JC-ESP32-P4-M3)
    gpio_set_direction(PA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PA_PIN, 1);
    ESP_LOGI(TAG, "ES8311 init done @ %lu Hz, PA=GPIO%d ON", (unsigned long)sr, PA_PIN);
}

void i2s_out_set_rate(uint32_t sr) {                // переконфиг клока под банк (THOR=22050)
    if (!s_tx) return;
    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(sr);
    i2s_channel_disable(s_tx);
    i2s_channel_reconfig_std_clock(s_tx, &clk);
    i2s_channel_enable(s_tx);
    ESP_LOGI(TAG, "I2S rate -> %lu", (unsigned long)sr);
}

void i2s_out_start(engine_t *engine) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = 4;
    chan_cfg.dma_frame_num = FRAMES_PER_BLOCK;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(engine->bank->sampleRate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCLK,
            .bclk = I2S_BCLK, .ws = I2S_LRCK, .dout = I2S_DOUT,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx));
    ESP_LOGI(TAG, "I2S up @ %lu Hz", (unsigned long)engine->bank->sampleRate);

    codec_init(engine->bank->sampleRate);           // MCLK уже идёт — кодек определит частоту

    xTaskCreatePinnedToCore(audio_task, "audio", 4096, engine, 10, NULL, 1);
}
