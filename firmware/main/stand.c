// stand.c — испытательный стенд: перемычки на GPIO имитируют зажигание/газ/КПП.
// ВАЖНО (авто-безопасность): стенд ВЫКЛЮЧЕН, пока его явно не включат:
//   - перемычка GEAR(28)->GND держана при старте прошивки (bench-режим), ИЛИ
//   - любая команда стенда из веб-пульта/TCP (сетевое намерение)
// Даже включённый стенд УСТУПАЕТ живому CAN: пока canAliveMs>0 стенд не пишет rpm.
// Пины (активный 0 = перемычка на GND, pull-up внутри):
//   IGN  GPIO25 — зажигание: коротнул -> "двигатель заведён" -> rpm на холостые
//   THR0 GPIO26, THR1 GPIO29 — два уровня газа: 0 замкнуто=0%, 1=50%, 2=100%
//   (GPIO27 = ACC_SENSE в боевом нетлисте — под стенд НЕ используем!)
//   GEAR GPIO28 — кнопка: короткое нажатие = передача вверх (+ enable при старте)
//   THR_ADC GPIO22 (ADC1_CH6) — потенциометр 0..3.3В, плавный газ (если >0.05В — перекрывает пины)
// КПП авто: rpm растёт к shiftRpm -> просадка -> следующая передача; отпустил газ — прострелы
// (детект в main loop). Настройки лежат в /sd/stand.cfg и правятся из веб-пульта.
#include "stand.h"
int sd_write_atomic(const char *path, const void *buf, size_t len);
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

#define PIN_IGN     GPIO_NUM_25
#define PIN_THR0    GPIO_NUM_26
#define PIN_THR1    GPIO_NUM_29            // 27 = ACC_SENSE в машине — не трогаем
#define PIN_GEAR    GPIO_NUM_28
#define PIN_THR_ADC GPIO_NUM_22
#define ADC_CH      ADC_CHANNEL_6          // GPIO22 = ADC1_CH6 на ESP32-P4 (GPIO18=SDIO CLK C6 — занят!)

typedef struct {
    float idleRpm, shiftRpm, redline, revPerSec;   // настройки (persist)
    uint8_t gears;                                  // число передач
    // runtime
    float rpm, thr;
    int   gear;
    uint32_t shiftHoldMs;                           // пауза роста после переключения
    volatile int webIgn;                            // -1/0/1: веб-зажигание (-1 = не задан)
    volatile float webThr;                          // <0 = веб газа нет
} stand_t;

static engine_t *s_eng;
static volatile uint8_t s_enabled;         // 0 = стенд выкл (авто-режим, пины игнорятся)
static volatile uint8_t s_neutral;         // 1 = газовка на месте (нейтраль, без передач)
static stand_t   s_st = {
    .idleRpm = 850.f, .shiftRpm = 5500.f, .redline = 6800.f, .revPerSec = 3800.f,
    .gears = 6, .rpm = 0.f, .thr = 0.f, .gear = 0,
    .webIgn = -1, .webThr = -1.f,
};
static adc_oneshot_unit_handle_t s_adc;

static void cfg_save(void) {
    uint16_t u[3] = { (uint16_t)s_st.idleRpm, (uint16_t)s_st.shiftRpm, (uint16_t)s_st.redline };
    uint8_t v[7]; memcpy(v, u, 6); v[6] = (uint8_t)s_st.gears;
    sd_write_atomic("/sd/stand.cfg", v, 7);
}
static void cfg_load(void) {
    FILE *f = fopen("/sd/stand.cfg", "rb");
    if (!f) return;
    uint16_t v[3] = {0};
    if (fread(v, 2, 3, f) == 3) {
        if (v[0] >= 300 && v[0] <= 2000) s_st.idleRpm = v[0];
        if (v[1] >= 2500 && v[1] <= 9000) s_st.shiftRpm = v[1];
        if (v[2] > v[1] && v[2] <= 12000) s_st.redline = v[2];
    }
    int g = fgetc(f);
    if (g >= 3 && g <= 8) s_st.gears = (uint8_t)g;
    fclose(f);
}

void stand_set_ign(engine_t *e, uint8_t on) { (void)e; s_enabled = 1; s_st.webIgn = on ? 1 : 0; }
void stand_set_thr(engine_t *e, float t)    { (void)e; s_enabled = 1; s_st.webThr = t; }
void stand_set_mode(engine_t *e, uint8_t neutral) { (void)e; s_enabled = 1; s_neutral = neutral ? 1 : 0; }
void stand_set_idle(engine_t *e, float rpm) { (void)e;
    if (rpm >= 300 && rpm <= 2000) { s_st.idleRpm = rpm; cfg_save(); } }
void stand_apply_pkg(engine_t *e, float idle, float redline) { (void)e;
    // калибровка из pkgrl пакета: холостые/отсечка реальной машины (без cfg_save —
    // не трогаем юзерские настройки; стенд живёт по текущим значениям)
    if (idle >= 300 && idle <= 2000)    s_st.idleRpm = idle;
    if (redline >= 3000 && redline <= 12000) {
        s_st.redline = redline;
        if (s_st.shiftRpm > redline - 500.f) s_st.shiftRpm = redline - 800.f;
    } }
float stand_redline(void) { return s_st.redline; }
void stand_gear_cmd(engine_t *e, int d) {     // ручное переключение из веба
    (void)e; s_enabled = 1;
    s_st.gear += d;
    if (s_st.gear < 0) s_st.gear = 0;
    if (s_st.gear > s_st.gears) s_st.gear = s_st.gears;
    s_st.rpm *= (d > 0) ? 0.62f : 1.30f;      // просадка/подброс как при реальном сдвиге
    if (s_st.rpm < s_st.idleRpm * 0.9f) s_st.rpm = s_st.idleRpm * 0.9f;
}

static void stand_task(void *arg) {
    uint8_t lastGearBtn = 1;                              // pull-up: 1 = не нажата
    while (1) {
        // --- входы ---
        uint8_t ignPin = !gpio_get_level(PIN_IGN);
        int thrPins = (!gpio_get_level(PIN_THR0)) + (!gpio_get_level(PIN_THR1));
        float thr = thrPins * 0.5f;
        int adc_raw = 0;
        if (s_adc && adc_oneshot_read(s_adc, ADC_CH, &adc_raw) == ESP_OK) {
            float tv = adc_raw / 4095.0f;
            if (tv > 0.03f && tv > thr) thr = tv;         // пот перекрывает пины
        }
        if (s_st.webThr >= 0.f) thr = s_st.webThr;        // веб перекрывает всё
        uint8_t ign = ignPin || (s_st.webIgn == 1);
        if (s_st.webIgn == 0 && !ignPin) ign = 0;

        uint8_t gb = gpio_get_level(PIN_GEAR);
        if (ign && lastGearBtn == 1 && gb == 0) stand_gear_cmd(NULL, +1);  // фронт нажатия
        lastGearBtn = gb;

        // --- динамика ---
        const float dt = 0.02f;                           // тик 20 мс
        if (!ign) {
            s_st.rpm -= 1200.f * dt;                      // заглох — спад
            if (s_st.rpm < 0) s_st.rpm = 0;
            s_st.gear = 0;
        } else {
            if (s_st.rpm < 100.f && thr < 0.05f)          // "проворот" стартера
                s_st.rpm = 400.f;
            if (s_neutral) {
                // газовка на месте: нейтраль, без передач — мотор крутится легко
                // и быстро падает к холостым (блип). Передачи и авто-сдвиги не трогаем.
                s_st.gear = 0;
                float ntarget = s_st.idleRpm + thr * (s_st.redline - s_st.idleRpm);
                float nrate = s_st.revPerSec * 1.6f;
                if (s_st.rpm < ntarget) { s_st.rpm += nrate * dt; if (s_st.rpm > ntarget) s_st.rpm = ntarget; }
                else                    { s_st.rpm -= nrate * 1.3f * dt; if (s_st.rpm < ntarget) s_st.rpm = ntarget; }
                if (s_st.rpm < s_st.idleRpm * 0.85f) s_st.rpm = s_st.idleRpm * 0.85f;
                s_st.thr = thr;
                goto stand_out;                           // мимо логики передач
            }
            if (thr > 0.05f && s_st.gear == 0) s_st.gear = 1;
            float target = s_st.idleRpm + thr * (s_st.redline - s_st.idleRpm);
            float rate = s_st.revPerSec / (s_st.gear > 0 ? s_st.gear : 1);
            if (s_st.shiftHoldMs) { s_st.shiftHoldMs -= 20; rate = 0; }
            if (s_st.rpm < target) { s_st.rpm += rate * dt; if (s_st.rpm > target) s_st.rpm = target; }
            else                   { s_st.rpm -= rate * 1.4f * dt; if (s_st.rpm < target) s_st.rpm = target; }
            if (s_st.rpm < s_st.idleRpm * 0.85f) s_st.rpm = s_st.idleRpm * 0.85f;
            // авто-КПП: дошли до точки — передача вверх с просадкой
            if (s_st.rpm >= s_st.shiftRpm && s_st.gear > 0 && s_st.gear < s_st.gears && thr > 0.15f) {
                s_st.gear++; s_st.rpm *= 0.62f; s_st.shiftHoldMs = 140;
            }
            // провал ниже рабочего диапазона на высокой передаче — вниз
            if (s_st.gear > 1 && s_st.rpm < s_st.idleRpm * 1.7f && thr < 0.1f) s_st.gear--;
        }
        s_st.thr = thr;
    stand_out:;

        // --- выход в движок: стенд только если включён И шина CAN молчит ---
        if (s_eng->canAliveMs > 0) s_eng->canAliveMs -= 20;      // живой CAN глушит стенд
        uint8_t own = s_enabled && s_eng->canAliveMs <= 0 && ign;
        s_eng->standActive = own ? 1 : 0;
        s_eng->standGear   = (uint8_t)s_st.gear;
        if (own) {
            // микро-флаттер оборотов как у живого мотора (на холостых ±15-25 об/мин):
            // медленное псевдослучайное блуждание ломает машинную периодичность лупов
            static uint32_t lfsr = 0xACE1;
            static float jit = 0.f;
            lfsr ^= lfsr << 13; lfsr ^= lfsr >> 17; lfsr ^= lfsr << 5;
            jit += (((lfsr >> 8) & 0xFFFF) / 65535.f - 0.5f - jit) * 0.08f; // ~250мс фильтр
            /* rpm_ovr активен (диагностический прогон) — не дерёмся за e->rpm:
             * иначе стенд и can-таск пишут по очереди -> обороты зигзагом */
            if (!s_eng->rpm_ovr_ms)
                s_eng->rpm = s_st.rpm + jit * 45.f * (s_st.rpm / 850.f);   // ±~12об/мин @850
            engine_set_throttle(s_eng, thr);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void stand_start(engine_t *e) {
    s_eng = e;
    cfg_load();
    gpio_config_t io = { .mode = GPIO_MODE_INPUT, .pull_up_en = 1, .pull_down_en = 0,
        .pin_bit_mask = (1ULL<<PIN_IGN)|(1ULL<<PIN_THR0)|(1ULL<<PIN_THR1)|(1ULL<<PIN_GEAR) };
    gpio_config(&io);
    // bench-режим: перемычка GEAR на GND при старте -> стенд сразу разрешён.
    // В машине перемычки нет -> s_enabled=0, пины не могут захватить управление.
    vTaskDelay(pdMS_TO_TICKS(300));                    // антидребезг/установка pull-up
    if (gpio_get_level(PIN_GEAR) == 0) { s_enabled = 1; ESP_LOGI("stand", "bench-режим: стенд ВКЛ (GEAR замкнут при старте)"); }
    adc_oneshot_unit_init_cfg_t uc = { .unit_id = ADC_UNIT_1 };
    if (adc_oneshot_new_unit(&uc, &s_adc) == ESP_OK) {
        adc_oneshot_chan_cfg_t cc = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
        adc_oneshot_config_channel(s_adc, ADC_CH, &cc);
    } else s_adc = NULL;
    xTaskCreate(stand_task, "stand", 3072, NULL, 4, NULL);
    ESP_LOGI("stand", "pины: IGN=%d THR=%d,%d GEAR=%d ADC=%d (enabled=%d)", PIN_IGN, PIN_THR0, PIN_THR1, PIN_GEAR, PIN_THR_ADC, s_enabled);
}
