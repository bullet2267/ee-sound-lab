#include "can_rpm.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcpp"          // legacy TWAI deprecated в IDF v6.1 — API рабочее
#include "driver/twai.h"
#pragma GCC diagnostic pop
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "can_rules.h"

// --- пины TWAI/CAN (под свою разводку) ---
// JC-ESP32P4-M3-DEV: GPIO14–19 заняты SDIO к C6. CAN на свободных 23/24.
#define CAN_TX  GPIO_NUM_23
#define CAN_RX  GPIO_NUM_24

// --- THOR-режим: события кнопок руля для main (0=нет,1=mute,2=след.пресет) ---
volatile int g_can_btn_evt = 0;

// Для PASSIVE: ID кадра с оборотами и раскладка байт зависят от модели.
#define RPM_FRAME_ID      0x0C0     // ПРИМЕР — замените на реальный
#define RPM_HI_BYTE       3
#define RPM_LO_BYTE       4
#define RPM_SCALE         0.25f     // многие ЭБУ: (A*256+B)/4

static const char *TAG = "can_rpm";

static float lpf(float prev, float x, float a) { return prev + a * (x - prev); }

// Если жив оверрайд отладки (ctrl cmd 0x05) — используем его вместо шины.
static inline int rpm_override(engine_t *e, uint16_t dec_ms) {
    if (e->rpm_ovr_ms) {
        e->rpm_ovr_ms = e->rpm_ovr_ms > dec_ms ? e->rpm_ovr_ms - dec_ms : 0;
        engine_set_rpm(e, e->rpm_ovr);
        return 1;
    }
    return 0;
}

static void obd2_poll_task(void *arg) {
    engine_t *e = (engine_t *)arg;
    float rpm = e->bank->rpmMin;
    uint32_t tick = 0, noResp = 0;
    float thrMin = 255.f;                            // learn: холостой уровень PID 0x11
    while (1) {
        // bus-off: recovery → ждём STOPPED → start (start в RECOVERING игнорится!)
        twai_status_info_t st;
        if (twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_BUS_OFF) {
            ESP_LOGW(TAG, "bus-off → recovery");
            twai_initiate_recovery();
            for (int i = 0; i < 100; i++) {              // до ~2с ждём выход из RECOVERING
                vTaskDelay(pdMS_TO_TICKS(20));
                if (twai_get_status_info(&st) != ESP_OK || st.state == TWAI_STATE_STOPPED)
                    break;
            }
            twai_start();
            ESP_LOGI(TAG, "TWAI restarted, state=%d", st.state);
        }
        if (rpm_override(e, 40)) { vTaskDelay(pdMS_TO_TICKS(40)); continue; }
        if (e->standActive) { vTaskDelay(pdMS_TO_TICKS(80)); continue; }   // стенд владеет входами
        twai_message_t rx; int got = 0;
        twai_message_t req = { .identifier = 0x7DF, .data_length_code = 8,
            .data = {0x02, 0x01, 0x0C, 0, 0, 0, 0, 0} };
        twai_transmit(&req, pdMS_TO_TICKS(20));

        // drain: фильтруем ответы 0x7E8-0x7EF с нужным PID (занятая шина — лишние кадры)
        for (int tr = 0; tr < 4 && !got; tr++) {
            if (twai_receive(&rx, pdMS_TO_TICKS(40)) != ESP_OK) break;
            if (rx.identifier >= 0x7E8 && rx.identifier <= 0x7EF &&
                rx.data_length_code >= 5 && rx.data[1] == 0x41 && rx.data[2] == 0x0C) {
                float r = ((rx.data[3] * 256) + rx.data[4]) / 4.0f;
                rpm = lpf(rpm, r, 0.35f);
                engine_set_rpm(e, rpm); got = 1;
                e->canAliveMs = 2000;                   // шина жива — стенд молчит
            }
        }
        twai_message_t rq2 = { .identifier = 0x7DF, .data_length_code = 8,
            .data = {0x02, 0x01, 0x11, 0, 0, 0, 0, 0} };
        twai_transmit(&rq2, pdMS_TO_TICKS(20));
        for (int tr = 0; tr < 4; tr++) {
            if (twai_receive(&rx, pdMS_TO_TICKS(40)) != ESP_OK) break;
            if (rx.identifier >= 0x7E8 && rx.identifier <= 0x7EF &&
                rx.data_length_code >= 4 && rx.data[1] == 0x41 && rx.data[2] == 0x11) {
                got = 1;
                float raw = rx.data[3];
                if (raw < thrMin) thrMin = raw;       // learn idle throttle level
                else if (thrMin < 255.f) thrMin += 0.05f;  // дрейф вверх — глюк в 0 не навсегда
                float span = 255.f - thrMin;
                float t = span > 1 ? (raw - thrMin) / span : 0;
                engine_set_throttle(e, t < 0 ? 0 : t > 1 ? 1 : t);
            }
        }
        // CAN-таймаут: нет ответов >1с → rpm в ноль (мотор заглушен → тишина)
        noResp = got ? 0 : noResp + 1;
        if (noResp > 8) { engine_set_rpm(e, 0); rpm = e->bank ? e->bank->rpmMin : 0;
                          thrMin = 255.f; }              // мотор выкл — learn заново
        // температура ОЖ PID 0x05 (A-40 °C) — реже, раз в ~1.5 c
        if ((++tick % 75) == 0) {
            twai_message_t rq3 = { .identifier = 0x7DF, .data_length_code = 8,
                .data = {0x02, 0x01, 0x05, 0, 0, 0, 0, 0} };
            twai_transmit(&rq3, pdMS_TO_TICKS(20));
            if (twai_receive(&rx, pdMS_TO_TICKS(50)) == ESP_OK) {
                if (rx.data_length_code >= 4 && rx.data[1] == 0x41 && rx.data[2] == 0x05)
                    e->coolantC = (uint8_t)(rx.data[3] > 40 ? rx.data[3] - 40 : 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(60));   // ~10-15 Гц опрос — не душим шлюз
    }
}

static void passive_task(void *arg) {
    engine_t *e = (engine_t *)arg;
    float rpm = e->bank->rpmMin;
    twai_message_t rx;
    uint32_t noFrame = 0;
    while (1) {
        twai_status_info_t st;
        if (twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_BUS_OFF) {
            ESP_LOGW(TAG, "bus-off → recovery");
            twai_initiate_recovery();
            for (int i = 0; i < 100; i++) {
                vTaskDelay(pdMS_TO_TICKS(20));
                if (twai_get_status_info(&st) != ESP_OK || st.state == TWAI_STATE_STOPPED)
                    break;
            }
            twai_start();
        }
        if (twai_receive(&rx, pdMS_TO_TICKS(40)) == ESP_OK) {
            noFrame = 0;
            if (rpm_override(e, 40)) continue;
            if (e->standActive) continue;            // стенд владеет входами
            if (rx.identifier == RPM_FRAME_ID && rx.data_length_code > RPM_LO_BYTE) {
                float r = (rx.data[RPM_HI_BYTE] * 256 + rx.data[RPM_LO_BYTE]) * RPM_SCALE;
                rpm = lpf(rpm, r, 0.4f);
                engine_set_rpm(e, rpm);
                e->canAliveMs = 2000;               // шина жива — стенд молчит
            }
        } else if (++noFrame > 25 && !e->rpm_ovr_ms) {   // >1с тишины в шине — мотор заглушен
            engine_set_rpm(e, 0);
            rpm = e->bank ? e->bank->rpmMin : 0;
        }
    }
}

// ================= THOR-режим: декодер сигналов по правилам авто =================
// Слоты: 0=ОБОРОТЫ двигателя (уже в rpm после >>2), 1=режим/передача 4=педаль; kind2=зажигание.
typedef struct {
    float    rpmRaw, speedMax;
    uint8_t  modeByte, pedalByte;
    uint8_t  hasSpeed, hasMode, hasPedal, hasIgn, ignState;
    uint8_t  lastMode;
    float    pedalMax;                              // шкала педали (cl из правила, иначе 255)
    float    lastSpeed;
    uint32_t lastRxMs, lastSpeedMs;
    uint32_t dumpCnt;                               // raw-лог совпавших кадров
} thor_can_t;

static int thor_rx_frame(thor_can_t *st, const car_can_t *cc,
                         uint32_t id, const uint8_t *d, int len) {
    int matched = 0;
    for (int i = 0; i < cc->n_rules; i++) {
        const can_rule_t *r = &cc->rules[i];
        if (r->id != id) continue;
        matched = 1;
        switch (r->kind) {
        case 0: {                                   // u16le(b0:b1)>>shift → speed
            if (r->b1 >= len) break;
            uint32_t v = ((uint32_t)d[r->b0] | ((uint32_t)d[r->b1] << 8)) >> r->shift;
            if (r->cl && v > r->cl) v = r->cl;
            st->rpmRaw = v; if (r->cl) st->speedMax = r->cl;
            st->hasSpeed = 1; st->lastSpeedMs = xTaskGetTickCount() * portTICK_PERIOD_MS;
            break; }
        case 1: {                                   // byte[b0] → slot
            if (r->b0 >= len) break;
            uint8_t v = d[r->b0];
            if (r->slot == 4) { st->pedalByte = v; st->hasPedal = 1; }
            else              { st->modeByte  = v; st->hasMode  = 1; }
            break; }
        case 2: {                                   // состояние зажигания/режима
            if (r->b0 >= len) break;
            st->ignState = d[r->b0]; st->hasIgn = 1;
            break; }
        case 3:                                     // BE-битовое поле (Motorola)
        case 4: {                                   // LE-битовое поле (Intel/VAG)
            uint64_t v;
            if (r->kind == 3) {
                uint64_t fr = 0;
                for (int j = 0; j < len && j < 8; j++) fr = (fr << 8) | d[j];
                int sh = 64 - r->b0 - r->b1;
                if (sh < 0) break;
                v = (fr >> sh) & ((1ULL << r->b1) - 1);
            } else {
                uint64_t fr = 0;
                for (int j = (len > 8 ? 8 : len) - 1; j >= 0; j--) fr = (fr << 8) | d[j];
                v = (fr >> r->b0) & ((1ULL << r->b1) - 1);
            }
            if (r->cl && v > r->cl) v = r->cl;      // клэмп до слота — rpmRaw тоже ограничивается
            if (r->slot == 4) { st->pedalByte = (uint8_t)(v > 255 ? 255 : v); st->hasPedal = 1;
                              if (r->cl) st->pedalMax = r->cl; }
            else if (r->slot == 1) { st->modeByte = (uint8_t)(v > 255 ? 255 : v); st->hasMode = 1; }
            else { st->rpmRaw = (float)v; if (r->cl) st->speedMax = r->cl;
                   st->hasSpeed = 1; st->lastSpeedMs = xTaskGetTickCount() * portTICK_PERIOD_MS; }
            break; }
        }
    }
    // native control: passthrough ID — кнопки руля/круиза
    for (int i = 0; i < cc->n_pass; i++) {
        if (cc->pass_ids[i] != id) continue;
        static uint8_t seen[16]; static uint8_t seenD[16][8];
        if (i < 16 && memcmp(seenD[i], d, len > 8 ? 8 : len) == 0) break;  // повтор кадра
        if (i < 16) { memcpy(seenD[i], d, len > 8 ? 8 : len); if (seen[i]++) break; } // первый кадр игнорим
        // THOR BMW: первая половина pass-группы = MUTE, вторая = переключение звука
        g_can_btn_evt = (i < cc->n_pass / 2) ? 1 : 2;
        break;
    }
    return matched;
}

static void thor_task(void *arg) {
    engine_t *e = (engine_t *)arg;
    const car_can_t *cc = e->carCan;
    thor_can_t st = {0};
    twai_message_t rx;
    float rpm = 0, thr = 0;
    while (1) {
        twai_status_info_t sti;
        if (twai_get_status_info(&sti) == ESP_OK && sti.state == TWAI_STATE_BUS_OFF) {
            twai_initiate_recovery();
            for (int i = 0; i < 100; i++) {
                vTaskDelay(pdMS_TO_TICKS(20));
                if (twai_get_status_info(&sti) != ESP_OK || sti.state == TWAI_STATE_STOPPED) break;
            }
            twai_start();
        }
        while (twai_receive(&rx, pdMS_TO_TICKS(30)) == ESP_OK) {
            if (rx.rtr) continue;
            if (thor_rx_frame(&st, cc, rx.identifier, rx.data, rx.data_length_code)) {
                st.lastRxMs = xTaskGetTickCount() * portTICK_PERIOD_MS;  // кадр НАШЕГО авто
                e->canAliveMs = 2000;
                // raw-дамп: первые 10 совпадений + каждый 50-й — сверка декода с тахометром
                if (++st.dumpCnt <= 10 || st.dumpCnt % 50 == 0) {
                    ESP_LOGI(TAG, "RX %03lx [%d] %02x %02x %02x %02x %02x %02x %02x %02x → rpm=%.0f thr=%d",
                             (unsigned long)rx.identifier, rx.data_length_code,
                             rx.data[0], rx.data[1], rx.data[2], rx.data[3],
                             rx.data[4], rx.data[5], rx.data[6], rx.data[7],
                             st.rpmRaw, st.pedalByte);
                }
            }
        }
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (e->standActive || rpm_override(e, 40)) { vTaskDelay(pdMS_TO_TICKS(40)); continue; }

        // зажигание: по байту состояния; без него — живая силовая шина = зажигание вкл
        // (у VAG/большинства powertrain CAN молчит при выключенном зажигании —
        //  так на холостом при rpm=0 двигатель не "глушится")
        int engineOn;
        if (st.hasIgn)  engineOn = (st.ignState == 1 || st.ignState == 2 ||
                                    st.ignState == 5 || st.ignState == 6);
        else            engineOn = (now - st.lastRxMs < 2000);
        if (now - st.lastRxMs > 2000) engineOn = 0;       // шина мертва — заглушён

        // дроссель: педаль из шины (шкала cl, иначе 255); fallback — производная rpm
        if (st.hasPedal)
            thr = st.pedalByte / (st.pedalMax > 0 ? st.pedalMax : 255.f);
        else {
            float dRpm = (st.rpmRaw - st.lastSpeed) / 0.2f;       // rpm/с
            float a = dRpm > 0 ? dRpm / 3000.f : 0.f;             // ~3000 rpm/с ≈ полный газ
            thr = lpf(thr, a > 1 ? 1 : a, 0.3f);
        }
        st.lastSpeed = st.rpmRaw;

        // slot0 — РЕАЛЬНЫЕ обороты из кадра (nMot u16le>>2): прямое следование за тахометром.
        if (engineOn) {
            rpm = lpf(rpm, st.rpmRaw, 0.35f);
        } else {
            rpm = 0;                                              // глохнет — без затухания
        }
        engine_set_throttle(e, thr);
        engine_set_rpm(e, rpm);
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

void can_thor_start(engine_t *engine, const car_can_t *cc) {
    engine->carCan = cc;
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI THOR listen-only, rules=%d", cc->n_rules);
    xTaskCreatePinnedToCore(thor_task, "canthor", 4096, engine, 6, NULL, 0);
}

void can_rpm_start(engine_t *engine, can_mode_t mode) {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX,
        mode == CAN_MODE_OBD2 ? TWAI_MODE_NORMAL : TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();   // powertrain CAN обычно 500 кбит/с
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI up, mode=%d", mode);

    if (mode == CAN_MODE_OBD2)
        xTaskCreatePinnedToCore(obd2_poll_task, "obd2", 4096, engine, 6, NULL, 0);
    else
        xTaskCreatePinnedToCore(passive_task, "canrx", 4096, engine, 6, NULL, 0);
}
