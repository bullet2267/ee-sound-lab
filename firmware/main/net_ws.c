#include "net_ws.h"
#include "trip_log.h"
#include "ota.h"
#include "stand.h"
#include "thor_proto.h"   /* g_thor_dbg — WS cmd 0x0A */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_random.h"   /* ПРАВКА (10): дрожание оборотов в drag_task */
#include "esp_timer.h"    /* ПРАВКА (11): drag profile — wall-clock t */
#include <string.h>
#include <stdlib.h>    /* ПРАВКА 2026-11-19: atoi для ?sec= */
#include <math.h>      /* ПРАВКА 2026-11-19: expf для сброса в драге */
#include <dirent.h>    /* ПРАВКА 2026-11-19 (5): скан /sd для /packs */

// Страница пульта вшита в прошивку (EMBED_TXTFILES "web/index.html" в CMakeLists).
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

#define MAX_CLIENTS 4

static const char *TAG = "net_ws";
static engine_t *s_eng;
static ctrl_preset_cb s_preset;
static httpd_handle_t s_srv = NULL;
static int s_fds[MAX_CLIENTS];      // fd websocket-клиентов, 0 = пусто

static void apply_cmd(uint8_t cmd, const uint8_t *p, int len) {
    switch (cmd) {
        case 0x01: if (len>=1) s_eng->masterGain = p[0] / 255.0f; break;
        case 0x02: if (len>=1 && s_preset) s_preset(p[0]); break;
        case 0x03: if (len>=1) engine_trigger_event(s_eng, p[0]); break;
        case 0x04: if (len>=1) trip_log_set(p[0] != 0); break;
        case 0x05: if (len>=2) { s_eng->rpm_ovr = p[0] | (p[1]<<8);        // тестовые обороты u16
                                 s_eng->rpm_ovr_ms = 3000; } break;
        // --- стенд ---
        case 0x06: if (len>=1) stand_set_ign(s_eng, p[0]);                    // зажигание 0/1
                   if (len>=2) stand_set_thr(s_eng, p[1] / 255.0f);           // газ 0..255
                   if (len>=3) { if (p[2]==1) stand_gear_cmd(s_eng,+1);       // передача
                                  else if (p[2]==2) stand_gear_cmd(s_eng,-1); }
                   if (len>=3 && p[2]==3) stand_set_thr(s_eng, -1.f);         // газ назад на пины
                   break;
        case 0x07: if (len>=2) stand_set_idle(s_eng, (float)(p[0] | (p[1]<<8))); break; // холостые rpm
        case 0x08: if (len>=1) engine_set_reverb(p[0] / 255.0f); break;                 // реверб «кожуха» 0..255
        case 0x09: if (len>=1) stand_set_mode(s_eng, p[0]); break;                      // режим стенда: 0=АКПП, 1=газовка на месте
        case 0x0A: if (len>=1) g_thor_dbg = p[0] != 0;                            // thor:-логи 0/1
                   ESP_LOGI(TAG, "thor dbg=%d", g_thor_dbg); break;
        default: ESP_LOGW(TAG, "unknown cmd 0x%02x", cmd);
    }
}

static void remember_fd(int fd) {
    ESP_LOGI(TAG, "ws frame on fd=%d info=%d", fd, httpd_ws_get_fd_info(s_srv, fd));
    for (int i=0;i<MAX_CLIENTS;i++) if (s_fds[i]==fd) return;
    for (int i=0;i<MAX_CLIENTS;i++) if (s_fds[i]==0) { s_fds[i]=fd; return; }
}

// GET / — отдаём страницу пульта (вшита в прошивку).
static esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, (const char *)index_html_start,
                    index_html_end - index_html_start - 1);  // -1: вшитый txt оканчивается \0
    return ESP_OK;
}

// GET /rec — арм захвата аудио 5с (моно int16 @ bank SR); GET /dump.wav — WAV.
extern void engine_dump_arm(uint32_t frames, uint32_t rate);
extern uint32_t engine_dump_pos(void), engine_dump_rate(void), engine_render_calls(void);
extern const int16_t *engine_dump_buf(void);
/* ПРАВКА 2026-11-19: длительность дампа параметром. Было жёстко 5с, а драг-
 * заезд 20с — одним дампом не снять, а сравнивать с рендерами надо целиком. */
static uint32_t rec_secs(httpd_req_t *req) {
    char q[32]; uint32_t sec = 5;
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        char v[8];
        if (httpd_query_key_value(q, "sec", v, sizeof(v)) == ESP_OK) {
            int s = atoi(v);
            if (s >= 1 && s <= 25) sec = (uint32_t)s;
        }
    }
    return sec;
}
static esp_err_t rec_handler(httpd_req_t *req) {
    uint32_t rate = (s_eng->bank && s_eng->bank->sampleRate) ? s_eng->bank->sampleRate : 22050;
    uint32_t sec = rec_secs(req);
    engine_dump_arm(rate * sec, rate);
    httpd_resp_set_type(req, "text/plain");
    if (engine_dump_buf()) {
        char m[48]; snprintf(m, sizeof(m), "armed %lus @%luHz", (unsigned long)sec, (unsigned long)rate);
        httpd_resp_sendstr(req, m);
    }
    else                  httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "dump malloc failed");
    return ESP_OK;
}

/* ПРАВКА 2026-11-19: ВОСПРОИЗВОДИМЫЙ ДРАГ-ЗАЕЗД на плате.
 * Жест в точности совпадает с render_pwr_drag.py, иначе дамп не сравнить с
 * рендерами и с эталонами. Шаг 20мс, обороты через rpm_ovr, газ через стенд.
 * 0-2с ХХ | 2.0-2.3 пуск | до 5.6 первая | 5.6-5.9 шифт | ... | 17-20 сброс. */
static volatile int s_drag_mode = 0;            /* 0=заезд, 1=подгазовки на месте */
static void drag_task(void *arg) {
    const float DRAG[][4] = {   /* t0, t1, rpm0, rpm1 ; газ отдельно ниже */
        {0.0f,  2.0f,  800.f,  800.f},
        {2.0f,  2.3f,  800.f, 2200.f},
        {2.3f,  5.6f, 2200.f, 6200.f},
        {5.6f,  5.9f, 6200.f, 4100.f},
        {5.9f,  9.0f, 4100.f, 6200.f},
        {9.0f,  9.3f, 6200.f, 4300.f},
        {9.3f, 13.0f, 4300.f, 6200.f},
        {13.0f,13.3f, 6200.f, 4500.f},
        {13.3f,17.0f, 4500.f, 6200.f},
    };
    /* ПРАВКА 2026-11-19 (7): mode=rev — «холостой + газовка на месте».
     * Короткие тычки педали разной силы + один держ ~2800 — диагностика
     * поведения движка на открытии/закрытии дросселя без длинного заезда. */
    const float REV[][4] = {
        {0.0f,  2.5f, 1000.f, 1000.f},   /* чистый холостой — k35 авторских 1000 */
        {2.5f,  2.9f, 1000.f, 2200.f},   /* тычок */
        {2.9f,  4.3f, 2200.f, 1000.f},   /* спад 1.4с — не моментальный */
        {4.3f,  4.8f, 1000.f, 3400.f},
        {4.8f,  6.3f, 3400.f, 1000.f},
        {6.3f,  6.9f, 1000.f, 4600.f},
        {6.9f,  8.2f, 4600.f, 1000.f},
        {8.2f,  9.6f, 1000.f, 2800.f},   /* подъём к держу ~1290об/с — темп
         * эталона (render_pwr2: 1100об/с). На 4500об/с хвосты весов выбывших
         * окон наслаивались на новые = «кусочки один на другой». Газ держа
         * (см. thr ниже) — жест идёт одним уровнем, как в эталоне. */
        {9.6f, 10.9f, 2800.f, 2800.f},   /* держ ПЛОСКИЙ: скриптовая просадка
         * 2800->2550 звучала как «газ подвис» (питч плывёт вниз при
         * держанном газе). Эталон (render_pwr2) держит rpm ровно. */
        {10.9f,12.2f, 2800.f, 1000.f},   /* спад из держа */
        {12.2f,12.7f, 1000.f, 5600.f},   /* резкий тычок */
        {12.7f,14.6f, 5600.f, 1000.f},
        {14.6f,15.0f, 1000.f, 2000.f},   /* двойной тычок */
        {15.0f,15.5f, 2000.f, 3600.f},
        {15.5f,17.2f, 3600.f, 1000.f},
        {17.2f,18.2f, 1000.f, 1000.f},
        {18.2f,18.8f, 1000.f, 4200.f},
        {18.8f,20.0f, 4200.f, 1000.f},
    };
    const float (*T)[4] = s_drag_mode ? REV : DRAG;
    const int NS_ = s_drag_mode ? (int)(sizeof(REV)/sizeof(REV[0]))
                                : (int)(sizeof(DRAG)/sizeof(DRAG[0]));
    /* ПРАВКА 2026-11-19 (9): стендовый захват на авторском уровне пакета.
     * masterGain затирался до ~0.004 мусорной WS/BLE-командой громкости —
     * все BOARD_*.wav писались -40дБ и юзер слушал «очень тихо». pkgGain —
     * mainVolume из pkgrl (S63=50 → 0.5); 1.0 забивает tanh-лимитер в кашу. */
    s_eng->masterGain = s_eng->pkgGain > 0.01f ? s_eng->pkgGain : 0.5f;
    const int64_t t0 = esp_timer_get_time();
    for (;;) {
        /* wall-clock t: vTaskDelay(1 tick@100Hz) реально спит 0-10мс —
         * ms-счётчик бежал быстрее секунд и сдвигал сегменты профиля. */
        float t = (float)(esp_timer_get_time() - t0) / 1000000.0f;
        if (t > 20.f) break;
        float rpm = 900.f, thr = 0.f;
        if (!s_drag_mode && t >= 17.0f) {         /* сброс: экспонента как в рендере */
            float u = (t - 17.0f) / 3.0f;
            rpm = 900.f + (6200.f - 900.f) * expf(-3.0f * u);
        } else {
            for (int i = 0; i < NS_; i++) {
                if (t >= T[i][0] && t < T[i][1]) {
                    float u = (t - T[i][0]) / (T[i][1] - T[i][0]);
                    int sag = (T[i][3] < T[i][2] && T[i][3] >= T[i][2] * 0.85f);
                    if (T[i][3] < T[i][2] && !sag) {   /* спад: быстрый старт */
                        float ue = 1.f - (1.f - u) * (1.f - u);  /* хвост к ХХ */
                        rpm = T[i][2] + (T[i][3] - T[i][2]) * ue;
                    } else {
                        rpm = T[i][2] + (T[i][3] - T[i][2]) * u;  /* линейно */
                    }
                    /* подъём: тычок — газ 0.9; подъём В ДЕРЖ — газ держа.
                     * Эталон ведёт ramp+hold одним уровнем thr=0.45; у нас
                     * 0.9 «передерживался» в держ ещё ~150мс после входа. */
                    if (T[i][3] > T[i][2]) {
                        int intoHold = (i + 1 < NS_
                                        && T[i+1][2] == T[i+1][3]
                                        && T[i+1][2] >= T[i][3] - 1.f
                                        && T[i+1][2] > 1200.f);
                        thr = intoHold ? 0.55f : 0.9f;
                    } else if (T[i][2] > 1200.f && (T[i][3] >= T[i][2] || sag))
                                                thr = 0.55f;   /* держ: эталон
                     * thr~0.45 + чуть тяжелее, иначе уровень провисает в 3x
                     * против тычков = «зависло-отвисло»; плавная просадка
                     * <=15% — тоже держ, иначе thr=0 уйдёт в сброс-бурление */
                    break;
                }
            }
        }
        /* дрожание ВХОДНОГО сигнала: живой CAN rpm квантуется ~±0.4%
         * (~±10об на 2800) — реальный уровень шума ЭБУ. Было ±3% (±84об):
         * на держе это гуляло питчем ~±30 центов («газ не отпускается»)
         * и трещало весами гранул на краях фейдов k5/k6 (~100-200об шириной)
         * — гранулы на границе окон дёргались 0↔40% = «тарахтенье». */
        static float wob = 0.f;
        wob += ((float)(esp_random() & 0xFF) / 255.f - 0.5f) * 0.0008f;
        if (wob > 0.004f) wob = 0.004f; else if (wob < -0.004f) wob = -0.004f;
        s_eng->rpm_ovr = (uint16_t)(rpm * (1.f + wob));
        s_eng->rpm_ovr_ms = 200;                /* перекрывается каждые 5мс */
        stand_set_thr(s_eng, thr);
        /* ПРЯМО в движок: без зажигания стенд не владеет throttleTarget —
         * газ профиля терялся, thr=0 весь прогон: зональные попы стреляли
         * в разгоне (хрип при нажатии) и слой сброса играл вместо нагрузки */
        engine_set_throttle(s_eng, thr);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    stand_set_thr(s_eng, 0.f);
    s_eng->rpm_ovr_ms = 0;
    vTaskDelete(NULL);
}
static esp_err_t drag_handler(httpd_req_t *req) {
    uint32_t rate = (s_eng->bank && s_eng->bank->sampleRate) ? s_eng->bank->sampleRate : 22050;
    uint32_t sec = rec_secs(req);
    if (sec < 20) sec = 20;
    char q[32] = {0};
    s_drag_mode = (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK)
               && strstr(q, "mode=rev") ? 1 : 0;
    /* bf=0 — стенд без прострелов: отделяет грязь бэкфайров от основы */
    s_eng->popMute = strstr(q, "bf=0") ? 1 : 0;
    engine_dump_arm(rate * sec, rate);
    httpd_resp_set_type(req, "text/plain");
    if (!engine_dump_buf()) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "dump malloc failed");
        return ESP_FAIL;
    }
    if (xTaskCreate(drag_task, "drag", 3072, NULL, 4, NULL) != pdPASS) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "task create failed");
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, s_drag_mode ? "rev started, 20s — then GET /dump.wav"
                                        : "drag started, 20s — then GET /dump.wav");
    return ESP_OK;
}
/* GET /vol?x=0..100 — громкость движка с веба (та же ручка, что BLE mainVolume) */
static esp_err_t vol_handler(httpd_req_t *req) {
    char q[32] = {0}; int v = -1;
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        char b[8] = {0};
        if (httpd_query_key_value(q, "x", b, sizeof(b)) == ESP_OK) v = atoi(b);
    }
    if (v >= 0) { if (v > 100) v = 100; s_eng->masterGain = v / 100.f; }
    httpd_resp_set_type(req, "text/plain");
    char out[32]; snprintf(out, sizeof(out), "mg=%.2f", s_eng->masterGain);
    httpd_resp_sendstr(req, out);
    return ESP_OK;
}
/* ПРАВКА 2026-11-19 (5): GET /packs — что РЕАЛЬНО лежит на SD и читается ли.
 * Поводом был pkg144: лог чистый, rc=0, а банк пустой (grains=0) из-за битого
 * f_05 на карте. Зеркальная копия того же пакета парсится в 34/45, значит
 * формат не при чём — ломается именно копия. Отпечаток grains/evs теперь
 * доступен ДО загрузки, по каждому паку, вместе с размерами файлов. */
static esp_err_t packs_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    char *out = malloc(2048);
    if (!out) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom"); return ESP_FAIL; }
    int w = snprintf(out, 2048, "[");
    uint16_t pk[16]; long smpSz[16], rlsSz[16]; int np = 0;
    for (int i = 0; i < 16; i++) { smpSz[i] = rlsSz[i] = -1; }
    DIR *dp = opendir("/sd"); struct dirent *de;
    while (dp && (de = readdir(dp))) {
        int isS = (strncmp(de->d_name, "f_04", 4) == 0);
        int isR = (strncmp(de->d_name, "f_05", 4) == 0);
        if (!isS && !isR) continue;
        unsigned long fid = strtoul(de->d_name + 2, NULL, 16);
        uint16_t p = (fid >> 8) & 0xFFFF;
        int k = -1;
        for (int i = 0; i < np; i++) if (pk[i] == p) k = i;
        if (k < 0) { if (np >= 16) continue; k = np++; pk[k] = p; }
        char path[64]; snprintf(path, sizeof(path), "/sd/%.48s", de->d_name);
        FILE *f = fopen(path, "rb");
        long sz = -1;
        if (f) { fseek(f, 0, SEEK_END); sz = ftell(f); fclose(f); }
        if (isS) smpSz[k] = sz; else rlsSz[k] = sz;
    }
    if (dp) closedir(dp);
    for (int i = 0; i < np; i++) {
        int gc = -1, ec = -1;                      /* -1 = не смогли посчитать */
        if (rlsSz[i] > 8 && rlsSz[i] < 262144) {
            DIR *d2 = opendir("/sd"); struct dirent *e2; char rp[64] = {0};
            char want[12]; snprintf(want, sizeof(want), "f_05%04x", pk[i]);
            while (d2 && (e2 = readdir(d2)))
                if (strncmp(e2->d_name, want, 8) == 0) snprintf(rp, sizeof(rp), "/sd/%.48s", e2->d_name);
            if (d2) closedir(d2);
            FILE *rf = rp[0] ? fopen(rp, "rb") : NULL;
            if (rf) {
                uint8_t *rb = malloc((size_t)rlsSz[i]);
                if (rb && fread(rb, 1, rlsSz[i], rf) == (size_t)rlsSz[i] &&
                    memcmp(rb, "RLS2", 4) == 0) {
                    int rn = (rb[4] << 8) | rb[5]; gc = 0; ec = 0;
                    for (int q = 0; q < rn; q++) {
                        uint32_t o = 6 + (uint32_t)q * 4;
                        if (o + 4 > (uint32_t)rlsSz[i]) break;
                        uint32_t ro = ((uint32_t)rb[o]<<24)|((uint32_t)rb[o+1]<<16)|
                                      ((uint32_t)rb[o+2]<<8)|rb[o+3];
                        if (ro + 6 > (uint32_t)rlsSz[i]) break;
                        int npair = (rb[ro+2] << 8) | rb[ro+3];
                        int k0 = -1;
                        for (int j = 0; j < npair; j++) {
                            uint32_t kp = ro + 4 + (uint32_t)j * 4;
                            if (kp + 4 > (uint32_t)rlsSz[i]) break;
                            if (((rb[kp] << 8) | rb[kp+1]) == 0) { k0 = (rb[kp+2] << 8) | rb[kp+3]; break; }
                        }
                        if (k0 == 4 || k0 == 5 || k0 == 6) ec++; else if (k0 >= 0) gc++;
                    }
                }
                free(rb); fclose(rf);
            }
        }
        w += snprintf(out + w, (w < 2000 ? 2048 - w : 1),
            "%s{\"pkg\":%u,\"smp\":%ld,\"rls\":%ld,\"grains\":%d,\"evs\":%d,\"ok\":%s}",
            i ? "," : "", pk[i], smpSz[i], rlsSz[i], gc, ec,
            (smpSz[i] > 0 && rlsSz[i] > 0 && gc > 0) ? "true" : "false");
        if (w > 1980) break;
    }
    snprintf(out + w, 2048 - w, "]");
    httpd_resp_sendstr(req, out);
    free(out);
    return ESP_OK;
}
static esp_err_t diag_handler(httpd_req_t *req) {
    char j[512];
    snprintf(j, sizeof(j),
        "{\"rpm\":%.0f,\"thr\":%.2f,\"preset\":%u,\"run\":%u,\"stand\":%u,"
        "\"bank\":%s,\"grains\":%u,\"sr\":%lu,\"evs\":%u,"
        "\"dump\":%lu,\"rc\":%lu,\"mg\":%.2f,\"ig\":%.2f,\"ip\":%.2f,\"wp\":%.2f,"
        "\"on\":%d,\"off\":%d,"
        "\"gm\":%.0f,\"gn\":%.1f,\"gw\":%.2f,\"law\":%.3f,"
        "\"ev\":%.0f,\"sgu\":%.0f,\"out\":%.0f}",
        s_eng->rpm, s_eng->throttle, s_eng->preset, s_eng->engineRunning,
        s_eng->standActive,
        s_eng->bank ? "yes" : "null",
        s_eng->bank ? s_eng->bank->grainCount : 0,
        s_eng->bank ? (unsigned long)s_eng->bank->sampleRate : 0UL,
        s_eng->bank ? s_eng->bank->eventCount : 0,
        (unsigned long)engine_dump_pos(), (unsigned long)engine_render_calls(),
        s_eng->masterGain, s_eng->idleGain, s_eng->idlePitchOff, s_eng->workPitchOff,
        s_eng->bank ? rsb_count_layer(s_eng->bank, 0) : 0,
        s_eng->bank ? rsb_count_layer(s_eng->bank, 1) : 0,
        s_eng->dbgGrMix, s_eng->dbgGrN, s_eng->dbgGrW, s_eng->dbgLaw,
        s_eng->dbgEv, s_eng->dbgSgu, s_eng->dbgOut);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, j);
    return ESP_OK;
}
// GET /dbg — таймлайн рендера: кольцо 400 записей по ~6мс (rpm thr gm gn gw law ev sgu out)
static esp_err_t dbg_handler(httpd_req_t *req) {
    char *out = malloc(400 * 80 + 16);
    if (!out) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom"); return ESP_OK; }
    int w = snprintf(out, 100, "rpm thr gm gn gw law ev sgu out\n");
    uint16_t p = s_eng->dbgLogPos;
    for (int i = 0; i < 400; i++) {
        const float *L = s_eng->dbgLog[(p + i) % 400];
        w += snprintf(out + w, 400*80 - w, "%.0f %.2f %.0f %.1f %.2f %.3f %.0f %.0f %.0f\n",
                      L[0], L[1], L[2], L[3], L[4], L[5], L[6], L[7], L[8]);
    }
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, out, w);
    free(out);
    return ESP_OK;
}
/* ПРАВКА 2026-11-20: GET /files — полный листинг /sd (имя+размер).
 * Поводом был 0x0046 DeleteSoundPackage: app в магазине снова рисовал пакет
 * «скачанным» — без листинга карты было не проверить, что remove() реально
 * сработал и ни один f_*<pkg> не остался. */
static esp_err_t files_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    char *out = malloc(3072);
    if (!out) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom"); return ESP_FAIL; }
    int w = snprintf(out, 3072, "[");
    DIR *dp = opendir("/sd"); struct dirent *de;
    int n = 0;
    while (dp && (de = readdir(dp))) {
        char path[64]; snprintf(path, sizeof(path), "/sd/%.48s", de->d_name);
        FILE *f = fopen(path, "rb"); long sz = -1;
        if (f) { fseek(f, 0, SEEK_END); sz = ftell(f); fclose(f); }
        w += snprintf(out + w, (w < 3000 ? 3072 - w : 1),
                      "%s\"%s\":%ld", n ? "," : "", de->d_name, sz);
        n++;
        if (w > 2960) break;
    }
    if (dp) closedir(dp);
    snprintf(out + w, 3072 - w, "]");
    httpd_resp_sendstr(req, out);
    free(out);
    return ESP_OK;
}
static esp_err_t dump_handler(httpd_req_t *req) {
    const int16_t *pcm = engine_dump_buf();
    uint32_t n = engine_dump_pos(), rate = engine_dump_rate();
    if (!pcm || !n) { httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no dump — /rec first"); return ESP_FAIL; }
    uint8_t wh[44] = { 'R','I','F','F', 36,0,0,0, 'W','A','V','E','f','m','t',' ',
                       16,0,0,0, 1,0, 1,0, 0,0,0,0, 0,0,0,0, 2,0, 16,0,
                       'd','a','t','a', 0,0,0,0 };
    uint32_t dsz = n * 2, rsz = 36 + dsz, br = rate * 2;
    memcpy(wh + 4, &rsz, 4); memcpy(wh + 24, &rate, 4); memcpy(wh + 28, &br, 4);
    memcpy(wh + 40, &dsz, 4);
    httpd_resp_set_type(req, "audio/wav");
    httpd_resp_send_chunk(req, (const char *)wh, 44);
    httpd_resp_send_chunk(req, (const char *)pcm, dsz);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// Обработчик /ctrl: handshake (GET) + входящие бинарные кадры команд.
static esp_err_t ws_handler(httpd_req_t *req) {
    // IDF6: handshake GET обрабатывается внутри httpd — сюда доходят ТОЛЬКО
    // фреймы. fd запоминаем на любом вызове — первый же клиентский кадр
    // (UI шлёт громкость при onopen) регистрирует нас в рассылке.
    remember_fd(httpd_req_to_sockfd(req));
    if (req->method == HTTP_GET) {                 // рукопожатие (если всё же доходит)
        ESP_LOGI(TAG, "ws client connected");
        return ESP_OK;
    }
    httpd_ws_frame_t f = { .type = HTTPD_WS_TYPE_BINARY };
    if (httpd_ws_recv_frame(req, &f, 0) != ESP_OK) return ESP_FAIL;  // узнать длину
    if (f.len == 0) return ESP_OK;
    if (f.len > 64) {                            // кадр больше буфера — ВЫЧИТАТЬ полностью,
        uint8_t dump[64];                        // иначе хвост отравляет следующий recv
        size_t left = f.len;
        while (left) {
            httpd_ws_frame_t d = { .type = HTTPD_WS_TYPE_BINARY, .payload = dump,
                                   .len = left > sizeof(dump) ? sizeof(dump) : left };
            if (httpd_ws_recv_frame(req, &d, d.len) != ESP_OK) return ESP_FAIL;
            left -= d.len;
        }
        return ESP_OK;
    }
    uint8_t buf[64];
    f.payload = buf;
    if (httpd_ws_recv_frame(req, &f, f.len) != ESP_OK) return ESP_FAIL;
    apply_cmd(buf[0], &buf[1], (int)f.len - 1);     // [cmd][payload...]
    return ESP_OK;
}

// Рассылка телеметрии всем ws-клиентам ~10 Гц (async-отправка из рабочей очереди httpd).
static void telem_task(void *arg) {
    while (1) {
        uint16_t rpm = (uint16_t)(s_eng->rpm < 0 ? 0 : s_eng->rpm);
        uint8_t payload[10] = {
            (uint8_t)(rpm & 0xFF), (uint8_t)(rpm >> 8),
            (uint8_t)(s_eng->throttle * 255.0f),
            (uint8_t)((s_eng->throttle > 0.5f ? 1 : 0) | (s_eng->bfLeft > 0 ? 2 : 0)),
            s_eng->preset, s_eng->coolantC, s_eng->errFlags,
            (uint8_t)((s_eng->standActive ? 0x80 : 0) | (s_eng->engineRunning ? 0x40 : 0)
                      | (s_eng->standGear & 0x0F)),   // b8: [стенд][заведён][gear 0-15]
            (uint8_t)(s_eng->canAliveMs > 0 ? 1 : 0),// b9: CAN жив — стенд замьючен
            (uint8_t)(s_eng->masterGain * 255.0f) }; // b10: громкость движка
        // Официальное API: список всех активных клиентов сервера — фильтруем WS.
        int cfds[16]; size_t n = sizeof(cfds)/sizeof(cfds[0]);
        if (httpd_get_client_list(s_srv, &n, cfds) == ESP_OK) {
            static int dbg = 0;
            for (size_t i = 0; i < n; i++) {
                if (httpd_ws_get_fd_info(s_srv, cfds[i]) != HTTPD_WS_CLIENT_WEBSOCKET) continue;
                httpd_ws_frame_t f = { .type = HTTPD_WS_TYPE_BINARY, .payload = payload, .len = 10 };
                esp_err_t e = httpd_ws_send_frame_async(s_srv, cfds[i], &f);
                if (dbg++ % 30 == 0) ESP_LOGI(TAG, "telem fd=%d/%zu send=%d", cfds[i], n, e);
            }
        }
        // запасной путь — fd из ws_handler (если get_client_list вернёт не то)
        for (int i = 0; i < MAX_CLIENTS; i++) {
            int fd = s_fds[i];
            if (fd == 0) continue;
            httpd_ws_frame_t f = { .type = HTTPD_WS_TYPE_BINARY, .payload = payload, .len = 10 };
            if (httpd_ws_send_frame_async(s_srv, fd, &f) != ESP_OK) s_fds[i] = 0; // отвалился
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void net_ws_start(engine_t *engine, ctrl_preset_cb on_preset) {
    s_eng = engine; s_preset = on_preset;
    memset(s_fds, 0, sizeof(s_fds));
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    /* ПРАВКА 2026-11-20: дефолтных 8 слотов не хватило — /ota не зарегался */
    cfg.max_uri_handlers = 16;
    if (httpd_start(&s_srv, &cfg) != ESP_OK) { ESP_LOGE(TAG, "httpd start failed"); return; }
    httpd_uri_t uri = {0};
    uri.uri = "/ctrl"; uri.method = HTTP_GET; uri.handler = ws_handler;
    uri.is_websocket = true;
    httpd_register_uri_handler(s_srv, &uri);

    httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = root_handler };
    httpd_register_uri_handler(s_srv, &root);   // страница пульта

    httpd_uri_t rec = { .uri = "/rec", .method = HTTP_GET, .handler = rec_handler };
    httpd_register_uri_handler(s_srv, &rec);    // арм аудио-дампа 5с
    httpd_uri_t dmp = { .uri = "/dump.wav", .method = HTTP_GET, .handler = dump_handler };
    httpd_register_uri_handler(s_srv, &dmp);    // скачать захваченный WAV
    httpd_uri_t dia = { .uri = "/diag", .method = HTTP_GET, .handler = diag_handler };
    httpd_register_uri_handler(s_srv, &dia);    // JSON-диагностика движка
    httpd_uri_t dbg = { .uri = "/dbg", .method = HTTP_GET, .handler = dbg_handler };
    httpd_register_uri_handler(s_srv, &dbg);    // таймлайн уровней рендера
    /* ПРАВКА 2026-11-19: воспроизводимый драг для сравнения с рендерами */
    httpd_uri_t drg = { .uri = "/drag", .method = HTTP_GET, .handler = drag_handler };
    httpd_register_uri_handler(s_srv, &drg);
    httpd_uri_t vol = { .uri = "/vol", .method = HTTP_GET, .handler = vol_handler };
    httpd_register_uri_handler(s_srv, &vol);
    /* ПРАВКА 2026-11-19 (5): список паков на SD с проверкой целостности */
    httpd_uri_t pks = { .uri = "/packs", .method = HTTP_GET, .handler = packs_handler };
    httpd_register_uri_handler(s_srv, &pks);
    /* ПРАВКА 2026-11-20: листинг /sd — верификация удаления пакетов */
    httpd_uri_t fls = { .uri = "/files", .method = HTTP_GET, .handler = files_handler };
    httpd_register_uri_handler(s_srv, &fls);

    ota_register(s_srv);                        // POST /ota — обновление прошивки

    xTaskCreatePinnedToCore(telem_task, "ws_telem", 4096, NULL, 4, NULL, 0);
    ESP_LOGI(TAG, "pult: http://<ip>/  ·  ws://<ip>/ctrl  ·  OTA: POST /ota");
}
