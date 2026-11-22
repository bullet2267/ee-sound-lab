// main.c — активный звук выхлопа на ESP32-P4.
// CAN(обороты) -> движок кроссфейда/питча (+турбо,+прострелы) -> I2S -> DAC -> class-D -> динамик.
// Управление: UART↔BLE-чип, Ethernet (TCP), WebSocket (веб-пульт) + OTA.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_ota_ops.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "nvs_flash.h"
#include "sdmmc_cmd.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include <dirent.h>
#include "driver/sdmmc_host.h"               // sdmmc_slot_config_t в IDF v6.1
#include "driver/sdmmc_default_configs.h"     // SDMMC_HOST_DEFAULT в IDF v6.1
#include "bank.h"
#include "engine.h"

#define BUILD_VER 585   // версия, которую плата репортит приложению; бампать на каждый релиз
                          // (зеркало отдаёт /api/v2/firmware с version_FW = текущему билду)
// тег версии внутри образа: CommitGroup находит "FWVR"+ver и пишет fw_ver.bin
// ДО долгого esp_ota_write — реконнект аппа сразу видит новую версию
volatile const uint8_t FW_VER_TAG[] = {'F','W','V','R',
    (uint8_t)(BUILD_VER & 0xFF), (uint8_t)(BUILD_VER >> 8),
    (uint8_t)((BUILD_VER & 0xFF) ^ 0xFF), (uint8_t)((BUILD_VER >> 8) ^ 0xFF)};
#include "i2s_out.h"
#include "can_rpm.h"
#include "can_rules.h"
#include "ctrl_uart.h"
#include "thor_proto.h"
#include "trip_log.h"
#include "net_ctrl.h"
#include "net_ws.h"
#include "stand.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_random.h"
#include "driver/gpio.h"

static const char *TAG = "revsound";

void thor_ble_rules_tick(uint32_t ms);
int sd_write_atomic(const char *path, const void *buf, size_t len);
int nvs_wr(const char *key, const void *buf, size_t len);
int nvs_rd(const char *key, void *buf, size_t *len);
void thor_ble_start(engine_t *e, void (*preset)(int),
                    uint16_t hw, uint16_t fw, uint16_t sn);
int  thor_ble_preset_lookup(int idx, uint16_t *pkg, uint16_t *mode);
void thor_ble_apply_mode_rules(uint16_t pkg, uint16_t mode);
void ble_init_and_services(void);

// esp_hosted владеет SDMMC-контроллером — mount не должен его переиницировать
static esp_err_t sdmmc_host_init_dummy(void)   { return ESP_OK; }
static esp_err_t sdmmc_host_deinit_dummy(void) { return ESP_OK; }

static rsb_bank_t s_banks[2];                       // двойной буфер — гонка-safe смена
static int        s_bank_idx;
static engine_t   s_engine;

static void pkgrl_apply(uint16_t pkg);
// presetIndex → packageId: app шлёт индекс из списка пресетов (pkg × 3 режима).
// Список = отсортированные уникальные pkg из f_04PPPP*.bin.
static int pkg_for_preset(int index, uint16_t *pkg_out) {
    uint16_t pk[16]; int np = 0;
    DIR *dp = opendir("/sd"); struct dirent *e;
    while (dp && (e = readdir(dp)) && np < 16) {
        if (strncmp(e->d_name, "f_04", 4) != 0) continue;
        unsigned long fid = strtoul(e->d_name + 2, NULL, 16);
        uint16_t p = (fid >> 8) & 0xFFFF;
        int dup = 0; for (int i = 0; i < np; i++) if (pk[i] == p) dup = 1;
        if (!dup) {
            int i = np++;                               // insert sorted
            while (i > 0 && pk[i-1] > p) { pk[i] = pk[i-1]; i--; }
            pk[i] = p;
        }
    }
    if (dp) closedir(dp);
    if (!np) return -1;
    int ent = index > 0 ? (index - 1) / 3 : 0;          // presetIndex 1-based; 3 режима на пакет
    if (ent >= np) ent = np - 1;
    *pkg_out = pk[ent];
    return pk[ent];
}

// Смена пресета: реальная работа (SD-чтение ~1МБ + ACK-ожидание) — в worker'е.
// on_preset лёгкий: только кладёт индекс в очередь — не блокирует NimBLE/httpd.
static QueueHandle_t s_preset_q;
static void preset_load(int index) {
    if (index <= 0) {                                // 0x0045 idx=0 — деактивация карточки
        s_engine.preset = 0;
        { uint8_t v = 0; nvs_wr("active", &v, 1); }
        ESP_LOGI(TAG, "preset deactivated");
        return;
    }
    int nidx = 1 - s_bank_idx;                       // грузим в неактивный слот
    rsb_bank_t *nb = &s_banks[nidx];
    uint16_t pkg = 0, mode = 0;
    /* THOR: f_04/f_05 пара — единственный формат банка у оригинала.
     * (.rsb-путь удалён: свой контейнер, терял k0 и шёл мимо pkgrl.) */
    if (thor_ble_preset_lookup(index, &pkg, &mode)) {   // нет pushed-списка —
        pkg_for_preset(index, &pkg);                    // fallback: pkg×3modes скан
        mode = (index - 1) % 3 + 1;
    }
    DIR *dp = opendir("/sd");
    char smp[48]={0}, rls[48]={0}; struct dirent *e;
    char want[12], wantr[12];
    snprintf(want, sizeof(want), "f_04%04x", pkg);
    snprintf(wantr, sizeof(wantr), "f_05%04x", pkg);
    while (dp && (e = readdir(dp))) {
        if (strncmp(e->d_name, want, 8) == 0) snprintf(smp, sizeof(smp), "/sd/%.40s", e->d_name);
        if (strncmp(e->d_name, wantr, 8) == 0) snprintf(rls, sizeof(rls), "/sd/%.40s", e->d_name);
    }
    if (dp) closedir(dp);
    int rc;
    if (smp[0] && rls[0]) rc = thor_load_bank(smp, rls, nb);
    else rc = -2;                                     // нет пары — ошибка, не fallback на чужой пакет
    if (!rc) {
        pkgrl_apply(pkg);                         // заводские дефолты движка/стенда из f_06
        thor_ble_apply_mode_rules(pkg, mode);     // + правила режима пресета (город/спорт/свой)
    }
    if (rc) { ESP_LOGW(TAG, "preset %d (pkg %u mode %u) load failed rc=%d", index, pkg, mode, rc); return; }
    int oidx = s_bank_idx;
    s_bank_idx = nidx;
    s_engine.bank = &s_banks[nidx];   // атомарный указатель — рендер сам сбросит голоса
    s_engine.preset = (uint8_t)index; // уедет в телеметрию
    i2s_out_set_rate(nb->sampleRate); // THOR=22050 — часы под банк
    // R05: ждём ACK от аудио-задачи (рендер перешёл на новый банк), потом free старого
    for (int i = 0; i < 50 && s_engine.bankAck != nb; i++)
        vTaskDelay(pdMS_TO_TICKS(10));            // до 500мс; без аудио-задачи — таймаут
    s_engine.pendingBank = &s_banks[oidx];  // отложенный free — сделает рендер (нет UAF при таймауте)
    { uint8_t v=(uint8_t)index;                          // F02: активный пресет переживает ребут
      nvs_wr("active", &v, 1); }                           // NVS — SD делит шину с C6
    ESP_LOGI(TAG, "preset %d → pkg %u loaded", index, pkg);
}

// .pkgrl (RLM2): записи-пресеты (rid 1..3 = режимы), формат как RLS2 —
// пары big-endian [key u16][value u16]. Параметры пакета (pkg31):
// k0=masterVol%(50/100/75), k12=redline(6500), k16=bfMinRpm(2500),
// k35=idleRpm(1000). ПРАВКА 2026-11-21: парс был со сдвигом на 2 байта
// (начинал с 0x18 вместо 0x16 — читал [val_i,key_{i+1}]): случайно ловил
// верные idle/redline, но case 33 получал 0 и ВЫКЛЮЧАЛ прострелы.
// Теперь полноценный парсер; применяем запись rid==1 (базовый режим;
// выбор по mode пресета — TODO). backfireCount НЕ трогаем — семантика
// k33 не подтверждена, а 28 сэмплов cls4 в банке должны стрелять.
static void pkgrl_apply(uint16_t pkg) {
    char want[12], path[48] = {0};
    snprintf(want, sizeof(want), "f_06%04x", pkg);
    DIR *dp = opendir("/sd"); struct dirent *e;
    while (dp && (e = readdir(dp)))
        if (strncmp(e->d_name, want, 8) == 0) { snprintf(path, sizeof(path), "/sd/%.40s", e->d_name); break; }
    if (dp) closedir(dp);
    if (!path[0]) return;
    FILE *f = fopen(path, "rb");
    if (!f) return;
    fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
    if (sz < 0x20 || sz > 4096) { fclose(f); return; }
    uint8_t *d = malloc(sz);
    if (fread(d, 1, sz, f) == (size_t)sz && !memcmp(d, "RLM2", 4)) {
        int n = (d[4] << 8) | d[5];
        float idle = 0, redline = 0;
        for (int i = 0; i < n; i++) {
            uint32_t ro = ((uint32_t)d[6+i*4] << 24) | ((uint32_t)d[7+i*4] << 16)
                        | ((uint32_t)d[8+i*4] << 8) | d[9+i*4];
            uint32_t re = (i + 1 < n)
                        ? (((uint32_t)d[6+(i+1)*4] << 24) | ((uint32_t)d[7+(i+1)*4] << 16)
                          | ((uint32_t)d[8+(i+1)*4] << 8) | d[9+(i+1)*4])
                        : (uint32_t)sz;
            if (ro + 4 > (uint32_t)sz || re > (uint32_t)sz) continue;
            uint16_t rid = (d[ro] << 8) | d[ro+1];
            if (rid != 1) continue;                  // базовый режим
            int np = (d[ro+2] << 8) | d[ro+3];
            uint16_t kv[92][2]; int nk = 0;              // пары записи rid==1
            for (int j = 0; j < np && nk < 92; j++) {
                if (ro + 4 + j*4 + 4 > re) break;
                kv[nk][0] = (d[ro+4+j*4] << 8) | d[ro+5+j*4];
                kv[nk][1] = (d[ro+6+j*4] << 8) | d[ro+7+j*4];
                nk++;
            }
            for (int j = 0; j < nk; j++) {
                switch (kv[j][0]) {
                case 0:  if (kv[j][1] <= 100) { s_engine.masterGain = kv[j][1] / 100.f;
                                              s_engine.pkgGain = s_engine.masterGain; } break;
                case 12: redline = kv[j][1]; break;
                case 16: if (kv[j][1] >= 1000 && kv[j][1] <= 6000) s_engine.bfMinRpm = kv[j][1]; break;
                case 35: idle = kv[j][1]; break;
                }
            }
            /* ПРАВКА 2026-11-21 (зоны): блоки 0x112/0x212/0x312 — профили
             * прострелов. +1=loRpm, +2=hiRpm, +3/+4=cntMin/cntMax очереди.
             * Порядок по убыванию порога; в рендере берётся первая зона
             * rpm>=lo -> низкие обороты = плотнее трескотня (как у THOR). */
            s_engine.bfZoneN = 0;
            memset(s_engine.bfZones, 0, sizeof(s_engine.bfZones));
            for (int i = 0; i < 3; i++) {
                uint16_t base = (uint16_t)((i + 1) * 256 + 0x12);
                uint16_t lo = 0, hi = 0, c0 = 0, c1 = 0, gms = 0, gp = 0;
                for (int j = 0; j < nk; j++) {
                    if      (kv[j][0] == base + 1) lo = kv[j][1];
                    else if (kv[j][0] == base + 2) hi = kv[j][1];
                    else if (kv[j][0] == base + 3) c0 = kv[j][1];
                    else if (kv[j][0] == base + 4) c1 = kv[j][1];
                    else if (kv[j][0] == base + 13) gms = kv[j][1];
                    else if (kv[j][0] == base + 14) gp  = kv[j][1];
                }
                if (lo && hi > lo && c0 >= 1 && c0 <= 10 && c1 >= c0 && c1 <= 10)
                    s_engine.bfZones[s_engine.bfZoneN++] = (typeof(s_engine.bfZones[0]))
                        {lo, hi, gms ? gms : 30, (uint8_t)c0, (uint8_t)c1,
                         (uint8_t)((gp >= 5 && gp <= 95) ? gp : 40)};
            }
            if (!s_engine.bfZoneN) {
                s_engine.bfZoneN = 1;
                s_engine.bfZones[0] = (typeof(s_engine.bfZones[0])){1700, 7000, 30, 2, 4, 40};
            }
        }
        stand_apply_pkg(&s_engine, idle, redline);
        ESP_LOGI(TAG, "pkg %u pkgrl: idle=%.0f redline=%.0f bfCount=%d zones=%d"
                 " [%u-%u:%u-%u g%u/%u|%u-%u:%u-%u g%u/%u|%u-%u:%u-%u g%u/%u]",
                 pkg, idle, redline, s_engine.backfireCount, s_engine.bfZoneN,
                 s_engine.bfZones[0].lo, s_engine.bfZones[0].hi,
                 s_engine.bfZones[0].cntMin, s_engine.bfZones[0].cntMax,
                 s_engine.bfZones[0].gapMs, s_engine.bfZones[0].gapP,
                 s_engine.bfZones[1].lo, s_engine.bfZones[1].hi,
                 s_engine.bfZones[1].cntMin, s_engine.bfZones[1].cntMax,
                 s_engine.bfZones[1].gapMs, s_engine.bfZones[1].gapP,
                 s_engine.bfZones[2].lo, s_engine.bfZones[2].hi,
                 s_engine.bfZones[2].cntMin, s_engine.bfZones[2].cntMax,
                 s_engine.bfZones[2].gapMs, s_engine.bfZones[2].gapP);
    }
    free(d); fclose(f);
}

static void preset_task(void *arg) {
    int idx;
    while (xQueueReceive(s_preset_q, &idx, portMAX_DELAY) == pdTRUE) preset_load(idx);
}
static void on_preset(int index) { xQueueOverwrite(s_preset_q, &index); }

static esp_err_t mount_sd(void) {
    // VDD слота microSD = on-chip LDO4 (см. схему JC-ESP32P4-M3-DEV) — включаем явно
    sd_pwr_ctrl_handle_t pwr = NULL;
    sd_pwr_ctrl_ldo_config_t ldo = { .ldo_chan_id = 4 };
    sd_pwr_ctrl_new_on_chip_ldo(&ldo, &pwr);
    esp_vfs_fat_sdmmc_mount_config_t m = {
        .format_if_mount_failed = false, .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };
    sdmmc_card_t *card;
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;              // SD карта на slot0; slot1 = esp_hosted (C6)
    host.init = sdmmc_host_init_dummy;          // контроллер уже поднят esp_hosted
    host.deinit = sdmmc_host_deinit_dummy;
    host.pwr_ctrl_handle = pwr;
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 1;   // 1-битная шина проще развести; при 4-бит — быстрее загрузка банка
    return esp_vfs_fat_sdmmc_mount("/sd", &host, &slot, &m, &card);
}

static void ota_valid_task(void *arg) {
    // rollback-arm: только если образ прожил 30 с реальной работы —
    // падение на mount_sd/ble/i2s до этого откатится загрузчиком
    vTaskDelay(pdMS_TO_TICKS(30000));
    esp_ota_mark_app_valid_cancel_rollback();
    ESP_LOGI(TAG, "OTA image marked valid (30s uptime)");
    vTaskDelete(NULL);
}

void app_main(void) {
    esp_log_level_set("NimBLE", ESP_LOG_WARN);        // notify-спам 10Гц — глушим
    esp_log_level_set("bt_nimble", ESP_LOG_WARN);
    // mark_app_valid НЕ здесь: rollback должен сработать при падении
    // на mount_sd/ble_init/i2s — помечаем через 30 с живой работы (см. таск ниже)
    ESP_LOGI(TAG, "build %u (tag %c%c%c%c)", (unsigned)BUILD_VER,
             FW_VER_TAG[0], FW_VER_TAG[1], FW_VER_TAG[2], FW_VER_TAG[3]);
    {   esp_err_t ne = nvs_flash_init();            // конфиги тут, а не на SD:
        if (ne == ESP_ERR_NVS_NO_FREE_PAGES ||      // SD делит SDMMC с C6 —
            ne == ESP_ERR_NVS_NEW_VERSION_FOUND) {  // запись в BLE-сессии падает
            nvs_flash_erase(); nvs_flash_init();
        }
    }
    s_preset_q = xQueueCreate(1, sizeof(int));        // длина 1, overwrite — свежий важнее
    xTaskCreate(preset_task, "preset", 6144, NULL, 5, NULL);
    if (mount_sd() != ESP_OK)
        ESP_LOGW(TAG, "SD mount failed — продолжаю без карты (BLE всё равно поднимется)");

    {   // OTA-образ от BLE-коммита: применяем ДО старта BLE.
        // esp_ota_write глушит icache — при живом коннекте вешал NimBLE >7с,
        // апп не дожидался Polling-ответа -> "Re-handshake failed".
        // Тут соединения нет вообще: столл никого не трогает.
        const char *img = "/sd/f_02000000.bin";
        uint8_t magic = 0; int have_img = 0;
        FILE *ff = fopen(img, "rb");
        if (ff) { have_img = 1; (void)fread(&magic, 1, 1, ff); fclose(ff); }
        if (have_img && magic != 0xE9) {
            remove(img);                                     // не-ESP образ (зашифрованный THOR) — никогда не применится
        } else if (magic == 0xE9) {
            ESP_LOGW(TAG, "pending BLE OTA image — applying before BLE start");
            const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
            esp_ota_handle_t oh = 0; esp_err_t orc = ESP_FAIL;
            FILE *of = fopen(img, "rb");
            if (part && of && esp_ota_begin(part, OTA_SIZE_UNKNOWN, &oh) == ESP_OK) {
                uint8_t ob[1024]; size_t orr; orc = ESP_OK;
                while ((orr = fread(ob, 1, sizeof(ob), of)) > 0)
                    if (esp_ota_write(oh, ob, orr) != ESP_OK) { esp_ota_abort(oh); orc = ESP_FAIL; break; }
                if (orc == ESP_OK && esp_ota_end(oh) == ESP_OK)
                    orc = esp_ota_set_boot_partition(part); else orc = ESP_FAIL;
            }
            if (of) fclose(of);
            if (orc == ESP_OK) { remove(img); esp_restart(); }  // второй ребут — в новую fw
            ESP_LOGE(TAG, "boot OTA apply failed rc=%d", orc);
            remove("/sd/fw_ver.bin");                        // честный откат версии
            remove(img);                                     // образ мусор — не зацикливаться
        }
    }

    /* Банк приходит только через preset_load (f_04/f_05 пара) — при
     * восстановлении активного пресета из NVS ниже. До того — тишина. */
    s_banks[0].sampleRate = 44100;          // валидный SR для I2S; рендер даст тишину

    engine_init(&s_engine, &s_banks[0]);

    {   // F02: восстановить активный пресет — NVS, SD только как миграция
        uint8_t v = 0; size_t al = 1; int have = 0;
        if (nvs_rd("active", &v, &al) == 0) have = 1;
        else { FILE *af = fopen("/sd/active.bin","rb");
               if (af) { have = fread(&v,1,1,af)==1; fclose(af);
                         if (have) nvs_wr("active", &v, 1); } }
        if (have) on_preset(v);
    }

    i2s_out_start(&s_engine);                     // аудио-рендер стартует здесь
    {   // CAN: есть canbin (f_01*) + правила по serie → THOR listen-only декодер.
        // нет — OBD2-опрос (универсально, но активно гадит в шину).
        uint16_t serie = can_serie_from_sd();
        const car_can_t *cc = serie ? can_rules_find(serie) : NULL;
        if (cc) { ESP_LOGI("main", "CAN THOR mode: serie=%u rules=%u", serie, cc->n_rules);
                  can_thor_start(&s_engine, cc); }
        else    { ESP_LOGW("main", "CAN OBD2 mode (no rules for serie=%u)", serie);
                  can_rpm_start(&s_engine, CAN_MODE_OBD2); }
    }
    stand_start(&s_engine);                       // стенд: IGN/THR/GEAR пины + веб-управление
    trip_log_init(&s_engine);                     // логгер поездки на SD (старт/стоп по BLE)
    // BLE на самом P4 через esp_hosted на бортовом C6 (thor_ble.c).
    // Заменяет внешний S3-модуль: команды идут прямо в engine_t.
    // hw 1536 / fw 551 — gen1 THOR (из карты прошивок сервера).
    ble_init_and_services();                      // NimBLE + GATT (ниже)
    thor_ble_start(&s_engine, on_preset, /*hw*/1536, /*fw*/BUILD_VER, /*sn*/7584);
    // ctrl_uart_start(&s_engine, on_preset);     // внешний S3 — опционально

    // Пульт по Ethernet (RJ45 на борту): PHY автодетект, DHCP, IP печатается в лог.
    // WiFi: C6 объявляет WLAN over SDIO, но Req_WifiInit (rpc 278) молчит 5с —
    // вероятно slave без wlan-RPC (нужна перепрошивка C6), Ethernet надёжнее.
    // httpd слушает INADDR_ANY — веб-пульт и /ctrl живут на eth-адресе автоматом.
    net_ctrl_start(&s_engine, on_preset);              // eth + TCP:3333 (netcat-контроль)
    net_ws_start(&s_engine, on_preset);                // http://<eth-ip>/ · ws /ctrl · POST /ota

    xTaskCreate(ota_valid_task, "otaval", 2048, NULL, 3, NULL);  // rollback-arm через 30с

    // Детект прострела: защёлка armed при thr>0.6 -> стреляет когда thr упал <0.15
    // на оборотах >bfMinRpm. Срабатывает при любой скорости отпускания педали
    // (старый thrPeak*0.85 затухал за ~150мс — плавный сброс ползунком не ловился).
    // Отсечка: rpm в красной зоне под газом -> одиночный хлопок cls4 (~0.9с перезарядка).
    //
    // ПРАВКА 2026-11-21 (трескотня): серия — только начало. RLM2 пакета держит
    // 3 зональных блока прострелов (пороги 3000/2500/1700, счётчики 2-4/3-5/4-6)
    // — THOR трещит ВСЮ перекатку, чем ниже rpm тем гуще. Поэтому:
    //  1) отпускание >=150мс -> серия (как раньше, gate rpm>bfMinRpm=2500);
    //  2) дальше, пока газ закрыт и rpm>1700 (пол зоны C RLM2) -> одиночный
    //     случайный хлопок каждые ~200-600мс;
    //  3) на холостых (~<1400) ещё ~20с после последнего газа — редкий
    //     одиночный каждые 4-9с (отстрелы ХХ).
    uint8_t bfArmed = 0; uint32_t bfLiftMs = 0;
    int32_t ovTmr = 0, idleTmr = 0, armAgo = 0;
    int bfBurst = 0;                             // хлопков осталось в текущей очереди
    uint32_t cutTmr = 0;
    uint8_t wasRunning = 0;
    while (1) {
        float thr = s_engine.throttleTarget;        // цель — не фильтрованная (детект сброса!)
        float rpm = s_engine.rpm;
        if (thr > 0.6f) { bfArmed = 1; bfLiftMs = 0; armAgo = 20000; ovTmr = 0; }
        else if (armAgo >= 50) armAgo -= 50;
        if (bfArmed && thr < 0.15f) {
            /* ПРАВКА 2026-11-19: прострелы только при удержании газа закрытым
             * >=150мс — иначе серия стреляет в середине каждого переключения
             * (газ там закрыт ~300мс), что слышалось как «наложение». Проверено
             * на драг-рендере: render_pwr_drag6.py. */
            bfLiftMs += 50;
            if (bfLiftMs >= 150) {
                bfArmed = 0; bfLiftMs = 0;
                if (rpm > s_engine.bfMinRpm)
                    engine_trigger_backfire(&s_engine);  // сброс на оборотах -> серия
                ovTmr = 600;                            // пауза, потом трескотня
            }
        } else bfLiftMs = 0;
        /* Перекатка: очереди хлопков по зонам RLM2 — первая зона где
         * rpm>=lo (пороги идут по убыванию => низкие обороты трещат гуще).
         * Между очередями пауза 0.6-1.5с; внутри — 30-150мс (как серия). */
        if (thr < 0.15f && rpm > 400.f) {
            const typeof(s_engine.bfZones[0]) *z = NULL;
            for (int i = 0; i < s_engine.bfZoneN; i++)
                if (rpm >= s_engine.bfZones[i].lo && rpm <= s_engine.bfZones[i].hi)
                    { z = &s_engine.bfZones[i]; break; }
            if (z) {
                ovTmr -= 50;
                if (ovTmr <= 0) {
                    if (bfBurst <= 0)
                        bfBurst = z->cntMin + (int)(esp_random()
                                  % (uint32_t)(z->cntMax - z->cntMin + 1));
                    if (bfBurst > 0) {
                        engine_pop(&s_engine); bfBurst--;
                        /* ПРАВКА 2026-11-21 (паузы, подтв. демо APK):
                         * внутри очереди — геометрические слоты gapMs*(1+skip),
                         * prob=gapP% на такт (k287/k288); зазоры демо на ~33мс
                         * решётке. Между очередями — k291..k289 (600..4000мс). */
                        if (bfBurst > 0) {
                            int ticks = 1;
                            while ((int)(esp_random() % 100) >= z->gapP && ticks < 64)
                                ticks++;
                            ovTmr = ticks * z->gapMs;
                        } else ovTmr = 600 + (int)(esp_random() % 3401);
                    }
                }
            } else { bfBurst = 0; ovTmr = 0; }
        } else bfBurst = 0;
        // холостые отстрелы: ~20с окна после последнего газа, редкие одиночные
        if (rpm > 400.f && rpm < 1400.f && thr < 0.15f && armAgo) {
            idleTmr -= 50;
            if (idleTmr <= 0) { engine_pop(&s_engine);
                                idleTmr = 4000 + (int)(esp_random() % 5000); }
        } else idleTmr = 1500;
        {   float rl = stand_redline();
            if (cutTmr) cutTmr -= 50;
            if (rl > 0.f && s_engine.rpm >= rl - 100.f && thr > 0.5f && !cutTmr) {
                engine_trigger_event(&s_engine, 2); // type2 = cls4 отсечка
                cutTmr = 900;
            } }
        // детект запуска: rpm пошёл с ~0 выше 400 — играем старт-звук по режиму
        uint8_t running = s_engine.rpm > 400;
        if (running && !wasRunning && s_engine.startMode)
            engine_trigger_event(&s_engine, 0);     // type0=startup (стартер/динамич.)
        if (!running && wasRunning)
            engine_trigger_event(&s_engine, 1);     // type1=shutoff (ENGINE_STOP)
        s_engine.engineRunning = running; wasRunning = running;
        // native control: кнопки руля/круиза из CAN (pass_ids) — 1=MUTE 2=след.пресет
        if (g_can_btn_evt) {
            int evt = g_can_btn_evt; g_can_btn_evt = 0;
            if (evt == 1) {                             // MUTE toggle (cruise knob UP)
                static float savedGain = 0;
                if (s_engine.masterGain > 0.001f) { savedGain = s_engine.masterGain;
                                                    s_engine.masterGain = 0; }
                else s_engine.masterGain = savedGain > 0 ? savedGain : 1.f;
            } else if (evt == 2)                        // звук вперёд (knob DOWN)
                on_preset((s_engine.preset + 1) % 8);
        }
        thor_ble_rules_tick(50);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
