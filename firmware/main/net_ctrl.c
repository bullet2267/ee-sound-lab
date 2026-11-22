#include "net_ctrl.h"
#include "trip_log.h"
#include "stand.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_eth_mac_esp.h"
#include "lwip/sockets.h"
#include <string.h>

#define PORT        3333
#define FRAME_SOF   0xC7
#define STAT_SOF    0xC8

// --- пины RMII/PHY (под свою плату; на P4 часть фиксирована, MDC/MDIO/RST гибкие) ---
#define PHY_ADDR        1
#define PIN_PHY_RST     51
#define PIN_SMI_MDC     31
#define PIN_SMI_MDIO    52
#define RMII_CLK_GPIO   50          // вход/выход 50 МГц RMII REF_CLK — зависит от PHY

static const char *TAG = "net_ctrl";
static engine_t *s_eng;
static ctrl_preset_cb s_preset;

static void apply_cmd(uint8_t cmd, const uint8_t *p, int len) {
    switch (cmd) {
        case 0x01: if (len>=1) s_eng->masterGain = p[0] / 255.0f; break;   // громкость
        case 0x02: if (len>=1 && s_preset) s_preset(p[0]); break;          // пресет
        case 0x03: if (len>=1) engine_trigger_event(s_eng, p[0]); break;   // событие
        case 0x04: if (len>=1) trip_log_set(p[0] != 0); break;             // лог поездки
        // --- стенд (тот же протокол, что и /ctrl) ---
        case 0x06: if (len>=1) stand_set_ign(s_eng, p[0]);
                   if (len>=2) stand_set_thr(s_eng, p[1] / 255.0f);
                   if (len>=3) { if (p[2]==1) stand_gear_cmd(s_eng,+1);
                                  else if (p[2]==2) stand_gear_cmd(s_eng,-1); }
                   if (len>=3 && p[2]==3) stand_set_thr(s_eng, -1.f);
                   break;
        case 0x07: if (len>=2) stand_set_idle(s_eng, (float)(p[0] | (p[1]<<8))); break;
        default: ESP_LOGW(TAG, "unknown cmd 0x%02x", cmd);
    }
}

// Обслуживание одного клиента: читаем команды, раз в ~100мс шлём телеметрию.
static void serve_client(int sock) {
    struct timeval tv = { .tv_sec = 0, .tv_usec = 100*1000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    uint8_t buf[64];
    while (1) {
        int n = recv(sock, buf, sizeof(buf), 0);
        if (n == 0) break;                      // клиент закрыл
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) break;
        // разбор кадров команд (может прийти несколько подряд)
        for (int i = 0; i + 2 < n; ) {
            if (buf[i] != FRAME_SOF) { i++; continue; }
            uint8_t cmd = buf[i+1], len = buf[i+2];
            if (i + 3 + len > n) break;
            apply_cmd(cmd, &buf[i+3], len);
            i += 3 + len;
        }
        // телеметрия
        uint16_t rpm = (uint16_t)(s_eng->rpm < 0 ? 0 : s_eng->rpm);
        uint8_t thr = (uint8_t)(s_eng->throttle * 255.0f);
        uint8_t state = (s_eng->throttle > 0.5f ? 1 : 0) | (s_eng->bfLeft > 0 ? 2 : 0);
        uint8_t f[9] = { STAT_SOF, 7, (uint8_t)(rpm&0xFF), (uint8_t)(rpm>>8),
                         thr, state, s_eng->preset, s_eng->coolantC, s_eng->errFlags };
        if (send(sock, f, sizeof(f), 0) < 0) break;
    }
    close(sock);
}

static void server_task(void *arg) {
    int srv = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    struct sockaddr_in a = { .sin_family=AF_INET, .sin_addr.s_addr=htonl(INADDR_ANY),
                             .sin_port=htons(PORT) };
    int yes=1; setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    bind(srv, (struct sockaddr*)&a, sizeof(a));
    listen(srv, 1);
    ESP_LOGI(TAG, "TCP control on :%d", PORT);
    while (1) {
        int c = accept(srv, NULL, NULL);
        if (c >= 0) { ESP_LOGI(TAG, "client connected"); serve_client(c); }
    }
}

static void on_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data) {
    ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
    ESP_LOGI(TAG, "Ethernet IP: " IPSTR " — пульт http://" IPSTR "/", IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.ip));
}

static void eth_init(void) {
    // netif/loop могли уже быть созданы (WiFi-ветка) — дубли не фатальны
    if (esp_netif_init() == ESP_ERR_INVALID_STATE) {}
    if (esp_event_loop_create_default() == ESP_ERR_INVALID_STATE) {}
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, on_got_ip, NULL));
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *netif = esp_netif_new(&netif_cfg);

    eth_esp32_emac_config_t emac_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_cfg.smi_gpio.mdc_num  = PIN_SMI_MDC;
    emac_cfg.smi_gpio.mdio_num = PIN_SMI_MDIO;
    emac_cfg.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;   // или EMAC_CLK_OUT — зависит от PHY
    emac_cfg.clock_config.rmii.clock_gpio = RMII_CLK_GPIO;

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_cfg, &mac_cfg);

    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr = PHY_ADDR;
    phy_cfg.reset_gpio_num = PIN_PHY_RST;
    esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_cfg);   // v6.1: авто-детект (IP101/RTL8201/LAN87xx)

    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_cfg, &eth));
    ESP_ERROR_CHECK(esp_netif_attach(netif, esp_eth_new_netif_glue(eth)));
    ESP_ERROR_CHECK(esp_eth_start(eth));                      // IP получит по DHCP
    ESP_LOGI(TAG, "Ethernet started (DHCP)");
}

void net_ctrl_start(engine_t *engine, ctrl_preset_cb on_preset) {
    s_eng = engine; s_preset = on_preset;
    eth_init();
    xTaskCreatePinnedToCore(server_task, "netctrl", 4096, NULL, 5, NULL, 0);
}
