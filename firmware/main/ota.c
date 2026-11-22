#include "ota.h"
#include "esp_ota_ops.h"
#include "esp_log.h"
#include "esp_system.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ota";

// POST /ota — тело запроса = новый .bin. Требует заголовок X-OTA-Token.
// Пишем в свободный OTA-раздел, при успехе переключаем загрузку и ребутимся.
// Откат защищён CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE: новый образ обязан
// дойти до app_main (esp_ota_mark_app_valid_cancel_rollback), иначе бутлоадер
// сам вернётся на предыдущий слот.
#ifndef OTA_TOKEN
#define OTA_TOKEN "CHANGE_ME_BEFORE_DEPLOY"   // менять при установке в машину!
#endif
static esp_err_t ota_post(httpd_req_t *req) {
    char tok[40] = {0};
    if (httpd_req_get_hdr_value_str(req, "X-OTA-Token", tok, sizeof(tok)) != ESP_OK
        || strcmp(tok, OTA_TOKEN) != 0) {
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "bad/missing X-OTA-Token");
        return ESP_FAIL;
    }
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no ota part"); return ESP_FAIL; }
    ESP_LOGI(TAG, "OTA -> %s (%u байт объявлено)", part->label, (unsigned)req->content_len);

    esp_ota_handle_t h = 0;
    if (esp_ota_begin(part, OTA_SIZE_UNKNOWN, &h) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota_begin"); return ESP_FAIL;
    }

    char buf[1460];
    int remaining = req->content_len, total = 0;
    while (remaining > 0) {
        int r = httpd_req_recv(req, buf, sizeof(buf) < (size_t)remaining ? sizeof(buf) : remaining);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (r <= 0) { esp_ota_abort(h); httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "recv"); return ESP_FAIL; }
        if (esp_ota_write(h, buf, r) != ESP_OK) {
            esp_ota_abort(h); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota_write"); return ESP_FAIL;
        }
        remaining -= r; total += r;
    }

    if (esp_ota_end(h) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota_end (образ повреждён?)"); return ESP_FAIL;
    }
    if (esp_ota_set_boot_partition(part) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "set_boot"); return ESP_FAIL;
    }
    ESP_LOGI(TAG, "OTA ок, %d байт, перезагрузка", total);
    httpd_resp_sendstr(req, "OK, перезагрузка...");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

void ota_register(httpd_handle_t server) {
    httpd_uri_t u = { .uri = "/ota", .method = HTTP_POST, .handler = ota_post };
    httpd_register_uri_handler(server, &u);
    ESP_LOGI(TAG, "OTA endpoint: POST /ota");
}
