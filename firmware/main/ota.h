// ota.h — обновление прошивки по HTTP (POST сырого .bin в /ota).
// Регистрируется на уже поднятом httpd (из net_ws).
#pragma once
#include "esp_http_server.h"

void ota_register(httpd_handle_t server);
