idf_component_register(
    SRCS "main.c" "bank.c" "engine.c" "i2s_out.c" "can_rpm.c"
         "can_rules.c" "can_rules_gen.c"
         "trip_log.c" "thor_ble.c" "thor_proto.c" "net_ws.c" "ota.c"
         "net_ctrl.c" "stand.c"
         # TODO(JC-board): ctrl_uart.c — UART-мост к внешнему S3 не нужен (BLE на C6)
    INCLUDE_DIRS "."
    REQUIRES driver fatfs esp_driver_sdmmc esp_driver_i2s esp_driver_twai esp_driver_uart
             esp_eth esp_netif esp_event lwip esp_http_server app_update esp_adc
             esp_hosted bt mbedtls nvs_flash esp_codec_dev esp_driver_i2c
    EMBED_TXTFILES "web/index.html"
)
