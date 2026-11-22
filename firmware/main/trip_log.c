#include "trip_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <stdio.h>

#define LOG_PATH   "/sd/trip_log.csv"
#define LOG_HZ     20            // 20 строк/с — достаточно для отладки звука

static const char *TAG = "triplog";
static engine_t *s_eng = NULL;
static volatile bool s_on = false;

bool trip_log_is_on(void){ return s_on; }
void trip_log_set(bool on){ s_on = on; ESP_LOGI(TAG, "logging %s", on ? "ON" : "OFF"); }

static void log_task(void *arg){
    FILE *f = NULL;
    int64_t t0 = 0;
    bool was = false;
    while (1){
        if (s_on && !was){                         // старт новой записи
            f = fopen(LOG_PATH, "w");
            // первые 3 колонки совместимы со стендом (bench читает time,rpm,throttle),
            // temp/preset — для анализа и точного воспроизведения условий.
            if (f){ fputs("time,rpm,throttle,temp,preset\n", f); t0 = esp_timer_get_time(); }
            else ESP_LOGE(TAG, "cannot open %s", LOG_PATH);
            was = true;
        } else if (!s_on && was){                   // стоп — закрыть файл
            if (f){ fclose(f); f = NULL; }
            was = false;
            ESP_LOGI(TAG, "log saved to %s", LOG_PATH);
        }
        if (s_on && f){
            float t = (esp_timer_get_time() - t0) / 1e6f;
            fprintf(f, "%.3f,%.0f,%.3f,%u,%u\n",
                    t, s_eng->rpm, s_eng->throttle, s_eng->coolantC, s_eng->preset);
            fflush(f);                              // чтобы не потерять при выключении зажигания
        }
        vTaskDelay(pdMS_TO_TICKS(1000 / LOG_HZ));
    }
}

void trip_log_init(engine_t *engine){
    s_eng = engine;
    xTaskCreatePinnedToCore(log_task, "triplog", 4096, NULL, 3, NULL, 0);
}
