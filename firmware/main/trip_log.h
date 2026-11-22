// trip_log.h — запись лога поездки на SD: time,rpm,throttle,temp,preset.
// Первые 3 колонки совместимы со стендом:  bench bank.rsb out.wav --csv trip_log.csv
#pragma once
#include "engine.h"
#include <stdbool.h>

void trip_log_init(engine_t *engine);  // поднять логгер (пишет, пока включён)
void trip_log_set(bool on);            // старт/стоп записи (например по BLE cmd 0x04)
bool trip_log_is_on(void);
