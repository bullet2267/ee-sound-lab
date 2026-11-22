// i2s_out.h — I2S std TX на внешний DAC (PCM5102A) или цифровой class-D (TAS5805M).
#pragma once
#include "engine.h"

void i2s_out_start(engine_t *engine);   // поднимает I2S и запускает аудио-задачу рендера
void i2s_out_set_rate(uint32_t sr);     // переключить частоту под новый банк
