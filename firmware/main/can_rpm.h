// can_rpm.h — чтение оборотов/газа с CAN (TWAI) на ESP32-P4.
#pragma once
#include "engine.h"

// Два режима:
//  - PASSIVE: слушаем шину и ловим кадр с известным ID оборотов (быстро, но ID зависит от машины)
//  - OBD2:    опрашиваем 0x7DF PID 0x0C (RPM) / 0x11 (throttle) / 0x05 (temp) — универсально
typedef enum { CAN_MODE_PASSIVE, CAN_MODE_OBD2 } can_mode_t;

#include "can_rules.h"
void can_rpm_start(engine_t *engine, can_mode_t mode);
void can_thor_start(engine_t *engine, const car_can_t *rules);
extern volatile int g_can_btn_evt;   // 0=нет 1=MUTE 2=след.пресет (читается в main)
