// ctrl_uart.h — приём команд управления от ESP32-C6/S3 (BLE) по UART на стороне P4.
#pragma once
#include "engine.h"

// Протокол кадра (от BLE-чипа к P4): [0xC7][cmd][len][payload...]
//   cmd 0x01 VOLUME   len1  payload: 0..255  -> masterGain
//   cmd 0x02 PRESET   len1  payload: индекс  -> перезагрузка банка (f_04/f_05)
//   cmd 0x03 EVENT    len1  payload: тип      -> engine_trigger_event
//   cmd 0x04 LOG      len1  payload: 0/1      -> запись лога поездки на SD
//   cmd 0x05 RPMTEST  len2  payload: rpm u16  -> тестовые обороты без CAN (~3 c на команду)
// Статус от P4 к BLE-чипу: [0xC8][len][payload(7)] rpm_lo,rpm_hi,thr,state,preset,temp,err
typedef void (*ctrl_preset_cb)(int index);   // колбэк перезагрузки банка (реализован в main)

void ctrl_uart_start(engine_t *engine, ctrl_preset_cb on_preset);
