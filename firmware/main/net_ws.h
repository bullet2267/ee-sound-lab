// net_ws.h — управление по WebSocket (чтобы веб-пульт работал по LAN).
// Требует поднятого Ethernet (net_ctrl.c делает eth_init) ИЛИ Wi-Fi.
// Эндпоинт: ws://<ip>/ctrl
//   приём (от браузера):  бинарное сообщение [cmd][payload...]   (как у BLE control-характеристики)
//   выдача (в браузер):   бинарное сообщение [rpm_lo,rpm_hi,thr,state,preset,temp,err] (7 байт), ~10 Гц
#pragma once
#include "engine.h"
#include "ctrl_uart.h"   // ctrl_preset_cb

void net_ws_start(engine_t *engine, ctrl_preset_cb on_preset);
