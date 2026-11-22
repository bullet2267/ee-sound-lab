// net_ctrl.h — управление по Ethernet (ESP32-P4 EMAC + внешний PHY).
// Тот же протокол, что ctrl_uart/BLE:
//   приём  команд:    [0xC7][cmd][len][payload]
//   выдача телеметрии: [0xC8][len][payload(7)] rpm_lo,rpm_hi,thr,state,preset,temp,err
// TCP-сервер на порту 3333. Нужен внешний PHY на RMII (IP101/RTL8201/LAN8720).
#pragma once
#include "engine.h"
#include "ctrl_uart.h"   // ctrl_preset_cb

void net_ctrl_start(engine_t *engine, ctrl_preset_cb on_preset);

[executed on device: KOMPUTER (0089e01f-ace6-4dff-b186-c09b6a80372f)]