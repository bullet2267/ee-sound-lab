// stand.h — стенд: перемычки/потенциометр имитируют зажигание и педаль газа.
// Активен когда зажат IGN-пин ИЛИ включён из веба. Когда стенд активен,
// CAN-задача входы не трогает (engine->standActive).
#pragma once
#include "engine.h"

void stand_start(engine_t *e);                      // таск чтения пинов + динамика rpm
void stand_set_ign(engine_t *e, uint8_t on);        // веб-зажигание (0/1, 2=только пины)
void stand_set_thr(engine_t *e, float t);           // веб-газ 0..1 (t<0 — вернуть пинам)
void stand_gear_cmd(engine_t *e, int d);            // ручное переключение +1/-1
void stand_set_mode(engine_t *e, uint8_t neutral);  // 0=езда (АКПП), 1=газовка на месте (нейтраль)
void stand_set_idle(engine_t *e, float rpm);        // холостые об/мин (persist /sd/stand.cfg)
void stand_apply_pkg(engine_t *e, float idle, float redline); // калибровка из pkgrl пакета
float stand_redline(void);                       // текущая красная зона (для детекта отсечки)
