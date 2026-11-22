#pragma once
#include <stdint.h>

// Правила декодирования CAN-кадров из car_profiles.json (THOR .canbin эмуляция).
// kind: 0=u16le(b0:b1)>>shift→slot   1=byte[b0]→slot (cl=шкала)   2=byte[b0]=состояние зажигания/режима
//       3=BE-битовое поле b0..b0+b1 (нумерация от MSB кадра, Motorola) → slot, cl=шкала
//       4=LE-битовое поле b0..b0+b1 (Intel/VAG: le64>>b0 & mask(b1)) → slot, cl=шкала
// slot: 0=ОБОРОТЫ двигателя (u16le>>2, до 8000), 1=режим/передача, 4=педаль/DriveSelect (0..cl)
typedef struct {
    uint32_t id;      // CAN-ID
    uint8_t  kind;
    uint8_t  b0, b1;  // байты (kind0/1/2) или bit_off+bit_len (kind3)
    uint8_t  shift;
    uint16_t cl;      // верх шкалы (speed=8000, byte=255 и т.п.)
    uint8_t  slot;
} can_rule_t;

typedef struct car_can_t {
    uint16_t serie;              // serie_id авто (u16 из CAN fileId)
    uint8_t  n_rules;
    const can_rule_t *rules;
    uint8_t  n_pass;
    const uint16_t *pass_ids;    // passthrough ID — кнопки руля/круиза (native control)
} car_can_t;

extern const car_can_t g_car_can[];
extern const int g_car_can_n;

const car_can_t *can_rules_find(uint16_t serie);
uint16_t can_serie_from_sd(void);   // serie_id из /sd/f_01<SSSS><VV>.bin
