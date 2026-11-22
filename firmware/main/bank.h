// bank.h — разбор банка THOR (SMP2+RLS2 пара, f_04/f_05) + типы зёрен/событий.
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t offset, length;        // в payload, байты
    uint16_t baseRpm, loopStart, loopEnd, flags; // flags bit0=loopable, bit1=layer(0=on/нагрузка,1=off/сброс), bit2=idle, bits3-5=класс k0 (1..6)
    // THOR-окно аддитивного микса (из RLS2 k2/k3/k5/k6): трапеция —
    // [fadeLo..flatLo] ramp-in, [flatLo..flatHi] full, [flatHi..fadeHi] ramp-out.
    // Все нули = окно не задано (старый банк) — рендер берёт режим 2-пар.
    uint16_t fadeLo, flatLo, flatHi, fadeHi, gainQ8, xfade; // xfade=длина шва лупа (RLS2 k9)
} rsb_grain_t;

typedef struct {
    uint32_t offset, length;
    uint8_t  type, flags; uint16_t _rsv;
} rsb_event_t;

typedef struct {
    uint16_t rpm, grainIndex, gain, loadGain; // gain/loadGain в Q8.8; rpm bit15 = слой (0=on,1=off)
} rsb_rule_t;

typedef struct {
    uint32_t sampleRate;
    uint8_t  format, channels;
    uint16_t rpmMin, rpmMax;
    uint16_t grainCount, eventCount, ruleCount;

    const rsb_grain_t *grains;      // копии таблиц (offset/length в PCM16-байтах после загрузки)
    const rsb_event_t *events;
    const rsb_rule_t  *rules;
    const int16_t     *payload;     // всегда PCM16 после загрузки (ADPCM декодируется здесь)
    uint32_t           payloadSamples;

    uint8_t  *_image;               // исходный образ файла (освобождается при ADPCM)
    void     *_grains, *_events, *_rules; // владение копиями таблиц
    int16_t  *_decoded;             // владение декодированным PCM (при ADPCM)
} rsb_bank_t;

// Загрузка банка — только оригинальный формат THOR (thor_load_bank ниже).
void rsb_free(rsb_bank_t *b);

// Хелперы движка:
int  rsb_grain_below(const rsb_bank_t *b, uint16_t rpm);
void rsb_rule_lookup(const rsb_bank_t *b, uint16_t rpm, uint16_t *gain, uint16_t *loadGain);
int  rsb_find_event(const rsb_bank_t *b, uint8_t type);
int  rsb_find_event_rand(const rsb_bank_t *b, uint8_t type, uint32_t seed);
int  rsb_find_event_rid(const rsb_bank_t *b, uint16_t rid);

// --- слои (v1.1): 0 = on-load (под газом), 1 = off-load (сброс газа) ---
int  rsb_count_layer(const rsb_bank_t *b, int layer);           // гранул в слое
int  rsb_grain_pair_l(const rsb_bank_t *b, uint16_t rpm, int layer, int *gi, int *gj); // соседние гранулы слоя
void rsb_rule_lookup_l(const rsb_bank_t *b, uint16_t rpm, int layer, uint16_t *gain, uint16_t *loadGain);
int  rsb_has_windows(const rsb_bank_t *b);                     // 1 = у гранул THOR-окна
float rsb_grain_env(const rsb_bank_t *b, int gi, float rpm);   // трапец-огибающая 0..1
int thor_load_bank(const char*, const char*, rsb_bank_t*);
