// engine.h — движок синтеза: аддитивный микс гранул по RPM-окнам + питч,
// плюс рандомизированные прострелы (backfire) и one-shot события.
#pragma once
#include "bank.h"

typedef struct {
    const rsb_bank_t *bank;
    const rsb_bank_t *lastBank;      // детект смены банка в рендере (гонка-safe)
    volatile const rsb_bank_t *bankAck; // рендер публикует банк, с которого реально играет

    // вход (пишется из CAN-задачи, читается в аудио-рендере)
    volatile float rpm;        // сглаженные обороты
    volatile float throttle;   // 0..1 нагрузка/газ (фильтрованная)
    float throttleTarget;      // цель фильтра газа
    float accResponse;         // 0..1 отклик акселератора (1=мгновенно, iXsound-стиль)
    // тестовый вход: если rpm_ovr_ms > 0, can-задачи используют rpm_ovr вместо шины
    volatile float    rpm_ovr;     // подставные обороты (отладка на столе)
    volatile uint16_t rpm_ovr_ms;  // таймер жизни оверрайда (тикает в can-задаче)

    // состояние проигрываемых грануль: on-слой (v0/v1) + off-слой (w0/w1)
    // (старый режим 2-пар для банков без THOR-окон)
    struct { int idx; float phase; } v0, v1, w0, w1;
    // аддитивный режим THOR — env-тайлинг + свободные фазы (2026-11-21):
    // все гранулы в своих окнах звучат с весом env*gainQ8*layerW, фаза
    // каждой — свободный бег rate=rpm/base (revs≈2.012 => рейт совпадает с
    // циклом; фаза-лок давал «бук-бук» — коррелированные нарезки одного
    // тейка синхронизировали огибающие на частоте цикла).
    float grGain[96];            // per-grain сглаженный вес (env*gainQ8, ~22мс)
    float grPh[96];              // per-grain фаза лупа (свободный бег, rate=rpm/base)
    float grJit[96];             // per-grain ±5% джиттер — обновляется раз в цикл
    int16_t grSrc[96];           // ротация 1/12: какая гранула сейчас в слоте i
    int16_t grPrev[96];          // предыдущий источник при ротации (фейд 5мс)
    float  grPrevPh[96];         // фаза предыдущего источника (доигрывает своим темпом)
    uint8_t grFade[96];          // остаток фейда ротации, сэмплы
    float   cycDm;               // wobble длительности цикла ×1±0.002
    float   rpmSm;               // rpm для питча/уровня — сглаженные ~15мс: вход
                                 // rpm_ovr ступенями раз в 20мс давал AM на 25-50Гц
    float   envRpm;              // unused: окна идут от rpmSm (одна траектория с циклом)
    float cyc;                   // фаза цикла двигателя (4-такт: 2 об/цикл)
    uint8_t multiMode;           // 1 = банк с окнами (THOR), 0 = старый режим 2-пар
    float layerMix;            // 0 = off-слой, 1 = on-слой (плавный переход)
    uint8_t layerOn;           // целевой слой по газу
    uint8_t hasOff;            // в банке есть off-гранулы/правила
    uint32_t offDwell;         // мс при закрытом газе (гистерезис)
    // ПРАВКА 2026-11-22: idle-срезы (класс1) играются ПОДРЯД как цельный тейк
    // (~556мс), а не луп одного короткого куска (тот давал 7Гц throb + щелчки
    // швов = «цыканье» на тихих местах).
    int16_t idleSeq[8];        // индексы idle-гранул по порядку
    uint8_t idleSeqN, idleCur; // сколько срезов; текущий
    float   idlePh;            // фаза внутри текущего среза (сэмплы)
    uint8_t offIdlePrev;       // источник off-слоя в прошлом блоке (1=подложка) —
                               // чтобы не переключать источник под слышимый off (цок на газе)

    // one-shot события (старт/прострел/лимитер), -1 = слот свободен.
    // Пул 4 голоса: серия прострелов играет НАЛОЖЕНИЕМ (как в оригинале —
    // попы cls4 звучат конкурентно); одиночный слот обрезал предыдущий
    // хлопок на полной амплитуде = треск/подхрипывания.
    int   evIdx[4]; float evPhase[4]; float evGain[4];   // гейн на голос — поп (0.6) не рубит уже играющий старт-ивент (1.0)

    // SGU-голос: one-shot из f_03 файла (PCM16 mono).
    volatile const int16_t *sguPcm;
    volatile uint32_t sguLen;
    volatile uint16_t sguRep;        // сколько повторов осталось (PlaySguSound rep)
    volatile float    sguPos, sguStep, sguGain;
    int16_t *sguBuf;              // владение буфером (освобождаем при смене)
    int16_t *sguPending;          // graveyard: free делает рендер, не caller (нет UAF)
    rsb_bank_t *pendingBank;      // старый банк — rsb_free в начале следующего блока рендера

    // --- планировщик прострелов (backfire) ---
    int   bfLeft;              // сколько хлопков осталось в серии
    int   bfTimer;             // отсчёт до следующего хлопка (семплы)
    uint32_t rng;              // PRNG для рандомизации

    float masterGain;          // 0..1 общий объём (ручка громкости)
    float pkgGain;             // mainVolume из pkgrl — стенд восстанавливает сюда

    // --- настройки из THOR app (WritePresetRule) ---
    float idleGain;            // 0..1 громкость холостого (rule 0x30)
    float idlePitchOff;        // ± тон холостого, множитель питча (rule 0x18)
    float workPitchOff;        // ± тон рабочих оборотов (rule 0x19)
    int   backfireCount;       // 0..10 количество прострелов (rule 0x21, WritePresetRule)
    int   popMute;             // стенд: bf=0 — тишина всех хлопков (диагностика)
    float bfMinRpm;            // порог оборотов для серии (pkgrl rule 17, default 2500)
    // Зональные профили прострелов из RLM2 (ключи 0x112/0x212/0x312+j):
    // lo/hi rpm-окно, cntMin..cntMax хлопков в очереди. Порядок в файле —
    // по убыванию порога: берётся ПЕРВАЯ зона где rpm>=lo (=> низкие обороты
    // дают более плотную трескотню). 0 зон -> fallback {1700,7000,2,4}.
    // gapMs/gapP: +13 (k287=30) / +14 (k288=40) — внутриочередной тайминг,
    // подтверждено демо APK: зазоры квантуются решёткой ~33мс = геометрич.
    // распределение (каждый gapMs-такт выстрел с вероятностью gapP%).
    struct { uint16_t lo, hi, gapMs; uint8_t cntMin, cntMax, gapP; } bfZones[3];
    uint8_t bfZoneN;
    uint16_t bfGapMs;            // активные значения на момент старта серии
    uint8_t  bfGapP;
    uint8_t startMode;         // 0=выкл,1=стартер,2=динамичный (rule 0x01/0x02)
    uint8_t engineRunning;     // внутренний флаг: двигатель "заведён"


    // --- стенд (stand.c) ---
    volatile uint8_t standActive; // 1 = стенд владеет rpm/газом (CAN не пишет)
    volatile uint8_t standGear;   // текущая передача симулятора
    volatile int32_t canAliveMs;  // >0: CAN жив (ставит can-задача, тает в stand-таске)
    const struct car_can_t *carCan; // правила CAN-декодера выбранного авто
    int16_t lastSmp;              // последний сэмпл кадра — для рампа тишины

    // --- телеметрия для BLE NOTIFY ---
    volatile uint8_t preset;   // текущий индекс пресета
    volatile uint8_t coolantC; // температура ОЖ, °C (из OBD PID 0x05), 0 если нет
    volatile uint8_t errFlags; // флаги ошибок (bit0: банк не загружен, ...)

    // --- диагностика уровней (/diag): EMA по блокам рендера ---
    volatile float dbgGrMix;   // rms зернового микса ДО level law
    volatile float dbgGrN;     // среднее число активных гранул (grGain>=порог)
    volatile float dbgGrW;     // средняя сумма grGain за кадр
    volatile float dbgLaw;     // текущий множитель level law (с mg)
    volatile float dbgEv;      // rms вклада ивентов (после evGain*mg)
    volatile float dbgSgu;     // rms вклада SGU-голоса
    volatile float dbgOut;     // rms выхода после clip16
    // таймлайн для /dbg: кольцо 400 × 9 float — по блоку рендера (~6мс)
    float dbgLog[400][9];
    volatile uint16_t dbgLogPos;
} engine_t;

void engine_init(engine_t *e, const rsb_bank_t *bank);
void engine_set_rpm(engine_t *e, float rpm);
void engine_set_throttle(engine_t *e, float t);   // задаёт throttleTarget
void engine_set_acc_response(engine_t *e, float r); // 0..1 инерция газа
void engine_set_reverb(float wet);                  // 0..1 реверб «кожуха» (0=выкл)
void engine_trigger_event(engine_t *e, uint8_t type);
void engine_fire(engine_t *e, int idx, float gain); // выстрелить ивент idx в пул 4 голосов
void engine_stop_event(engine_t *e);                // глушит все one-shot голоса (StopSguSound)
void engine_trigger_backfire(engine_t *e);          // серия рандомных хлопков на сбросе газа
void engine_pop(engine_t *e);                        // одиночный случайный хлопок (перекатка/ХХ)
// SGU-звук: забирает владение буфером (предыдущий free'ится). fileRate — частота PCM.
void engine_play_sgu(engine_t *e, int16_t *pcm, uint32_t n, uint32_t fileRate, float gain, uint16_t rep);
void engine_stop_sgu(engine_t *e);

// Заполнить буфер stereo int16 (L=R) на nframes кадров. Частота = bank->sampleRate.
void engine_render(engine_t *e, int16_t *out, int nframes);
