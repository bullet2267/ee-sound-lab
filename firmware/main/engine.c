#include "engine.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_heap_caps.h"
#include "esp_random.h"

// перегруз-защита: линейно до ±32767, дальше жёсткий клип.
// tanh-лимитер убран — гнул сигнал уже на -3дБ при 28к входе (грязь на пиках).
static inline int16_t clip16(int32_t v) {
    if (v >  32767) return 32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

// Быстрый PRNG (xorshift32) — без libc rand, безопасно в аудио-потоке.
static inline uint32_t xrand(uint32_t *s) {
    uint32_t x = *s ? *s : 0x1234567u;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *s = x; return x;
}
static inline float frand(uint32_t *s) { return (xrand(s) >> 8) / 16777216.0f; } // 0..1

/* EQ «коробки» и rpm-наклон удалены целиком (были отключены — eq_run
 * возвращал x): синтезированная АЧХ не подтверждена банком. PCM идёт как есть. */

// Линейная интерполяция внутри гранулы (loop между loopStart..loopEnd), дробная фаза.
// сэмпл гранулы в позиции pos loop-домена (с кроссфейдом стыка петли);
// pos >= 0 в «координатах фазы»: 0 соответствует сэмплу ls+xf
static inline float grain_at(const rsb_bank_t *b, const rsb_grain_t *g, float pos) {
    uint32_t base = g->offset / 2;                 // в семплах
    uint32_t len  = g->length / 2;
    uint32_t ls   = g->loopStart;
    uint32_t le   = (g->loopEnd && g->loopEnd <= len) ? g->loopEnd : len;
    if (le <= ls + 1) le = len;
    uint32_t loopLen = le - ls;
    if (!loopLen || !len) return 0.f;               // пустая гранула — тишина, не /0

    // кроссфейд стыка петли: последние xf сэмплов смешиваем с головой,
    // эффективный луп длиной L=loopLen-xf играет со сдвига xf —
    // переход F(L-1)≈s(ls+xf-1) → F(0)=s(ls+xf) непрерывен, «тика» нет
    uint32_t xf = g->xfade ? g->xfade : 154;           // RLS2 k9 (~700/300 сэмплов)
    if (xf > loopLen / 2) xf = loopLen / 2;            // k9 > половины — берём пол-лупа
    uint32_t L = loopLen - xf;
    if (!L) { xf = 0; L = loopLen; }

    uint32_t i0 = (uint32_t)pos % L;
    float    fr = pos - (uint32_t)pos;

    if (xf && i0 >= L - xf) {
        // зона фейда: tail = ls+xf+i0 ∈ [le-xf, le), head = ls+t ∈ [ls, ls+xf)
        uint32_t t = i0 - (L - xf);
        float w = (float)(t + 1) / (float)(xf + 1);
        uint32_t ti = ls + xf + i0, hi = ls + t;
        float ta = b->payload[base + ti];
        float tb = b->payload[base + ((ti + 1 < le) ? ti + 1 : ti)];
        float ha = b->payload[base + hi];
        float hb = b->payload[base + ((hi + 1 < le) ? hi + 1 : hi)];
        return (ta + (tb - ta) * fr) * (1.f - w) + (ha + (hb - ha) * fr) * w;
    }
    uint32_t idx0 = ls + xf + i0;
    uint32_t idx1 = (i0 + 1 < L) ? idx0 + 1 : ls + xf;
    float s0 = b->payload[base + idx0];
    float s1 = b->payload[base + idx1];
    return s0 + (s1 - s0) * fr;
}

// Прямой fetch по позиции в сэмплах, луп = вся запись (модель CY_pwr2:
// у sync-гранул pos=cyc*L — запись и есть один цикл двигателя, стык
// ложится на границу цикла, xfade не нужен и не был в оригинале).
static inline float pcm_at(const rsb_bank_t *b, const rsb_grain_t *g, float pos) {
    uint32_t len = g->length / 2;
    if (!len) return 0.f;
    uint32_t base = g->offset / 2;
    uint32_t i0 = (uint32_t)pos % len;
    float fr = pos - (float)(uint32_t)pos;
    uint32_t i1 = (i0 + 1 < len) ? i0 + 1 : 0;
    float s0 = b->payload[base + i0], s1 = b->payload[base + i1];
    return s0 + (s1 - s0) * fr;
}

static inline float grain_sample(const rsb_bank_t *b, int gi, float *phase, float ratio) {
    const rsb_grain_t *g = &b->grains[gi];
    float out = grain_at(b, g, *phase);
    uint32_t len = g->length / 2, ls = g->loopStart;
    uint32_t le = (g->loopEnd && g->loopEnd <= len) ? g->loopEnd : len;
    if (le <= ls + 1) le = len;
    uint32_t loopLen = le - ls;
    if (!loopLen || !len) { *phase = 0; return out; }
    uint32_t xf = g->xfade ? g->xfade : 154;
    if (xf > loopLen / 2) xf = loopLen / 2;
    uint32_t L = loopLen - xf;
    if (!L) L = loopLen;
    float p = *phase + ratio;
    while (p >= (float)L) p -= L;
    *phase = p;
    return out;
}

// ---- аудио-дамп: захват отрендеренного PCM в RAM для диагностики звука ----
static int16_t *s_dumpBuf; static uint32_t s_dumpLen, s_dumpPos, s_dumpRate;
void engine_dump_arm(uint32_t frames, uint32_t rate) {
    free(s_dumpBuf); s_dumpBuf = NULL; s_dumpPos = s_dumpLen = 0;
    s_dumpBuf = heap_caps_malloc(frames * sizeof(int16_t),
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);   // 220KБ — только PSRAM
    if (s_dumpBuf) { s_dumpLen = frames; s_dumpRate = rate; }
}
uint32_t engine_dump_pos(void)  { return s_dumpPos; }
uint32_t engine_dump_rate(void) { return s_dumpRate; }
const int16_t *engine_dump_buf(void) { return s_dumpBuf; }
static inline void dump_put(int16_t s) {
    if (s_dumpBuf && s_dumpPos < s_dumpLen) s_dumpBuf[s_dumpPos++] = s;
}

void engine_init(engine_t *e, const rsb_bank_t *bank) {
    memset(e, 0, sizeof(*e));
    e->bank = bank;
    e->rpm = bank->rpmMin;
    e->throttle = 0.f; e->throttleTarget = 0.f;
    /* эталон render_pwr2: thr_sm += d*0.0007 за сэмпл -> tau=64.8мс.
     * accResponse=0.9565 даёт тот же tau через формулу фильтра ниже.
     * (0.7 -> tau=270мс было выдумкой: газ «подвисал на пике» ~0.6с после
     * смены цели — артефакт держа 9-10с в REV-профиле.) */
    e->accResponse = 0.9565f;
    for (int k = 0; k < 4; k++) { e->evIdx[k] = -1; e->evPhase[k] = 0.f; e->evGain[k] = 1.f; }
    e->masterGain = 1.0f; e->pkgGain = 1.0f;
    e->idleGain = 1.0f; e->idlePitchOff = 1.0f; e->workPitchOff = 1.0f;
    e->backfireCount = 3; e->bfMinRpm = 2500.f; e->startMode = 0; e->engineRunning = 0;
    e->bfZoneN = 1;                              // fallback-зона до загрузки RLM2
    e->bfZones[0] = (typeof(e->bfZones[0])){1700, 7000, 30, 2, 4, 40};
    e->bfGapMs = 30; e->bfGapP = 40;
    e->rng = 0xC0FFEEu;
    e->v0.idx = 0; e->v1.idx = (bank->grainCount > 1) ? 1 : 0;
    e->w0 = e->v0; e->w1 = e->v1;
    e->hasOff = (rsb_count_layer(bank, 1) > 0);
    e->layerOn = 0; e->layerMix = 0.f;   // холостой — off-слой
}

void engine_set_rpm(engine_t *e, float rpm)      { e->rpm = rpm; }
void engine_set_throttle(engine_t *e, float t)   { e->throttleTarget = t < 0 ? 0 : (t > 1 ? 1 : t); }
void engine_set_acc_response(engine_t *e, float r) { e->accResponse = r < 0 ? 0 : (r > 1 ? 1 : r); }

/* Пул ивентов: свободный слот, иначе — самый старый (у него самый большой
 * evPhase — его хвост меньше всего слышен при обрезке). */
void engine_fire(engine_t *e, int idx, float gain) {
    int slot = 0; float oldest = -1.f;
    for (int k = 0; k < 4; k++) {
        if (e->evIdx[k] < 0) { slot = k; break; }
        if (e->evPhase[k] > oldest) { oldest = e->evPhase[k]; slot = k; }
    }
    e->evIdx[slot] = idx; e->evPhase[slot] = 0.f; e->evGain[slot] = gain;
}

void engine_trigger_event(engine_t *e, uint8_t type) {
    int i = rsb_find_event(e->bank, type);
    if (i >= 0) engine_fire(e, i, 1.f);
}

void engine_stop_event(engine_t *e) { for (int k = 0; k < 4; k++) e->evIdx[k] = -1; }

void engine_play_sgu(engine_t *e, int16_t *pcm, uint32_t n, uint32_t fileRate, float gain, uint16_t rep) {
    int16_t *old = e->sguBuf;
    e->sguPcm = NULL; e->sguBuf = NULL;              // рендер больше не читает старый
    e->sguPending = old;                             // free в начале следующего блока —
                                                     // текущая итерация рендера могла уже взять sp
    uint32_t br = (e->bank && e->bank->sampleRate) ? e->bank->sampleRate : 22050;
    e->sguBuf = pcm; e->sguLen = n; e->sguPos = 0.f;
    e->sguStep = fileRate ? (float)fileRate / (float)br : 1.f;
    e->sguRep = rep; e->sguGain = gain; e->sguPcm = pcm;
}
void engine_stop_sgu(engine_t *e) { e->sguPcm = NULL; }

// Серия хлопков — количество из app (backfireCount, 0 = выкл).
// Тайминг берём из активной зоны RLM2: каждые gapMs мс «такт», выстрел
// с вероятностью gapP% -> зазоры на решётке gapMs (как в демо APK ~33мс).
void engine_trigger_backfire(engine_t *e) {
    int n = e->backfireCount;
    if (n <= 0 || e->popMute) { e->bfLeft = 0; return; }
    e->bfGapMs = 30; e->bfGapP = 40;
    for (int i = 0; i < e->bfZoneN; i++)
        if (e->rpm >= e->bfZones[i].lo && e->rpm <= e->bfZones[i].hi)
            { e->bfGapMs = e->bfZones[i].gapMs; e->bfGapP = e->bfZones[i].gapP; break; }
    e->bfLeft = n;                                  // ровно count (0=выкл обработан выше)
    e->bfTimer = 0;                                 // первый — сразу
}

// Одиночный случайный хлопок из пула cls4 — трескотня перекатки.
// engine_trigger_event(2) всегда берёт первый rid45 (rsb_find_event);
// здесь rand — каждый выстрел другой сэмпл, как серия bfLeft.
void engine_pop(engine_t *e) {
    if (!e->bank || e->popMute) return;
    int i = rsb_find_event_rand(e->bank, 2, xrand(&e->rng));
    if (i >= 0) engine_fire(e, i, 1.f);
}

static volatile uint32_t s_renderCalls;
uint32_t engine_render_calls(void) { return s_renderCalls; }

void engine_render(engine_t *e, int16_t *out, int nframes) {
    s_renderCalls++;
    // graveyard: буферы, помеченные callers — безопасно: рендер сам их освобождает
    int16_t *dead = e->sguPending;
    if (dead) { e->sguPending = NULL; free(dead); }
    rsb_bank_t *deadBank = e->pendingBank;
    if (deadBank) { e->pendingBank = NULL; rsb_free(deadBank); }
    const rsb_bank_t *b = e->bank;
    e->bankAck = b;                               // ACK: рендер работает с этим банком
    if (b != e->lastBank) {                     // банк сменился — сброс голосов/эвентов
        e->lastBank = b;
        e->v0.idx = e->v1.idx = e->w0.idx = e->w1.idx = 0;
        e->v0.phase = e->v1.phase = e->w0.phase = e->w1.phase = 0;
        for (int k = 0; k < 4; k++) e->evIdx[k] = -1;
        e->layerMix = 1.f;
        e->bfLeft = 0; e->bfTimer = 0;
        e->hasOff = (b && rsb_count_layer(b, 1) > 0);
        /* ПРАВКА 2026-11-22: ВЫКЛ синтез-гранулятор. Играем ОРИГИНАЛ —
         * честный ladder-crossfade реальных лупов (как revheadz/THOR):
         * rsb_grain_pair_l -> 2 соседних лупа слоя, питч rpm/baseRpm,
         * кроссфейд по RPM, бленд on/off по газу. Никакого синтеза. */
        e->multiMode = 0;
        for (int k = 0; k < 96; k++) { e->grGain[k] = 0.f; e->grJit[k] = 1.f;
                                     /* стартовая фаза случайная — иначе все слоты
                                      * идут синфазно (скрипт: ph=rng.random()) */
                                     e->grPh[k] = frand(&e->rng);
                                     e->grSrc[k] = (int16_t)k; e->grPrev[k] = -1; e->grFade[k] = 0;
                                     e->grPrevPh[k] = 0.f; }
        e->cyc = 0.f; e->cycDm = 1.f; e->envRpm = e->rpm; e->rpmSm = e->rpm;
    }
    if (!b || b->grainCount == 0 || b->sampleRate == 0 || e->rpm < 300.f) {
        // заглушен/нет банка: доигрываем активный one-shot (shutoff!), иначе
        // плавный спад к нулю ~15мс вместо щелчка в динамике
        memset(out, 0, nframes * 2 * sizeof(int16_t));
        if (b) {
            bool any = false;
            for (int k = 0; k < 4; k++) if (e->evIdx[k] >= 0) any = true;
            if (any) {
                for (int n = 0; n < nframes; n++) {
                    int32_t m = 0;
                    for (int k = 0; k < 4; k++) {
                        int ei = e->evIdx[k];
                        if (ei < 0) continue;
                        const rsb_event_t *ev = &b->events[ei];
                        uint32_t base = ev->offset / 2, len = ev->length / 2;
                        uint32_t p = (uint32_t)e->evPhase[k];
                        if (p < len) { m += (int32_t)((float)b->payload[base + p] * e->evGain[k] * e->masterGain);
                                       e->evPhase[k] += 1.f; }
                        else e->evIdx[k] = -1;
                    }
                    int16_t s = clip16(m);
                    out[2*n] = out[2*n+1] = s; e->lastSmp = s;
                }
                return;
            }
        }
        // экспонентальный спад от последнего сэмпла — нет постоянки, нет щелчка
        if (e->lastSmp != 0) {
            int16_t s = e->lastSmp;
            for (int n = 0; n < nframes && s != 0; n++) {
                s = (int16_t)(s * 0.97f);            // ~15мс до тишины @44.1к
                out[2*n] = out[2*n+1] = s;
            }
            e->lastSmp = s;
        }
        return;
    }
    float sr = (float)b->sampleRate;

    uint16_t rpm = (uint16_t)(e->rpm < 0 ? 0 : e->rpm);

    // фильтр газа (accResponse): tau = 0.03с при r=1 … 0.83с при r=0
    {
        float tau = 0.03f + (1.f - e->accResponse) * 0.8f;
        float alpha = (float)nframes / (sr * tau);
        if (alpha > 1.f) alpha = 1.f;
        e->throttle += (e->throttleTarget - e->throttle) * alpha;
    }

    // --- выбор слоя с гистерезисом: газ >0.15 -> on, <0.08 дольше 150мс -> off ---
    if (e->throttle > 0.15f) { e->layerOn = 1; e->offDwell = 0; }
    else if (e->throttle < 0.08f) {
        e->offDwell += (uint32_t)(nframes * 1000 / sr);
        if (e->offDwell > 150) e->layerOn = 0;
    } else e->offDwell = 0;
    if (!e->hasOff) e->layerOn = 1;      // старые банки — только on-слой
    float lmTarget = e->layerOn ? 1.f : 0.f;
    float lmStep   = 1.0f / (sr * 0.12f);        // ~120 мс переход

    // тон: плавный переход idlePitchOff -> workPitchOff по rpm (общий для обоих режимов)
    float tn = (b->rpmMax > b->rpmMin) ? (e->rpm - b->rpmMin) / (float)(b->rpmMax - b->rpmMin) : 0.f;
    if (tn < 0) tn = 0;
    if (tn > 1) tn = 1;
    float pitchOff = e->idlePitchOff + (e->workPitchOff - e->idlePitchOff) * tn;
    float idleMix = (e->rpm < 1500.f) ? 1.f - (e->rpm / 1500.f) : 0.f;   // <1500 — idle gain
    float comGain = e->masterGain * (1.f - idleMix + idleMix * e->idleGain);
    /* динамику задаёт сам банк (env + gainQ8 по оборотам) — рукотворная
     * rpm/thr-горка убрана: загоняла микс в лимитер и схлопывала динамику. */

    // --- старый режим 2-пар (банки без THOR-окон) ---
    int gi = 0, gj = 0, ki = 0, kj = 0;
    float onRatio0 = 1.f, onRatio1 = 1.f, onA0 = 1.f, onA1 = 0.f, onGain = comGain;
    float offRatio0 = 1.f, offRatio1 = 1.f, offA0 = 1.f, offA1 = 0.f, offGain = 0.f;
    if (!e->multiMode) {
        if (rsb_grain_pair_l(b, rpm, 0, &gi, &gj) != 0) { gi = 0; gj = (b->grainCount > 1); }
        e->v0.idx = gi; e->v1.idx = gj;
        uint16_t r0 = b->grains[gi].baseRpm, r1 = b->grains[gj].baseRpm;
        float xf = (r1 > r0) ? (float)(rpm - r0) / (float)(r1 - r0) : 0.f;
        if (xf < 0) xf = 0;
        if (xf > 1) xf = 1;
        onRatio0 = ((r0 > 0) ? e->rpm / (float)r0 : 1.f) * pitchOff;
        onRatio1 = ((r1 > 0) ? e->rpm / (float)r1 : 1.f) * pitchOff;
        onA1 = xf; onA0 = 1.f - xf;
        uint16_t gQ, lQ;
        rsb_rule_lookup_l(b, rpm, 0, &gQ, &lQ);
        onGain = ((gQ / 256.0f) + (lQ / 256.0f) * e->throttle)
                 * e->masterGain * (1.f - idleMix + idleMix * e->idleGain);
        if (e->hasOff && rsb_grain_pair_l(b, rpm, 1, &ki, &kj) == 0) {
            e->w0.idx = ki; e->w1.idx = kj;
            uint16_t q0 = b->grains[ki].baseRpm, q1 = b->grains[kj].baseRpm;
            float xo = (q1 > q0) ? (float)(rpm - q0) / (float)(q1 - q0) : 0.f;
            if (xo < 0) xo = 0;
            if (xo > 1) xo = 1;
            offA1 = xo; offA0 = 1.f - xo;
            offRatio0 = (q0 > 0) ? e->rpm / (float)q0 : 1.f;
            offRatio1 = (q1 > 0) ? e->rpm / (float)q1 : 1.f;
            rsb_rule_lookup_l(b, rpm, 1, &gQ, &lQ);
            offGain = ((gQ / 256.0f) + (lQ / 256.0f) * (1.f - e->throttle)) * e->masterGain;
        } else if (e->hasOff) { e->w0.idx = gi; e->w1.idx = gj; }
    }
    // multiMode: назначения голосов нет — ретриггер-модель работает внутри
    // сэмплового цикла: каждый голос играет гранулу целиком (без петли),
    // по окончании берёт СЛУЧАЙНУЮ гранулу из rpm-окна (вес = env*gain*layerW).
    // Гранулы — нарезки непрерывной записи: поток разных кусков = живой бёрбл,
    // без периодики лупа. k9 = длина фейда хвоста записи (они обрываются на
    // полной амплитуде — без фейда был бы щелчок каждые ~0.1с).



    float accGr = 0.f, accEv = 0.f, accSgu = 0.f, accOut = 0.f;
    float accGrN = 0.f, accGrW = 0.f, accLaw = 0.f;

    for (int n = 0; n < nframes; n++) {
        // плавный переход слоя (~120 мс)
        e->layerMix += (lmTarget > e->layerMix) ? lmStep : -lmStep;
        if (e->layerMix < 0.f) e->layerMix = 0.f;
        if (e->layerMix > 1.f) e->layerMix = 1.f;

        float mix;
        if (e->multiMode) {
            // THOR-лупы: голос крутит гранулу в петле (k9-кроссфейд стыка)
            // пока rpm в её окне; переход на соседнюю — idx2 по затуханию env.
            // Голоса 0-2 — рабочий класс, 3 — холостой; pick фильтрует класс
            // и исключает занятые гранулы => полное покрытие окна без провалов.
            // selRpm клампится к краю банка — выше последнего бэнда голоса
            // продолжают играть на верхней грануле (иначе тишина >6500).
            /* окна и цикл — от ОДНОГО сглаженного rpm (модель render_pwr2:
             * env(g,rpm) и cyc по одному значению). Отдельный тяжёлый фильтр
             * envRpm (~45мс) отставал от фазы цикла на свипе ~58об и додвигал
             * веса ~135мс ПОСЛЕ остановки оборотов = «подвисание» на входе в
             * держ. rpmSm ~15мс гасит ступени 20мс входа и держит края окон. */
            e->rpmSm += (e->rpm - e->rpmSm) * 0.003f;
            float rpmS = e->rpmSm;
            float selRpm = rpmS;
            if (b->rpmMax && selRpm > (float)b->rpmMax) selRpm = (float)b->rpmMax;
            mix = 0.f;
            /* ===== ПРАВКА 2026-11-19: ОДИН КЛАСС ЗА РАЗ =====================
             * Было: layW=1.f, то есть cls2 и cls3 звучали параллельно с полным
             * весом. Измерено на паке PWR (_corr.py): cls2 x cls3 одного бэнда
             * коррелируют до +0.953 — это одна и та же запись в двух классах.
             * Две коррелированные копии на общей фазе = комб-фильтр; на рендере
             * он давал 5.7-7.7 дБ ряби на субгармонике частоты вспышек.
             * Проверено: render_pwr_test.py (разброс гармоник 6.3 -> 4.9 дБ,
             * попал в диапазон 10 эталонов 3.6-5.7) и render_pwr_drag.py
             * (ступеньки на шифтах 13.5 -> 9.1 дБ, СКО выровнено 2.0-2.3).
             * Отображение lay в bank.c НЕ менял, чтобы не перевернуть старый
             * on/off путь: по нему слой 1 = cls2 (нагрузка), 0 = cls3 (сброс).
             */
            /* ===== ПОРТ ПРИНЯТОЙ МОДЕЛИ CY_pwr2 (render_pwr2.py, «идеально») ==
             * - общий цикл cyc += rpm/(2.012*60*sr)*dm, dm=1±0.002 на границе
             * - sync-гранулы (revs≈2.012=цикл) играют pos=(cyc+pjit)*L — залочены
             *   на фазу цикла; остальные — свободной фазой rpm/base
             * - per-cycle: gjit=1±0.05, pjit=rand*0.003, ротация 1/12 -> случайная
             *   гранула того же класса в ±35% base, кроссфейд 5мс
             * - веса: cls2 (нагрузка) ×(0.25+0.75·thr), cls3 (сброс) ×(1−0.85·thr),
             *   cls1 ×idleGain; cls5/6 подложки ×(0.55/0.3)·clamp((2000-rpm)/800)
             * - level law: mix/2 · (0.5+0.5·thr) · (rpm/800)^1.2 · lowrpm-lift */
            const int XF = (int)(0.005f * sr);               // 5мс фейд ротации
            /* rpmSm ~15мс: вход стенда/CAN-оверрайда ступенями по 20мс
             * давал ступеньки питча и уровня = AM на 25-50Гц («зависло»).
             * Медленное блуждание оборотов сохраняется, ступени — нет. */
            if (rpmS > 300.f) {
                e->cyc += rpmS / (2.012f * 60.f * sr) * e->cycDm;
                if (e->cyc >= 1.f) {
                    e->cyc -= 1.f;
                    e->cycDm = 1.f + 0.002f * (frand(&e->rng) * 2.f - 1.f);
                    for (int i = 0; i < b->grainCount && i < 96; i++) {
                        e->grJit[i]  = 1.f + 0.05f * (frand(&e->rng) * 2.f - 1.f);
                        /* ротация с вероятностью 1/12 за цикл */
                        if (frand(&e->rng) < 1.f / 12.f) {
                            const rsb_grain_t *sg = &b->grains[i];
                            int cls = (sg->flags >> 3) & 7;
                            uint32_t sb = sg->baseRpm;
                            if (!cls || !sb || cls >= 5) continue;   /* подложки не ротируются (модель) */
                            int pick[96], pc = 0;
                            for (int j = 0; j < b->grainCount && j < 96 && pc < 96; j++) {
                                if (j == i) continue;
                                const rsb_grain_t *o = &b->grains[j];
                                if (((o->flags >> 3) & 7) != cls) continue;
                                uint32_t ob = o->baseRpm;
                                if (ob * 100 >= sb * 65 && ob * 100 <= sb * 135) pick[pc++] = j;
                            }
                            if (pc) {
                                e->grPrev[i]   = e->grSrc[i];
                                e->grPrevPh[i] = e->grPh[i];   // старый доигрывает
                                e->grSrc[i]  = (int16_t)pick[(int)(frand(&e->rng) * pc) % pc];
                                e->grFade[i] = (uint8_t)(XF > 255 ? 255 : XF);
                            }
                        }
                    }
                }
            }
            float thr = e->throttle;
            for (int i = 0; i < b->grainCount && i < 96; i++) {
                const rsb_grain_t *g = &b->grains[i];
                int cls = (g->flags >> 3) & 7;
                float w;
                if (cls >= 5) {
                    /* длинные подложки: фактор (2000-rpm)/800 клэмпим [0,1] —
                     * без клэмпа cls6 на <1200 об летел выше своего кэпа 0.30.
                     * gainQ8 (k4) применяется — в pkg31 у подложек авторский
                     * гейн как у обычных гранул. */
                    float f = (2000.f - rpmS) / 800.f;
                    if (f < 0.f) f = 0.f; else if (f > 1.f) f = 1.f;
                    w = (cls == 5 ? 0.55f : 0.30f) * f * (g->gainQ8 / 256.f);
                } else {
                    float ev = rsb_grain_env(b, i, selRpm);
                    w = ev * (g->gainQ8 / 256.f);
                    if (cls == 2)      w *= 0.25f + 0.75f * thr;
                    else if (cls == 3) w *= 1.0f - 0.85f * thr;
                    else if (cls == 1) w *= e->idleGain;
                }
                e->grGain[i] += (w - e->grGain[i]) * 0.002f;
                accGrW += e->grGain[i];
                if (e->grGain[i] < 0.0003f) continue;
                accGrN += 1.f;
                int src = (e->grSrc[i] >= 0 && e->grSrc[i] < b->grainCount)
                        ? e->grSrc[i] : i;
                const rsb_grain_t *s = &b->grains[src];
                float Ls = s->length * 0.5f;
                /* ПРАВКА 2026-11-20 (бук-бук): свободные фазы у ВСЕХ гранул.
                 * Цикл-лок (pos=cyc*L) давал всем гранулам одинаковый период —
                 * их внутренние вспышечные импульсы слипались в один удар за
                 * цикл = «бурление бук-бук-бук», вспышечный ритм (у оригинала
                 * ~150-200Гц на огибающей разгона) гасился до 13-84Гц.
                 * Свободный ход: revs у гранул 1.98-2.05 (не ровно цикл) ->
                 * периоды чуть разные -> импульсы дрейфуют и переплетаются =
                 * сплошной рык как у оригинала. Фаза дробная 0..1 — при
                 * ротации переходит в новую гранулу без скачка позиции. */
                e->grPh[i] += (s->baseRpm ? rpmS / s->baseRpm : 0.f) / Ls;
                if (e->grPh[i] >= 1.f) e->grPh[i] -= 1.f;
                float v = pcm_at(b, s, e->grPh[i] * Ls);
                if (e->grFade[i]) {
                    int pv = e->grPrev[i];
                    if (pv >= 0 && pv < b->grainCount) {
                        const rsb_grain_t *p = &b->grains[pv];
                        /* старый источник доигрывает со своей фазы и своим
                         * темпом — grPrevPh тащится отдельно */
                        e->grPrevPh[i] += (p->baseRpm ? rpmS / p->baseRpm : 0.f)
                                        / (p->length * 0.5f);
                        if (e->grPrevPh[i] >= 1.f) e->grPrevPh[i] -= 1.f;
                        float v2 = pcm_at(b, p, e->grPrevPh[i] * p->length * 0.5f);
                        float a = (float)e->grFade[i] / (float)(XF > 255 ? 255 : XF);
                        v = v * (1.f - a) + v2 * a;
                    }
                    e->grFade[i]--;
                }
                mix += v * e->grGain[i] * e->grJit[i];
            }
            accGr += mix * mix;
            /* level law из принятого рендера: газ + (rpm/800)^1.2 + лифт низа.
             * ×0.5 — калибровка по оригинальному демо S63 (top-rms ~10.3k,
             * pk32767 без плоских вершин): 0.55 давал top-rms 11341 и 0.1%
             * клипа на пиках тычков; 0.8 — перегруз 0.77% (хедрум был настроен
             * когда газ до движка не доходил, thr=0 весь прогон). */
            mix /= 2.f;
            float t25 = rpmS / 2500.f; if (t25 > 1.f) t25 = 1.f;
            float law = (0.5f + 0.5f * thr) * powf(rpmS / 800.f, 1.2f)
                 * (1.f + 1.8f * (1.f - t25)) * e->masterGain * 0.5f;
            accLaw += law;
            mix *= law;
        } else {
            float s0 = grain_sample(b, e->v0.idx, &e->v0.phase, onRatio0);
            float s1 = grain_sample(b, e->v1.idx, &e->v1.phase, onRatio1);
            mix = (s0 * onA0 + s1 * onA1) * onGain * e->layerMix;
            if (e->hasOff && e->layerMix < 1.f) {
                float t0 = grain_sample(b, e->w0.idx, &e->w0.phase, offRatio0);
                float t1 = grain_sample(b, e->w1.idx, &e->w1.phase, offRatio1);
                mix += (t0 * offA0 + t1 * offA1) * offGain * (1.f - e->layerMix);
            }
        }

        /* турбо-свист (синус+шум) удалён — синтез, в оригинале нет */

        if (e->bfLeft > 0) {
            if (e->bfTimer <= 0) {
                int i = rsb_find_event_rand(b, 2, xrand(&e->rng)); // rng продвигается — вариант реально случаен
                if (i >= 0) engine_fire(e, i, 1.f);
                e->bfLeft--;
                /* Геометрическая модель слотов (подтв. демо APK ~33мс решёткой):
                 * такт bfGapMs, выстрел прошёл — следующий слот через N тактов,
                 * N ~ geometric(p=bfGapP): mean = gapMs/p = 30/0.4 = 75мс. */
                int ticks = 1;
                while (frand(&e->rng) * 100.f >= (float)e->bfGapP && ticks < 64) ticks++;
                e->bfTimer = (int)(sr * e->bfGapMs * ticks / 1000.f);
            } else e->bfTimer--;
        }

        /* ивенты — пул 4 конкурентных голоса: прострелы cls4 накладываются
         * друг на друга, а не обрезаются (обрезка = треск на стыке). */
        for (int k = 0; k < 4; k++) {
            int ei = e->evIdx[k];
            if (ei < 0) continue;
            const rsb_event_t *ev = &b->events[ei];
            uint32_t base = ev->offset / 2, len = ev->length / 2;
            uint32_t p = (uint32_t)e->evPhase[k];
            if (p < len) { float ev = (float)b->payload[base + p] * e->evGain[k] * e->masterGain;
                           mix += ev; accEv += ev * ev;
                           e->evPhase[k] += 1.0f; }
            else e->evIdx[k] = -1;
        }

        {   // SGU-голос: линейная интерполяция PCM16, gain из PlaySguSound
            const int16_t *sp = (const int16_t *)e->sguPcm;
            if (sp && e->sguPos < e->sguLen) {
                uint32_t i0 = (uint32_t)e->sguPos;
                float fr = e->sguPos - i0;
                float s = sp[i0] + ((i0 + 1 < e->sguLen ? sp[i0 + 1] : 0.f) - sp[i0]) * fr;
                float sv = s * e->sguGain * e->masterGain;
                mix += sv; accSgu += sv * sv;
                e->sguPos += e->sguStep;
                if (e->sguPos >= e->sguLen) {
                    if (e->sguRep > 1) { e->sguRep--; e->sguPos = 0.f; }  // rep>1 — повторяем
                    else e->sguPcm = NULL;
                }
            }
        }

        int16_t s = clip16((int32_t)mix);
        accOut += (float)s * s;
        out[2*n]   = s;    // L
        out[2*n+1] = s;    // R
        e->lastSmp = s;    // для рампа тишины при rpm<300
        dump_put(s);       // диагностический захват (если armed)
    }

    if (nframes > 0) {
        const float k = 1.f / 32.f;                  // EMA ~32 блока (~0.1-0.2с)
        e->dbgGrMix += (sqrtf(accGr / nframes) - e->dbgGrMix) * k;
        e->dbgGrN   += (accGrN / nframes - e->dbgGrN) * k;
        e->dbgGrW   += (accGrW / nframes - e->dbgGrW) * k;
        e->dbgLaw   += (accLaw / nframes - e->dbgLaw) * k;
        e->dbgEv    += (sqrtf(accEv / nframes) - e->dbgEv) * k;
        e->dbgSgu   += (sqrtf(accSgu / nframes) - e->dbgSgu) * k;
        e->dbgOut   += (sqrtf(accOut / nframes) - e->dbgOut) * k;
        static int dbgDec;
        if (++dbgDec >= 8) {                       // ~46мс/запись, кольцо ~18.5с
            dbgDec = 0;
            float *L = e->dbgLog[e->dbgLogPos];
            L[0]=e->rpm;      L[1]=e->throttle; L[2]=sqrtf(accGr/nframes);
            L[3]=accGrN/nframes; L[4]=accGrW/nframes; L[5]=accLaw/nframes;
            L[6]=sqrtf(accEv/nframes); L[7]=sqrtf(accSgu/nframes);
            L[8]=sqrtf(accOut/nframes);
            e->dbgLogPos = (e->dbgLogPos + 1) % 400;
        }
    }
}
