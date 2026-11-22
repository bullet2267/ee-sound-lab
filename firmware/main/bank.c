#include "bank.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"   /* ПРАВКА 2026-11-19 (5): диагностика загрузки банка */
static const char *BTAG = "bank";

/* Формат .rsb (RSB1 + IMA-ADPCM) УДАЛЁН — наш выдуманный контейнер, у
 * оригинала банки это пара SMP2+RLS2 (f_04/f_05) + RLM2 (f_06). Хуже того,
 * загруженный .rsb пропускал pkgrl/mode-rules и терял класс k0 (класс
 * угадывался эвристикой). Все банки грузятся только через thor_load_bank.
 * Если понадобится банк без серверной пары — упаковываем его в настоящий
 * RLS2/SMP2 офлайн-тулингом, а не в RSB1. */

void rsb_free(rsb_bank_t *b) {
    if (b->_image)   free(b->_image);
    if (b->_decoded) free(b->_decoded);
    if (b->_grains)  free(b->_grains);
    if (b->_events)  free(b->_events);
    if (b->_rules)   free(b->_rules);
    memset(b, 0, sizeof(*b));
}

int rsb_grain_below(const rsb_bank_t *b, uint16_t rpm) {
    int lo = 0, hi = b->grainCount - 1, res = 0;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        if (b->grains[mid].baseRpm <= rpm) { res = mid; lo = mid + 1; }
        else hi = mid - 1;
    }
    return res;
}

void rsb_rule_lookup(const rsb_bank_t *b, uint16_t rpm, uint16_t *gain, uint16_t *loadGain) {
    if (b->ruleCount == 0) { *gain = 0x0100; *loadGain = 0; return; }
    const rsb_rule_t *r = b->rules;
    if (rpm <= r[0].rpm)               { *gain = r[0].gain; *loadGain = r[0].loadGain; return; }
    if (rpm >= r[b->ruleCount-1].rpm)  { *gain = r[b->ruleCount-1].gain; *loadGain = r[b->ruleCount-1].loadGain; return; }
    for (int i = 1; i < b->ruleCount; i++) {
        if (rpm <= r[i].rpm) {
            uint16_t r0 = r[i-1].rpm, r1 = r[i].rpm;
            uint32_t t = (uint32_t)(rpm - r0) * 256 / (r1 - r0);   // Q0.8
            *gain     = r[i-1].gain     + (int32_t)(r[i].gain     - r[i-1].gain)     * (int32_t)t / 256;
            *loadGain = r[i-1].loadGain + (int32_t)(r[i].loadGain - r[i-1].loadGain) * (int32_t)t / 256;
            return;
        }
    }
    *gain = 0x0100; *loadGain = 0;
}

int rsb_find_event(const rsb_bank_t *b, uint8_t type) {
    for (int i = 0; i < b->eventCount; i++) if (b->events[i].type == type) return i;
    return -1;
}

// рандомный ивент типа (пул вариантов прострелов в THOR-пакетах)
int rsb_find_event_rand(const rsb_bank_t *b, uint8_t type, uint32_t seed) {
    int cnt = 0;
    for (int i = 0; i < b->eventCount; i++) if (b->events[i].type == type) cnt++;
    if (!cnt) return -1;
    int pick = (int)(seed % (uint32_t)cnt);
    for (int i = 0; i < b->eventCount; i++)
        if (b->events[i].type == type && pick-- == 0) return i;
    return -1;
}

// ивент по rid записи (хранится в _rsv) — для PlaySguSound(sid)
int rsb_find_event_rid(const rsb_bank_t *b, uint16_t rid) {
    for (int i = 0; i < b->eventCount; i++) if (b->events[i]._rsv == rid) return i;
    return -1;
}

int rsb_count_layer(const rsb_bank_t *b, int layer) {
    int n = 0;
    for (int i = 0; i < b->grainCount; i++)
        if (((b->grains[i].flags >> 1) & 1) == layer) n++;
    return n;
}

int rsb_grain_pair_l(const rsb_bank_t *b, uint16_t rpm, int layer, int *gi, int *gj) {
    int lo = -1, hi = -1;
    for (int i = 0; i < b->grainCount; i++) {
        if (((b->grains[i].flags >> 1) & 1) != layer) continue;
        if (b->grains[i].baseRpm <= rpm) lo = i;
        else if (hi < 0) hi = i;
    }
    if (lo < 0 && hi < 0) return -1;         // слой пуст
    if (lo < 0) lo = hi;
    if (hi < 0) hi = lo;
    *gi = lo; *gj = hi;
    return 0;
}

void rsb_rule_lookup_l(const rsb_bank_t *b, uint16_t rpm, int layer, uint16_t *gain, uint16_t *loadGain) {
    const rsb_rule_t *r = b->rules;
    int first = -1, last = -1;
    for (int i = 0; i < b->ruleCount; i++) {
        if ((r[i].rpm >> 15) != layer) continue;
        if (first < 0) first = i;
        last = i;
    }
    if (first < 0) { *gain = 0x0100; *loadGain = 0; return; }
    uint16_t f0 = r[first].rpm & 0x7FFF, l0 = r[last].rpm & 0x7FFF;
    if (rpm <= f0) { *gain = r[first].gain; *loadGain = r[first].loadGain; return; }
    if (rpm >= l0) { *gain = r[last].gain;  *loadGain = r[last].loadGain;  return; }
    int prev = first;
    for (int i = first + 1; i <= last; i++) {
        if ((r[i].rpm >> 15) != layer) continue;
        uint16_t r1 = r[i].rpm & 0x7FFF;
        if (rpm <= r1) {
            uint16_t r0 = r[prev].rpm & 0x7FFF;
            if (r1 <= r0) { *gain = r[i].gain; *loadGain = r[i].loadGain; return; }
            uint32_t t = (uint32_t)(rpm - r0) * 256 / (r1 - r0);
            *gain     = r[prev].gain     + (int32_t)(r[i].gain     - r[prev].gain)     * (int32_t)t / 256;
            *loadGain = r[prev].loadGain + (int32_t)(r[i].loadGain - r[prev].loadGain) * (int32_t)t / 256;
            return;
        }
        prev = i;
    }
    *gain = r[last].gain; *loadGain = r[last].loadGain;
}

int rsb_has_windows(const rsb_bank_t *b) {
    if (!b || !b->grainCount) return 0;
    for (int i = 0; i < b->grainCount; i++)
        if (b->grains[i].fadeHi || b->grains[i].flatHi) return 1;
    return 0;
}

// трапец-огибающая THOR-окна: [fadeLo..flatLo] ramp-in, [flatLo..flatHi] full,
// [flatHi..fadeHi] ramp-out, вне — 0
float rsb_grain_env(const rsb_bank_t *b, int gi, float rpm) {
    const rsb_grain_t *g = &b->grains[gi];
    if (!(g->fadeHi || g->flatHi)) return 0.f;
    if (rpm < (float)g->fadeLo || rpm > (float)g->fadeHi) return 0.f;
    if (rpm < (float)g->flatLo)
        return (g->flatLo > g->fadeLo) ? (rpm - g->fadeLo) / (float)(g->flatLo - g->fadeLo) : 1.f;
    if (rpm <= (float)g->flatHi) return 1.f;
    return (g->fadeHi > g->flatHi) ? (g->fadeHi - rpm) / (float)(g->fadeHi - g->flatHi) : 0.f;
}

// ================= THOR-пакет (SMP2 + RLS2) -> rsb_bank_t =================
static uint16_t rd16be(const uint8_t *p) { return (uint16_t)(p[0]<<8 | p[1]); }
static uint32_t rd32be(const uint8_t *p) { return (uint32_t)(p[0]<<24 | p[1]<<16 | p[2]<<8 | p[3]); }
static uint16_t q88f(float x) { int v = (int)(x*256.f+0.5f); return (uint16_t)(v<0?0:v>0xFFFF?0xFFFF:v); }

typedef struct { uint16_t id; const uint8_t *pcm; uint32_t plen; } smp_rec_t;

int thor_load_bank(const char *smp_path, const char *rls_path, rsb_bank_t *b) {
    memset(b, 0, sizeof(*b));
    FILE *f = fopen(smp_path, "rb"); if (!f) return -1;
    fseek(f, 0, SEEK_END); long ssz = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *img = malloc(ssz);
    if (!img || fread(img, 1, ssz, f) != (size_t)ssz) { fclose(f); free(img); return -2; }
    fclose(f);
    if (ssz < 8 || ssz > 24L*1024*1024 || memcmp(img, "SMP2", 4)) { free(img); return -3; }
    int n = rd16be(img+4);
    if (6 + (uint32_t)n*4 > (uint32_t)ssz) { free(img); return -3; }   // таблица офсетов
    smp_rec_t *recs = malloc((size_t)n*sizeof(smp_rec_t)); if (!recs) { free(img); return -4; }
    for (int i = 0; i < n; i++) {
        uint32_t o = rd32be(img+6+i*4), e = i+1<n ? rd32be(img+6+(i+1)*4) : (uint32_t)ssz;
        if (o + 6 > (uint32_t)ssz || e < o + 6) { recs[i].plen = 0; recs[i].id = 0; recs[i].pcm = img; continue; }
        if (e > (uint32_t)ssz) e = ssz;
        const uint8_t *r = img+o;
        recs[i].id = rd16be(r); recs[i].plen = rd32be(r+2);
        recs[i].pcm = r+6;
        if (recs[i].plen > e-o-6) recs[i].plen = e-o-6;
        recs[i].plen &= ~1u;
    }
    FILE *rf = fopen(rls_path, "rb"); if (!rf) { free(recs); free(img); return -5; }
    fseek(rf,0,SEEK_END); long rsz_f = ftell(rf); fseek(rf,0,SEEK_SET);
    uint8_t *rimg = malloc(rsz_f);
    if (!rimg || fread(rimg,1,rsz_f,rf)!=(size_t)rsz_f){ fclose(rf); free(rimg); free(recs); free(img); return -6; }
    fclose(rf);
    if (rsz_f < 8 || rsz_f > 4L*1024*1024 || memcmp(rimg, "RLS2", 4)) { free(rimg); free(recs); free(img); return -7; }
    int rn = rd16be(rimg+4);
    if (6 + (uint32_t)rn*4 > (uint32_t)rsz_f) { free(rimg); free(recs); free(img); return -7; }

    rsb_grain_t *G = malloc((size_t)rn*sizeof(rsb_grain_t));
    rsb_rule_t  *R = malloc((size_t)rn*sizeof(rsb_rule_t));
    if (!G || !R) { free(G); free(R); free(rimg); free(recs); free(img); return -8; }

    // собрать гранулы с правилами: k0=класс, base=k1, band=k2..k3, gain=k4
    // классы THOR (по длинам PCM pkg31/144): 1=холостая цепочка гранул (~0.14с,
    // банды 300→1100), 2=off-рабочий слой, 3=on-рабочий слой, 4=пул прострелов,
    // 5=старт-ваншот (~1.7с), 6=глушение-ваншот (~0.7с)
    typedef struct { uint16_t id, k0, lo, hi, base, gain, k5, k6, k9; const uint8_t *pcm; uint32_t plen; } gr_t;
    gr_t *tmp = malloc((size_t)rn*sizeof(gr_t)); if(!tmp){free(G);free(R);free(rimg);free(recs);free(img);return -9;}
    int gc = 0;
    // класс→тип события: 5→0(старт), 6→1(глушение), 4→2(прострелы).
    // 1/2/3 — рабочие гранулы: слой off(1) = cls1(idle)+cls2(сброс), on(0) = cls3.
    static const uint8_t cls_ev[7] = {0xFF,0xFF,0xFF,0xFF,2,0,1};
    rsb_event_t *EV = malloc((size_t)rn*sizeof(rsb_event_t));
    if (!EV) { free(tmp); free(G); free(R); free(rimg); free(recs); free(img); return -9; }
    int ecount = 0;
    for (int i = 0; i < rn; i++) {
        uint32_t o = rd32be(rimg+6+i*4), e = i+1<rn ? rd32be(rimg+6+(i+1)*4) : (uint32_t)rsz_f;
        if (o+4 > (uint32_t)rsz_f || e < o+4) continue;      // e<o — underflow защита
        const uint8_t *r = rimg+o;
        uint16_t rid = rd16be(r), np = rd16be(r+2);
        uint16_t kv[11] = {0};
        for (int j = 0; j < np; j++) {
            if (4+j*4+4 <= e-o) { uint16_t kid = rd16be(r+4+j*4); if (kid < 11) kv[kid] = rd16be(r+6+j*4); }
        }
        const smp_rec_t *rec = NULL;
        for (int j = 0; j < n; j++) if (recs[j].id == rid) { rec = &recs[j]; break; }
        if (!rec) continue;
        uint16_t cls = kv[0];
        if (cls < 7 && cls_ev[cls] != 0xFF) {              // класс-ивент → таблица событий
            EV[ecount].offset = (uint32_t)(rec->pcm - img);
            EV[ecount].length = rec->plen;
            EV[ecount].type   = cls_ev[cls];
            EV[ecount].flags  = 0;
            EV[ecount]._rsv   = rid;                     // rid — для PlaySguSound
            ecount++;
            continue;                                    // в гранулы НЕ попадает!
        }
        tmp[gc].id = rid; tmp[gc].k0 = cls;
        tmp[gc].base = kv[1] ? kv[1] : (kv[3] ? kv[3] : 800);
        tmp[gc].lo = kv[2]; tmp[gc].hi = kv[3];
        tmp[gc].gain = kv[4] ? kv[4] : 45;
        tmp[gc].k5 = kv[5]; tmp[gc].k6 = kv[6];              // fade-зоны окна
        tmp[gc].k9 = kv[9];                                  // длина кроссфейда лупа (сэмплы)
        tmp[gc].pcm = rec->pcm; tmp[gc].plen = rec->plen; gc++;
    }
    free(rimg); free(recs);
    /* ПРАВКА 2026-11-19 (5): молчаливая ловушка. Было: при gc==0 функция
     * возвращала 0, движок получал пустой банк и просто молчал — именно так
     * выглядел битый f_05 на SD (diag grains=0 evs=0 при чистом логе rc=0).
     * Зеркальная копия того же пакета парсится без пропусков (34/45), значит
     * формат тут не при чём. Теперь это ошибка с диагностикой в логе. */
    if (gc == 0) {
        ESP_LOGE(BTAG, "thor_load_bank: 0 grains! smp=%ld B recs=%d  rls=%ld B rules=%d evs=%d",
                 ssz, n, rsz_f, rn, ecount);
        free(tmp); free(G); free(R); free(EV); free(img);
        return -14;
    }
    ESP_LOGI(BTAG, "thor_load_bank: smp=%ld B recs=%d rls=%ld B rules=%d -> grains=%d evs=%d",
             ssz, n, rsz_f, rn, gc, ecount);
    // сортировка по baseRpm (простая вставками)
    for (int i = 1; i < gc; i++) { gr_t t = tmp[i]; int j=i-1;
        while (j>=0 && tmp[j].base > t.base) { tmp[j+1]=tmp[j]; j--; } tmp[j+1]=t; }

    /* Нормировка уровней записей (nrm) УБРАНА — выдумка: перекраивала
     * авторские отношения k4 между гранулами (тихие записи бустила до +42%,
     * громкие давила). Эталон render_pwr2 использует чистое k4/maxg. */
    uint16_t lo = 0xFFFF, hi = 0;
    for (int i = 0; i < gc; i++) {
        // слой: cls2=on-load(0), cls3/cls1/cls5/cls6=off(1) — принятая модель
        // CY_pwr2: cls2=нагрузка (k4 растёт с rpm), cls3=сброс (k4 падает).
        uint16_t lay = (uint16_t)(tmp[i].k0 == 2 ? 0 : 1);
        uint32_t off = (uint32_t)(tmp[i].pcm - img);
        G[i].offset = off; G[i].length = tmp[i].plen;
        G[i].baseRpm = tmp[i].base; G[i].loopStart = 0;
        G[i].loopEnd = (uint16_t)(tmp[i].plen/2 > 0xFFFF ? 0 : tmp[i].plen/2);  // 0 = вся запись
        /* ПРАВКА 2026-11-19: бит2 = «слой холостого, звучит всегда в своём окне».
         * Без него cls1 получал тот же слой 1, что и cls2, и при эксклюзивном
         * выборе класса (газ=0 -> слой 0) живой холостой заглушался бы вовсе. */
        /* ПРАВКА 2026-11-21: биты 3-5 = класс k0 — движку нужны cls2/3/5/6
         * для весов слоёв (cls2=нагрузка, cls3=сброс, cls5/6=подложки). */
        G[i].flags = (uint16_t)(1 | (lay << 1) | (tmp[i].k0 == 1 ? 4 : 0) | ((tmp[i].k0 & 7) << 3));
        // THOR-окно микса: k5/k6 = зоны кроссфейда ВНУТРИ своего бэнда (перекрытия
        // с соседями по цепочке): ramp-in [lo, lo+k5], flat [lo+k5, hi-k6], ramp-out [hi-k6, hi]
        G[i].fadeLo = tmp[i].lo;   G[i].fadeHi = tmp[i].hi;
        uint32_t fl = (uint32_t)tmp[i].lo + tmp[i].k5;
        G[i].flatLo = (uint16_t)(fl > tmp[i].hi ? tmp[i].hi : fl);
        int32_t  fh = (int32_t)tmp[i].hi - tmp[i].k6;
        G[i].flatHi = (uint16_t)(fh < 0 ? 0 : fh);
        /* вес = чистое k4 (как в эталоне: gnorm=k4/maxg — без нормировки
         * уровня записи; константа /45 поглощается калибровкой хедрума) */
        G[i].gainQ8 = q88f(tmp[i].gain / 45.f);
        G[i].xfade  = tmp[i].k9;                             // 0 -> дефолт в grain_sample
        if (tmp[i].lo && tmp[i].lo < lo) lo = tmp[i].lo;
        if (tmp[i].hi > hi) hi = tmp[i].hi;
        R[i].rpm = (tmp[i].hi ? (tmp[i].hi & 0x7FFF) : 0x7FFF) | (uint16_t)(lay << 15);
        R[i].grainIndex = (uint16_t)i;
        R[i].gain = q88f(tmp[i].gain/45.f); R[i].loadGain = q88f(0.25f);
    }
    // правила по rpm возрастанию — rsb_rule_lookup_l интерполирует по отсортированным
    for (int i = 1; i < gc; i++) { rsb_rule_t t = R[i]; int j=i-1;
        while (j>=0 && (R[j].rpm&0x7FFF) > (t.rpm&0x7FFF)) { R[j+1]=R[j]; j--; } R[j+1]=t; }
    free(tmp);
    // PCM копируем в выровненный буфер — офсеты гранул всегда чётные (int16)
    /* RLS may reference one SMP record more than once. Allocate for the
     * actual copies, not the source image size. Offsets are uint32_t. */
    size_t pcm_bytes = 0;
    for (int i = 0; i < gc; i++) {
        size_t len = G[i].length;
        if (G[i].offset > (size_t)ssz || len > (size_t)ssz - G[i].offset ||
            len > UINT32_MAX - pcm_bytes) {
            free(G); free(R); free(EV); free(img); return -10;
        }
        pcm_bytes += len; /* plen is already rounded down to even. */
    }
    for (int i = 0; i < ecount; i++) {
        size_t len = EV[i].length;
        if (EV[i].offset > (size_t)ssz || len > (size_t)ssz - EV[i].offset ||
            len > UINT32_MAX - pcm_bytes) {
            free(G); free(R); free(EV); free(img); return -10;
        }
        pcm_bytes += len;
    }
    int16_t *dec = malloc(pcm_bytes ? pcm_bytes : sizeof(int16_t));
    if (!dec) { free(G); free(R); free(EV); free(img); return -10; }
    uint32_t pos = 0;
    for (int i = 0; i < gc; i++) {
        uint32_t off = (uint32_t)(G[i].offset);
        uint32_t len = G[i].length;
        G[i].offset = pos;
        memcpy((uint8_t*)dec + pos, img + off, len);
        pos += (len + 1) & ~1u;
        G[i].length = len;
    }
    for (int i = 0; i < ecount; i++) {               // ивенты — следом за гранулами
        uint32_t off = EV[i].offset, len = EV[i].length;
        EV[i].offset = pos;
        memcpy((uint8_t*)dec + pos, img + off, len);
        pos += (len + 1) & ~1u;
    }
    free(img);
    b->sampleRate=22050; b->format=0; b->channels=1;
    b->rpmMin = lo==0xFFFF?300:lo; b->rpmMax = hi?hi:7000;
    b->grainCount=(uint16_t)gc; b->eventCount=(uint16_t)ecount; b->ruleCount=(uint16_t)gc;
    b->grains=G; b->events=EV; b->rules=R;
    b->_grains=G; b->_events=(void*)EV; b->_rules=R;
    b->payload=(const int16_t*)dec; b->payloadSamples=pos/2;
    b->_image=NULL; b->_decoded=dec;
    return 0;
}
