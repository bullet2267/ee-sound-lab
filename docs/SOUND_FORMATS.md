# Sound-pack formats (SMP2 / RLS2 / RLM2)

A sound pack = 3 files on SD:
- `f_04<pkg><ver>` — SMP2 PCM samples
- `f_05...`        — RLS2 playback rules (grains / layers / events)
- `f_06...`        — RLM2 preset params (UI)

PCM is **s16le, mono, 22050 Hz**.

## .sample — SMP2
```
SMP2 | u16be count | u32be offsets[count] | records...
record: u16be id | u32be pcmLen | s16le PCM[pcmLen]
```
Each record = one grain/loop; no cue tables inside.

## .smprl — RLS2 (playback rules), 48-byte records
`u16be rule_id` (= sample id) + 11 pairs `(key u16be, val u16be)`:
- `k0` — class: 1=idle, 2=load layer, 3=release/off layer, 4=backfire pool,
  5=startup one-shot, 6=shutoff one-shot
- `k1` — nominal RPM of the grain (pitch = rpm / k1)
- `k2`, `k3` — RPM band (e.g. 300–1100, 1101–1400 …)
- `k4` — gain (falls with RPM: 45→29)
- `k5`, `k6` — envelope params
- `k9`, `k10` — fade in/out crossfade (ms)

## .pkgrl — RLM2 (preset params)
Records: `u16be id | u16be pairCount | pairs (key u16be, val u16be)`.
Holds global RPM thresholds, fade timings, redline, idle, and **zonal
backfire blocks** `0x112/0x212/0x312` (lo/hi rpm + min/max pops per burst).
The engine is data-driven: THOR does not special-case code per pack — all
tuning lives in the RLM2/RLS2 data. This project follows the same architecture.

Server packs use `type1_v5.sample` + `type2_v5.smprl` (+ `type3_v5.pkgrl`);
PWR / ns_1 loop packs come as `.rsb` (RSB1 container).
