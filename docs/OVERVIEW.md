# Architecture overview

THOR-compatible active-exhaust controller on ESP32-P4: reads engine RPM over
CAN (passive sniff; OBD-2 fallback), plays granular engine-sound packs
(SMP2 samples + RLS2 rules) through I2S → amplifier → speaker.

```
firmware/main/
  engine.c/.h    retrigger grain engine: class selection, crossfade, level law, limiter
  bank.c/.h      pack loader: SMP2 samples + RLS2 rules -> grains/layers/events
  stand.c/.h     bench model: ignition / throttle / gear (for reproducible test drags)
  i2s_out.c/.h   I2S output + power-amp gate (silence -> amp off)
  can_rpm.c/.h   RPM from CAN (TWAI TX=GPIO23 RX=GPIO24); OBD-2 fallback
  can_rules*.c   per-model CAN frame -> RPM decode rules
  net_ws.c/.h    HTTP + WebSocket LAN console (/diag /packs /drag /dump.wav /ctrl)
  net_ctrl.c/.h  TCP :3333 control
  ota.c/.h       POST /ota firmware update (token-gated; placeholder token here)
  main.c         tasks / glue
```

## Control surface (besides CAN)
- `http://<ip>/` — LAN console. WebSocket `/ctrl` binary frames `[cmd][payload]`:
  `0x01` volume, `0x02` preset, `0x03` event, `0x05` rpm override (u16),
  `0x06` bench (ign/thr/gear), `0x07` idle, `0x0A` debug logs.
- `GET /diag`, `GET /packs` (grain/event fingerprint), `GET /drag?sec=N`
  (reproducible scripted drag), `GET /dump.wav` (capture).

## Sound packs
3 files on SD per pack: SMP2 samples (`f_04…`), RLS2 rules (`f_05…`),
RLM2 preset params (`f_06…`). See `SOUND_FORMATS.md`.

## Build
ESP-IDF v6.1, `esp32p4`. `idf.py build`; flash app at `0x20000`. The BLE
pairing/crypto layer is not included in this public repo (see `AGENTS.md`).
