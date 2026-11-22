# AGENTS.md — context for AI agents

> This repository is written to be read by AI coding/DSP agents as much as by
> humans. If you are an agent: read this file top to bottom first.

## What this is

Open **ESP32-P4** firmware that plays a realistic car **active-exhaust engine
sound**, driven by engine RPM over CAN (passive sniff; OBD-2 fallback). It is a
faithful, data-driven re-implementation of how a commercial active-exhaust unit
("THOR") and the RevHeadz engine-sound app actually work. Independent, for
interoperability / personal use; **not affiliated with any vendor; no vendor
branding, app code, or copyrighted recordings are included here.**

## Design principle: play the ORIGINAL recordings, don't synthesize

**This is the most important thing.** The engine does NOT synthesize sound. It
plays the original recorded loops and crossfades them by RPM and throttle —
exactly the model used by both THOR (SMP2 samples + RLS2 rules) and RevHeadz
(FMOD banks with onBands/offBands). An earlier granular-synthesis experiment
was removed because it sounded artificial; the faithful crossfade player is the
only path now (`engine.c`, `multiMode = 0`).

### The model (identical in THOR and RevHeadz)
- **Two ladders of recorded loops**, each loop tagged with a nominal RPM:
  - **on-load** (under throttle): class `k0=2`; gain rises with RPM.
  - **off-load** (overrun / closed throttle): class `k0=3` (+ idle class `k0=1`).
- At runtime, for the current RPM pick the two adjacent loops in each ladder,
  **pitch** each to `rpm / nominalRpm`, **crossfade** between them by RPM
  position, and **blend on vs off by throttle**.
- **One-shots:** startup (`k0=5`), shutoff (`k0=6`), backfire pool (`k0=4`),
  up/downshift — triggered as events, played over the ladders.
- RPM is smoothed; idle has a low-RPM loop.

## Where to look (all in `firmware/main/`)

- **`engine.c`** — the player. The faithful path is the `if (!e->multiMode)`
  block (grain-pair selection, pitch, RPM crossfade, throttle blend) plus the
  per-sample mix in the `else` branch (`grain_sample` loops with seam
  crossfade). One-shot events (backfire/startup/shutoff) are appended after.
  **Start here.**
- **`bank.c` / `bank.h`** — loads a THOR pack (SMP2 `f_04` + RLS2 `f_05`) into
  grains with: `baseRpm` (nominal), `loopStart/loopEnd`, `gainQ8` (k4), `xfade`
  (loop seam, k9), `flags` (bit1 layer 0=on/1=off, bits3-5 class). Helpers
  `rsb_grain_pair_l` (adjacent loops of a layer), `rsb_rule_lookup_l`,
  `rsb_grain_env`.
- **`stand.c`** — bench model (ignition/throttle/gear) for reproducible drags.
- **`i2s_out.c`**, **`net_ws.c`** — I2S output; LAN control
  (`/diag /packs /drag?sec=N /dump.wav`, WebSocket `/ctrl`).
- `docs/SOUND_FORMATS.md` — SMP2/RLS2/RLM2 details.

## Current state (measured on the live board, pkg31)

Faithful player, captured via `/drag` + analyzed with `tools/audio_audit.py`:

| metric | granular synth (old) | faithful player (now) |
|---|---|---|
| load-layer tremolo (4–45 Hz) | ~28% | **0.7%** |
| clipped samples | ~13% | 0.14% (negligible) |
| load spectrum | broadband wash | clean harmonics that track RPM |

The spectrogram now shows distinct harmonic lines sweeping with RPM across the
gears — real loops being pitched/crossfaded, not synthesis.

**Open item:** **idle tremolo ~30%** (4–45 Hz). The idle loops (class 1) are
slices of one recording and beat against each other at idle. Everything above
idle is clean. This is the main thing left to improve.

## How to reproduce / test

The board is the author's private bench (not internet-reachable), so agents
**review code and reason about the model**; the author runs captures. Loop:
1. `tools/board_cap.py OUT.wav 20` → scripted drag → `/dump.wav`.
2. `tools/audio_audit.py OUT.wav` → metrics + spectrogram PNG (`audit/`).
`audio_audit.py` needs only `numpy`; run it on any board WAV without hardware.

## Build / flash

ESP-IDF v6.1, target `esp32p4`, `idf.py build`. App at flash `0x20000`.
The BLE pairing/crypto layer (`thor_ble.c`, `thor_proto.*`) is intentionally
**not published** (circumvention concern, unrelated to sound); the public tree
is for sound-model review, not a turn-key build.

## Concrete asks for reviewers

1. **Idle tremolo ~30%:** the class-1 idle loops beat at idle. Best way to play
   idle from slices of one take without the beat — a single seamless idle loop,
   a leader-with-hysteresis, or phase-aligned crossfade? (See `rsb_grain_env`
   and the idle handling in `engine.c`.)
2. **Idle wobble / RPM smoothing:** RevHeadz `cfg/*.txt` uses a spring-damper
   on RPM and a ~7 Hz idle wobble. Worth adding to the ESP32 player for realism?
3. General review of the ladder-crossfade player vs how a real engine recording
   set should be sequenced (loop seams, pitch range per loop, on/off blend).

## Contributing

PRs against `firmware/main/`, or issues naming which metric you target
(idle tremolo / loudness / crossfade). Keep it data-driven (the engine is tuned
by pack data, not code special-cases). See `NOTICE.md`.
