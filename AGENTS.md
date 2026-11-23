# AGENTS.md — context for AI agents

> This repository is written to be read by AI coding/DSP agents as much as by
> humans. If you are an agent: read this file top to bottom first. It tells you
> what the project is, what model it uses now, what is still imperfect, and
> exactly where to look.

## What this is

An **independent, open** firmware project that reproduces a naturalistic
car-engine **active-exhaust sound** on an **ESP32-P4** board. It reads engine
RPM over CAN (passive sniff; OBD-2 fallback) and plays back recorded
engine-sound packs through I2S → amplifier → speaker.

The playback engine and the pack formats were **reverse-engineered for
interoperability** from a commercially sold active-exhaust unit (referred to
here generically; this project is **not affiliated with, endorsed by, or
connected to** any such vendor, and uses none of its branding). The goal is a
clean-room-style, data-driven re-implementation of the *audio model*, for the
author's private use on his own cars — **not** a product.

## Current model (IMPORTANT — read this before the git history)

The engine no longer synthesizes sound granularly. It now plays the
**original recorded loops faithfully**, the same way the reference unit and
RevHeadz do:

- Each pack is a set of recorded loops tagged by **class** and **base RPM**.
- Two adjacent-RPM loops of a layer are **pitch-shifted** by `rpm/baseRpm` and
  **crossfaded by RPM** (a ladder), so pitch tracks revs continuously.
- An **on-load** layer (class 2) and an **off/overrun** layer (classes 1+3)
  are **blended by throttle** (hysteresis so it doesn't chatter at the edge).
- **One-shot events** — startup (5), shutoff (6), backfire/pops (4) — are
  mixed in from a 4-voice pool.
- No oscillators, no noise, no granular re-synthesis. Everything audible is
  PCM from the pack.

This is selected in `engine.c` by `e->multiMode = 0` (the "two-pair ladder"
path in `engine_render`). The old additive/granular path (`multiMode = 1`) is
still compiled but **not used**; ignore it unless you are explicitly comparing.

## Measured state (from the live board, `tools/audio_audit.py`)

The switch to faithful playback resolved the dominant defects:

| metric | granular era | faithful (now) | target |
|---|---|---|---|
| under-throttle tremolo (4-45 Hz) | ~28% | **~0.7%** | <20% |
| clipped samples | ~13% | **~0.1%** | 0 |
| spectrogram | granular "wash" | real harmonics tracking RPM | — |

Anti-clip is a soft knee at 30000 (`softclip` in `engine.c`); below the knee
the signal is untouched (no global trim, full dynamics).

## What is still being polished (by ear, on real recordings)

These are subtle and are where fresh DSP eyes help most:

1. **Idle (class 1) reconstruction.** Idle is slices of one short take. Looping
   a single slice beat at ~7 Hz ("throb") and the slice seams ticked. Current
   fix (`engine.c`, `offIdle` path + `idleSeq`): play the idle slices **in
   order as one continuous take**, and at each slice boundary interpolate into
   the **first sample of the next slice** (slices are contiguous in the
   recording) so the seam is continuous. Open question: is an ordered-slice
   playthrough the right reconstruction, or is there a cleaner way to build a
   seamless idle bed from such slices?
2. **Loop-seam clicks ("vinyl tick").** Addressed three ways, all in
   `engine.c`/`bank.c`: a **raised-cosine** crossfade over the loop seam, a
   **minimum crossfade** length (~27 ms), and a **zero-crossing snap** of the
   loop start/end to rising zero-crossings in `bank.c` (so head and tail match
   level and slope). Residual ticks are now micro-level.
3. **Band-crossing transition clicks (latest).** When RPM moves a voice from
   one loop to an adjacent one, the voice would read a **new recording at the
   old phase** → a click at throttle onset, at end-of-decay (rev → ~0), and a
   micro-click on each gear shift. Current fix (`engine.c`, both on- and
   off-layer): when a grain "slides" between the two voices on a band change,
   **carry its phase** across so playback position is continuous. This is the
   most recent change and the one under active listening tests.
4. **Loudness** sits a touch hot vs the reference demo; a by-ear call, not
   urgent, and deliberately left until monitored on the real speaker.

## How the audio engine works (where to look)

All in `firmware/main/`:

- **`engine.c` / `engine.h`** — the core. The faithful path is the `else`
  branch of the per-sample loop in `engine_render` (two on-voices `v0/v1`, two
  off-voices `w0/w1`, RPM crossfade, throttle layer blend, `offIdle` idle bed,
  event pool, `softclip`). **Start here.** The band-crossing phase-carry fix is
  right after each `rsb_grain_pair_l(...)` call.
- **`bank.c` / `bank.h`** — pack loader. Builds the bank from SMP2 samples +
  RLS2 rules (grain classes → layers/events, RPM bands, gains, loop crossfade),
  and does the zero-crossing loop-point snap.
- **`stand.c` / `stand.h`** — bench model (ignition, throttle, gear) that drives
  the reproducible "drag" run.
- **`i2s_out.c`** — I2S output + power-amp gate.
- **`net_ws.c` / `net_ctrl.c`** — LAN control: HTTP `/diag`, `/packs`,
  `/drag?sec=N` (and `?mode=rev`, `?bf=0`), `/dump.wav`, `/vol`, and a
  WebSocket `/ctrl` (binary opcodes; `0x08` = cabin reverb 0..255).
- `can_*` — RPM from CAN. `main.c` — glue / tasks, backfire scheduling,
  redline cut. `ota.c` — OTA.

Grain classes: 1=idle, 2=on-load layer, 3=off/overrun layer, 4=backfire pool,
5=startup one-shot, 6=shutoff one-shot. See `docs/SOUND_FORMATS.md`.

## How to reproduce / test the sound

The board is the author's private bench (not reachable from the internet), so
**agents review the code and reason about the model**; the author runs the
capture loop and posts results. The loop (reproducible):

1. `board_cap.py OUT.wav 20` → ignition on, set master gain, trigger a 20 s
   scripted drag (idle → pulls with gear shifts → release to ~0), pull
   `/dump.wav`. `?mode=rev` gives idle + throttle blips instead.
2. `tools/audio_audit.py OUT.wav` → clip count, RMS profile, tremolo depth
   (4-45 Hz), loop auto-correlation, octave balance, and a dependency-free
   spectrogram PNG.
3. Compare against the targets above.

`audio_audit.py` needs only `numpy`. With a WAV of the board output you can run
the audit directly without the board.

## Build / flash

ESP-IDF v6.1, target `esp32p4`. `idf.py build flash -p <PORT>`. Flash offsets:
`0x20000` app (`rev_sound.bin`), `0x2000` bootloader, `0x8000` partition table,
`0x10000` otadata.

> **Note:** the BLE pairing/crypto layer (`thor_ble.c`, `thor_proto.*`) is
> **intentionally not published here** — publishing a commercial device's
> key-derivation would be a circumvention concern and is unrelated to the sound
> problem. The public tree therefore will **not link as-is**; it is provided
> for **sound-model review**, not a turn-key build. `main.c` and `net_ws.c`
> reference those symbols/headers, but everything needed to understand and fix
> the audio path is present.

## Concrete asks for reviewers

1. **Band-crossing transitions (fix #3 above):** is carrying the grain phase on
   a band swap the right approach, or should adjacent loops be phase-aligned at
   load time (e.g. aligned to a common crank-cycle landmark) so a swap never
   needs a phase jump at all? Any residual click should be at throttle onset,
   end-of-decay, and gear shifts.
2. **Idle bed (fix #1):** best way to build a seamless idle from ordered slices
   of one take — ordered playthrough (current), one pre-stitched long loop, or
   a crossfaded ring?
3. **Loop seams (fix #2):** is the raised-cosine + min-crossfade + ZC-snap
   combination sound, or is a constant-power (equal-power) crossfade preferable
   for the overrun loops?
4. General review of the ladder-crossfade + throttle-blend model vs how the
   reference unit / RevHeadz layer their recordings.

## How to contribute

Open a PR against `firmware/main/`, or file an issue describing the change and
its expected effect on the measured metrics or the named by-ear defects. Keep
changes data-driven where possible (the engine is meant to be tuned by pack
data, not code special-cases). State which issue your change targets so the
author can verify with a before/after board capture.

See `NOTICE.md` for scope and legal notes.
