# AGENTS.md — context for AI agents

> This repository is written to be read by AI coding/DSP agents as much as by
> humans. If you are an agent: read this file top to bottom first. It tells you
> what the project is, what is broken, and exactly where to look.

## What this is

An **independent, open** firmware project that synthesizes a naturalistic
car-engine **active-exhaust sound** on an **ESP32-P4** board. It reads engine
RPM over CAN (passive sniff; OBD-2 fallback), and plays back granular
engine-sound packs through I2S → amplifier → speaker.

The playback engine and the pack formats were **reverse-engineered for
interoperability** from a commercially sold active-exhaust unit (referred to
here generically; this project is **not affiliated with, endorsed by, or
connected to** any such vendor, and uses none of its branding). The goal is a
clean-room-style, data-driven re-implementation of the *audio model*, not a
copy of any proprietary binary.

## The one problem we want help with

**Make the synthesized sound natural** — match real engine recordings instead
of sounding "synthetic / toy-like / raspy."

The dominant defect (**hard clipping**) is already **fixed and verified** (see
below). The remaining issue is subtler and is where fresh DSP eyes would help.

## Current state (measured, not guessed)

Captured from the live board, analyzed with `tools/audio_audit.py`:

| metric | before fix | now (fixed) | target |
|---|---|---|---|
| clipped samples (>=32000) | ~57,500 (13%) | **0** | 0 |
| peak | 32767 (flat-topped) | 31084 | <32767 |
| top RMS under load | ~24,500 | ~13,500 | ~10,300 (ref demo) |
| load-layer tremolo (4-45 Hz) | - | **~28%** | **<20%** |

- **Clipping (SOLVED):** output level law had been calibrated while throttle
  was broken (`thr=0`), so once throttle worked the signal ran ~2.3x too hot
  and hard-clipped, producing a broadband hash that masked the harmonics (a
  uniform bright wash across the whole spectrogram). Fix: `OUT_TRIM 0.42` +
  a soft limiter (knee at 26k) in `firmware/main/engine.c`. After the fix the
  spectrogram shows energy concentrated in the low band with a natural
  roll-off, and distinct idle harmonics.
- **Load-layer tremolo ~28% (OPEN):** amplitude modulation at 4-45 Hz on the
  load layer, ~39.5 Hz, above the <20% target. Likely the grain-retrigger beat
  (free-phase crossfade between overlapping grains). This is the main suspect
  for the residual "roughness." **This is the best place to help.**
- **Loudness still ~30% hot** vs the reference demo (top-RMS 13.5k vs 10.3k);
  trivially a one-line `OUT_TRIM` change, but it's a by-ear call - not urgent.

## How the audio engine works (where to look)

All in `firmware/main/`:

- **`engine.c` / `engine.h`** - the core. Retrigger/ping-pong grain playback
  (`idx`/`idx2`), exclusive class selection with hysteresis, leader-by-class,
  `clsMul` level-matching at transitions, RMS normalization, the output level
  law, and the final `softlim(mix * OUT_TRIM)`. **Start here.**
- **`bank.c` / `bank.h`** - pack loader. Builds the bank from SMP2 samples +
  RLS2 rules (grain classes -> layers/events, RPM bands, gains, crossfades).
- **`stand.c` / `stand.h`** - bench/dyno model (ignition, throttle, gear) used
  by the test harness to drive a reproducible "drag" run.
- **`i2s_out.c`** - I2S output + power-amp gate.
- **`net_ws.c` / `net_ctrl.c`** - LAN control: HTTP `/diag`, `/packs`,
  `/drag?sec=N`, `/dump.wav`, and a WebSocket `/ctrl` (binary opcodes).
- `can_*` - RPM from CAN. `main.c` - glue / tasks. `ota.c` - OTA.

Grain classes: 1=idle, 2=load layer, 3=release/off layer, 4=backfire pool,
5=startup one-shot, 6=shutoff one-shot. See `docs/SOUND_FORMATS.md`.

## How to reproduce / test the sound

The board is the author's private bench (not reachable from the internet), so
**agents review the code and reason about the model**; the author runs the
capture loop and posts results. The loop (reproducible):

1. `tools/board_cap.py OUT.wav 20` -> drives ignition + throttle, triggers a
   20 s scripted drag, pulls `/dump.wav`.
2. `tools/audio_audit.py OUT.wav` -> prints clip count, RMS profile, tremolo
   depth (4-45 Hz), loop auto-correlation, octave balance, and renders a
   spectrogram PNG to `audit/`.
3. Compare the spectrogram + numbers against the reference demo targets above.

`audio_audit.py` needs only `numpy` (PNG writer is dependency-free). If you
have a WAV of the board output, you can run the audit directly without the
board.

## Build / flash

ESP-IDF v6.1, target `esp32p4`. Standard `idf.py build`. Flash offsets:
`0x20000` app (`rev_sound.bin`), `0x2000` bootloader, `0x8000` partition table,
`0x10000` otadata. Partition table and `sdkconfig.defaults` are in `firmware/`.

> **Note:** the BLE pairing/crypto layer (`thor_ble.c`, `thor_proto.*`) is
> **intentionally not published here** - publishing a commercial device's
> key-derivation would be a circumvention concern and is unrelated to the sound
> problem. The public tree therefore will **not link as-is**; it is provided
> for **sound-model review**, not a turn-key build. Everything needed to
> understand and fix the audio path is present.

## Concrete asks for reviewers

1. **Kill the 28% load-layer tremolo** without flattening the firing pulses a
   real engine has. Is the free-phase grain crossfade in `engine.c` beating?
   Would phase-locking grain retrigger to the crank period (instead of free
   phase) reduce the 39.5 Hz modulation while keeping the ~150-200 Hz firing
   texture?
2. **Review the level law** (`law = (0.5+0.5*thr)*(rpm/800)^1.2*(1+1.8*(1-t25))
   *masterGain*0.5` in `engine.c`): is the `(rpm/800)^1.2` curve sensible, or
   should it be shaped differently per class?
3. **Idle fullness:** idle is built from slices of one recording (cls1). Is the
   leader-by-class + hysteresis approach (`idleLead`) the right way to avoid the
   "bu-bu-bu" pulse-train, or is there a cleaner reconstruction?
4. General DSP review of the retrigger model vs a real engine's spectrum.

## How to contribute

Open a PR against `firmware/main/`, or file an issue describing the change and
the expected effect on the measured metrics above. Keep changes data-driven
where possible (the engine is meant to be tuned by pack data, not code
special-cases). State which metric your change targets (clip / tremolo / octave
balance / loudness) so the author can verify with a before/after capture.

See `NOTICE.md` for scope and legal notes.
