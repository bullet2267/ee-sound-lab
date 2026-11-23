# Dev log — audio engine evolution

Condensed history of how the sound model got to its current state. Useful for
agents to avoid re-treading dead ends.

## Loops → retrigger model
Early versions looped whole recordings. Proven wrong by the pack structure:
grains are slices of a continuous recording (e.g. idle = slices rid2–5 of one
~556 ms take; a backfire pack = a pool of 4×0.13 s one-shots). Switched to a
**retrigger** model: a voice plays a grain to its end; on the tail (last `k9`
crossfade samples) the next random grain from the current RPM window is mixed
in — ping-pong crossfade `idx/idx2`, `phase/phase2`.

Key findings along the way:
- **cls2/cls3 are not on/off layers** but parallel ladders crossfaded by RPM
  (cls2 load gain rises 44→95, cls3 release gain falls 44→12). Layer-gating was
  removed from the retrigger path.
- **4 voices, not 16** — too many voices smear the firing impulses (blur the
  "bark"). Reference idle modulation depth ~0.63; ours ~0.68.
- **Free grain phases** (ПРАВКА 2026-11-20): cycle-locking (`pos = cyc*L`) gave
  every grain the same period, so their internal flash impulses coalesced into
  one thud per cycle = "boo-boo-boo." Free phase (grain `revs` 1.98–2.05, not
  exactly one cycle) lets impulses drift and interleave = continuous growl.
- **Idle leader + hysteresis** (2026-11-21): cls1 slices are pieces of one idle
  take; playing them simultaneously makes a pulse-train on itself. Fixed with
  `idleLead = argmax-env` + 1.35 hysteresis and a shared crank phase so slice
  changes are seamless.
- **Data-driven backfires:** RLM2 zonal blocks `0x112/0x212/0x312` give
  (lo/hi rpm, min/max pops). Lower RPM = denser crackle. The engine reads these
  from pack data rather than hard-coding thresholds.

## 2026-11-22 — clipping fix (verified on board)
The output level law had been calibrated while throttle was broken (`thr=0`).
Once throttle worked, the law ran ~2.3× hot: a live capture showed top-RMS
~24.5k and **13% of samples hard-clipped** (flat tops at 32767). On the
spectrogram this is a broadband hash flooding the harmonics = the "rasp / toy"
sound. Reference demo top-RMS ~10.3k. Fix: `OUT_TRIM = 0.42` (≈10.3/24.5) plus
a soft limiter (knee 26k) that catches rare transients without bending the body
of the signal. After: **0 clipped samples**, peak 31084, energy concentrated
low with a natural roll-off and distinct idle harmonics.

## Open
- **Load-layer tremolo ~28%** (4–45 Hz, ~39.5 Hz). Above the <20% target. Prime
  suspect for residual roughness. Candidate: the free-phase grain crossfade is
  beating — would crank-phase-locked retrigger reduce it while keeping the
  firing texture? See `AGENTS.md` ask #1.
- Loudness ~30% hot vs reference (one-line `OUT_TRIM`, by-ear).

## 2026-11-22 (2) — dropped synthesis, play the originals
Reverted the granular-synth direction entirely. The clipping "fix" (OUT_TRIM +
limiter) was removed because it flattened transients and killed the character
(backfire attack, throttle-peak overlay, exhaust brightness). More importantly,
the whole granular-retrigger model was the wrong approach: THOR and RevHeadz
both just play recorded loops crossfaded by RPM/throttle. Switched the engine to
the faithful ladder-crossfade path (`multiMode = 0`). Result on the board:
load-layer tremolo 28% → 0.7%, clipping 13% → 0.14%, and the spectrogram shows
real harmonics tracking RPM instead of a granular wash. Remaining: idle tremolo
~30% (class-1 idle loops beating).

## 2026-11-22 (3) — faithful-path refinements: seams, idle, transitions
With the faithful ladder-crossfade path now the only one used, the remaining
work is micro-level artifacts, chased by ear on real recordings:

- **Loop-seam "vinyl tick"** (`engine.c` `grain_at`/`grain_sample`, `bank.c`):
  the loop seam now uses a **raised-cosine** crossfade (`w = 0.5 - 0.5·cos(π·wl)`),
  a **minimum crossfade** of ~600 samples (~27 ms), and `bank.c` snaps each
  working loop's start/end to **rising zero-crossings** so head and tail match
  level and slope. Idle/short event clips are skipped by the snap. Result by
  ear: the tick dropped to barely audible.
- **Idle bed** (`engine.c` `offIdle` + `idleSeq`, `engine.h`): instead of
  looping one short slice (7 Hz throb + seam ticks), collect the class-1 slices
  in order and play them as **one continuous take**; at each slice boundary
  interpolate into the **first sample of the next slice** (slices are contiguous
  in the recording). By ear: throb gone on quiet idle, clean.
- **Louder events** (`engine.c`): backfire/pop/cut `engine_fire` gains raised to
  2.5 (pops were too weak to hear).
- **Cabin reverb** (`engine.c`, WS cmd `0x08`): optional Schroeder reverb
  (3 combs + allpass, 25–37 ms) modelling the enclosure resonance. Default OFF
  (dry is cleaner); `0x08 <0..255>` sets wet.
- **Band-crossing phase-carry** (latest, `engine.c`, on- and off-layer): on a
  band change a grain "slides" from one voice to the other; previously the
  receiving voice read the **new** recording from the **old** phase = a click at
  throttle onset, at end-of-decay (rev→0), and a micro-click per gear shift. Now
  the grain's phase is carried across the swap so playback position stays
  continuous. Under active listening tests.

A reverted dead end: 4-point Catmull-Rom interpolation (`cubic4`) was tried in
place of linear to fight pitch-up aliasing, but by ear it sounded "dirtier";
reverted to linear (helper left defined but unused).

## Open (faithful path)
- Confirm the band-crossing phase-carry removes the onset/decay/gear-shift
  ticks without introducing a pitch glitch on the swap.
- Idle reconstruction: ordered-slice playthrough vs one pre-stitched loop vs
  equal-power ring — which is cleanest?
- Loudness slightly hot vs reference (by-ear, deferred to real-speaker monitor).
