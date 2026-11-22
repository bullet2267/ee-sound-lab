# ee-sound-lab — naturalistic active-exhaust engine-sound synth (ESP32-P4)

Open firmware that synthesizes a realistic car-engine **active-exhaust sound**
on an **ESP32-P4**, driven by engine RPM over CAN, using granular playback of
engine-sound packs. An independent, data-driven re-implementation of the audio
model of a commercial active-exhaust unit, built for interoperability.
**Not affiliated with any such vendor; no vendor branding is used.**

**AI agents: read [`AGENTS.md`](AGENTS.md) first.** It is the entry point.

## The ask

Make the synthesized sound **natural** (match real recordings, not "toy-like").
The hard-clipping defect is fixed and verified; the open issue is a ~28%
amplitude tremolo on the load layer. Full details, metrics, and where to look:
[`AGENTS.md`](AGENTS.md).

## Layout

```
firmware/main/     ESP32-P4 audio engine (engine.c = core), CAN→engine→I2S, LAN control
firmware/          ESP-IDF v6.1 project (CMake, partitions, sdkconfig.defaults)
tools/             audio_audit.py (metrics + dependency-free spectrogram PNG),
                   board_cap.py (capture a scripted drag from the live board)
docs/              SOUND_FORMATS.md, DEV_LOG.md, OVERVIEW.md
```

## Quick start (sound review, no hardware)

```
pip install numpy
python tools/audio_audit.py some_capture.wav   # prints metrics, writes audit/*.png
```

The spectrogram PNG is the fastest way to "see" the defect. See `AGENTS.md`
for the full capture/analyze loop and the review questions.

## Contributing

PRs against `firmware/main/`, or issues describing the change and which metric
it targets (clip / tremolo / octave balance / loudness). See `NOTICE.md`.
