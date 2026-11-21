# eurorack-mi-sgnir

A slim, self-contained source tree for building custom firmware for the Mutable Instruments
**Rings** Eurorack module — plus (work in progress) a custom alternative mode. Folder
names inside the tree stay as upstream.

This is **not** a GitHub fork of the `pichenettes/eurorack` monorepo: it is a snapshot of only
what the Rings firmware needs, with the vendored dependencies included, so that
`git clone && make` works without submodules or the rest of the monorepo.

## Base (pinned upstream commits)

| Path | Source | Commit |
|---|---|---|
| `rings/` | [pichenettes/eurorack](https://github.com/pichenettes/eurorack) `master` | `f4cfa7697095a28d292bff758beb51deb49b5a86` (rings tip `c5f910fb25`) |
| `stmlib/` | [pichenettes/stmlib](https://github.com/pichenettes/stmlib) | `e3bd7c9cc00e4364166f9905c0509b6ffd0535ec` |
| `stm_audio_bootloader/` | [pichenettes/stm-audio-bootloader](https://github.com/pichenettes/stm-audio-bootloader) | `1ec7d6374e2d305cb710170eb80de9b66e795792` |

The `stmlib` and `stm_audio_bootloader` commits are exactly the submodule pins of the monorepo
commit above. All upstream sources are unmodified in the base commit of this repo; every change
on top is its own commit.

## Download and install

Ready-made firmware is on the [releases page](../../releases): `sgnir-<version>.wav` is installed
by playing it into the IN jack of Rings — step-by-step guide in
[docs/FLASHING.md](docs/FLASHING.md). Releases are built by GitHub Actions
(`.github/workflows/release.yml`) when a version tag `v*` is pushed.

## Building

Tested with Arm GNU Toolchain **12.2.Rel1** on macOS. The upstream build chain targets
gcc 4.8-era defaults; with a modern GCC, override the C++ dialect (no source changes needed):

```bash
TOOLCHAIN_PATH=/path/to/arm-gnu-toolchain/arm-none-eabi/ \
  make -f rings/makefile \
  CPPFLAGS="-fno-exceptions -fno-rtti -std=gnu++11 -Wno-deprecated" \
  bin
```

Result: `build/rings/rings.bin` (application image, link address `0x08008000`,
`stmlib/linker_scripts/stm32f4xx_flash_application_large.ld`). Reference build from the pristine
base tree: 84 016 bytes, SHA-256
`b481538e5a34d41668fa47efa2437e7de5d6f9236b38c13f30ed2f20823806ca` (not bit-reproducibility
tested across machines).

After a fresh clone, `make` may try to regenerate `rings/resources.cc` with the Python 2 resource
compiler (the checked-in file looks older than `rings/resources/*.py`) and fail with an `xrange`
error. The checked-in resources are correct: run `touch rings/resources.cc rings/resources.h` once
and build again.

The `size` make target pipes through `figlet`/`cowsay` — install them or ignore the error.

The firmware WAV for the audio bootloader is made with the Python 3 port of the original
encoder in `tools/` (needs numpy):

```bash
python3 tools/qpsk_encoder.py -t stm32f4 -s 48000 -b 12000 -c 6000 -p 256 \
    -o build/rings/rings.wav build/rings/rings.bin
python3 tools/qpsk_decoder.py build/rings/rings.wav -o /tmp/roundtrip.bin  # optional check
```

### Host tests

The Schliere DSP has host tests (transfer functions from impulse responses, checked against the
textbook allpass and the Smear manual, plus the Rings additions):

```bash
c++ -std=c++11 -O2 -DTEST -I. -o schliere_test \
    rings/test/schliere_test.cc rings/dsp/schliere.cc && ./schliere_test
```

An equivalence test against the original OWL patch lives in the workbench repo
(`tests/schliere/`).

### Profiling

`FillBuffer()` records its peak cycle count (DWT cycle counter) in the global
`fill_buffer_cycles_peak`; read it over SWD. Budget per block: 84 000 cycles (24 samples at
48 kHz, 168 MHz). Without a debugger, `rings/diagnostics.h` has two switches: a CPU meter on
LED 1 in the Schliere bank, and a benchmark mode that blinks the load in percent (see the
comments there).

Measured on the module (2026-11-21, benchmark builds, peaks, knobs at noon, no input):
Schliere S 62 % at 48 kHz, 71 % with the 96 kHz feedback path (v0.1.0: 81 % / 89 %), L 82 %;
base load without the DSP 13 %; grounded models 71–84 % depending on the model (stock code).
A running excitation (ping, the longest flavor) adds 1–3 points: L 85 %, S 63 / 72 %.
Each extra stage of the chain costs about 0.5 % of the block budget. The flash prefetch (enabled at boot, not
set by `SystemInit`) saved about 10 %; running Schliere from RAM was 18 % slower and is not
used. The Schliere chain is unrolled straight-line code and limited by instruction fetch from
flash: a stage with one fused multiply-add (4 bytes shorter) measured faster than the
cycle-optimal one without. Keep the stage states within the first 1020 bytes of the
`Schliere` object (the `vldr`/`vstr` offset range): placed further back, GCC reloaded an
address per stage and S cost 5 points more.

## Differences from the stock firmware

Not yet tested on hardware.

### Frequency locking

Hold **both buttons for more than 0.2 s**: both LEDs blink **red** (frequency will be locked)
or **green** (frequency will be unlocked). The change happens when you release the buttons.
While locked, V/OCT, FM input and FM attenuverter work as usual, and the FREQUENCY knob becomes
an **octave switch**: it keeps its usual scale (one octave per fifth of its travel) but snaps to
whole octaves of the locked pitch. Locking never changes the pitch; turn the knob about a fifth
of its travel to go up or down one octave. The lock and the locked pitch are saved and survive
a power cycle. Idea taken from the [lylepmills alt firmware](https://github.com/lylepmills/eurorack),
whose octave switch is absolute (7 fixed steps) instead.

### Calibration: hold both buttons for 6 seconds

The stock firmware enters V/OCT calibration on **any** press of both buttons. Here, calibration
starts only after holding **both buttons for 6 s** (the LEDs show the frequency-locking preview
until then; releasing earlier only toggles frequency locking). A short press of both buttons
does nothing. Single-button long presses are unchanged (3 s).

Calibration procedure (unchanged from stock):

1. Unpatch all CV inputs, hold both buttons for 6 s. The left LED blinks yellow.
2. Patch exactly **1 V** into V/OCT, press the left button. The right LED blinks yellow.
3. Patch exactly **3 V** into V/OCT, press the left button. On success the module returns to
   normal operation and saves the calibration; on failure both LEDs flash red and nothing is
   changed.

In step 1, pressing both buttons briefly instead starts the factory calibration of the
jack-detection threshold — leave that alone unless you know what it does.

Unlike stock, the CV input zero points captured in step 2 are only applied if step 3 succeeds,
so an aborted calibration cannot leave them behind.

### Schliere (third bank)

A long chain of second-order allpass filters with feedback, inspired by the Smoothie Audio
*Smear*: transients smear into chirps, with feedback it becomes a resonator. Port of the
author's Schliere patch for the AC/DC (OWL), in two variants (right button, see below):

| Variant | Chain | Taps L / R (detents 1 … 7) | Feedback path |
|---|---|---|---|
| **S** (default) | 61 stages (122 poles) | L 2 … 98 poles, R 2 … 122 poles | 48 kHz, or 96 kHz as on the AC/DC |
| **L** | 98 stages (196 poles, two Smear sections in series) | L 2 … 158 poles, R 2 … 196 poles | 48 kHz only |

L is variant SchliereL of the AC/DC (160 stages there) cut to what fits the Rings processor:
the smear lasts about 1.6 times as long and has more notches between two detents.

**Selecting it:** hold the right button for 3 s to cycle through the three banks: grounded
models → bonus models (LED 2 pulsing) → **Schliere (LED 2 blinking hard)** → grounded models.

#### Knobs, CVs and jacks

| Control | Function |
|---|---|
| FREQUENCY + V/OCT + FM (CV + att.) | Frequency where the delay acts: continuous over the whole knob, 20 Hz … 20 kHz (≈ 632 Hz at noon, ≈ 11 octaves of travel); V/OCT 1 V/oct with Rings' calibration; FM through its attenuverter |
| STRUCTURE + CV + att. | **Tap**: where L is taken from the chain, seven detents (S: 2, 18, 34, 50, 66, 82, 98 poles; L: 2, 28, 54, 80, 106, 132, 158 poles; R about 5/4 as far down the chain). Each detent has a hold zone; between two detents the taps are crossfaded, which creates phaser-like notches. Turning it changes how long and how chirpy the smear is |
| BRIGHTNESS + CV + att. | **Resonance** (Q 0.01 … 16): low = the bass arrives late, high = a narrow band around FREQUENCY rings on |
| DAMPING + CV + att. | **Feedback**: knob 0 … ≈ 95 %, up to 97 % with CV (soft knee above 90 %). A negative CV inverts the loop (peaks and troughs swap). L goes back to the chain input through a 10 Hz highpass and a soft limiter; the loop never self-oscillates |
| POSITION + CV + att. | **Spread**: distance between the L and R taps. Fully CCW = mono (R = L), noon = classic stereo (R further down the chain), fully CW = R at the chain end (S: 122 poles, L: 196 poles). Hold zones at all three positions |
| STRUM | **Excite**: a trigger at the jack injects an excitation into the chain (flavor: left button). With STRUM **and** IN both unpatched, a V/OCT note change excites instead (as in the resonator models). Onsets of the input never excite |
| IN | Audio input (no noise gate) |
| ODD / EVEN | L / R. With only ODD patched, Rings mixes both outputs in hardware [manual]; L and R are the same chain at different taps, so the mono sum has comb notches and sounds thinner — turn POSITION fully CCW for a clean mono signal |

#### Buttons

| Button | Function |
|---|---|
| Left, short | **Dry/wet**: 100 → 75 → 50 → 25 → 100 % wet (default 100 %). 50 % gives the deepest phaser notches |
| Left, hold 3 s | **Excite flavor**: impulse → mallet → ping → impulse. *Impulse*: a single sample, bright click followed by the chirp. *Mallet*: a smooth pulse one period of FREQUENCY long, so nearly all of it lands where the chain delays (at and below FREQUENCY): almost no click, mostly chirp. *Ping*: a short sine burst at FREQUENCY, about 2 Q cycles long (3–16 cycles, ≤ 40 ms), a pitched "boing", strongest at high BRIGHTNESS. Every hit varies slightly (level ±1 dB, random polarity, mallet width / ping length ±10 %), so repeated triggers do not sound identical |
| Right, press | **Excite** the moment the button goes down (also when it is then held) |
| Right, hold 1–3 s | **Variant**, switched on release: S 48 kHz → S 96 kHz → L → S 48 kHz. S 48 kHz is the default; S 96 kHz is the AC/DC variant S (feedback product oversampled), audibly a bit different, ≈ 9 % more CPU. Changing between S and L fades the wet signal out, restarts the chain silent and fades back in (5 ms each way), so ringing tails are cut |
| Right, hold 3 s | Next bank (no variant change) |
| Both, hold > 0.2 s | Frequency locking (see above). While locked, the FREQUENCY knob snaps to whole octaves of the locked frequency (≈ one octave per 9 % of travel) |
| Both, hold 6 s | Calibration (see above) |

The right button does not excite while the left one is held.

#### LEDs

**LED 2 (right)** blinks hard while the Schliere bank is active. Its color is the variant:
**green = S 48 kHz**, **yellow = S 96 kHz**, **red = L** (the order of the right-button cycle).
While the right button is held between 1 and 3 s, it already shows the variant that will be
selected on release.

**LED 1 (left)** shows one thing at a time, in this order of priority:

| LED 1 | Meaning |
|---|---|
| green / yellow / red for 1.5 s after holding the left button 3 s | excite flavor: green = impulse, yellow = mallet, red = ping |
| red / yellow / green / off for 1.5 s after a short press of the left button | dry/wet: 100 / 75 / 50 / 25 % wet |
| green for 1.5 s while turning STRUCTURE | the tap knob reached one of the seven detents (follows the knob only, not the CV) |
| red, held for 0.3 s | the soft limiter in the feedback loop is working (loud input, high feedback or resonance) — not an error |
| briefly dark | a trigger arrived at STRUM |
| off | nothing to report |

Frequency-locking preview and calibration take over both LEDs as described above.

#### Saved settings and safety

Bank, dry/wet step, excite flavor and the variant are saved and survive a power cycle. After a
rollback to v0.1.0, L becomes S 48 kHz. After a rollback to v0.1.0 or v0.2.0, ping becomes the
impulse and mallet becomes their noise burst (which the mallet replaced); a saved noise burst
reads as mallet after an update.
If Schliere ever overloads the processor (over budget for 0.2 s), the module leaves the
Schliere bank by itself.

Differences from the AC/DC version: CVs are read once per 24-sample block (no audio-rate CV), no
CV dead zone (Rings has calibrated offsets and attenuverters), `tanh` replaced by stmlib's
`SoftClip`, two outputs instead of four, 48 kHz feedback path by default, L with 98 instead of
160 stages and without the 96 kHz path. Spread, excite and dry/wet are new.

### Settings compatibility

The new settings live in formerly unused bytes of the settings block, whose size is unchanged.
Calibration, polyphony and model saved by the stock firmware are kept when updating, and the
stock firmware can still read the settings after a rollback.

## Planned changes

Developed in the (separate) workbench repo `sgnir-workbench`:

1. Schliere: level adjustment after measuring Rings' input/output ranges, worst-case CPU
   load with a signal at the input.
2. New chord tables from the [lylepmills fork](https://github.com/lylepmills/eurorack)
   (optional).

## Credits

- **Mutable Instruments / Émilie Gillet** — Rings firmware, stmlib, stm_audio_bootloader (MIT)
- **Lyle Mills** — alternative Rings firmware features (MIT)
- Schliere DSP (`rings/dsp/schliere.*`, MIT) originates from the author's OWL patch for the Befaco/Rebel Technology AC/DC
  (acdc-workbench project)

## License

MIT — see [LICENSE](LICENSE). Upstream sources keep their original MIT headers
(© Émilie Gillet).
