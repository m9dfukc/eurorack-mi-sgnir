# eurorack-mi-sgnir

A slim, self-contained source tree for building custom firmware for the Mutable Instruments
**Rings** Eurorack module — plus (work in progress) a custom alternative mode. Named *sgnir*
because Mutable Instruments asked firmware forks not to reuse the original module names; folder
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

The `size` make target pipes through `figlet`/`cowsay` — install them or ignore the error.

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
48 kHz, 168 MHz).

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

A long chain of 61 second-order allpass filters with feedback, inspired by the Smoothie Audio
*Smear*: transients smear into chirps, with feedback it becomes a resonator. Port of variant
**S** of the author's Schliere patch for the AC/DC (OWL).

**Selecting it:** a long press (3 s) of the right button cycles through three banks: grounded
models → bonus models (LED 2 pulsing) → **Schliere (LED 2 blinking hard)** → grounded models.

| Control | Function |
|---|---|
| FREQUENCY + V/OCT + FM | Frequency, continuous over the whole knob (20 Hz … 20 kHz), V/OCT 1 V/oct (calibrated) |
| STRUCTURE + CV + att. | Tap: where L is taken from the chain, seven detents with hold zones (crossfades in between create notches) |
| BRIGHTNESS + CV + att. | Resonance (Q 0.01 … 16) |
| DAMPING + CV + att. | Feedback, knob 0 … 95 %; a negative CV inverts the loop |
| POSITION + CV + att. | Spread: CCW mono (R = L), noon classic stereo, CW R at the chain end |
| STRUM | Excite: an impulse or a short noise burst into the chain (only a trigger at the jack) |
| IN | Audio (no noise gate) |
| ODD / EVEN | L / R |
| Left button | Dry/wet: 0 → 25 → 50 → 75 → 100 % wet (default 100 %). 50 % gives the deepest phaser notches |
| Right button (short) | Excite flavor: impulse (LED 2 green) / noise burst (LED 2 yellow) |
| Both buttons | Frequency locking as above; while locked, the FREQUENCY knob snaps to whole octaves of the locked frequency (about one octave per 9 % of travel) |

**LED 1:** after a press of the left button, the dry/wet step for 1.5 s (off / green / yellow /
red / red blinking = 0 / 25 / 50 / 75 / 100 %); after the tap knob reaches a detent, green for
1.5 s; otherwise red while the limiter in the feedback loop works.

Differences from the AC/DC version: CVs are read once per 24-sample block (no audio-rate CV), no
CV dead zone (Rings has calibrated offsets and attenuverters), `tanh` replaced by stmlib's
`SoftClip`, two outputs instead of four. Spread, excite and dry/wet are new.

### Settings compatibility

The new settings live in formerly unused bytes of the settings block, whose size is unchanged.
Calibration, polyphony and model saved by the stock firmware are kept when updating, and the
stock firmware can still read the settings after a rollback.

## Planned changes

Developed in the (separate) workbench repo `eurorack-mi-rings`:

1. Schliere: variants O and A on the right button (the switches are implemented), level
   adjustment after measuring Rings' input/output ranges, CPU profiling on hardware.
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
