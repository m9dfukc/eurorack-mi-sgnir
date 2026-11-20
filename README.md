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

### Settings compatibility

The new settings live in formerly unused bytes of the settings block, whose size is unchanged.
Calibration, polyphony and model saved by the stock firmware are kept when updating, and the
stock firmware can still read the settings after a rollback.

## Planned changes

Developed in the (separate) workbench repo `eurorack-mi-rings`:

1. **Schliere** — a chain of 61 second-order allpass filters with feedback (inspired by the
   Smoothie Audio *Smear*), as an additional mode next to the resonator models.
2. New chord tables from the [lylepmills fork](https://github.com/lylepmills/eurorack)
   (optional).

## Credits

- **Mutable Instruments / Émilie Gillet** — Rings firmware, stmlib, stm_audio_bootloader (MIT)
- **Lyle Mills** — alternative Rings firmware features (MIT)
- Schliere DSP originates from the author's OWL patch for the Befaco/Rebel Technology AC/DC
  (acdc-workbench project)

## License

MIT — see [LICENSE](LICENSE). Upstream sources keep their original MIT headers
(© Émilie Gillet).
