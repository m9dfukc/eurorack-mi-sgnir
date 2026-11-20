# Installing the firmware (audio update)

Rings updates its firmware through its **audio bootloader**: you play a WAV file into the IN
jack. No programmer and no opening of the module are needed. The bootloader itself is never
overwritten, so a failed update can always be repeated, and you can always go back to the
official Mutable Instruments firmware the same way.

Calibration and settings should be kept: they live in a flash sector the update does not
touch (derived from the bootloader source, not yet confirmed on hardware).

## What you need

- The file `sgnir-<version>.wav` from the release.
- Something that plays a WAV file at a decent level: a computer or phone with a headphone
  output, and a cable from there to the IN jack of Rings (3.5 mm to 3.5 mm mono patch cable or
  a headphone-to-Eurorack adapter cable).
- Turn off everything that alters the sound: equalizers, "sound enhancers", volume
  normalization, Bluetooth. Mute notifications.

## Steps

1. Power the case off. Patch the audio output of the player into **IN**. Leave the other jacks
   unpatched.
2. Hold the **left button** and power the case on. Release the button. Both LEDs blink yellow
   in alternation: the bootloader waits for audio.
3. Turn the **FREQUENCY** knob to about noon. In the bootloader it sets the input gain.
4. Start playing the WAV at a fairly high volume.
   - While data is received, LED 1 blinks green and LED 2 shows the level: it should be mostly
     **green to yellow**. Red means too loud; nothing or a failure after some seconds usually
     means too quiet.
   - Both LEDs light briefly yellow whenever a block is written to flash.
5. When the file has played to the end, Rings restarts by itself with the new firmware.

The WAV is about 1.5 minutes long.

## If it fails

- **Both LEDs blink red:** a reception error. Press the **left button** once: the bootloader
  restarts reception (LEDs blink yellow again). Adjust the volume (or FREQUENCY) and play the
  WAV again **from the beginning**. Do not power-cycle in between; the firmware is incomplete at
  this point (harmless, but it would not start properly).
- **Nothing happens, Rings starts normally:** the bootloader was not active. The left button
  must already be held while the module is powered on.
- The most common causes: playback too quiet or too loud, sound processing on the player,
  notification sounds during playback.

## Going back to the official firmware

Play the official Rings firmware WAV the same way (Mutable Instruments' last release is 1.1).
The official firmware simply ignores the settings added by sgnir.

## The .bin file

`sgnir-<version>.bin` is the same firmware as a raw image for flashing over SWD (link address
`0x08008000`, behind the bootloader). You only need it with a debugger/programmer; for the
normal update use the WAV.
