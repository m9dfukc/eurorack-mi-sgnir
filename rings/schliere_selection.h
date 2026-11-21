// Copyright 2026 m9dfukc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// Schliere options saved in the settings, and the variant chosen with the
// right button. Free of hardware includes, so the host tests cover it.

#ifndef RINGS_SCHLIERE_SELECTION_H_
#define RINGS_SCHLIERE_SELECTION_H_

#include "rings/dsp/schliere.h"

namespace rings {

// Bits of State::schliere_options (default 0: impulse excite, S at 48 kHz).
// Firmware without the L bit (v0.1.0) masks it away and falls back to S.
const uint8_t kSchliereOptionNoiseBurst = 1;  // excite with a noise burst
const uint8_t kSchliereOptionOversampled = 2;  // S: feedback path at 96 kHz
const uint8_t kSchliereOptionLong = 4;  // variant L (48 kHz only)
const uint8_t kSchliereOptionMask = 7;

enum SchliereSelection {
  SCHLIERE_SELECTION_S_48K,
  SCHLIERE_SELECTION_S_96K,
  SCHLIERE_SELECTION_L,
};

// The L bit wins: L has no 96 kHz path.
inline SchliereSelection SchliereSelectionFromOptions(uint8_t options) {
  if (options & kSchliereOptionLong) {
    return SCHLIERE_SELECTION_L;
  }
  return options & kSchliereOptionOversampled
      ? SCHLIERE_SELECTION_S_96K
      : SCHLIERE_SELECTION_S_48K;
}

inline uint8_t SchliereOptionsWithSelection(
    uint8_t options, SchliereSelection selection) {
  options &= ~(kSchliereOptionOversampled | kSchliereOptionLong);
  switch (selection) {
    case SCHLIERE_SELECTION_S_96K:
      return options | kSchliereOptionOversampled;
    case SCHLIERE_SELECTION_L:
      return options | kSchliereOptionLong;
    default:
      return options;
  }
}

// Right button held 1..3 s: S 48 kHz -> S 96 kHz -> L -> S 48 kHz, the order
// of the LED 2 colors green / yellow / red.
inline SchliereSelection NextSchliereSelection(SchliereSelection selection) {
  switch (selection) {
    case SCHLIERE_SELECTION_S_48K:
      return SCHLIERE_SELECTION_S_96K;
    case SCHLIERE_SELECTION_S_96K:
      return SCHLIERE_SELECTION_L;
    default:
      return SCHLIERE_SELECTION_S_48K;
  }
}

inline SchliereChain SchliereChainFor(SchliereSelection selection) {
  return selection == SCHLIERE_SELECTION_L
      ? SCHLIERE_CHAIN_L
      : SCHLIERE_CHAIN_S;
}

}  // namespace rings

#endif  // RINGS_SCHLIERE_SELECTION_H_
