// Copyright 2015 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
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
// Settings storage.

#ifndef RINGS_SETTINGS_H_
#define RINGS_SETTINGS_H_

#include "stmlib/stmlib.h"

#include "rings/drivers/adc.h"

namespace rings {

// Marks a valid sgnir extension block in the State struct. Settings saved by
// the stock firmware carry arbitrary bytes there (formerly padding).
const uint32_t kStateExtensionId = 0x53474e31;  // 'SGN1'

// Bits of State::schliere_options (default 0: impulse excite, 48 kHz).
const uint8_t kSchliereOptionNoiseBurst = 1;  // excite with a noise burst
const uint8_t kSchliereOptionOversampled = 2;  // feedback path at 96 kHz

struct CalibrationData {
  float pitch_offset;
  float pitch_scale;
  float offset[ADC_CHANNEL_NUM_OFFSETS];
  float normalization_detection_threshold;
  uint8_t padding[4];
};

struct State {
  uint8_t polyphony;
  uint8_t model;
  uint8_t easter_egg;
  uint8_t color_blind;
  
  // sgnir extensions, placed in the former padding so that the layout of the
  // stock fields (and the calibration data) is unchanged.
  uint32_t extension_id;
  float locked_transpose;
  uint8_t frequency_locked;
  uint8_t schliere;  // 1 = the Schliere bank is active
  uint8_t schliere_dry_steps;  // 0..3: wet = 100 / 75 / 50 / 25 %
  uint8_t schliere_options;  // kSchliereOption* bits
};

struct SettingsData {
  CalibrationData calibration_data; // 36 bytes
  State state;  // 16 bytes
  uint8_t padding[8];
};

// The storage layer rejects blocks of a different size: changing it would
// discard the calibration data saved by the stock firmware.
static_assert(sizeof(SettingsData) == 60, "SettingsData must stay 60 bytes");

class Settings {
 public:
  Settings() { }
  ~Settings() { }
  
  void Init();
  void Save();
  
  inline CalibrationData* mutable_calibration_data() {
    return &data_.calibration_data;
  }
  
  inline void ToggleEasterEgg() {
    data_.state.easter_egg = !data_.state.easter_egg;
  }
  
  inline void LockFrequency(float transpose) {
    data_.state.locked_transpose = transpose;
    data_.state.frequency_locked = 1;
  }
  
  inline void UnlockFrequency() {
    data_.state.frequency_locked = 0;
  }
  
  // Dry/wet steps of the Schliere bank: 100, 75, 50, 25 % wet.
  inline float schliere_mix() const {
    return 1.0f - 0.25f * static_cast<float>(data_.state.schliere_dry_steps);
  }
  
  inline bool schliere_option(uint8_t option) const {
    return (data_.state.schliere_options & option) != 0;
  }
  
  inline void ToggleSchliereOption(uint8_t option) {
    data_.state.schliere_options ^= option;
  }
  
  inline void NextSchliereMix() {
    // One step drier: 100 -> 75 -> 50 -> 25 -> 100 % wet.
    uint8_t& dry = data_.state.schliere_dry_steps;
    dry = dry >= 3 ? 0 : dry + 1;
  }
  
  inline State* mutable_state() {
    return &data_.state;
  }

  inline const State& state() const {
    return data_.state;
  }
  
  // True when no calibration data has been found on flash sector 1, that is
  // to say when the module has just been flashed.
  inline bool freshly_baked() const {
    return freshly_baked_;
  }
  
 private:
  void InitStateExtension();
  
  bool freshly_baked_;
  SettingsData data_;
  uint16_t version_token_;
  
  DISALLOW_COPY_AND_ASSIGN(Settings);
};

}  // namespace rings

#endif  // RINGS_SETTINGS_H_
