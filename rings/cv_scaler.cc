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
// Filtering and scaling of ADC values + input calibration.

#include "rings/cv_scaler.h"

#include <algorithm>

#include "stmlib/dsp/dsp.h"
#include "stmlib/system/storage.h"
#include "stmlib/utils/random.h"

#include "rings/dsp/part.h"
#include "rings/dsp/patch.h"

namespace rings {
  
using namespace std;
using namespace stmlib;

// Same smoothing as Rings' own STRUCTURE CV path for now: unfiltered ADC
// noise on tap/feedback would amplitude-modulate the output. [open] tune.
const float kSchliereCvLpCoefficient = 0.1f;

/* static */
ChannelSettings CvScaler::channel_settings_[ADC_CHANNEL_LAST] = {
  { LAW_LINEAR, true, 1.00f },  // ADC_CHANNEL_CV_FREQUENCY
  { LAW_LINEAR, true, 0.1f },  // ADC_CHANNEL_CV_STRUCTURE
  { LAW_LINEAR, true, 0.1f },  // ADC_CHANNEL_CV_BRIGHTNESS
  { LAW_LINEAR, true, 0.05f },  // ADC_CHANNEL_CV_DAMPING
  { LAW_LINEAR, true, 0.01f },  // ADC_CHANNEL_CV_POSITION
  { LAW_LINEAR, false, 1.00f },  // ADC_CHANNEL_CV_V_OCT
  { LAW_LINEAR, false, 0.01f },  // ADC_CHANNEL_POT_FREQUENCY
  { LAW_LINEAR, false, 0.01f },  // ADC_CHANNEL_POT_STRUCTURE
  { LAW_LINEAR, false, 0.01f },  // ADC_CHANNEL_POT_BRIGHTNESS
  { LAW_LINEAR, false, 0.01f },  // ADC_CHANNEL_POT_DAMPING
  { LAW_LINEAR, false, 0.01f },  // ADC_CHANNEL_POT_POSITION
  { LAW_QUARTIC_BIPOLAR, false, 0.005f },  // ADC_CHANNEL_ATTENUVERTER_FREQUENCY
  { LAW_QUADRATIC_BIPOLAR, false, 0.005f },  //ADC_CHANNEL_ATTENUVERTER_STRUCTURE,
  { LAW_QUADRATIC_BIPOLAR, false, 0.005f },  // ADC_CHANNEL_ATTENUVERTER_BRIGHTNESS,
  { LAW_QUADRATIC_BIPOLAR, false, 0.005f },  // ADC_CHANNEL_ATTENUVERTER_DAMPING,
  { LAW_QUADRATIC_BIPOLAR, false, 0.005f },  // ADC_CHANNEL_ATTENUVERTER_POSITION,
};

void CvScaler::Init(CalibrationData* calibration_data) {
  calibration_data_ = calibration_data;

  adc_.Init();
  trigger_input_.Init();

  transpose_ = 0.0f;
  
  fill(&adc_lp_[0], &adc_lp_[ADC_CHANNEL_LAST], 0.0f);
  copy(
      &calibration_data_->offset[0],
      &calibration_data_->offset[ADC_CHANNEL_NUM_OFFSETS],
      &pending_offset_[0]);
  
  normalization_probe_.Init();
  normalization_detector_exciter_.Init(0.01f, 0.5f);
  normalization_detector_trigger_.Init(0.05f, 0.9f);
  normalization_detector_v_oct_.Init(0.01f, 0.5f);
  
  inhibit_strum_ = 0;
  pot_transpose_ = 0.0f;
  octave_ = 0;
  schliere_octave_ = 0;
  fill(&cv_fast_[0], &cv_fast_[ADC_CHANNEL_NUM_OFFSETS], 0.0f);
  fm_cv_ = 0.0f;
  
  normalization_probe_enabled_ = true;
  normalization_probe_forced_state_ = false;
}

void CvScaler::DetectAudioNormalization(Codec::Frame* in, size_t size) {
  int32_t count = 0;
  short* input_samples = &in->r;
  for (size_t i = 0; i < size; i += 8) {
    short s = input_samples[i * 2];
    if (s > 50 && s < 1500) {
      ++count;
    } else if (s > -1500 && s < -50) {
      --count;
    }
  }
  float y = static_cast<float>(count) / static_cast<float>(size >> 3);
  float x = normalization_probe_value_[1] ? -1.0f : 1.0f;
  
  normalization_detector_exciter_.Process(x, y);
  if (normalization_detector_exciter_.normalized()) {
    for (size_t i = 0; i < size; ++i) {
      input_samples[i * 2] = 0;
    }
  }
}

void CvScaler::DetectNormalization() {
  if (normalization_probe_value_[0] == trigger_input_.DummyRead()) {
    normalization_detector_trigger_.Process(1.0f, 1.0f);
  } else {
    normalization_detector_trigger_.Process(1.0f, -1.0f);
  }
  
  float x = adc_.float_value(ADC_CHANNEL_CV_V_OCT) - calibration_data_->normalization_detection_threshold;
  float y = normalization_probe_value_[0] ? -1.0f : 1.0f;
  if (x > -0.5f && x < 0.5f) {
    x = x < 0.0f ? -1.0f : 1.0f;
    normalization_detector_v_oct_.Process(x, y);
  } else {
    normalization_detector_v_oct_.Process(0.0f, y);
  }
  
  normalization_probe_value_[1] = normalization_probe_value_[0];
  normalization_probe_value_[0] = Random::GetWord() >> 31;
  bool new_state = normalization_probe_enabled_
      ? normalization_probe_value_[0]
      : normalization_probe_forced_state_;
  normalization_probe_.Write(new_state);
}

#define ATTENUVERT(destination, NAME, min, max) \
  { \
    float value = adc_lp_[ADC_CHANNEL_CV_ ## NAME]; \
    value *= adc_lp_[ADC_CHANNEL_ATTENUVERTER_ ## NAME]; \
    value += adc_lp_[ADC_CHANNEL_POT_ ## NAME]; \
    CONSTRAIN(value, min, max) \
    destination = value; \
  }

void CvScaler::Read(
    Patch* patch,
    PerformanceState* performance_state,
    const State& state) {
  // Process all CVs / pots.
  for (size_t i = 0; i < ADC_CHANNEL_LAST; ++i) {
    const ChannelSettings& settings = channel_settings_[i];
    float value = adc_.float_value(i);
    if (settings.remove_offset) {
      value = calibration_data_->offset[i] - value;
    }
    switch (settings.law) {
      case LAW_QUADRATIC_BIPOLAR:
        {
          value = value - 0.5f;
          float value2 = value * value * 4.0f * 3.3f;
          value = value < 0.0f ? -value2 : value2;
        }
        break;

      case LAW_QUARTIC_BIPOLAR:
        {
          value = value - 0.5f;
          float value2 = value * value * 4.0f;
          float value4 = value2 * value2 * 3.3f;
          value = value < 0.0f ? -value4 : value4;
        }
        break;

      default:
        break;
    }
    adc_lp_[i] += settings.lp_coefficient * (value - adc_lp_[i]);
    if (i < ADC_CHANNEL_NUM_OFFSETS) {
      // Lightly filtered copy of the CVs for Schliere, which smooths its
      // modulation paths itself. [hypothesis] tune against ADC noise.
      cv_fast_[i] += kSchliereCvLpCoefficient * (value - cv_fast_[i]);
    }
  }
  
  ATTENUVERT(patch->structure, STRUCTURE, 0.0f, 0.9995f);
  ATTENUVERT(patch->brightness, BRIGHTNESS, 0.0f, 1.0f);
  ATTENUVERT(patch->damping, DAMPING, 0.0f, 1.0f);
  ATTENUVERT(patch->position, POSITION, 0.0f, 1.0f);
  
  float fm = adc_lp_[ADC_CHANNEL_CV_FREQUENCY] * 48.0f;
  float error = fm - fm_cv_;
  if (fabs(error) >= 0.8f) {
    fm_cv_ = fm;
  } else {
    fm_cv_ += 0.02f * error;
  }
  performance_state->fm = fm_cv_ * adc_lp_[ADC_CHANNEL_ATTENUVERTER_FREQUENCY];
  CONSTRAIN(performance_state->fm, -48.0f, 48.0f);
  
  float transpose = 60.0f * adc_lp_[ADC_CHANNEL_POT_FREQUENCY];
  float hysteresis = transpose - transpose_ > 0.0f ? -0.3f : +0.3f;
  transpose_ = static_cast<int32_t>(transpose + hysteresis + 0.5f);
  
  float note = calibration_data_->pitch_offset;
  note += adc_lp_[ADC_CHANNEL_CV_V_OCT] * calibration_data_->pitch_scale;
  
  performance_state->note = note;
  performance_state->tonic = 12.0f + transpose_;
  
  DetectNormalization();
  
  // Strumming / internal exciter triggering logic.
  bool internal_strum = normalization_detector_trigger_.normalized();
  bool internal_exciter = normalization_detector_exciter_.normalized();
  bool internal_note = normalization_detector_v_oct_.normalized();
  performance_state->internal_exciter = internal_exciter;
  performance_state->internal_strum = internal_strum;
  performance_state->internal_note = internal_note;
  performance_state->strum = trigger_input_.rising_edge();
  
  if (performance_state->internal_note) {
    // Remove quantization when nothing is plugged in the V/OCT input.
    performance_state->note = 0.0f;
    performance_state->tonic = 12.0f + transpose;
  }
  
  pot_transpose_ = performance_state->tonic - 12.0f;
  if (state.frequency_locked) {
    // Alt function of the FREQUENCY pot: it keeps its scale but snaps to
    // whole octaves of the locked pitch (no jump at lock time).
    float octave = (pot_transpose_ - state.locked_transpose) / 12.0f;
    hysteresis = octave - octave_ > 0.0f ? -0.1f : +0.1f;
    octave_ = static_cast<int32_t>(floorf(octave + hysteresis + 0.5f));
    performance_state->tonic = 12.0f + state.locked_transpose + \
        12.0f * static_cast<float>(octave_);
  }
  
  // Hysteresis on chord.
  float chord = calibration_data_->offset[ADC_CHANNEL_CV_STRUCTURE] - \
      adc_.float_value(ADC_CHANNEL_CV_STRUCTURE);
  chord *= adc_lp_[ADC_CHANNEL_ATTENUVERTER_STRUCTURE];
  chord += adc_lp_[ADC_CHANNEL_POT_STRUCTURE];
  chord *= static_cast<float>(kNumChords - 1);
  hysteresis = chord - chord_ > 0.0f ? -0.1f : +0.1f;
  chord_ = static_cast<int32_t>(chord + hysteresis + 0.5f);
  CONSTRAIN(chord_, 0, kNumChords - 1);
  performance_state->chord = chord_;
  
  adc_.Convert();
  trigger_input_.Read();
}

void CvScaler::ReadSchliere(
    const Patch& patch,
    const PerformanceState& performance_state,
    const State& state,
    SchliereParameters* p) {
  // Continuous frequency over the whole knob (about 11 octaves), not the
  // semitone-quantized Rings transpose. Frequency locking captures the raw
  // pot position.
  float pot = adc_lp_[ADC_CHANNEL_POT_FREQUENCY];
  pot_transpose_ = 60.0f * pot;
  p->frequency = pot;
  p->octaves = (performance_state.note + performance_state.fm) / 12.0f;
  if (state.frequency_locked) {
    // Alt function: the pot snaps to whole octaves of the locked frequency.
    float locked = state.locked_transpose / 60.0f;
    float octave = (pot - locked) * kSchliereKnobOctaves;
    float hysteresis = octave - schliere_octave_ > 0.0f ? -0.1f : +0.1f;
    schliere_octave_ = static_cast<int32_t>(
        floorf(octave + hysteresis + 0.5f));
    p->frequency = locked;
    p->octaves += static_cast<float>(schliere_octave_);
  }
  
  p->resonance = patch.brightness;
  p->tap = adc_lp_[ADC_CHANNEL_POT_STRUCTURE];
  p->tap_cv = cv_fast_[ADC_CHANNEL_CV_STRUCTURE] * \
      adc_lp_[ADC_CHANNEL_ATTENUVERTER_STRUCTURE];
  p->feedback = adc_lp_[ADC_CHANNEL_POT_DAMPING];
  p->feedback_cv = cv_fast_[ADC_CHANNEL_CV_DAMPING] * \
      adc_lp_[ADC_CHANNEL_ATTENUVERTER_DAMPING];
  p->spread = patch.position;
  p->mix = 1.0f - 0.25f * static_cast<float>(state.schliere_dry_steps);
  
  // A trigger at the STRUM jack excites. With STRUM unpatched, Rings' internal
  // strum (see Strummer) fires on a V/OCT note change or, with V/OCT
  // unpatched, on every onset of the input. Keep only the note change, and
  // only while IN is unpatched too: with IN patched, the internal strum would
  // add transients to the signal being processed.
  p->excite = performance_state.strum && (
      !performance_state.internal_strum ||
      performance_state.internal_exciter);
  p->excite_type = SchliereExciteFromOptions(state.schliere_options);
}

}  // namespace rings
