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
// Schliere: a long chain of second-order allpass filters with feedback,
// inspired by the Smoothie Audio Smear. Port of the OWL patch from the
// acdc-workbench project (variants S, O, Z, W, A as switches).
//
// Signal flow: input (scaled by sqrt(1 - fb^2)) + fb * loop -> 61 identical
// allpass stages -> two taps (L, R) crossfaded between seven fixed detents.
// L goes back to the input through a 10 Hz highpass and a soft limiter.
//
// Differences from the OWL patch:
// - Knobs and CVs arrive once per block; CVs are ramped over the block.
// - No CV dead zone (Rings has calibrated CV offsets and attenuverters).
// - tanh() in the soft knees is replaced by stmlib's SoftClip().
// - Spread (R tap distance), excite (STRUM) and dry/wet mix are new.
// - Two outputs (L, R) instead of four (no diff and tail outputs).

#ifndef RINGS_DSP_SCHLIERE_H_
#define RINGS_DSP_SCHLIERE_H_

#include "stmlib/stmlib.h"

// Optional placement of the per-sample code, e.g.
// __attribute__((section(".ramtext"), long_call)) for RAM.
// Measured on the module (diag5): running from RAM is 18 % SLOWER than from
// flash with prefetch (instruction fetches and data share the S-bus), so
// this stays empty. Kept as a hook for experiments.
#define SCHLIERE_RAM_CODE

namespace rings {

const size_t kSchliereStages = 61;
const float kSchliereKnobOctaves = 10.965784f;  // ln(2000) / ln(2)
const size_t kSchliereTaps = 7;

enum SchliereVariant {
  SCHLIERE_VARIANT_S,  // O with a lightly smoothed feedback CV
  SCHLIERE_VARIANT_O,  // chain input (feedback product) at 96 kHz
  SCHLIERE_VARIANT_Z,  // feedback amount smoothed at 1.5 kHz
  SCHLIERE_VARIANT_W,  // feedback amount smoothed at 12 kHz
  SCHLIERE_VARIANT_A,  // original: hard edges, no smoothing
};

enum SchliereExcite {
  SCHLIERE_EXCITE_IMPULSE,
  SCHLIERE_EXCITE_NOISE_BURST,
};

struct SchliereParameters {
  float frequency;  // knob, 0..1 = 20 Hz .. 20 kHz (logarithmic)
  float octaves;  // V/OCT + FM, in octaves
  float resonance;  // knob, 0..1 = Q 0.01 .. 16
  float tap;  // knob, 0..1 = detent 1 .. 7
  float tap_cv;  // CV in knob units, bipolar
  float feedback;  // knob, 0..1 = 0 .. 95 %
  float feedback_cv;  // CV in knob units, bipolar (negative = inverted loop)
  float spread;  // 0 = mono, 0.5 = classic L/R layout, 1 = R at the chain end
  float mix;  // 0 = dry, 1 = wet
  bool excite;  // inject an excitation at the start of this block
  SchliereExcite excite_type;
};

// 2x halfband filter: two branches of first-order allpasses, polyphase IIR.
class SchliereHalfband {
 public:
  void Init();

  // Defined here so that it inlines into the per-sample code, where `first`
  // is a constant and the loop unrolls. Coefficients: Valenzuela/
  // Constantinides, elliptic, transition 0.045; flat to 20 kHz, at least
  // 77 dB stopband from 28 kHz.
  inline float Branch(float x, int32_t first) {
    static const float coefficients[kNumCoefficients] = {
      0.06396838f, 0.22734146f, 0.42965512f,
      0.62193724f, 0.78589107f, 0.92904445f
    };
    for (int32_t i = first; i < kNumCoefficients; i += 2) {
      float y = coefficients[i] * (x - y_[i]) + x_[i];
      x_[i] = x;
      y_[i] = y;
      x = y;
    }
    return x;
  }

 private:
  static const int32_t kNumCoefficients = 6;
  float x_[kNumCoefficients];
  float y_[kNumCoefficients];
};

class Schliere {
 public:
  Schliere() { }
  ~Schliere() { }

  void Init(SchliereVariant variant);
  SCHLIERE_RAM_CODE void Process(
      const SchliereParameters& parameters,
      const float* in,
      float* out_l,
      float* out_r,
      size_t size);

  // True while the limiter in the feedback loop works (held for 0.3 s).
  inline bool limiter_active() const { return limiter_blocks_ > 0; }

  // Chain input at 96 kHz (variants S, O). Switchable at runtime for CPU
  // measurements on the module.
  inline void set_oversampled_feedback(bool on) {
    switches_.oversampled_feedback = on;
  }
  inline bool oversampled_feedback() const {
    return switches_.oversampled_feedback;
  }

  // Detent 1..7 the tap knob rests on (with hysteresis), 0 between detents.
  // Follows the knob only, not the CV.
  inline int32_t tap_detent() const { return tap_detent_; }

 private:
  struct Switches {
    bool linear_crossfade;
    bool smooth_tap_weights;
    bool hard_feedback_limit;
    float feedback_smoothing_hz;  // after the knee, 0 = off
    float feedback_cv_smoothing_hz;  // before the knee, 0 = off
    bool oversampled_feedback;
  };

  void Reset();
  void ComputeCoefficients(float knob_f, float knob_q, float octaves,
                           float* g, float* k) const;
  float FeedbackAmount(float knob_plus_cv) const;
  float ChainInput(float in, float cv);
  float ChainInputOversampled(float in, float cv);
  SCHLIERE_RAM_CODE float Excitation();
  void UpdateTapDetent();
  bool Broken() const;

  Switches switches_;

  float state_[2 * kSchliereStages];
  float node_[kSchliereStages + 1];
  float tap_lp_[kSchliereTaps][2];

  // Coefficients, stepped per sample.
  float g_, k_, g_step_, k_step_;

  // Smoothed knobs and ramped CVs.
  float frequency_, resonance_, tap_, feedback_, spread_, mix_;
  float tap_cv_, feedback_cv_;
  float feedback_cv_lp_[2];

  // Feedback loop.
  float loop_lp_;
  float loop_state_;
  float loop_peak_;
  float feedback_now_;
  float feedback_lp_[2];

  // Chain input at 96 kHz.
  SchliereHalfband up_in_, up_loop_, down_;
  float down_odd_;
  float cv_previous_;

  // Excitation.
  float burst_level_;
  float burst_lp_;
  uint32_t rng_state_;
  bool impulse_pending_;

  // Precomputed filter and slew coefficients.
  float loop_hp_;
  float tap_weight_g_;
  float feedback_g_;
  float feedback_cv_g_;
  float slew_sample_;
  float slew_block_;
  size_t slew_block_size_;
  float burst_decay_;
  float burst_lp_coefficient_;

  int32_t limiter_blocks_;
  int32_t tap_detent_;
  bool primed_;

  // Bit j set: tap j has a non-zero smoothed weight. (At the end of the
  // class, so the offsets of the members above stay as they were.)
  uint32_t tap_active_;

  DISALLOW_COPY_AND_ASSIGN(Schliere);
};

}  // namespace rings

#endif  // RINGS_DSP_SCHLIERE_H_
