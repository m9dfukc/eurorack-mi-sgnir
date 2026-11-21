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
// Variant L (after SchliereL on the AC/DC, 160 stages there): the same signal
// path with a chain of 98 stages (196 poles, two Smear sections of 98 poles
// in series) and wider tap spacing, 48 kHz only. 82 instead of 62 % of the
// block budget [measured, diag9, flash log 2026-11-21].
//
// Differences from the OWL patch:
// - Knobs and CVs arrive once per block; CVs are ramped over the block.
// - No CV dead zone (Rings has calibrated CV offsets and attenuverters).
// - tanh() in the soft knees is replaced by stmlib's SoftClip().
// - Spread (R tap distance), excite (STRUM: impulse, mallet, ping) and
//   dry/wet mix are new.
// - Two outputs (L, R) instead of four (no diff and tail outputs).

#ifndef RINGS_DSP_SCHLIERE_H_
#define RINGS_DSP_SCHLIERE_H_

#include "stmlib/stmlib.h"

#include "rings/dsp/dsp.h"

// Optional placement of the per-sample code, e.g.
// __attribute__((section(".ramtext"), long_call)) for RAM.
// Measured on the module (diag5): running from RAM is 18 % SLOWER than from
// flash with prefetch (instruction fetches and data share the S-bus), so
// this stays empty. Kept as a hook for experiments.
#define SCHLIERE_RAM_CODE

namespace rings {

const size_t kSchliereStages = 61;  // chain S
const size_t kSchliereStagesL = 98;  // chain L
const float kSchliereKnobOctaves = 10.965784f;  // ln(2000) / ln(2)
const size_t kSchliereTaps = 7;

// Chain length and tap layout. L and R taps are about 4 : 5 in every layout.
enum SchliereChain {
  SCHLIERE_CHAIN_S,  // 61 stages: L every 8 (1 ... 49), R every 10 (1 ... 61)
  SCHLIERE_CHAIN_L,  // 98 stages: L every 13 (1 ... 79), R every 16-17 (1 ... 98)
};

enum SchliereVariant {
  SCHLIERE_VARIANT_S,  // O with a lightly smoothed feedback CV
  SCHLIERE_VARIANT_O,  // chain input (feedback product) at 96 kHz
  SCHLIERE_VARIANT_Z,  // feedback amount smoothed at 1.5 kHz
  SCHLIERE_VARIANT_W,  // feedback amount smoothed at 12 kHz
  SCHLIERE_VARIANT_A,  // original: hard edges, no smoothing
};

// Excitation on a STRUM trigger. Mallet and ping follow the chain's
// frequency (and the ping its resonance), so their energy lands where the
// chain delays: at and below FREQUENCY (research note
// docs/research/notes/excite-signals-2026-11-21.md in the workbench).
enum SchliereExcite {
  SCHLIERE_EXCITE_IMPULSE,  // single sample: bright click, then the chirp
  SCHLIERE_EXCITE_MALLET,  // Hann pulse, one period of FREQUENCY long
  SCHLIERE_EXCITE_PING,  // Hann-windowed sine at FREQUENCY, about 2 Q cycles
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

  // Chain input at 96 kHz (variants S, O). Switchable at runtime. Chain S
  // only: the L chains always run at 48 kHz.
  inline void set_oversampled_feedback(bool on) {
    switches_.oversampled_feedback = on;
  }
  inline bool oversampled_feedback() const {
    return switches_.oversampled_feedback;
  }

  // Selects the chain. Before the first block (after Init) at once;
  // afterwards the wet signal fades out, the chain restarts silent with the
  // new length, and the wet signal fades back in (5 ms each way).
  inline void set_chain(SchliereChain chain) {
    pending_chain_ = chain;
    if (!primed_) {
      chain_ = chain;
    }
  }
  inline SchliereChain chain() const { return chain_; }

  // Detent 1..7 the tap knob rests on (with hysteresis), 0 between detents.
  // Follows the knob only, not the CV.
  inline int32_t tap_detent() const { return tap_detent_; }

  // Random micro-variation of every excitation (level +-1 dB, polarity,
  // mallet width / ping length +-10 %), against identical repeated hits.
  // On after Init(); the host tests switch it off.
  inline void set_excite_variation(bool on) { excite_variation_ = on; }

 private:
  struct Switches {
    bool linear_crossfade;
    bool smooth_tap_weights;
    bool hard_feedback_limit;
    float feedback_smoothing_hz;  // after the knee, 0 = off
    float feedback_cv_smoothing_hz;  // before the knee, 0 = off
    bool oversampled_feedback;
  };

  // Per-block values for the sample loop.
  struct Block {
    float a1, a2, a1_step, a2_step;
    float tap_cv_step, feedback_cv_step;
    bool exciting;
    bool first;
    uint32_t tap_active;
  };

  void Reset();
  void ComputeCoefficients(float knob_f, float knob_q, float octaves,
                           float* g, float* k) const;
  float FeedbackAmount(float knob_plus_cv) const;
  float ChainInput(float in, float cv);
  float ChainInputOversampled(float in, float cv);
  void StartExcitation(SchliereExcite type, float g, float k);
  bool RenderExcitation(size_t size);
  // The sample loop, one instance per chain layout (Layout: see schliere.cc).
  template<typename Layout>
  void Run(const SchliereParameters& parameters, const float* in,
           float* out_l, float* out_r, size_t size, Block* block);
  void UpdateTapDetent();
  bool Broken() const;

  Switches switches_;

  // Stage states first: every state of the L chain (784 bytes) stays
  // within the +-1020 byte offset range of vldr/vstr. Placed behind the
  // scalars, GCC kept one precomputed address per stage on the stack and
  // reloaded it in every stage: S 64 instead of 59 % [measured, diag8,
  // flash log 2026-11-21].
  float state_[2 * kSchliereStagesL];
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

  // Excitation: one generator, a retrigger restarts it. The Hann window
  // 0.5 (1 - cos(n theta)) and the ping's sine run as recursions, so no trig
  // call per sample.
  SchliereExcite excite_type_;
  int32_t excite_remaining_;  // samples left, 0 = idle
  float excite_amplitude_;
  float window_cos_;  // cos(n theta)
  float window_cos_previous_;  // cos((n - 1) theta)
  float window_cos_theta_x2_;  // 2 cos(theta)
  float ping_cos_, ping_sin_;  // cos / sin(n omega0)
  float ping_rotation_cos_, ping_rotation_sin_;
  uint32_t rng_state_;
  bool excite_variation_;

  // Precomputed filter and slew coefficients.
  float loop_hp_;
  float tap_weight_g_;
  float feedback_g_;
  float feedback_cv_g_;
  float slew_sample_;
  float slew_block_;
  size_t slew_block_size_;

  int32_t limiter_blocks_;
  int32_t tap_detent_;
  bool primed_;

  // Bit j set: tap j has a non-zero smoothed weight.
  uint32_t tap_active_;

  // Chain selection and the wet gate that fades around a change.
  SchliereChain chain_;
  SchliereChain pending_chain_;
  float gate_;
  float gate_step_;

  // Chain output of the last sample (the taps' nodes live on the stack in
  // Run(), they are only needed within one sample).
  float chain_end_;

  // The excitation of the current block, rendered before the sample loop:
  // the loop only adds it, the unrolled chain stays untouched.
  float excite_buffer_[kMaxBlockSize];

  DISALLOW_COPY_AND_ASSIGN(Schliere);
};

}  // namespace rings

#endif  // RINGS_DSP_SCHLIERE_H_
